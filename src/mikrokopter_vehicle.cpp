// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mikrokopter_vehicle.h>

constexpr std::chrono::milliseconds
    Mikrokopter_vehicle::TELEMETRY_REQUEST_INTERVAL,
    Mikrokopter_vehicle::TELEMETRY_REPORT_INTERVAL,
    Mikrokopter_vehicle::LINK_MONITOR_INTERVAL;

Mikrokopter_vehicle::Mikrokopter_vehicle(Mikrokopter_protocol::Ptr protocol):
    Vehicle(ugcs::vsm::mavlink::MAV_TYPE_QUADROTOR,
            static_cast<ugcs::vsm::mavlink::MAV_AUTOPILOT>(ugcs::vsm::mavlink::ugcs::MAV_AUTOPILOT_MIKROKOPTER),
            ugcs::vsm::Vehicle::Capabilities(ugcs::vsm::Vehicle::Capability::RETURN_HOME_AVAILABLE),
            protocol->Get_serial_number(),
            "MikroKopter"),
    protocol(protocol),
    sys_status(true, true, Sys_status::Control_mode::UNKNOWN,
               Sys_status::State::UNKNOWN, std::chrono::seconds(0))
{
    VEHICLE_LOG_INF(*this, "MikroKopter vehicle connected.");
    Set_system_status(sys_status);
    auto props = ugcs::vsm::Properties::Get_instance();
    wp_event_value = props->Get_int("vehicle.mikrokopter.wp_event_value");
    if (wp_event_value < 0 || wp_event_value > 255) {
        VEHICLE_LOG_ERR(*this, "wp_event_value out of range, disabling WP events");
        wp_event_value = 0;
    }
}

void
Mikrokopter_vehicle::On_enable()
{
    protocol->Get_pkt_stats(last_pkts_received, last_pkts_error);

    telemetry_stream = protocol->Subscribe(Mikrokopter_protocol::Command_id::OSD_DATA,
                                           Mikrokopter_protocol::Address::NC);
    Schedule_telemetry_read();
    Request_telemetry();

    telemetry_timer = ugcs::vsm::Timer_processor::Get_instance()->Create_timer(
        TELEMETRY_REQUEST_INTERVAL,
        ugcs::vsm::Make_callback(&Mikrokopter_vehicle::Request_telemetry, this),
        Get_completion_ctx());

    link_monitor_timer = ugcs::vsm::Timer_processor::Get_instance()->Create_timer(
        LINK_MONITOR_INTERVAL,
        ugcs::vsm::Make_callback(&Mikrokopter_vehicle::Link_monitor_timer, this),
        Get_completion_ctx());
}

void
Mikrokopter_vehicle::On_disable()
{
    auto req = ugcs::vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Mikrokopter_vehicle::On_disable_handler,
                    Shared_from_this(),
                    req));
    Get_processing_ctx()->Submit_request(req);
    req->Wait_done(false);
}

void
Mikrokopter_vehicle::On_disable_handler(ugcs::vsm::Request::Ptr request)
{
    link_monitor_timer->Cancel();
    link_monitor_timer = nullptr;
    telemetry_stream->Close();
    telemetry_timer->Cancel();

    task_upload_op.Abort();
    mission_clear_op.Abort();
    telemetry_request_op.Abort();
    echo_request_op.Abort();
    telemetry_stream_read_op.Abort();
    command_op.Abort();

    request->Complete();
}

void
Mikrokopter_vehicle::Handle_vehicle_request(ugcs::vsm::Vehicle_task_request::Handle request)
{
    /* Firstly clear current mission. */
    proto::Data<proto::Point> data;
    data->type = proto::Point_type::INVALID;
    task_upload_op.Abort();
    task_upload_op = protocol->Command(Mikrokopter_protocol::Command_id::SEND_WP,
                      Mikrokopter_protocol::Address::NC,
                      std::move(data),
                      Mikrokopter_protocol::Command_id::SEND_WP_RESP,
                      Mikrokopter_protocol::Make_data_handler(
                          &Mikrokopter_vehicle::On_task_upload, this,
                          std::make_shared<State_info_upload_task>(request)),
                      Get_completion_ctx());
}

double
Mikrokopter_vehicle::Lookup_wait(const std::vector<ugcs::vsm::Action::Ptr> &actions,
                                 size_t idx)
{
    while (idx < actions.size()) {
        ugcs::vsm::Action::Ptr action = actions[idx];
        if (action->Get_type() == ugcs::vsm::Action::Type::MOVE) {
            break;
        }
        if (action->Get_type() == ugcs::vsm::Action::Type::WAIT) {
            return action->Get_action<ugcs::vsm::Action::Type::WAIT>()->wait_time;
        }
        idx++;
    }
    return 0;
}

bool
Mikrokopter_vehicle::Retransmit_waypoint(State_info_upload_task::Ptr si,
                                         proto::Data<proto::Point> &wp)
{
    si->num_retrans++;
    if (si->num_retrans > MAX_RETRANS) {
        return false;
    }
    if (!si->num_act_processed) {
        /* Retransmit clear request. */
        wp->type = proto::Point_type::INVALID;
        return true;
    }
    if (si->panorama_range != 0.0) {
        si->panorama_angle -= si->panorama_step;
        si->num_uploaded--;
        Create_panorama_wp(si, wp);
        return true;
    }
    si->num_act_processed--;
    si->num_uploaded--;
    Create_waypoint(si, wp, true);
    return true;
}

bool
Mikrokopter_vehicle::Create_panorama_wp(State_info_upload_task::Ptr si,
                                        proto::Data<proto::Point> &wp)
{
    double resid = si->panorama_range - si->panorama_angle + si->panorama_step;
    if (fabs(resid) < fabs(si->panorama_range) / 100.0) {
        si->panorama_range = 0.0;
        return false;
    }
    if (fabs(resid) > fabs(si->panorama_step)) {
        resid = si->panorama_step;
    }

    wp->position = proto::Gps_pos::From_position(*si->last_position);
    wp->tolerance_radius = si->tolerance_radius;
    wp->speed = si->speed * 10;
    wp->altitude_rate = si->speed * 10;
    wp->wp_event_channel_value = 100;
    wp->cam_angle = si->cur_camera_angle;
    wp->type = proto::Point_type::WP;
    wp->index = si->num_uploaded + 1;
    wp->name[0] = 'P';

    int hdg = si->cur_heading + si->panorama_angle * 180.0 / M_PI +
              (si->panorama_range > 0 ? 0.5 : -0.5);
    if (hdg > 359) {
        hdg -= 360;
    } else if (hdg < 0) {
        hdg += 360;
    }
    if (hdg == 0) {
        hdg = 1;
    }
    wp->heading = hdg;

    wp->hold_time = si->panorama_delay + 0.5;
    si->panorama_angle += resid;
    si->num_uploaded++;
    return true;
}

bool
Mikrokopter_vehicle::Create_waypoint(State_info_upload_task::Ptr si,
                                     proto::Data<proto::Point> &wp,
                                     bool is_retrans)
{
    if (!is_retrans) {
        si->num_retrans = 0;
    }
    if (si->panorama_range != 0.0) {
        if (Create_panorama_wp(si, wp)) {
            return true;
        }
    }
    while (si->num_act_processed < si->request->actions.size()) {
        ugcs::vsm::Action::Ptr action = si->request->actions[si->num_act_processed];
        si->num_act_processed++;

        if (action->Get_type() == ugcs::vsm::Action::Type::CHANGE_SPEED) {
            ugcs::vsm::Change_speed_action::Ptr a =
                action->Get_action<ugcs::vsm::Action::Type::CHANGE_SPEED>();
            si->speed = a->speed;

        } else if (action->Get_type() == ugcs::vsm::Action::Type::MOVE) {
            ugcs::vsm::Move_action::Ptr a =
                action->Get_action<ugcs::vsm::Action::Type::MOVE>();
            wp->hold_time = Lookup_wait(si->request->actions,
                                        si->num_act_processed + 1);
            wp->type = proto::Point_type::WP;
            wp->index = si->num_uploaded + 1;
            if (a->acceptance_radius < 0.1) {
                /* Set default. */
                si->tolerance_radius = 4;
            } else if (a->acceptance_radius < 1.0) {
                /* Set minimum. */
                si->tolerance_radius = 1;
            } else {
                /* Round the given value. */
                si->tolerance_radius = a->acceptance_radius + 0.5;
            }
            wp->tolerance_radius = si->tolerance_radius;
            wp->heading = si->cur_heading;
            ugcs::vsm::Geodetic_tuple pos = a->position.Get_geodetic();
            pos.altitude -= si->launch_elevation;
            si->last_position = pos;
            wp->position = proto::Gps_pos::From_position(pos);
            wp->speed = si->speed * 10;
            wp->altitude_rate = si->speed * 10;
            wp->wp_event_channel_value = 0;
            wp->cam_angle = si->cur_camera_angle;
            wp->name[0] = 'P';
            si->num_uploaded++;
            return true;

        } else if (action->Get_type() == ugcs::vsm::Action::Type::POI) {
            ugcs::vsm::Poi_action::Ptr a =
                action->Get_action<ugcs::vsm::Action::Type::POI>();
            if (a->active) {
                wp->type = proto::Point_type::POI;
                ugcs::vsm::Geodetic_tuple pos = a->position.Get_geodetic();
                pos.altitude -= si->launch_elevation;
                wp->position = proto::Gps_pos::From_position(pos);
                wp->index = si->num_uploaded + 1;
                wp->name[0] = 'P';
                si->cur_heading = -wp->index;
                si->num_uploaded++;
                return true;
            }
            si->cur_heading = 0;

        } else if (action->Get_type() == ugcs::vsm::Action::Type::HEADING) {
            ugcs::vsm::Heading_action::Ptr a =
                action->Get_action<ugcs::vsm::Action::Type::HEADING>();
            si->cur_heading = a->heading * 180.0 / M_PI;

        } else if (action->Get_type() == ugcs::vsm::Action::Type::WAIT) {
            ugcs::vsm::Wait_action::Ptr a =
                action->Get_action<ugcs::vsm::Action::Type::WAIT>();
            if (si->last_position) {
                wp->position = proto::Gps_pos::From_position(*si->last_position);
                wp->tolerance_radius = si->tolerance_radius;
                wp->speed = si->speed * 10;
                wp->altitude_rate = si->speed * 10;
                wp->wp_event_channel_value = 0;
                wp->cam_angle = si->cur_camera_angle;
                wp->type = proto::Point_type::WP;
                wp->index = si->num_uploaded + 1;
                wp->name[0] = 'P';
                wp->heading = si->cur_heading;
                wp->hold_time = a->wait_time;
                si->num_uploaded++;
                return true;
            }

        } else if (action->Get_type() == ugcs::vsm::Action::Type::PANORAMA) {
            ugcs::vsm::Panorama_action::Ptr a =
                action->Get_action<ugcs::vsm::Action::Type::PANORAMA>();

            si->panorama_range = a->angle;
            if (a->angle > 0) {
                if (a->step == 0) {
                    si->panorama_step = M_PI * 10.0 / 180.0;
                } else {
                    si->panorama_step = a->step;
                }
            } else {
                if (a->step == 0) {
                    si->panorama_step = -M_PI * 10.0 / 180.0;
                } else {
                    si->panorama_step = -a->step;
                }
            }

            si->panorama_delay = static_cast<double>(a->delay.count()) / 1000.0 ;
            if (si->panorama_delay > 255) {
                si->panorama_delay = 255;
            }

            si->panorama_angle = si->panorama_step;

            if (si->cur_heading < 0) {
                si->cur_heading = 0;
            }

            wp->position = proto::Gps_pos::From_position(*si->last_position);
            wp->tolerance_radius = si->tolerance_radius;
            wp->speed = si->speed * 10;
            wp->altitude_rate = si->speed * 10;
            wp->wp_event_channel_value = 0;
            wp->cam_angle = si->cur_camera_angle;
            wp->type = proto::Point_type::WP;
            wp->index = si->num_uploaded + 1;
            wp->name[0] = 'P';
            wp->heading = si->cur_heading;
            if (wp->heading.Get() == 0) {
                wp->heading = 1;
            }
            wp->hold_time = si->panorama_delay + 0.5;
            si->num_uploaded++;
            return true;

        } else if (action->Get_type() == ugcs::vsm::Action::Type::CAMERA_CONTROL) {
            ugcs::vsm::Camera_control_action::Ptr a =
                action->Get_action<ugcs::vsm::Action::Type::CAMERA_CONTROL>();
            si->cur_camera_angle = a->tilt * 180.0 / M_PI;

        } else if (action->Get_type() == ugcs::vsm::Action::Type::CAMERA_TRIGGER) {
            if (si->last_position && wp_event_value) {
                wp->position = proto::Gps_pos::From_position(*si->last_position);
                wp->tolerance_radius = si->tolerance_radius;
                wp->speed = si->speed * 10;
                wp->altitude_rate = si->speed * 10;
                wp->wp_event_channel_value = wp_event_value;
                wp->cam_angle = si->cur_camera_angle;
                wp->type = proto::Point_type::WP;
                wp->index = si->num_uploaded + 1;
                wp->name[0] = 'P';
                wp->heading = si->cur_heading;
                int wp_time = std::ceil(wp_event_value * 8.0 / 100.0);
                wp->hold_time = wp_time;
                si->num_uploaded++;
                return true;
            }
        }
    }
    return false;
}

void
Mikrokopter_vehicle::On_task_upload(ugcs::vsm::Io_result result,
                                    Mikrokopter_protocol::Data_ptr data,
                                    State_info_upload_task::Ptr si)
{
    proto::Data<proto::Point> wp;
    if (result != ugcs::vsm::Io_result::OK ||
        proto::Data<proto::Send_wp_response>(*data)->count != si->num_uploaded) {

        if (result == ugcs::vsm::Io_result::TIMED_OUT ||
            (data && proto::Data<proto::Send_wp_response>(*data)->count == si->num_uploaded - 1)) {

            if (Retransmit_waypoint(si, wp)) {
                if (si->num_uploaded) {
                    VEHICLE_LOG_WRN(*this, "Waypoint [%zu] uploading failed, retrying",
                                si->num_uploaded - 1);
                } else {
                    VEHICLE_LOG_WRN(*this, "Waypoint clear request failed, retrying");
                }
            } else {
                VEHICLE_LOG_WRN(*this, "Mission upload failed after retransmissions");
                protocol->Close();
                return;
            }
        } else {
            if (data) {
                VEHICLE_LOG_WRN(*this, "Mission upload failed, result [%d] count [%d] num_uploaded [%zu]",
                            result, proto::Data<proto::Send_wp_response>(*data)->count,
                            si->num_uploaded);
            } else {
                VEHICLE_LOG_WRN(*this, "Mission upload failed, result [%d]", result);
            }
            protocol->Close();
            return;
        }
    } else if (!Create_waypoint(si, wp)) {
        /* All actions processed. */
        si->Complete();
        return;
    }
    task_upload_op.Abort();
    task_upload_op = protocol->Command(Mikrokopter_protocol::Command_id::SEND_WP,
                      Mikrokopter_protocol::Address::NC,
                      std::move(wp),
                      Mikrokopter_protocol::Command_id::SEND_WP_RESP,
                      Mikrokopter_protocol::Make_data_handler(
                          &Mikrokopter_vehicle::On_task_upload, this, si),
                      Get_completion_ctx());
}

void
Mikrokopter_vehicle::Handle_vehicle_request(ugcs::vsm::Vehicle_clear_all_missions_request::Handle request)
{
    proto::Data<proto::Point> data;
    data->type = proto::Point_type::INVALID;
    /* Send waypoint with invalid position which is treated as clearing. */
    mission_clear_op.Abort();
    mission_clear_op = protocol->Command(Mikrokopter_protocol::Command_id::SEND_WP,
                      Mikrokopter_protocol::Address::NC,
                      std::move(data),
                      Mikrokopter_protocol::Command_id::SEND_WP_RESP,
                      Mikrokopter_protocol::Make_data_handler(
                          &Mikrokopter_vehicle::On_mission_clear, this,
                          std::make_shared<State_info_clear_mission>(request)),
                      Get_completion_ctx());
}

void
Mikrokopter_vehicle::Handle_vehicle_request(ugcs::vsm::Vehicle_command_request::Handle request)
{
    if (request->Get_type() == ugcs::vsm::Vehicle_command::Type::RETURN_HOME) {
        proto::Data<proto::Point> data;
            data->type = proto::Point_type::INVALID;
            /* Send waypoint with invalid position which is treated as clearing. */
            command_op.Abort();
            command_op = protocol->Command(
                Mikrokopter_protocol::Command_id::SEND_WP,
                Mikrokopter_protocol::Address::NC,
                std::move(data),
                Mikrokopter_protocol::Command_id::SEND_WP_RESP,
                Mikrokopter_protocol::Make_data_handler(
                    &Mikrokopter_vehicle::On_command, this,
                    std::make_shared<State_info_command>(request)),
                Get_completion_ctx());
    }
}

void
Mikrokopter_vehicle::On_mission_clear(ugcs::vsm::Io_result result,
                                      Mikrokopter_protocol::Data_ptr data,
                                      State_info_clear_mission::Ptr si)
{
    if (result == ugcs::vsm::Io_result::OK &&
        proto::Data<proto::Send_wp_response>(*data)->count == 0) {

        si->Complete();
    } else {
        VEHICLE_LOG_WRN(*this, "Mission clear failed");
        protocol->Close();
    }
}

void
Mikrokopter_vehicle::On_command(ugcs::vsm::Io_result result,
                                Mikrokopter_protocol::Data_ptr data,
                                State_info_command::Ptr si)
{
    if (si->request->Get_type() == ugcs::vsm::Vehicle_command::Type::RETURN_HOME) {
        if (result == ugcs::vsm::Io_result::OK &&
            proto::Data<proto::Send_wp_response>(*data)->count == 0) {

            si->Complete();
        } else {
            VEHICLE_LOG_WRN(*this, "Mission clear failed in scope of RETURN_HOME command");
            protocol->Close();
        }
    }
}

bool
Mikrokopter_vehicle::Request_telemetry()
{
    proto::Data<proto::Osd_request> data;
    data->interval =
        std::chrono::milliseconds(TELEMETRY_REPORT_INTERVAL).count() / 10;
    telemetry_request_op.Abort();
    telemetry_request_op = protocol->Command(
            Mikrokopter_protocol::Command_id::REQUEST_OSD_DATA,
            Mikrokopter_protocol::Address::NC,
            std::move(data),
            Mikrokopter_protocol::Command_id::NONE,
            ugcs::vsm::Make_dummy_callback<void, ugcs::vsm::Io_result,
                                     Mikrokopter_protocol::Data_ptr>(),
            Get_completion_ctx());
    return true;
}

void
Mikrokopter_vehicle::Schedule_telemetry_read()
{
    telemetry_stream_read_op.Abort();
    telemetry_stream_read_op = telemetry_stream->Read(
        Mikrokopter_protocol::Stream::Make_packet_handler(
            &Mikrokopter_vehicle::On_telemetry, this),
        Get_completion_ctx());
}

void
Mikrokopter_vehicle::On_telemetry(Mikrokopter_protocol::Data_ptr data)
{
    auto nav = proto::Data<proto::Navi_data>(*data);
    auto report = Open_telemetry_report();

    report->Set<ugcs::vsm::tm::Battery_voltage>(static_cast<double>(nav->bat_voltage) / 10);

    report->Set<ugcs::vsm::tm::Battery_current>(static_cast<double>(nav->current) / 10);

    report->Set<ugcs::vsm::tm::Position>(nav->current_position.Get_position_telemetry());

    report->Set<ugcs::vsm::tm::Gps_satellites_count>(nav->satellites_in_use);

    report->Set<ugcs::vsm::tm::Ground_speed>(static_cast<double>(nav->ground_speed) / 100);

    report->Set<ugcs::vsm::tm::Course>(nav->heading.Get() / 180.0 * M_PI);

    report->Set<ugcs::vsm::tm::Attitude::Pitch>(-static_cast<double>(nav->pitch) / 180.0 * M_PI);

    report->Set<ugcs::vsm::tm::Attitude::Roll>(-static_cast<double>(nav->roll) / 180.0 * M_PI);

    report->Set<ugcs::vsm::tm::Attitude::Yaw>(static_cast<double>(nav->yaw) / 180.0 * M_PI);

    report->Set<ugcs::vsm::tm::Climb_rate>(nav->variometer.Get());//XXX units?

    /* No information about units. 1:21 ratio was found experimentally. */
    report->Set<ugcs::vsm::tm::Rel_altitude>(static_cast<double>(nav->altimeter.Get()) / 20.0);

    if (nav->nc_flags & static_cast<uint8_t>(proto::Nc_flags::CH)) {
        sys_status.control_mode = Sys_status::Control_mode::AUTO;
    } else {
        sys_status.control_mode = Sys_status::Control_mode::MANUAL;
    }
    if (nav->fc_status_flags & static_cast<uint8_t>(proto::Fc_status_flags::MOTOR_RUN)) {
        sys_status.state = Sys_status::State::ARMED;
    } else {
        sys_status.state = Sys_status::State::DISARMED;
    }
    Set_system_status(sys_status);

    Schedule_telemetry_read();
}

bool
Mikrokopter_vehicle::Link_monitor_timer()
{
    size_t pkts_received, pkts_error;
    protocol->Get_pkt_stats(pkts_received, pkts_error);
    int num_expected = LINK_MONITOR_INTERVAL / TELEMETRY_REPORT_INTERVAL;
    double d_rcvd = static_cast<double>(pkts_received) - last_pkts_received;
    double d_error = static_cast<double>(pkts_error) - last_pkts_error;
    if (d_rcvd < num_expected) {
        VEHICLE_LOG_DBG(*this, "DL packet loss detected: %d packets",
                static_cast<int>(num_expected - d_rcvd));
        d_error += num_expected - d_rcvd;
    } else if (d_rcvd > num_expected) {
        num_expected = d_rcvd;
    }
    last_pkts_received = pkts_received;
    last_pkts_error = pkts_error;
    if (d_rcvd < 0 || d_error < 0 || d_error > num_expected) {
        return true;
    }
    auto report = Open_telemetry_report();
    double q = 1.0 - d_error / num_expected;

    last_link_quality = last_link_quality * (1.0 - LINK_QUALITY_RA_QUOT) +
        LINK_QUALITY_RA_QUOT * q;
    report->Set<ugcs::vsm::tm::Link_quality>(last_link_quality);

    if (!echo_active) {
        echo_active = true;
        proto::Data<proto::Echo> data;
        data->pattern = ECHO_PATTERN;
        echo_request_op.Abort();
        echo_request_op = protocol->Command(
                          Mikrokopter_protocol::Command_id::LINK_TEST,
                          Mikrokopter_protocol::Address::NC,
                          std::move(data),
                          Mikrokopter_protocol::Command_id::LINK_TEST_RESP,
                          Mikrokopter_protocol::Make_data_handler(
                              &Mikrokopter_vehicle::Echo_handler, this),
                          Get_completion_ctx(),
                          std::chrono::milliseconds(200),
                          15);
    }
    return true;
}

void
Mikrokopter_vehicle::Echo_handler(ugcs::vsm::Io_result result,
                                  Mikrokopter_protocol::Data_ptr pkt)
{
    if (result != ugcs::vsm::Io_result::OK ||
        proto::Data<proto::Echo>(*pkt)->pattern != ECHO_PATTERN) {

        lost_echo_count++;
        if (lost_echo_count > MAX_ECHO_LOST) {
            VEHICLE_LOG_WRN(*this, "MikroKopter device link lost");
            protocol->Close();
        } else {
            VEHICLE_LOG_WRN(*this, "Echo response lost: %d", lost_echo_count);
            echo_active = false;
        }
    } else {
        echo_active = false;
        if (lost_echo_count) {
            VEHICLE_LOG_INF(*this, "Echo reception continued after %d loses",
                     lost_echo_count);
            lost_echo_count = 0;
        }
    }
}
