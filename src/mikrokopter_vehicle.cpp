// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mikrokopter_vehicle.h>

constexpr std::chrono::milliseconds
    Mikrokopter_vehicle::TELEMETRY_REQUEST_INTERVAL,
    Mikrokopter_vehicle::TELEMETRY_REPORT_INTERVAL,
    Mikrokopter_vehicle::LINK_MONITOR_INTERVAL;

Mikrokopter_vehicle::Mikrokopter_vehicle(Mikrokopter_protocol::Ptr protocol):
    Vehicle(vsm::mavlink::MAV_TYPE_QUADROTOR,
            static_cast<vsm::mavlink::MAV_AUTOPILOT>(vsm::mavlink::ugcs::MAV_AUTOPILOT_MIKROKOPTER),
            protocol->Get_serial_number(),
            "MikroKopter"),
    protocol(protocol)
{
    LOG_INFO("MikroKopter vehicle connected, serial number '%s'",
             serial_number.c_str());
    Set_system_status(static_cast<vsm::mavlink::MAV_MODE_FLAG>(0) /*XXX*/,
                      vsm::mavlink::MAV_STATE_ACTIVE /*XXX*/,
                      Custom_mode(true, true),
                      std::chrono::seconds(0));
}

void
Mikrokopter_vehicle::On_enable()
{
    protocol->Get_pkt_stats(last_pkts_received, last_pkts_error);

    telemetry_stream = protocol->Subscribe(Mikrokopter_protocol::Command_id::OSD_DATA,
                                           Mikrokopter_protocol::Address::NC);
    Schedule_telemetry_read();
    Request_telemetry();

    telemetry_timer = vsm::Timer_processor::Get_instance()->Create_timer(
        TELEMETRY_REQUEST_INTERVAL,
        vsm::Make_callback(&Mikrokopter_vehicle::Request_telemetry, this),
        Get_completion_ctx());

    link_monitor_timer = vsm::Timer_processor::Get_instance()->Create_timer(
        LINK_MONITOR_INTERVAL,
        vsm::Make_callback(&Mikrokopter_vehicle::Link_monitor_timer, this),
        Get_completion_ctx());
}

void
Mikrokopter_vehicle::On_disable()
{
    auto req = vsm::Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Mikrokopter_vehicle::On_disable_handler,
                    Shared_from_this(),
                    req));
    Get_processing_ctx()->Submit_request(req);
    req->Wait_done(false);
}

void
Mikrokopter_vehicle::On_disable_handler(vsm::Request::Ptr request)
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

    request->Complete();
}

void
Mikrokopter_vehicle::Handle_vehicle_request(vsm::Vehicle_task_request::Handle request)
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
Mikrokopter_vehicle::Lookup_wait(const std::vector<vsm::Action::Ptr> &actions,
                                 size_t idx)
{
    while (idx < actions.size()) {
        vsm::Action::Ptr action = actions[idx];
        if (action->Get_type() == vsm::Action::Type::MOVE) {
            break;
        }
        if (action->Get_type() == vsm::Action::Type::WAIT) {
            return action->Get_action<vsm::Action::Type::WAIT>()->wait_time;
        }
        idx++;
    }
    return 0;
}

bool
Mikrokopter_vehicle::Create_waypoint(State_info_upload_task::Ptr si,
                                     proto::Data<proto::Point> &wp)
{
    while (si->num_act_processed < si->request->actions.size()) {

        vsm::Action::Ptr action = si->request->actions[si->num_act_processed];

        if (action->Get_type() == vsm::Action::Type::CHANGE_SPEED) {
            vsm::Change_speed_action::Ptr a =
                action->Get_action<vsm::Action::Type::CHANGE_SPEED>();
            si->speed = a->speed;
            si->num_act_processed++;

        } else if (action->Get_type() == vsm::Action::Type::MOVE) {
            vsm::Move_action::Ptr a =
                action->Get_action<vsm::Action::Type::MOVE>();
            if (!si->launch_elevation) {
                si->launch_elevation = a->elevation;
            }
            wp->hold_time = Lookup_wait(si->request->actions,
                                        si->num_act_processed + 1);
            wp->type = proto::Point_type::WP;
            wp->index = si->num_uploaded + 1;
            if (a->acceptance_radius < 0.1) {
                /* Set default. */
                wp->tolerance_radius = 4;
            } else if (a->acceptance_radius < 1.0) {
                /* Set minimum. */
                wp->tolerance_radius = 1;
            } else {
                /* Round the given value. */
                wp->tolerance_radius = a->acceptance_radius + 0.5;
            }
            wp->heading = a->heading * 180.0 / M_PI;
            vsm::Geodetic_tuple pos = a->position.Get_geodetic();
            pos.altitude -= *si->launch_elevation;
            wp->position = proto::Gps_pos::From_position(pos);
            wp->speed = si->speed * 10;
            wp->altitude_rate = si->speed * 10;
            wp->wp_event_channel_value = 100;
            wp->name[0] = 'P';
            si->num_act_processed++;
            si->num_uploaded++;
            return true;
        } else {
            si->num_act_processed++;
        }
    }
    return false;
}

void
Mikrokopter_vehicle::On_task_upload(vsm::Io_result result,
                                    Mikrokopter_protocol::Data_ptr data,
                                    State_info_upload_task::Ptr si)
{
    if (result != vsm::Io_result::OK ||
        proto::Data<proto::Send_wp_response>(*data)->count != si->num_uploaded) {

        LOG_WARNING("Mission upload failed, result [%d] count [%d] num_uploaded [%ld]",
                result, proto::Data<proto::Send_wp_response>(*data)->count, si->num_uploaded);
        protocol->Close();
        return;
    }
    proto::Data<proto::Point> wp;
    if (!Create_waypoint(si, wp)) {
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
Mikrokopter_vehicle::Handle_vehicle_request(vsm::Vehicle_clear_all_missions_request::Handle request)
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
Mikrokopter_vehicle::On_mission_clear(vsm::Io_result result,
                                      Mikrokopter_protocol::Data_ptr data,
                                      State_info_clear_mission::Ptr si)
{
    if (result == vsm::Io_result::OK &&
        proto::Data<proto::Send_wp_response>(*data)->count == 0) {

        si->Complete();
    } else {
        LOG_WARNING("Mission clear failed");
        protocol->Close();
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
            vsm::Make_dummy_callback<void, vsm::Io_result,
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

    report->Set<vsm::tm::Battery_voltage>(static_cast<double>(nav->bat_voltage) / 10);

    report->Set<vsm::tm::Battery_current>(static_cast<double>(nav->current) / 10);

    report->Set<vsm::tm::Position>(nav->current_position.Get_position_telemetry());

    report->Set<vsm::tm::Gps_satellites_count>(nav->satellites_in_use);

    report->Set<vsm::tm::Ground_speed>(static_cast<double>(nav->ground_speed) / 100);

    report->Set<vsm::tm::Heading>(nav->heading.Get());

    report->Set<vsm::tm::Pitch>(-static_cast<double>(nav->pitch) / 180.0 * M_PI);

    report->Set<vsm::tm::Roll>(-static_cast<double>(nav->roll) / 180.0 * M_PI);

    report->Set<vsm::tm::Yaw>(static_cast<double>(nav->yaw) / 180.0 * M_PI);

    report->Set<vsm::tm::Climb_rate>(nav->variometer.Get());//XXX units?

    /* No information about units. 1:21 ratio was found experimentally. */
    report->Set<vsm::tm::Rel_altitude>(static_cast<double>(nav->altimeter.Get()) / 21.0);

    Schedule_telemetry_read();
}

bool
Mikrokopter_vehicle::Link_monitor_timer()
{
    size_t pkts_received, pkts_error;
    protocol->Get_pkt_stats(pkts_received, pkts_error);
    int num_expected = TELEMETRY_REQUEST_INTERVAL / TELEMETRY_REPORT_INTERVAL;
    double d_rcvd = static_cast<double>(pkts_received) - last_pkts_received;
    double d_error = static_cast<double>(pkts_error) - last_pkts_error;
    if (d_rcvd < num_expected) {
        LOG("DL packet loss detected: %d packets", static_cast<int>(num_expected - d_rcvd));
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
    report->Set<vsm::tm::Link_quality>(last_link_quality);

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
Mikrokopter_vehicle::Echo_handler(vsm::Io_result result,
                                  Mikrokopter_protocol::Data_ptr pkt)
{
    if (result != vsm::Io_result::OK ||
        proto::Data<proto::Echo>(*pkt)->pattern != ECHO_PATTERN) {

        LOG_WARNING("MikroKopter device link lost");
        protocol->Close();
    } else {
        echo_active = false;
    }
}
