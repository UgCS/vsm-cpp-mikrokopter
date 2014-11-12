// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#ifndef MIKROKOPTER_VEHICLE_H_
#define MIKROKOPTER_VEHICLE_H_

#include <ugcs/vsm/vsm.h>
#include <ugcs/vsm/optional.h>

#include <mikrokopter_protocol.h>
#include <protocol_data.h>

class Mikrokopter_vehicle: public ugcs::vsm::Vehicle {
    DEFINE_COMMON_CLASS(Mikrokopter_vehicle, ugcs::vsm::Vehicle)
public:

    Mikrokopter_vehicle(Mikrokopter_protocol::Ptr protocol);

private:
    static constexpr std::chrono::milliseconds
        /** Interval for telemetry request timer. */
        TELEMETRY_REQUEST_INTERVAL = std::chrono::seconds(2),
        /** Requested telemetry report rate. */
        TELEMETRY_REPORT_INTERVAL = std::chrono::milliseconds(400),
        /** Link quality is evaluated with this interval. */
        LINK_MONITOR_INTERVAL = std::chrono::milliseconds(1200);

    /** Maximal number of retransmissions for waypoints. */
    static constexpr int MAX_RETRANS = 3;
    /** Maximal number of echo requests which may be lost before device is
     * considered to be disconnected.
     */
    static constexpr int MAX_ECHO_LOST = 2;
    /** Pattern used in echo command for device presence detection. */
    static constexpr uint16_t ECHO_PATTERN = 0x5aa5;

    Mikrokopter_protocol::Ptr protocol;
    /** Timer for periodic requesting telemetry information. */
    ugcs::vsm::Timer_processor::Timer::Ptr telemetry_timer;
    Mikrokopter_protocol::Stream::Ref telemetry_stream;
    /** Periodic timer for link quality reports. */
    ugcs::vsm::Timer_processor::Timer::Ptr link_monitor_timer;
    /** Last downlink statistics. */
    size_t last_pkts_received, last_pkts_error;
    double last_link_quality = 1.0;
    /** Quotient for link quality rolling average calculation. */
    static constexpr double LINK_QUALITY_RA_QUOT = 0.5;
    /** Is echo request currently pending. */
    bool echo_active = false;
    /** Number of echo requests lost. */
    int lost_echo_count = 0;
    /** Current system status. */
    Sys_status sys_status;
    /** Value sent in wp-event-channel field for camera trigger action. */
    int wp_event_value;

    /** Current task upload operation. */
    ugcs::vsm::Operation_waiter task_upload_op;

    /** Current mission clear operation. */
    ugcs::vsm::Operation_waiter mission_clear_op;

    /** Current telemetry request operation. */
    ugcs::vsm::Operation_waiter telemetry_request_op;

    /** Current echo request operation. */
    ugcs::vsm::Operation_waiter echo_request_op;

    /** Current telemetry stream read operation. */
    ugcs::vsm::Operation_waiter telemetry_stream_read_op;

    /** Current instant vehicle operation. */
    ugcs::vsm::Operation_waiter command_op;

    /** Context for vehicle requests execution. */
    template <class RequestType>
    struct State_info {
        typename RequestType::Handle request;

        State_info(typename RequestType::Handle request):
            request(request)
        {}

        void
        Complete(ugcs::vsm::Vehicle_request::Result result = ugcs::vsm::Vehicle_request::Result::OK)
        {
            request = result;
        }
    };

    /** Context for mission clear request. */
    struct State_info_clear_mission: State_info<ugcs::vsm::Vehicle_clear_all_missions_request> {
        typedef std::shared_ptr<State_info_clear_mission> Ptr;

        State_info_clear_mission(const ugcs::vsm::Vehicle_clear_all_missions_request::Handle &request):
            State_info(request)
        {}
    };

    /** Context for instant command. */
    struct State_info_command: State_info<ugcs::vsm::Vehicle_command_request> {
        typedef std::shared_ptr<State_info_command> Ptr;

        State_info_command(const ugcs::vsm::Vehicle_command_request::Handle &request):
            State_info(request)
        {}
    };

    /** Context for task upload request. */
    struct State_info_upload_task: State_info<ugcs::vsm::Vehicle_task_request> {
        typedef std::shared_ptr<State_info_upload_task> Ptr;

        /** Launch site absolute height. */
        double launch_elevation;
        /** Number of waypoints uploaded so far. */
        size_t num_uploaded = 0,
        /** Number of actions processed so far. */
               num_act_processed = 0;
        /** Currently set speed, m/s. */
        double speed = 3.0;
        /** Retransmissions counter. */
        int num_retrans = 0;
        /** Current heading value in degrees. 0 - default behavior, negative
         * values - POI index. See proto::Point::heading.
         */
        int cur_heading = 0;
        /** Last waypoint position. */
        ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple> last_position;
        /** Last waypoint tolerance radius in meters. */
        int tolerance_radius;
        /** Panorama total angular range. Negative is CCW. Zero if no active
         * panorama action.
         */
        double panorama_range = 0.0,
        /** Angular step for panorama waypoints. Should be negative if CCW. */
               panorama_step,
        /** Currently reached panorama angle. */
               panorama_angle,
        /** Delay in seconds for each panorama step. */
               panorama_delay;
        /** Camera angle (pitch? deg?). */
        int cur_camera_angle = 0;

        State_info_upload_task(const ugcs::vsm::Vehicle_task_request::Handle &request):
            State_info(request)
        {
            launch_elevation = this->request->Get_takeoff_altitude();
        }
    };

    /** Enable handler. */
    virtual void
    On_enable() override;

    /** Disable event from base class. */
    virtual void
    On_disable() override;

    /** Disable handler in a vehicle context. */
    void
    On_disable_handler(ugcs::vsm::Request::Ptr);

    /** Tasks received from UCS. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_task_request::Handle request) override;

    /** UCS requesting to clear up all missions on a vehicle. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_clear_all_missions_request::Handle request) override;

    /** Instant command for a vehicle. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_command_request::Handle request) override;

    /** Mission cleared. */
    void
    On_mission_clear(ugcs::vsm::Io_result result, Mikrokopter_protocol::Data_ptr data,
                     State_info_clear_mission::Ptr si);

    /** Instant command execution step finished. */
    void
    On_command(ugcs::vsm::Io_result result, Mikrokopter_protocol::Data_ptr data,
               State_info_command::Ptr si);

    /** Mission cleared before upload. */
    void
    On_task_upload(ugcs::vsm::Io_result result, Mikrokopter_protocol::Data_ptr data,
                   State_info_upload_task::Ptr si);

    /** Create next waypoint descriptor.
     *
     * @return True if waypoint created, false if all actions processed.
     */
    bool
    Create_waypoint(State_info_upload_task::Ptr si,
                    proto::Data<proto::Point> &wp,
                    bool is_retrans = false);

    /** Create next waypoint for panorama action.
     *
     * @return True if waypoint created. False if panorama is complete.
     */
    bool
    Create_panorama_wp(State_info_upload_task::Ptr si,
                       proto::Data<proto::Point> &wp);

    /** Create waypoint descriptor for last transmitted one.
     *
     * @return true if waypoint for retransmission created, false if
     *      retransmissions limit exceeded.
     */
    bool
    Retransmit_waypoint(State_info_upload_task::Ptr si, proto::Data<proto::Point> &wp);

    /** Check if there is a wait action before next move action.
     * @param actions Actions to check.
     * @param idx Start index for actions to check.
     * @return Delay in seconds if wait action found, zero otherwise.
     */
    double
    Lookup_wait(const std::vector<ugcs::vsm::Action::Ptr> &actions, size_t idx);

    bool
    Request_telemetry();

    void
    Schedule_telemetry_read();

    void
    On_telemetry(Mikrokopter_protocol::Data_ptr data);

    bool
    Link_monitor_timer();

    void
    Echo_handler(ugcs::vsm::Io_result result, Mikrokopter_protocol::Data_ptr pkt);
};

#endif /* MIKROKOPTER_VEHICLE_H_ */
