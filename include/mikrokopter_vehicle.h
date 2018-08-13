// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#ifndef _MIKROKOPTER_VEHICLE_H_
#define _MIKROKOPTER_VEHICLE_H_

#include <ugcs/vsm/vsm.h>
#include <ugcs/vsm/optional.h>

#include <mikrokopter_protocol.h>


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
    /** Maximal requested telemetry rate, bytes per second. */
    static constexpr uint16_t MAX_TELEMETRY_RATE = 0x8000;

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
    size_t last_pkts_error = 0;
    size_t last_pkts_received = 0;
    double last_link_quality = 1.0;
    /** Quotient for link quality rolling average calculation. */
    static constexpr double LINK_QUALITY_RA_QUOT = 0.5;
    /** Is echo request currently pending. */
    bool echo_active = false;
    /** Number of echo requests lost. */
    int lost_echo_count = 0;
    /** Value sent in wp-event-channel field for camera trigger action. */
    int wp_event_value;
    /** Last received error code. */
    mk_proto::Error_code last_error = mk_proto::Error_code::OK;

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

        template<class... Args>
        void
        Fail(const char * fmt, Args&&... args)
        {
            if (fmt) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
                request.Fail(fmt, std::forward<Args>(args)...);
#pragma GCC diagnostic pop
            } else {
                request.Succeed();
            }
        }

        void
        Succeed()
        {
             request.Succeed();
        }
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
        double speed = 3.0,
        /** Currently climb rate, m/s. */
               climb_rate = 2.0;
        /** Retransmissions counter. */
        int num_retrans = 0;
        /** Current heading value in degrees. */
        ugcs::vsm::Optional<int> cur_heading;
        /** Current POI index. */
        ugcs::vsm::Optional<int> cur_poi;
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
        /** True if dummy waypoint for unsetting POI is received. */
        bool final_poi_unset = false;

        State_info_upload_task(const ugcs::vsm::Vehicle_task_request::Handle &request):
            State_info(request)
        {
            launch_elevation = this->request->Get_takeoff_altitude();
        }

        /** Check if next action(s) is(are) wait. Consume them and return
         * accumulated wait time for them.
         */
        double
        Lookup_wait();

        /** Get heading value for waypoint structure.
         * @return Value to set in proto::Point::heading. Zero - not set.
         */
        int
        Get_heading()
        {
            if (cur_heading) {
                if (*cur_heading == 0) {
                    return 1;
                }
                return *cur_heading;
            }
            if (cur_poi) {
                return -*cur_poi;
            }
            return 0;
        }
    };

    /** Mission being uploaded. */
    class Mission {
    public:
        Mission(Mikrokopter_vehicle &vehicle,
                ugcs::vsm::Vehicle_task_request::Handle request);

        ~Mission();

        void
        Succeed();

        template<class... Args>
        void
        Fail(const char * fmt, Args&&... args)
        {
            if (si) {
                si->Fail(fmt, std::forward<Args>(args)...);
                si = nullptr;
            }
        }

        /** Initialize waypoint descriptor with next transmitted waypoint data.
         *
         * @return true if waypoint for created, false if all waypoints transmitted.
         */
        bool
        Transmit_waypoint(mk_proto::Data<mk_proto::Point> &wp);

        /** Create waypoint descriptor for last transmitted one.
         *
         * @return true if waypoint for retransmission created, false if
         *      retransmissions limit exceeded.
         */
        bool
        Retransmit_waypoint(mk_proto::Data<mk_proto::Point> &wp);

        Mikrokopter_vehicle &vehicle;
        State_info_upload_task::Ptr si;
        /** Generated flight plan. */
        std::vector<mk_proto::Data<mk_proto::Point>> flight_plan;
        /** Index of the currently item being uploaded. -1 for clear request. */
        int cur_item_idx = -1;
        /** Number of retransmissions for current item. */
        int num_retrans = 0;
    };

    /** Current mission being uploaded. */
    std::unique_ptr<Mission> cur_mission;

    /** Vertical speed calculator based on altitude changes. New protocol does not provide reliable
     * vertical speed indication.
     */
    class Vertical_speed_calc {
    public:
        /** Feed next altitude value.
         *
         * @param
         * @return Current averaged value of vertical speed.
         */
        double
        Feed_altitude(double altitude);
    private:
        /** Tau for timed rolling averaged, s. */
        static constexpr double TAU = 1.0;
        /** Current vertical speed value. */
        double cur_value = 0;
        double last_altitude = 0;
        bool last_valid = false;
        /** Timestamp of last measurement. */
        std::chrono::high_resolution_clock::time_point last_measurement;
    };

    Vertical_speed_calc vert_speed_calc;

    /** Enable handler. */
    virtual void
    On_enable() override;

    /** Disable event from base class. */
    virtual void
    On_disable() override;

    /** Tasks received from UCS. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_task_request::Handle request) override;

    /** Instant command for a vehicle. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_command_request::Handle request) override;

    /** Instant command execution step finished. */
    void
    On_command(ugcs::vsm::Io_result result, Mikrokopter_protocol::Data_ptr data,
               State_info_command::Ptr si);

    /** Mission cleared before upload. */
    void
    On_task_upload(ugcs::vsm::Io_result result, Mikrokopter_protocol::Data_ptr data);

    /** Create next waypoint descriptor.
     *
     * @return True if waypoint created, false if all actions processed.
     */
    bool
    Create_waypoint(State_info_upload_task::Ptr si,
                    mk_proto::Data<mk_proto::Point> &wp);

    /** Create next waypoint for panorama action.
     *
     * @return True if waypoint created. False if panorama is complete.
     */
    bool
    Create_panorama_wp(State_info_upload_task::Ptr si,
                       mk_proto::Data<mk_proto::Point> &wp);

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

    void
    Set_error_code(mk_proto::Error_code error_code);
};

#endif /* _MIKROKOPTER_VEHICLE_H_ */
