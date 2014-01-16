// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#ifndef MIKROKOPTER_VEHICLE_H_
#define MIKROKOPTER_VEHICLE_H_

#include <vsm/vsm.h>
#include <vsm/optional.h>

#include <mikrokopter_protocol.h>
#include <protocol_data.h>

class Mikrokopter_vehicle: public vsm::Vehicle {
    DEFINE_COMMON_CLASS(Mikrokopter_vehicle, vsm::Vehicle)
public:

    Mikrokopter_vehicle(Mikrokopter_protocol::Ptr protocol);

private:
    static constexpr std::chrono::milliseconds
        /** Interval for telemetry request timer. */
        TELEMETRY_REQUEST_INTERVAL = std::chrono::seconds(2),
        /** Requested telemetry report rate. */
        TELEMETRY_REPORT_INTERVAL = std::chrono::milliseconds(400),
        /** Link quality is evaluated with this interval. */
        LINK_MONITOR_INTERVAL = std::chrono::milliseconds(2000);

    /** Pattern used in echo command for device presence detection. */
    static constexpr uint16_t ECHO_PATTERN = 0x5aa5;

    Mikrokopter_protocol::Ptr protocol;
    /** Timer for periodic requesting telemetry information. */
    vsm::Timer_processor::Timer::Ptr telemetry_timer;
    Mikrokopter_protocol::Stream::Ref telemetry_stream;
    /** Periodic timer for link quality reports. */
    vsm::Timer_processor::Timer::Ptr link_monitor_timer;
    /** Last downlink statistics. */
    size_t last_pkts_received, last_pkts_error;
    double last_link_quality = 1.0;
    /** Quotient for link quality rolling average calculation. */
    static constexpr double LINK_QUALITY_RA_QUOT = 0.5;
    /** Is echo request currently pending. */
    bool echo_active = false;

    /** Current task upload operation. */
    vsm::Operation_waiter task_upload_op;

    /** Current mission clear operation. */
    vsm::Operation_waiter mission_clear_op;

    /** Current telemetry request operation. */
    vsm::Operation_waiter telemetry_request_op;

    /** Current echo request operation. */
    vsm::Operation_waiter echo_request_op;

    /** Current telemetry stream read operation. */
    vsm::Operation_waiter telemetry_stream_read_op;

    /** Context for vehicle requests execution. */
    template <class RequestType>
    struct State_info {
        typename RequestType::Handle request;

        State_info(typename RequestType::Handle request):
            request(request)
        {}

        void
        Complete(vsm::Vehicle_request::Result result = vsm::Vehicle_request::Result::OK)
        {
            request = result;
        }
    };

    /** Context for mission clear request. */
    struct State_info_clear_mission: State_info<vsm::Vehicle_clear_all_missions_request> {
        typedef std::shared_ptr<State_info_clear_mission> Ptr;

        State_info_clear_mission(const vsm::Vehicle_clear_all_missions_request::Handle &request):
            State_info(request)
        {}
    };

    /** Context for task upload request. */
    struct State_info_upload_task: State_info<vsm::Vehicle_task_request> {
        typedef std::shared_ptr<State_info_upload_task> Ptr;

        /** Launch site absolute height. */
        vsm::Optional<double> launch_elevation;
        /** Number of waypoints uploaded so far. */
        size_t num_uploaded = 0,
        /** Number of actions processed so far. */
               num_act_processed = 0;
        /** Currently set speed, m/s. */
        double speed = 3.0;

        State_info_upload_task(const vsm::Vehicle_task_request::Handle &request):
            State_info(request)
        {}
    };

    /** Enable handler. */
    virtual void
    On_enable() override;

    /** Disable event from base class. */
    virtual void
    On_disable() override;

    /** Disable handler in a vehicle context. */
    void
    On_disable_handler(vsm::Request::Ptr);

    /** Tasks received from UCS. */
    virtual void
    Handle_vehicle_request(vsm::Vehicle_task_request::Handle request) override;

    /** UCS requesting to clear up all missions on a vehicle. */
    virtual void
    Handle_vehicle_request(vsm::Vehicle_clear_all_missions_request::Handle request) override;

    /** Mission cleared. */
    void
    On_mission_clear(vsm::Io_result result, Mikrokopter_protocol::Data_ptr data,
                     State_info_clear_mission::Ptr si);

    /** Mission cleared before upload. */
    void
    On_task_upload(vsm::Io_result result, Mikrokopter_protocol::Data_ptr data,
                   State_info_upload_task::Ptr si);

    /** Create next waypoint descriptor.
     *
     * @return True if waypoint created, false if all actions processed.
     */
    bool
    Create_waypoint(State_info_upload_task::Ptr si, proto::Data<proto::Point> &wp);

    /** Check if there is a wait action before next move action.
     * @param actions Actions to check.
     * @param idx Start index for actions to check.
     * @return Delay in seconds if wait action found, zero otherwise.
     */
    double
    Lookup_wait(const std::vector<vsm::Action::Ptr> &actions, size_t idx);

    bool
    Request_telemetry();

    void
    Schedule_telemetry_read();

    void
    On_telemetry(Mikrokopter_protocol::Data_ptr data);

    bool
    Link_monitor_timer();

    void
    Echo_handler(vsm::Io_result result, Mikrokopter_protocol::Data_ptr pkt);
};

#endif /* MIKROKOPTER_VEHICLE_H_ */
