// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#ifndef _VEHICLE_MANAGER_H_
#define _VEHICLE_MANAGER_H_

#include <mikrokopter_vehicle.h>

class Vehicle_manager {
public:
    void
    Enable();

    void
    Disable();

    /** The manager main loop. */
    void
    Run();

    /** Can be called asynchronously to cause the Run() method to exit. */
    void
    Stop();

private:
    /** Configured port. */
    class Port {
    public:
        /** Opened stream if the port available. */
        ugcs::vsm::Io_stream::Ref stream;
        /** Bound protocol instance. */
        Mikrokopter_protocol::Ptr protocol;
        /** Vehicle instance if created and the port is bound to it. */
        Mikrokopter_vehicle::Ptr vehicle;
    };

    typedef std::map<std::string, Port> Ports_map;

    /** All active ports. Indexed by stream name. */
    Ports_map ports;
    /** All created vehicles. */
    std::map<std::string, Mikrokopter_vehicle::Ptr> vehicles;
    /** Protects ports and vehicles lists. */
    std::mutex lists_mutex;
    /** Context for serving SDK requests. */
    ugcs::vsm::Request_processor::Ptr proc_context;
    /** Context used for processing completion notifications from SDK. */
    ugcs::vsm::Request_completion_context::Ptr comp_context;
    /** Ports polling timer. */
    ugcs::vsm::Timer_processor::Timer::Ptr poll_timer;

    std::atomic_bool stop_request =  { false };

    /** Check if some of configured ports can be opened. */
    bool
    Poll_ports();

    void
    Protocol_ready_handler(Port *port);

    void
    Disconnect_vehicle(Mikrokopter_vehicle::Ptr vehicle);

    Ports_map::iterator
    Disconnect_port(Port &port);

    /** Port connection handler. */
    void
    Port_connect_handler(
            std::string port_name,
            int baud,
            ugcs::vsm::Socket_address::Ptr,
            ugcs::vsm::Io_stream::Ref stream);
};


#endif /* _VEHICLE_MANAGER_H_ */
