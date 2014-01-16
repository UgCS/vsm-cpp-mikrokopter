// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#ifndef VEHICLE_MANAGER_H_
#define VEHICLE_MANAGER_H_

#include <mikrokopter_vehicle.h>

class Vehicle_manager {
public:

    Vehicle_manager();

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
        vsm::Io_stream::Ref stream;
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
    vsm::Request_processor::Ptr proc_context;
    /** Context used for processing completion notifications from SDK. */
    vsm::Request_completion_context::Ptr comp_context;
    /** Ports polling timer. */
    vsm::Timer_processor::Timer::Ptr poll_timer;

    std::atomic<bool> stop_request;

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
    Port_connect_handler(std::string port_name, int baud,
                         vsm::Io_stream::Ref stream);
};


#endif /* VEHICLE_MANAGER_H_ */
