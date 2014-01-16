// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <vehicle_manager.h>

Vehicle_manager::Vehicle_manager()
{

}

void
Vehicle_manager::Enable()
{
    proc_context = vsm::Request_processor::Create("Mikrokopter vehicle manager processor");
    proc_context->Enable();
    comp_context = vsm::Request_completion_context::Create(
            "Mikrokopter vehicle manager completion",
            proc_context->Get_waiter());
    comp_context->Enable();

    poll_timer = vsm::Timer_processor::Get_instance()->Create_timer(
        std::chrono::seconds(1),
        vsm::Make_callback(&Vehicle_manager::Poll_ports, this),
        comp_context);

    vsm::Transport_detector::Get_instance()->Add_detector(
            "vehicle.mikrokopter.serial",
            vsm::Transport_detector::Make_connect_handler(
                &Vehicle_manager::Port_connect_handler, this),
            proc_context);
}

void
Vehicle_manager::Disable()
{
    poll_timer->Cancel();
    poll_timer = nullptr;
    /* Disable processor to indicate to transport detector that we are not
     * anymore ready to accept new submissions.
     */
    proc_context->Disable();
    proc_context = nullptr;
    for (auto &entry: ports) {
        Port &port = entry.second;
        if (port.vehicle) {
            Disconnect_vehicle(port.vehicle);
            port.vehicle = nullptr;
        }
        if (port.protocol) {
            port.protocol->Disable();
            port.protocol = nullptr;
        }
        if (port.stream) {
            port.stream->Close();
            port.stream = nullptr;
        }
    }
    ports.clear();
    comp_context->Disable();
    comp_context = nullptr;
    ASSERT(vehicles.empty());
}

void
Vehicle_manager::Run()
{
    comp_context->Get_waiter()->Wait_and_process(
        std::initializer_list<vsm::Request_container::Ptr> {comp_context, proc_context},
        std::chrono::milliseconds::zero(),
        0,
        vsm::Make_callback([this]{ return static_cast<bool>(stop_request); }));
}

void
Vehicle_manager::Stop()
{
    stop_request = true;
}

bool
Vehicle_manager::Poll_ports()
{
    std::unique_lock<std::mutex> lock(lists_mutex);
    for (auto it  = ports.begin(); it != ports.end();) {
        Port &port = it->second;
        if ((port.stream && port.stream->Is_closed()) ||
            (port.protocol &&
             port.protocol->Get_state() == Mikrokopter_protocol::State::CLOSED)) {

            LOG_INFO("Disconnected port %s", port.stream->Get_name().c_str());

            it = Disconnect_port(port);
        } else {
            it++;
        }
    }
    return true;
}

void
Vehicle_manager::Protocol_ready_handler(Port *port)
{
    std::unique_lock<std::mutex> lock(lists_mutex);
    if (!port->stream || !port->protocol ||
        port->protocol->Get_state() != Mikrokopter_protocol::State::OPERATIONAL) {
        return;
    }
    LOG_INFO("MikroKopter ready on port %s", port->stream->Get_name().c_str());
    std::string sn = port->protocol->Get_serial_number();
    auto it = vehicles.find(sn);
    if (it == vehicles.end()) {
        it = vehicles.emplace(sn, Mikrokopter_vehicle::Create(port->protocol)).first;
        it->second->Enable();
    } else {
        LOG_WARNING("Serial number conflict, new vehicle connection ignored");
        Disconnect_port(*port);
        return;
    }
    port->vehicle = it->second;
}

void
Vehicle_manager::Disconnect_vehicle(Mikrokopter_vehicle::Ptr vehicle __UNUSED)
{
    vehicle->Disable();
    vehicles.erase(vehicle->Get_serial_number());
    LOG_INFO("MikroKopter vehicle disconnected, serial number '%s'",
             vehicle->Get_serial_number().c_str());
}

Vehicle_manager::Ports_map::iterator
Vehicle_manager::Disconnect_port(Port &port)
{
    if (port.vehicle) {
        Disconnect_vehicle(port.vehicle);
    }
    port.vehicle = nullptr;
    port.protocol->Disable();
    port.protocol = nullptr;
    port.stream->Close();
    auto it = ports.find(port.stream->Get_name());
    ASSERT(it != ports.end());
    return ports.erase(it);
}

void
Vehicle_manager::Port_connect_handler(std::string, int, vsm::Io_stream::Ref stream)
{
    std::unique_lock<std::mutex> lock(lists_mutex);
    auto res = ports.emplace(stream->Get_name(), Port());
    if (!res.second) {
        /* Already present. */
        Disconnect_port(res.first->second);
        res = ports.emplace(stream->Get_name(), Port());
        ASSERT(res.second);
    }
    Port &port = res.first->second;

    port.stream = stream;
    port.protocol = Mikrokopter_protocol::Create(port.stream);
    port.protocol->Set_ready_handler(
        vsm::Make_callback(&Vehicle_manager::Protocol_ready_handler,
                           this,
                           &port));
    port.protocol->Enable();
}