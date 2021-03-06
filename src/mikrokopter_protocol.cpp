// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <mikrokopter_protocol.h>
#include <protocol_data.h>

#include <sstream>


using namespace ugcs::vsm;

/* ****************************************************************************/
/* Mikrokopter_protocol::Stream class. */

Mikrokopter_protocol::Stream::~Stream()
{
    for (Pending_request &req : pending_requests) {
        req.request->Abort();
    }
    pending_requests.clear();
}

void
Mikrokopter_protocol::Stream::Add_ref()
{
    atomic_fetch_add(&ref_count, 1);
}

void
Mikrokopter_protocol::Stream::Release_ref()
{
    int res = atomic_fetch_sub(&ref_count, 1);
    if (res <= 0) {
        VSM_EXCEPTION(Internal_error_exception, "Reference counter underflow");
    } else if (res == 1) {
        Close();
    }
}

bool
Mikrokopter_protocol::Stream::Is_closed() const
{
    std::unique_lock<std::mutex> lock(close_mutex);
    return is_closed;
}

void
Mikrokopter_protocol::Stream::Close()
{
    std::unique_lock<std::mutex> lock(close_mutex);
    is_closed = true;
}

bool
Mikrokopter_protocol::Stream::On_packet(Data_ptr data)
{
    std::unique_lock<std::mutex> lock(close_mutex);
    if (is_closed) {
        packet_queue.clear();
        return false;
    }
    lock.unlock();

    Request::Locker req_lock;
    Pending_request *req = nullptr;
    while (pending_requests.size()) {
        req = &pending_requests.front();
        req_lock = req->request->Lock();
        if (req->request->Is_processing()) {
            break;
        }
        req_lock = Request::Locker();
        pending_requests.pop_front();
    }

    if (pending_requests.size() == 0) {
        if (packet_queue.size() < MAX_PKT_QUEUE_SIZE) {
            packet_queue.push_back(data);
        }
        return true;
    }

    Data_ptr pkt;
    if (packet_queue.size()) {
        pkt = packet_queue.front();
        packet_queue.pop_front();
        packet_queue.push_back(data);
    } else {
        pkt = data;
    }

    req->handler.Set_arg<0>(pkt);
    req->request->Complete(Request::Status::OK, std::move(req_lock));
    return true;
}

Operation_waiter
Mikrokopter_protocol::Stream::Read(Packet_handler handler,
                                   Request_completion_context::Ptr comp_ctx)
{
    Request::Ptr req = Request::Create();
    req->Set_processing_handler(
        Make_callback(&Mikrokopter_protocol::Stream::Handle_read, this,
                      req, handler));
    req->Set_completion_handler(comp_ctx, handler.Get_callback());
    protocol.Submit_request(req);
    return req;
}

void
Mikrokopter_protocol::Stream::Handle_read(Request::Ptr request,
                                          Packet_handler handler)
{
    pending_requests.emplace_back(Pending_request{request, handler});
}

/* ****************************************************************************/
/* Mikrokopter_protocol class. */

constexpr std::chrono::milliseconds Mikrokopter_protocol::DEFAULT_TIMEOUT;

Mikrokopter_protocol::Mikrokopter_protocol(Io_stream::Ref stream):
    Request_processor("Mikrokopter protocol processor"),
    stream(stream)
{}

void
Mikrokopter_protocol::On_enable()
{
    Request_processor::On_enable();
    comp_ctx = Request_completion_context::Create(
            "Mikrokopter protocol completion",
            Get_waiter());
    comp_ctx->Enable();
    worker = Request_worker::Create(
        "Mikrokopter protocol worker",
        std::initializer_list<Request_container::Ptr> { Shared_from_this(), comp_ctx });
    worker->Enable();

    Schedule_read();

    auto req = Request::Create();
    req->Set_processing_handler(
        Make_callback(&Mikrokopter_protocol::On_enable_handler,
                      Shared_from_this(), req));
    Submit_request(req);
}

void
Mikrokopter_protocol::On_enable_handler(Request::Ptr request)
{
    /* Send magic packet to ensure UART is not redirected from NC. */
    const uint8_t magic_pkt[] = {0x1B, 0x1B, 0x55, 0xAA, 0x00};
    Send_packet(Io_buffer::Create(magic_pkt, sizeof(magic_pkt)));

    /* Start detection. */
    mk_proto::Data<mk_proto::Echo> data;
    data->pattern = DETECTION_PATTERN;
    link_detection_op = Command(
            Command_id::LINK_TEST, Address::NC, std::move(data),
            Command_id::LINK_TEST_RESP,
            Make_data_handler(&Mikrokopter_protocol::Detection_handler, this),
            comp_ctx);
    request->Complete();
}

void
Mikrokopter_protocol::On_disable()
{
    auto req = Request::Create();
    req->Set_processing_handler(
            Make_callback(
                    &Mikrokopter_protocol::On_disable_handler,
                    Shared_from_this(),
                    req));
    Submit_request(req);
    req->Wait_done(false);

    Set_disabled();

    worker->Disable();
    worker = nullptr;
    comp_ctx->Disable();
    comp_ctx = nullptr;
}

void
Mikrokopter_protocol::On_disable_handler(Request::Ptr request)
{
    link_detection_op.Abort();
    read_op.Abort();
    write_op.Abort();
    for (auto &h : response_handlers) {
        if (h.second->request) {
            h.second->request->Abort();
            h.second->request = nullptr;
        }
        if (h.second->to_timer) {
            h.second->to_timer->Cancel();
            h.second->to_timer = nullptr;
        }
    }
    response_handlers.clear();
    streams.clear();
    request->Complete();
}

void
Mikrokopter_protocol::Close()
{
    Request::Ptr req = Request::Create();
    req->Set_processing_handler(
        Make_callback(&Mikrokopter_protocol::Handle_close, this, req));
    Submit_request(req);
}

void
Mikrokopter_protocol::Handle_close(Request::Ptr request)
{
    state = State::CLOSED;
    stream->Close();
    request->Complete();
}

void
Mikrokopter_protocol::Schedule_read()
{
    read_op.Abort();
    read_op = stream->Read(0, 1,
                 Make_read_callback(&Mikrokopter_protocol::On_data_received,
                                    this),
                 comp_ctx);
}

Operation_waiter
Mikrokopter_protocol::Command(Command_id id, Address address,
                              Data &&data,
                              Command_id response_id,
                              Data_handler response_handler,
                              Request_completion_context::Ptr comp_ctx,
                              std::chrono::milliseconds timeout,
                              int num_retransmissions)
{
    Io_buffer::Ptr pkt = Build_packet(id, address, std::move(data));
    Request::Ptr req = Request::Create();
    req->Set_processing_handler(
        Make_callback(&Mikrokopter_protocol::Handle_command, this,
                      req, address, response_id, response_handler, pkt,
                      timeout, num_retransmissions));
    req->Set_completion_handler(comp_ctx ? comp_ctx : this->comp_ctx,
                                response_handler.Get_callback());
    Submit_request(req);
    return req;
}

Io_buffer::Ptr
Mikrokopter_protocol::Build_packet(Command_id id, Address address,
                                   std::vector<uint8_t> &&data)
{
    std::vector<uint8_t> packet;
    packet.push_back('#');
    packet.push_back('a' + static_cast<uint8_t>(address));
    packet.push_back(static_cast<uint8_t>(id));
    Base64_encode(data, packet);
    auto crc = Calculate_crc(&packet.front(), packet.size());
    packet.push_back(crc.first);
    packet.push_back(crc.second);
    packet.push_back('\r');
    return Io_buffer::Create(std::move(packet));
}

void
Mikrokopter_protocol::Base64_encode(const std::vector<uint8_t> &data,
                                    std::vector<uint8_t> &out)
{
    for (auto it = data.begin(); it != data.end();) {
        uint8_t a, b, c;
        a = *it++;
        b = it != data.end() ? *it++ : 0;
        c = it != data.end() ? *it++ : 0;
        out.push_back('=' + (a >> 2));
        out.push_back('=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4)));
        out.push_back('=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6)));
        out.push_back('=' + (c & 0x3f));
    }
}

Mikrokopter_protocol::Data_ptr
Mikrokopter_protocol::Base64_decode(const uint8_t *data, size_t size)
{
    /* Should be multiple of four. */
    if (size & 3) {
        return nullptr;
    }
    Data_ptr out = std::make_shared<Data>();
    out->reserve(size / 4 * 3);
    while (size) {
        uint8_t a = *data++ - '=';
        uint8_t b = *data++ - '=';
        uint8_t c = *data++ - '=';
        uint8_t d = *data++ - '=';
        size -= 4;
        out->push_back((a << 2) | (b >> 4));
        out->push_back(((b & 0x0f) << 4) | (c >> 2));
        out->push_back(((c & 0x03) << 6) | d);
    }
    return out;
}

std::pair<uint8_t, uint8_t>
Mikrokopter_protocol::Calculate_crc(uint8_t *data, size_t size)
{
    uint16_t crc = 0;
    while (size) {
        crc += *data;
        data++;
        size--;
    }
    crc &= 0xfff;
    return std::pair<uint8_t, uint8_t>('=' + (crc >> 6), '=' + (crc & 0x3f));
}

void
Mikrokopter_protocol::Send_packet(Io_buffer::Ptr pkt)
{
    write_queue.push_back(pkt);
    if (write_queue.size() == 1) {
        write_op = stream->Write(
            pkt,
            Make_write_callback(&Mikrokopter_protocol::Handle_send, this),
            comp_ctx);
    }
}

void
Mikrokopter_protocol::Handle_send(Io_result result)
{
    if (result != Io_result::OK) {
        LOG_WARNING("Packet sending failed, result %d", static_cast<int>(result));
    }
    write_queue.pop_front();
    if (write_queue.size()) {
        Io_buffer::Ptr pkt = write_queue.front();
        write_op = stream->Write(
            pkt,
            Make_write_callback(&Mikrokopter_protocol::Handle_send, this),
            comp_ctx);
    }
}

void
Mikrokopter_protocol::Handle_command(Request::Ptr request,
                                     Address address,
                                     Command_id response_id,
                                     Data_handler response_handler,
                                     Io_buffer::Ptr request_pkt,
                                     std::chrono::milliseconds timeout,
                                     int num_retransmissions)
{
    if (response_id == Command_id::NONE) {
        Send_packet(request_pkt);
        request->Complete();
        return;
    }
    Address_id aid = {address, response_id};

    Response_handler::Ptr hdl = std::make_shared<Response_handler>(
         Response_handler{request, response_handler, std::move(request_pkt),
                          timeout, nullptr, num_retransmissions, {}});
    auto res = response_handlers.emplace(aid, hdl);
    if (!res.second) {
        /* Same key already exists, queue the request. */
        res.first->second->next.push_back(hdl);
    } else {
        Start_request(aid, *hdl);
    }
}

void
Mikrokopter_protocol::Start_request(const Address_id &aid, Response_handler &h)
{
    if (h.timeout.count()) {
        h.to_timer =
            Timer_processor::Get_instance()->Create_timer(
                h.timeout,
                Make_callback(&Mikrokopter_protocol::Timeout_handler,
                              this, aid),
                comp_ctx);
    }
    Send_packet(h.request_pkt);
}

bool
Mikrokopter_protocol::Timeout_handler(Address_id aid)
{
    auto it = response_handlers.find(aid);
    if (it == response_handlers.end()) {
        return false;
    }
    Response_handler::Ptr h = it->second;

    /* Send retransmission if necessary. */
    if (h->num_retrans_left) {
        h->num_retrans_left--;
        Send_packet(h->request_pkt);
        return true;
    }

    /** Timed out, should be removed. */
    h->to_timer = nullptr;
    Finish_request(it);
    return false;
}

void
Mikrokopter_protocol::Finish_request(Response_map::iterator &it, Data_ptr pkt)
{
    Response_handler::Ptr h = it->second;
    if (!h->next.empty()) {
        Response_handler::Ptr next_h = h->next.front();
        h->next.pop_front();
        next_h->next = std::move(h->next);
        it->second = next_h;
        Start_request(it->first, *next_h);
    } else {
        response_handlers.erase(it);
    }

    if (h->to_timer) {
        h->to_timer->Cancel();
        h->to_timer = nullptr;
    }

    auto lock = h->request->Lock();
    if (h->request->Is_processing()) {
        if (pkt) {
            h->handler.Set_args(Io_result::OK, pkt);
        } else {
            h->handler.Set_args(Io_result::TIMED_OUT, nullptr);
        }
        h->request->Complete(Request::Status::OK, std::move(lock));
        h->request = nullptr;
    }
}

void
Mikrokopter_protocol::On_data_received(Io_buffer::Ptr buf, Io_result result)
{
    if (result == Io_result::OK) {
        std::string data = buf->Get_string();
        for (char c : data) {
            On_char_received(c);
        }
        Schedule_read();
    } else {
        LOG_INFO("Stream read failed, closing: %d", static_cast<int>(result));
        stream->Close();
        state = State::CLOSED;
    }
}

void
Mikrokopter_protocol::On_char_received(int c)
{
    if (c == '\r') {
        if (!packet_buf.empty()) {
            On_packet_received_raw(std::move(packet_buf));
            packet_buf.clear();
        }
        return;
    }
    if (packet_buf.size() >= MAX_PACKET_LENGTH) {
        LOG("Incoming packet length exceeded, dropping data");
        packet_buf.clear();
        Pkt_stats::Pkt stat(pkt_stats);
        return;
    }
    packet_buf += c;
}

void
Mikrokopter_protocol::On_packet_received_raw(std::string &&packet)
{
    Pkt_stats::Pkt stat(pkt_stats);

    /* Validate the packet. */
    if (packet.size() < MIN_PACKET_LENGTH) {
        LOG("Too short packet");
        return;
    }
    if (packet[0] != '#') {
        LOG("Invalid start byte");
        return;
    }
    if (packet[1] < 'a') {
        LOG("Invalid address");
        return;
    }
    Address_id aid(static_cast<Address>(packet[1] - 'a'),
                   static_cast<Command_id>(packet[2]));
    if (aid.first == Address::NONE || aid.second == Command_id::NONE) {
        LOG("Invalid address/ID");
        return;
    }

    Data_ptr data = Base64_decode(reinterpret_cast<uint8_t *>(&packet[3]),
                                  packet.size() - 5);
    if (!data) {
        LOG("Base64 decoding error");
        return;
    }

    auto crc = Calculate_crc(reinterpret_cast<uint8_t *>(&packet.front()),
                             packet.size() - 2);
    if (packet[packet.size() - 2] != crc.first ||
        packet[packet.size() - 1] != crc.second)
    {
        LOG("CRC error");
        return;
    }

    stat.Succeded();
    On_packet_received(aid, data);
}

void
Mikrokopter_protocol::On_packet_received(Address_id aid, Data_ptr payload)
{
    auto it = response_handlers.find(aid);
    if (it != response_handlers.end()) {
        Finish_request(it, payload);
        return;
    }
    /* Check streams. */
    std::unique_lock<std::mutex>(stream_mutex);
    auto stream_it = streams.find(aid);
    if (stream_it == streams.end()) {
        return;
    }
    if (!stream_it->second->On_packet(payload)) {
        streams.erase(stream_it);
    }
}

void
Mikrokopter_protocol::Detection_handler(Io_result result, Data_ptr pkt)
{
    if (result == Io_result::OK &&
        mk_proto::Data<mk_proto::Echo>(*pkt)->pattern == DETECTION_PATTERN)
    {
        state = State::OPERATIONAL;
        if (ready_handler) {
            ready_handler();
        }
    } else {
        LOG("Detection failed, closing stream");
        state = State::CLOSED;
        stream->Close();
    }
}

Mikrokopter_protocol::Stream::Ref
Mikrokopter_protocol::Subscribe(Command_id id, Address address)
{
    Address_id aid = {address, id};
    if (aid.first == Address::NONE || aid.second == Command_id::NONE) {
        VSM_EXCEPTION(Invalid_param_exception, "Invalid address/ID");
    }
    Stream::Ptr stream = Stream::Create(*this);
    std::unique_lock<std::mutex>(stream_mutex);
    auto res = streams.emplace(aid, stream);
    if (!res.second) {
        VSM_EXCEPTION(Invalid_op_exception, "The stream already exists");
    }
    return stream;
}

namespace {

std::map<mk_proto::Error_code, const char *> error_messages = {
    {mk_proto::Error_code::OK, "OK"},
    {mk_proto::Error_code::FC_NOT_COMPATIBLE, "FC version not compatible with NC"},
    {mk_proto::Error_code::MK3MAG_NOT_COMPTIBLE, "Mk3Mag version not compatible with NC"},
    {mk_proto::Error_code::NO_FC_COMM, "No communication with FC"},
    {mk_proto::Error_code::NO_COMPASS_COMM, "No communication with compass"},
    {mk_proto::Error_code::NO_GPS_COMM, "No communication with GPS receiver"},
    {mk_proto::Error_code::BAD_COMPASS_VALUE, "Bad compass value"},
    {mk_proto::Error_code::RC_SIGNAL_LOST, "RC signal lost"},
    {mk_proto::Error_code::FC_SPI_RX_ERROR, "No communication with FC via SPI"},
    {mk_proto::Error_code::NO_NC_COMM, "The communication between FC and NC was suddenly interrupted"},
    {mk_proto::Error_code::FC_NICK_GYRO, "Gyro sensor error (pitch)"},
    {mk_proto::Error_code::FC_ROLL_GYRO, "Gyro sensor error (roll)"},
    {mk_proto::Error_code::FC_YAW_GYRO, "Gyro sensor error (yaw)"},
    {mk_proto::Error_code::FC_NICK_ACC, "Accelerometer error (pitch)"},
    {mk_proto::Error_code::FC_ROLL_ACC, "Accelerometer error (roll)"},
    {mk_proto::Error_code::FC_Z_ACC, "Accelerometer error (Z-axis)"},
    {mk_proto::Error_code::PRESSURE_SENSOR, "Pressure sensor error"},
    {mk_proto::Error_code::FC_I2C, "I2C communication with the brush-less controller is disturbed"},
    {mk_proto::Error_code::BL_MISSING, "Brush-less controller missing"},
    {mk_proto::Error_code::MIXER_ERROR, "Too many brush-less controllers detected"},
    {mk_proto::Error_code::CAREFREE_ERROR, "CareFree is used, but the value of the Mk3Mag is invalid"},
    {mk_proto::Error_code::GPS_LOST, "GPS signal was lost"},
    {mk_proto::Error_code::MAGNET_ERROR, "Magnetic field of the compass-sensor is different from "
                                         "the magnetic field during calibration."},
    {mk_proto::Error_code::MOTOR_RESTART, "Brush-less controller tries motor restarting (possibly blocked runner)"},
    {mk_proto::Error_code::BL_LIMITATION, "Brush-less controller overload"},
    {mk_proto::Error_code::WAYPOINT_RANGE, "Waypoint out of range"},
    {mk_proto::Error_code::NO_SD_CARD, "No SD card"},
    {mk_proto::Error_code::SD_LOGGING_ABORTED, "SD card writing error"},
    {mk_proto::Error_code::FLYING_RANGE, "Maximal flight range exceeded"},
    {mk_proto::Error_code::MAX_ALTITUDE, "Maximal altitude exceeded"},
    {mk_proto::Error_code::NO_GPS_FIX, "Armed without GPS fix"},
    {mk_proto::Error_code::COMPASS_NOT_CALIBRATED, "Compass not calibrated"},
    {mk_proto::Error_code::BL_SELFTEST_ERROR, "Brush-less controller self-test error"},
    {mk_proto::Error_code::NO_EXT_COMPASS, "External compass error"},
    {mk_proto::Error_code::COMPASS_ERROR, "Compass error"},
    {mk_proto::Error_code::FAILSAFE_POS, "Flying to fail-safe position"},
    {mk_proto::Error_code::REDUNDANCY, "Redundancy degraded"},
    {mk_proto::Error_code::REDUNDANCY_TEST, "Redundancy test mode"},
    {mk_proto::Error_code::GPS_UPDATE_RATE, "GPS update rate too low"},
    {mk_proto::Error_code::CANBUS, "CAN-bus error"},
    {mk_proto::Error_code::RC_5V_SUPPLY, "5V supply voltage low"},
    {mk_proto::Error_code::POWER_SUPPLY, "Power supply failure"},
    {mk_proto::Error_code::ACC_NOT_CALIBRATED, "Accelerometer not calibrated"},
    {mk_proto::Error_code::PARACHUTE, "Parachute engaged, drone disarmed"},
    {mk_proto::Error_code::OUTSIDE_FLYZONE, "Fly-zone breached"},
    {mk_proto::Error_code::NO_FLYZONE, "Fly-zone not loaded"},
};

} // namespace

std::string
Mikrokopter_protocol::Get_error_message(mk_proto::Error_code code)
{
    auto it = error_messages.find(code);
    if (it == error_messages.end()) {
        std::stringstream ss;
        ss << "Unknown error code: " << static_cast<int>(code);
        return ss.str();
    }
    return it->second;
}
