// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#ifndef _MIKROKOPTER_PROTOCOL_H_
#define _MIKROKOPTER_PROTOCOL_H_

#include <ugcs/vsm/vsm.h>

#include <protocol_data.h>


class Mikrokopter_protocol: public ugcs::vsm::Request_processor {
    DEFINE_COMMON_CLASS(Mikrokopter_protocol, Request_container)

public:
    /** Protocol state. */
    enum class State {
        /** Querying channel to determine capabilities. */
        INITIALIZING,
        /** Channel capabilities retrieved, it is functional now. */
        OPERATIONAL,
        /** Channel is closed. */
        CLOSED
    };

    /** Packet address value. */
    enum class Address: uint8_t {
        NONE = 0,
        /** Flight controller. */
        FC = 1,
        /** Navigation controller. */
        NC = 2,
        /** Magnetometer. */
        MK3MAG = 3,
        /** Brush-less motor controller. */
        BL = 5
    };

    /** Command ID values. */
    enum class Command_id: uint8_t {
        NONE = 0,
        /** Serial link test. */
        LINK_TEST = 'z',
        /** Serial link test response. */
        LINK_TEST_RESP = 'Z',
        /** Send waypoint. */
        SEND_WP = 'w',
        /** Send waypoint response. */
        SEND_WP_RESP = 'W',
        /** Request waypoint. */
        REQUEST_WP = 'x',
        /** Request waypoint response. */
        REQUEST_WP_RESP = 'X',
        /** Request OSD data. */
        REQUEST_OSD_DATA = 'o',
        /** OSD data from NC. */
        OSD_DATA = 'O'
    };

    /** Payload or whole packet data. */
    typedef std::vector<uint8_t> Data;
    /** Shared pointer for data buffer. */
    typedef std::shared_ptr<Data> Data_ptr;

    /** Handler for the event of protocol transition to OPERATIONAL state. */
    typedef ugcs::vsm::Callback_proxy<void> Ready_handler;
    /** Handler for the received packets.
     * @param result Operation result. Can be TIMED_OUT.
     * @param data Decoded payload data.
     */
    typedef ugcs::vsm::Callback_proxy<void, ugcs::vsm::Io_result, Data_ptr> Data_handler;

    static constexpr std::chrono::milliseconds DEFAULT_TIMEOUT =
        std::chrono::milliseconds(400);

    /** Builder for data handler. */
    DEFINE_CALLBACK_BUILDER(Make_data_handler,
                            (ugcs::vsm::Io_result, Data_ptr),
                            (ugcs::vsm::Io_result::OK, nullptr));

    /** Stream for specific messages subscription. */
    class Stream:
        public std::enable_shared_from_this<Stream> {
        DEFINE_COMMON_CLASS(Stream, Stream)

    public:
        /** Reference type. */
        typedef ugcs::vsm::Reference_guard<Ptr> Ref;

        typedef ugcs::vsm::Callback_proxy<void, Data_ptr> Packet_handler;
        /** Builder for packet handler. */
        DEFINE_CALLBACK_BUILDER(Make_packet_handler, (Data_ptr), (nullptr));

        Stream(Mikrokopter_protocol &protocol):
            protocol(protocol)
        {}

        ~Stream();

        /** Close the stream. */
        void
        Close();

        bool
        Is_closed() const;

        /** Read next packet. */
        ugcs::vsm::Operation_waiter
        Read(Packet_handler handler,
             ugcs::vsm::Request_completion_context::Ptr comp_ctx =
                 ugcs::vsm::Request_temp_completion_context::Create());

    private:
        friend Ref;
        friend class Mikrokopter_protocol;

        static constexpr size_t MAX_PKT_QUEUE_SIZE = 32;

        Mikrokopter_protocol &protocol;
        /** Reference counter. */
        std::atomic_int ref_count = { 0 };
        bool is_closed = false;
        mutable std::mutex close_mutex;

        struct Pending_request {
            ugcs::vsm::Request::Ptr request;
            Packet_handler handler;
        };

        std::list<Pending_request> pending_requests;
        std::list<Data_ptr> packet_queue;

        /** Add reference to the stream. */
        void
        Add_ref();

        /** Release reference for the stream. The stream is closed when last
         * reference is released.
         */
        void
        Release_ref();

        /** Accept incoming packet.
         *
         * @param data Packet data.
         * @return True to continue listening, false to unsubscribe the stream.
         */
        bool
        On_packet(Data_ptr data);

        void
        Handle_read(ugcs::vsm::Request::Ptr request, Packet_handler handler);
    };

    Mikrokopter_protocol(ugcs::vsm::Io_stream::Ref stream);

    /** Intercept event of protocol transition to OPERATIONAL state. */
    void
    Set_ready_handler(Ready_handler handler)
    {
        ready_handler = handler;
    }

    /** Send command to the device.
     *
     * @param id Command ID.
     * @param address Destination address on the board.
     * @param data Command payload data. Can be empty if no payload required.
     * @param response_id Response packet ID. Can be NONE if no response expected
     *      (or should be waited for). Retransmissions are never sent in such case.
     * @param response_handler Handler to invoke for the response processing.
     * @param comp_ctx Completion context for response handler invocation. Can
     *      be null to use default one.
     * @param timeout Response waiting timeout. If exceeded, the response
     *      handler is invoked with TIMED_OUT result or retransmission is sent
     *      if enabled. Zero indicates infinite timeout.
     * @param num_retransmissions Number of retransmissions to send. Zero to
     *      disable retransmissions.
     */
    ugcs::vsm::Operation_waiter
    Command(Command_id id, Address address,
            Data &&data,
            Command_id response_id = Command_id::NONE,
            Data_handler response_handler =
                ugcs::vsm::Make_dummy_callback<void, ugcs::vsm::Io_result, Data_ptr>(),
            ugcs::vsm::Request_completion_context::Ptr comp_ctx =
                ugcs::vsm::Request_temp_completion_context::Create(),
            std::chrono::milliseconds timeout = DEFAULT_TIMEOUT,
            int num_retransmissions = 3);

    /** Add handler for incoming packets.
     *
     * @param address Board address for the packets.
     * @param id Command ID to add handler for.
     * @param handler Handler to invoke with the received data.
     * @param comp_ctx Completion context for the handler invocation. Can be
     *      null to use default one.
     */
    void
    Add_packet_handler(Address address, Command_id id,
                       Data_handler handler,
                       ugcs::vsm::Request_completion_context::Ptr comp_ctx = nullptr);

    std::string
    Get_port_name()
    {
        return stream->Get_name();
    }

    State
    Get_state() const
    {
        return state;
    }

    /** Close this protocol instance. */
    void
    Close();

    /** Subscribe to specific incoming messages. */
    Stream::Ref
    Subscribe(Command_id id, Address address);

    /** Get packet processing statistics. */
    void
    Get_pkt_stats(size_t &received, size_t &errors)
    {
        pkt_stats.Get(received, errors);
    }

    static std::string
    Get_error_message(mk_proto::Error_code code);

private:
    class Pkt_stats {
    public:
        class Pkt {
        public:
            Pkt(Pkt_stats &stats):
                stats(stats)
            {}

            ~Pkt()
            {
                stats.Pkt_received(error);
            }

            void
            Succeded()
            {
                error = false;
            }
        private:
            Pkt_stats &stats;
            bool error = true;
        };

        void
        Pkt_received(bool has_error = false)
        {
            std::unique_lock<std::mutex> lock(mutex);
            received++;
            if (has_error) {
                errors++;
            }
        }

        void
        operator++(int)
        {
            Pkt_received();
        }

        void
        Get(size_t &received, size_t &errors)
        {
            std::unique_lock<std::mutex> lock(mutex);
            received = this->received;
            errors = this->errors;
        }

    private:
        std::mutex mutex;
        /** Total number of packets received. */
        size_t received = 0;
        /** Number of packets decoding errors. */
        size_t errors = 0;
    };
    /** Minimal length of the incoming packet (start byte, address, ID, CRC). */
    static constexpr size_t MIN_PACKET_LENGTH = 5;
    /** Maximal incoming packet size. */
    static constexpr size_t MAX_PACKET_LENGTH = 512;
    /** Pattern used in echo command for device presence detection. */
    static constexpr uint16_t DETECTION_PATTERN = 0x5aa5;
    /** Current protocol state. */
    State state = State::INITIALIZING;
    /** Associated stream. */
    ugcs::vsm::Io_stream::Ref stream;
    /** Readiness handler if any. */
    Ready_handler ready_handler;
    /** Internal completion context. */
    ugcs::vsm::Request_completion_context::Ptr comp_ctx;
    /** Worker for processor and completion context. */
    ugcs::vsm::Request_worker::Ptr worker;
    /** Packet buffer. */
    std::string packet_buf;
    /** Packet statistics. */
    Pkt_stats pkt_stats;
    /** Current link detection operation. */
    ugcs::vsm::Operation_waiter link_detection_op;
    /** Current stream read operation. */
    ugcs::vsm::Operation_waiter read_op;
    /** Current stream write operation. */
    ugcs::vsm::Operation_waiter write_op;
    /** Queued packets for writing. */
    std::list<ugcs::vsm::Io_buffer::Ptr> write_queue;

    /** Represents installed response processing handler. */
    struct Response_handler {
        typedef std::shared_ptr<Response_handler> Ptr;

        ugcs::vsm::Request::Ptr request;
        /** Response handler. */
        Data_handler handler;
        /** Request data to use for retransmissions. */
        ugcs::vsm::Io_buffer::Ptr request_pkt;
        /** Request timeout duration, zero to disable timeout. */
        std::chrono::milliseconds timeout;
        /** Timeout timer if was specified. */
        ugcs::vsm::Timer_processor::Timer::Ptr to_timer;
        /** Number of retransmissions left to send. */
        int num_retrans_left;

        /** Next requests queued to the same destination. */
        std::list<Ptr> next;
    };

    /** Identifier for a packet being waited for. */
    typedef std::pair<Address, Command_id> Address_id;

    typedef std::map<Address_id, Response_handler::Ptr> Response_map;
    /** Currently installed response processing handlers. */
    Response_map response_handlers;

    typedef std::map<Address_id, Stream::Ptr> Stream_map;
    /** Active streams. */
    Stream_map streams;
    /** Protects streams map. */
    std::mutex stream_mutex;

    virtual void
    On_enable() override;

    virtual void
    On_disable() override;

    /** Enable in a protocol context. */
    void
    On_enable_handler(ugcs::vsm::Request::Ptr);

    /** Disable in a protocol context. */
    void
    On_disable_handler(ugcs::vsm::Request::Ptr);

    /** Incoming raw data handler. */
    void
    On_data_received(ugcs::vsm::Io_buffer::Ptr buf, ugcs::vsm::Io_result result);

    /** Schedule next read operation for the input stream. */
    void
    Schedule_read();

    /** Character received. */
    void
    On_char_received(int c);

    void
    On_packet_received_raw(std::string &&packet);

    void
    On_packet_received(Address_id aid, Data_ptr payload);

    void
    Handle_command(ugcs::vsm::Request::Ptr request,
                   Address address,
                   Command_id response_id,
                   Data_handler response_handler,
                   ugcs::vsm::Io_buffer::Ptr request_pkt,
                   std::chrono::milliseconds timeout,
                   int num_retransmissions);

    /** Build packet for sending. */
    ugcs::vsm::Io_buffer::Ptr
    Build_packet(Command_id id, Address address,
                 Data &&data);

    /** Encode the provided data by MK base64 format. Output appended to the
     * provided buffer.
     */
    void
    Base64_encode(const Data &data, Data &out);

    /** Decode base64-encoded data from the string. Return null if decodig error
     * occurred.
     */
    Data_ptr
    Base64_decode(const uint8_t *data, size_t size);

    /** Calculate checksum for the provided data.
     *
     * @return Two characters which should be use to encode the checksum in a
     *      packet.
     */
    std::pair<uint8_t, uint8_t>
    Calculate_crc(uint8_t *data, size_t size);

    /** Called when request timeout elapses. */
    bool
    Timeout_handler(Address_id aid);

    void
    Detection_handler(ugcs::vsm::Io_result result, Data_ptr pkt);

    /** Finalize request processing.
     * @param it Iterator pointing to corresponding handler record.
     * @param pkt Data packet if received, null if timed out.
     */
    void
    Finish_request(Response_map::iterator &it, Data_ptr pkt = nullptr);

    void
    Start_request(const Address_id &aid, Response_handler &h);

    void
    Handle_close(ugcs::vsm::Request::Ptr);

    void
    Send_packet(ugcs::vsm::Io_buffer::Ptr pkt);

    /** Handle packet sending completion. */
    void
    Handle_send(ugcs::vsm::Io_result result);
};

#endif /* _MIKROKOPTER_PROTOCOL_H_ */
