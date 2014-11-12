// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/** Mikrokopter protocol data structures. */

#ifndef PROTOCOL_DATA_H_
#define PROTOCOL_DATA_H_

namespace proto {

/** Used in Serial Link Test command. */
struct Echo {
    /** Echo pattern. */
    ugcs::vsm::Le_uint16 pattern;
} __PACKED;

enum class Gps_status: uint8_t {
    INVALID,
    NEW_DATA,
    PROCESSED
};

enum class Point_type: uint8_t {
    INVALID = 255,
    WP = 0,
    POI = 1
};

struct Gps_pos {
    /** In 1E-7 degrees */
    ugcs::vsm::Le_int32 longitude,
    /** In 1E-7 degrees */
                        latitude,
    /** In decimetres. */
                        altitude;
    Gps_status status;

    ugcs::vsm::Wgs84_position
    Get_position() const
    {
        return ugcs::vsm::Geodetic_tuple(
            static_cast<double>(latitude) / 1e7 / 180.0 * M_PI,
            static_cast<double>(longitude) / 1e7 / 180.0 * M_PI,
            static_cast<double>(altitude) / 10.0);
    }

    /** In telemetry reports altitude has different units. */
    ugcs::vsm::Wgs84_position
    Get_position_telemetry() const
    {
        return ugcs::vsm::Geodetic_tuple(
            static_cast<double>(latitude) / 1e7 / 180.0 * M_PI,
            static_cast<double>(longitude) / 1e7 / 180.0 * M_PI,
            static_cast<double>(altitude) / 1000.0);
    }

    static Gps_pos
    From_position(const ugcs::vsm::Wgs84_position &pos)
    {
        ugcs::vsm::Geodetic_tuple gt = pos.Get_geodetic();
        return Gps_pos {gt.longitude * 180.0 / M_PI * 1e7,
                        gt.latitude * 180.0 / M_PI * 1e7,
                        gt.altitude * 10,
                        Gps_status::PROCESSED};
    }
} __PACKED;

struct Gps_pos_dev {
    /** Distance to target in cm. */
    ugcs::vsm::Le_uint16 distance;
    /** Course to target in deg. */
    ugcs::vsm::Le_int16 bearing;
} __PACKED;

struct Point {
    /** The GPS position of the waypoint. */
    Gps_pos position;
    /** Orientation, 0 no action, 1...360 fix heading,
     * negative = Index to POI in WP List.
     */
    ugcs::vsm::Le_int16 heading;
    /** In meters, if the MK is within that range around the target, then the
     * next target is triggered.
     */
    uint8_t tolerance_radius;
    /**  In seconds, if the was once in the tolerance area around a WP, this
     * time defines the delay before the next WP is triggered.
     */
    uint8_t hold_time;
    /** Future implementation. */
    uint8_t event_flag;
    /** To identify different waypoints. */
    uint8_t index;
    Point_type type;
    uint8_t  wp_event_channel_value;
    /** Rate to change the setpoint, 0.1 m/s */
    uint8_t altitude_rate;
    /** Rate to change the Position, 0.1 m/s */
    uint8_t speed;
    /** Camera servo angle XXX ? */
    uint8_t cam_angle;
    /** Name of that point (ASCII) */
    uint8_t name[4];
    uint8_t reserve[2];
} __PACKED;

/** Payload for "Request Waypoint" command. */
struct Wp_request {
    /** Waypoint index. */
    uint8_t index;
} __PACKED;

/** Payload for response to "Request Waypoint" command. */
struct Wp_response {
    /** Total number of waypoints. */
    uint8_t count;
    /** Waypoint index. */
    uint8_t index;
    /** Waypoint descriptor. */
    Point wp;
} __PACKED;

/** Payload for response to "Send Waypoint" command. */
struct Send_wp_response {
    /** Total number of waypoints. */
    uint8_t count;
};

/** OSD data request. */
struct Osd_request {
    /** Sending interval, 10ms. */
    uint8_t interval;
} __PACKED;

/** NC status report. */
struct Navi_data {
    uint8_t version;
    Gps_pos current_position;
    Gps_pos target_position;
    Gps_pos_dev target_pos_deviation;
    Gps_pos home_position;
    Gps_pos_dev home_pos_deviation;
    /** Index of current waypoints running from 0 to WaypointNumber-1 */
    uint8_t wp_index;
    /** Number of stored waypoints */
    uint8_t wp_count;
    /** Number of satellites used for position solution */
    uint8_t satellites_in_use;
    /** Height according to air pressure. Unit is 5cm. */
    ugcs::vsm::Le_int16 altimeter;
    /** Climb(+) and sink(-) rate */
    ugcs::vsm::Le_int16 variometer;
    /** In seconds */
    ugcs::vsm::Le_uint16 flying_time;
    /** Battery Voltage in 0.1 Volts */
    uint8_t bat_voltage;
    /** Speed over ground in cm/s (2D) */
    ugcs::vsm::Le_uint16 ground_speed;
    /** Current flight direction in deg as angle to north (course). */
    ugcs::vsm::Le_int16 heading;
    /** Current compass value in deg */
    ugcs::vsm::Le_int16 yaw;
    /** Current pitch angle in deg */
    int8_t pitch;
    /** Current roll angle in deg */
    int8_t roll;
    uint8_t rc_quality;
    /** Flags from FC */
    uint8_t fc_status_flags;
    /** Flags from NC */
    uint8_t nc_flags;
    /** 0 --> okay */
    uint8_t error_code;
    /** Current operation radius around the Home Position in m */
    uint8_t operating_radius;
    /** Velocity in vertical direction in cm/s */
    ugcs::vsm::Le_int16 top_speed;
    /**  Time in s to stay at the given target, counts down to 0 if target has
     * been reached.
     */
    uint8_t target_hold_time;
    uint8_t fc_status_flags_2;
    /** Setpoint for altitude */
    ugcs::vsm::Le_int16 setpoint_altitude;
    /** For future use */
    uint8_t gas;
    /** Actual current in 0.1A steps */
    ugcs::vsm::Le_uint16 current;
    /** Used capacity in mAh */
    ugcs::vsm::Le_uint16 used_capacity;
} __PACKED;

/** Navi_data::nc_flags bits. */
enum class Nc_flags {
    FREE = 0x01,
    PH = 0x02,
    CH = 0x04,
    RANGE_LIMIT = 0x08,
    NOSERIALLINK = 0x10,
    TARGET_REACHED = 0x20,
    MANUAL = 0x40,
    GPS_OK = 0x80
};

/** Navi_data::fc_status_flags bits. */
enum class Fc_status_flags {
    MOTOR_RUN = 0x01,
    FLY = 0x02,
    CALIBRATE = 0x04,
    START = 0x08,
    EMERGENCY_LANDING = 0x10,
    LOWBAT = 0x20,
    VARIO_TRIM_UP = 0x40,
    VARIO_TRIM_DOWN = 0x80
};

/** Wrapper for payload data structure. */
template <class PayloadType>
class Data {
public:
    typedef std::vector<uint8_t> BufferType;

    Data(const BufferType &buf):
        payload(buf)
    {
        if (payload.size() < sizeof(PayloadType)) {
            payload.resize(sizeof(PayloadType), 0);
        }
    }

    Data(BufferType &&buf):
        payload(std::move(buf))
    {
        if (payload.size() < sizeof(PayloadType)) {
            payload.resize(sizeof(PayloadType), 0);
        }
    }

    Data():
        payload(sizeof(PayloadType), 0)
    {}

    Data(const Data &) = default;
    Data(Data &&) = default;

    PayloadType &
    operator *()
    {
        return *reinterpret_cast<PayloadType *>(&payload.front());
    }

    const PayloadType &
    operator *() const
    {
        return *reinterpret_cast<const PayloadType *>(&payload.front());
    }

    PayloadType *
    operator->()
    {
        return &(**this);
    }

    const PayloadType *
    operator->() const
    {
        return &(**this);
    }

    BufferType
    GetData() const &
    {
        return payload;
    }

    BufferType &&
    GetData() &&
    {
        return std::move(payload);
    }

    operator BufferType() const &
    {
        return GetData();
    }

    operator BufferType() &&
    {
        return std::move(*this).GetData();
    }

private:
    BufferType payload;
};

} /* namespace proto */

#endif /* PROTOCOL_DATA_H_ */
