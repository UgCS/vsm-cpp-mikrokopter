// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/** Mikrokopter protocol data structures. */

#ifndef PROTOCOL_DATA_H_
#define PROTOCOL_DATA_H_

namespace mk_proto {

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
    POI = 1,
    FAILSAFE = 2,
    LAND = 3
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
    /** Camera auto-triggering distance, m */
    uint8_t auto_trigger_distance;
    uint8_t reserve;
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
    /** Maximal telemetry rate, bytes per second. */
    ugcs::vsm::Le_uint16 max_bps;
} __PACKED;

/** NC status report (old version). */
struct Navi_data_old {
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
enum Nc_flags {
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

struct Navi_data_hdr {
    /** Packet type index. */
    uint8_t index;
    ugcs::vsm::Le_int32 longitude;
    ugcs::vsm::Le_int32 latitude;
    /** Altitude, 5cm units */
    ugcs::vsm::Le_int16 altitude;
    /** 10cm/s units. */
    uint8_t ground_speed;
    /** See @ref Status_flags enum. */
    uint8_t status_flags;
} __PACKED;

struct Navi_data_tiny: Navi_data_hdr {
    /** Status from a connected CamCtrl unit: 'R' = REC  'c' = Ready  '!' = Error  ...etc */
    uint8_t cam_ctrl_char;
} __PACKED;

struct Navi_data_flags: Navi_data_hdr {
    /** See @ref Status_flags enum. */
    uint8_t status_flags_ex;
    uint8_t nc_flags;
    uint8_t reserved;
    /** See @ref Error_code enum. */
    uint8_t error_code;
    uint8_t speak_hott;
    /** display as ascii character ('+' = 'climb' etc) */
    uint8_t vario_char;
    /** display as ascii character ('H' = 'Home' etc) */
    uint8_t gps_mode_char;
    /** status byte of the BL-Ctrls */
    uint8_t bl_min_of_max_pwm;
} __PACKED;

struct Navi_data_target: Navi_data_hdr {
    ugcs::vsm::Le_int32 target_longitude;
    ugcs::vsm::Le_int32 target_latitude;
    ugcs::vsm::Le_int16 target_altitude;
    uint8_t rc_quality;
} __PACKED;

struct Navi_data_home: Navi_data_hdr {
    ugcs::vsm::Le_int32 home_longitude;
    ugcs::vsm::Le_int32 home_latitude;
    ugcs::vsm::Le_int16 home_altitude;
    /** current WP operation radius around the Home Position in m */
    ugcs::vsm::Le_uint16 wp_operating_radius;
    uint8_t lipo_cell_count;
    /** 10m units. */
    uint8_t descend_range;
    /** 10m units. */
    uint8_t manual_flying_range;
    uint8_t reserved1, reserved2;
} __PACKED;

struct Navi_data_deviation: Navi_data_hdr {
    /** In seconds. */
    ugcs::vsm::Le_uint16 flying_time;
    /** 10cm units. */
    ugcs::vsm::Le_uint16 distance_to_home;
    /** 2deg units. */
    uint8_t heading_to_home;
    /** 10cm units. */
    ugcs::vsm::Le_uint16 distance_to_target;
    /** 2deg units. */
    uint8_t heading_to_target;
    /** Pitch angle in degrees. */
    int8_t angle_nick;
    /** Roll angle in degrees. */
    int8_t angle_roll;
    /** number of satellites used for position solution */
    uint8_t sats_in_use;
} __PACKED;

struct Navi_data_wp: Navi_data_hdr {
    /** index of current waypoints running from 0 to wp_count-1 */
    uint8_t wp_index;
    /** number of stored waypoints */
    uint8_t wp_count;
    /** time in s to stay at the given target, counts down to 0 if target has been reached */
    uint8_t target_hold_time;
    /** the current value of the event channel */
    uint8_t wp_event_channel;
    uint8_t reserve;
} __PACKED;

struct Navi_data_volatile: Navi_data_hdr {
    /** 0.1V units. */
    ugcs::vsm::Le_uint16 bat_voltage;
    /** 0.1A units. */
    ugcs::vsm::Le_uint16 current;
    /** In mAh */
    ugcs::vsm::Le_uint16 used_capacity;
    /** climb(+) and sink(-) rate (??) */
    int8_t variometer;
    /** 2deg units. */
    uint8_t heading;
    /** 2deg units. */
    uint8_t compass_heading;
    uint8_t throttle;
    ugcs::vsm::Le_uint16 shutter_control;
    ugcs::vsm::Le_int16 setpoint_altitude;
} __PACKED;

/** Navi_data_hdr::status_flags bits. */
enum class Status_flags {
    CAREFREE = 0x01,
    ALTITUDE_CONTROL = 0x02,
    CALIBRATE = 0x04,
    OUT1_ACTIVE = 0x08,
    OUT2_ACTIVE = 0x10,
    LOW_BATTERY = 0x20,
    VARIO_TRIM_UP = 0x40,
    VARIO_TRIM_DOWN = 0x80
};

enum Navi_data_index {
    TINY = 10,
    FLAGS = 11,
    TARGET = 12,
    HOME = 13,
    DEVIATION = 14,
    WP = 15,
    VOLATILE = 16
};

/** Navi_data_flags::status_flags_ex bits. */
enum Status_flags_ex {
    MOTOR_RUN = 0x01,
    FLY = 0x02,
    RC_FAILSAFE_ACTIVE = 0x04,
    START = 0x08,
    EMERGENCY_LANDING = 0x10,
    WAIT_FOR_TAKEOFF = 0x20,
    AUTO_STARTING = 0x40,
    AUTO_LANDING = 0x80
};

enum class Error_code {
    OK = 0,
    FC_NOT_COMPATIBLE = 1,
    MK3MAG_NOT_COMPTIBLE = 2,
    NO_FC_COMM = 3,
    NO_COMPASS_COMM = 4,
    NO_GPS_COMM = 5,
    BAD_COMPASS_VALUE = 6,
    RC_SIGNAL_LOST = 7,
    FC_SPI_RX_ERROR = 8,
    NO_NC_COMM = 9,
    FC_NICK_GYRO = 10,
    FC_ROLL_GYRO = 11,
    FC_YAW_GYRO = 12,
    FC_NICK_ACC = 13,
    FC_ROLL_ACC = 14,
    FC_Z_ACC = 15,
    PRESSURE_SENSOR = 16,
    FC_I2C = 17,
    BL_MISSING = 18,
    MIXER_ERROR = 19,
    CAREFREE_ERROR = 20,
    GPS_LOST = 21,
    MAGNET_ERROR = 22,
    MOTOR_RESTART = 23,
    BL_LIMITATION = 24,
    WAYPOINT_RANGE = 25,
    NO_SD_CARD = 26,
    SD_LOGGING_ABORTED = 27,
    FLYING_RANGE = 28,
    MAX_ALTITUDE = 29,
    NO_GPS_FIX = 30,
    COMPASS_NOT_CALIBRATED = 31,
    BL_SELFTEST_ERROR = 32,
    NO_EXT_COMPASS = 33,
    COMPASS_ERROR = 34,
    FAILSAFE_POS = 35,
    REDUNDANCY = 36,
    REDUNDANCY_TEST = 37,
    GPS_UPDATE_RATE = 38,
    CANBUS = 39,
    RC_5V_SUPPLY = 40,
    POWER_SUPPLY = 41,
    ACC_NOT_CALIBRATED = 42,
    PARACHUTE = 43,
    OUTSIDE_FLYZONE = 44,
    NO_FLYZONE = 45
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
    Data &
    operator=(const Data &) = default;
    Data &
    operator=(Data &&) = default;

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
