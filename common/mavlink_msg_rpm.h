#pragma once
// MESSAGE RPM PACKING

#define MAVLINK_MSG_ID_RPM 402

MAVPACKED(
typedef struct __mavlink_rpm_t {
 uint64_t time_usec; /*< [us] Timestamp (since system boot).*/
 float indicated_frequency_hz; /*<  Indicated rotor Frequency in Hz*/
 float estimated_accurancy_hz; /*<  estimated accurancy in Hz*/
 float indicated_frequency_rpm; /*<  indicated rotor Frequency in Revolution per minute*/
 float estimated_accurancy_rpm; /*<  estimated accurancy in Revolution per minute*/
}) mavlink_rpm_t;

#define MAVLINK_MSG_ID_RPM_LEN 24
#define MAVLINK_MSG_ID_RPM_MIN_LEN 24
#define MAVLINK_MSG_ID_402_LEN 24
#define MAVLINK_MSG_ID_402_MIN_LEN 24

#define MAVLINK_MSG_ID_RPM_CRC 116
#define MAVLINK_MSG_ID_402_CRC 116



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RPM { \
    402, \
    "RPM", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rpm_t, time_usec) }, \
         { "indicated_frequency_hz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rpm_t, indicated_frequency_hz) }, \
         { "estimated_accurancy_hz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rpm_t, estimated_accurancy_hz) }, \
         { "indicated_frequency_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rpm_t, indicated_frequency_rpm) }, \
         { "estimated_accurancy_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rpm_t, estimated_accurancy_rpm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RPM { \
    "RPM", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rpm_t, time_usec) }, \
         { "indicated_frequency_hz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rpm_t, indicated_frequency_hz) }, \
         { "estimated_accurancy_hz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rpm_t, estimated_accurancy_hz) }, \
         { "indicated_frequency_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rpm_t, indicated_frequency_rpm) }, \
         { "estimated_accurancy_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rpm_t, estimated_accurancy_rpm) }, \
         } \
}
#endif

/**
 * @brief Pack a rpm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (since system boot).
 * @param indicated_frequency_hz  Indicated rotor Frequency in Hz
 * @param estimated_accurancy_hz  estimated accurancy in Hz
 * @param indicated_frequency_rpm  indicated rotor Frequency in Revolution per minute
 * @param estimated_accurancy_rpm  estimated accurancy in Revolution per minute
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float indicated_frequency_hz, float estimated_accurancy_hz, float indicated_frequency_rpm, float estimated_accurancy_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RPM_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_frequency_hz);
    _mav_put_float(buf, 12, estimated_accurancy_hz);
    _mav_put_float(buf, 16, indicated_frequency_rpm);
    _mav_put_float(buf, 20, estimated_accurancy_rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPM_LEN);
#else
    mavlink_rpm_t packet;
    packet.time_usec = time_usec;
    packet.indicated_frequency_hz = indicated_frequency_hz;
    packet.estimated_accurancy_hz = estimated_accurancy_hz;
    packet.indicated_frequency_rpm = indicated_frequency_rpm;
    packet.estimated_accurancy_rpm = estimated_accurancy_rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RPM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
}

/**
 * @brief Pack a rpm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (since system boot).
 * @param indicated_frequency_hz  Indicated rotor Frequency in Hz
 * @param estimated_accurancy_hz  estimated accurancy in Hz
 * @param indicated_frequency_rpm  indicated rotor Frequency in Revolution per minute
 * @param estimated_accurancy_rpm  estimated accurancy in Revolution per minute
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float indicated_frequency_hz,float estimated_accurancy_hz,float indicated_frequency_rpm,float estimated_accurancy_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RPM_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_frequency_hz);
    _mav_put_float(buf, 12, estimated_accurancy_hz);
    _mav_put_float(buf, 16, indicated_frequency_rpm);
    _mav_put_float(buf, 20, estimated_accurancy_rpm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPM_LEN);
#else
    mavlink_rpm_t packet;
    packet.time_usec = time_usec;
    packet.indicated_frequency_hz = indicated_frequency_hz;
    packet.estimated_accurancy_hz = estimated_accurancy_hz;
    packet.indicated_frequency_rpm = indicated_frequency_rpm;
    packet.estimated_accurancy_rpm = estimated_accurancy_rpm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RPM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
}

/**
 * @brief Encode a rpm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rpm_t* rpm)
{
    return mavlink_msg_rpm_pack(system_id, component_id, msg, rpm->time_usec, rpm->indicated_frequency_hz, rpm->estimated_accurancy_hz, rpm->indicated_frequency_rpm, rpm->estimated_accurancy_rpm);
}

/**
 * @brief Encode a rpm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rpm_t* rpm)
{
    return mavlink_msg_rpm_pack_chan(system_id, component_id, chan, msg, rpm->time_usec, rpm->indicated_frequency_hz, rpm->estimated_accurancy_hz, rpm->indicated_frequency_rpm, rpm->estimated_accurancy_rpm);
}

/**
 * @brief Send a rpm message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (since system boot).
 * @param indicated_frequency_hz  Indicated rotor Frequency in Hz
 * @param estimated_accurancy_hz  estimated accurancy in Hz
 * @param indicated_frequency_rpm  indicated rotor Frequency in Revolution per minute
 * @param estimated_accurancy_rpm  estimated accurancy in Revolution per minute
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rpm_send(mavlink_channel_t chan, uint64_t time_usec, float indicated_frequency_hz, float estimated_accurancy_hz, float indicated_frequency_rpm, float estimated_accurancy_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RPM_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_frequency_hz);
    _mav_put_float(buf, 12, estimated_accurancy_hz);
    _mav_put_float(buf, 16, indicated_frequency_rpm);
    _mav_put_float(buf, 20, estimated_accurancy_rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, buf, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#else
    mavlink_rpm_t packet;
    packet.time_usec = time_usec;
    packet.indicated_frequency_hz = indicated_frequency_hz;
    packet.estimated_accurancy_hz = estimated_accurancy_hz;
    packet.indicated_frequency_rpm = indicated_frequency_rpm;
    packet.estimated_accurancy_rpm = estimated_accurancy_rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, (const char *)&packet, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#endif
}

/**
 * @brief Send a rpm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rpm_send_struct(mavlink_channel_t chan, const mavlink_rpm_t* rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rpm_send(chan, rpm->time_usec, rpm->indicated_frequency_hz, rpm->estimated_accurancy_hz, rpm->indicated_frequency_rpm, rpm->estimated_accurancy_rpm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, (const char *)rpm, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#endif
}

#if MAVLINK_MSG_ID_RPM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rpm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float indicated_frequency_hz, float estimated_accurancy_hz, float indicated_frequency_rpm, float estimated_accurancy_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_frequency_hz);
    _mav_put_float(buf, 12, estimated_accurancy_hz);
    _mav_put_float(buf, 16, indicated_frequency_rpm);
    _mav_put_float(buf, 20, estimated_accurancy_rpm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, buf, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#else
    mavlink_rpm_t *packet = (mavlink_rpm_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->indicated_frequency_hz = indicated_frequency_hz;
    packet->estimated_accurancy_hz = estimated_accurancy_hz;
    packet->indicated_frequency_rpm = indicated_frequency_rpm;
    packet->estimated_accurancy_rpm = estimated_accurancy_rpm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, (const char *)packet, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#endif
}
#endif

#endif

// MESSAGE RPM UNPACKING


/**
 * @brief Get field time_usec from rpm message
 *
 * @return [us] Timestamp (since system boot).
 */
static inline uint64_t mavlink_msg_rpm_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field indicated_frequency_hz from rpm message
 *
 * @return  Indicated rotor Frequency in Hz
 */
static inline float mavlink_msg_rpm_get_indicated_frequency_hz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field estimated_accurancy_hz from rpm message
 *
 * @return  estimated accurancy in Hz
 */
static inline float mavlink_msg_rpm_get_estimated_accurancy_hz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field indicated_frequency_rpm from rpm message
 *
 * @return  indicated rotor Frequency in Revolution per minute
 */
static inline float mavlink_msg_rpm_get_indicated_frequency_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field estimated_accurancy_rpm from rpm message
 *
 * @return  estimated accurancy in Revolution per minute
 */
static inline float mavlink_msg_rpm_get_estimated_accurancy_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a rpm message into a struct
 *
 * @param msg The message to decode
 * @param rpm C-struct to decode the message contents into
 */
static inline void mavlink_msg_rpm_decode(const mavlink_message_t* msg, mavlink_rpm_t* rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rpm->time_usec = mavlink_msg_rpm_get_time_usec(msg);
    rpm->indicated_frequency_hz = mavlink_msg_rpm_get_indicated_frequency_hz(msg);
    rpm->estimated_accurancy_hz = mavlink_msg_rpm_get_estimated_accurancy_hz(msg);
    rpm->indicated_frequency_rpm = mavlink_msg_rpm_get_indicated_frequency_rpm(msg);
    rpm->estimated_accurancy_rpm = mavlink_msg_rpm_get_estimated_accurancy_rpm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RPM_LEN? msg->len : MAVLINK_MSG_ID_RPM_LEN;
        memset(rpm, 0, MAVLINK_MSG_ID_RPM_LEN);
    memcpy(rpm, _MAV_PAYLOAD(msg), len);
#endif
}
