#pragma once
// MESSAGE ROTOR_FREQUENCY PACKING

#define MAVLINK_MSG_ID_ROTOR_FREQUENCY 49001

MAVPACKED(
typedef struct __mavlink_rotor_frequency_t {
 float measured_frequency_rpm; /*<  Measured Frequency*/
 float estimated_accurancy_rpm ; /*<  Estimated Accurancy*/
}) mavlink_rotor_frequency_t;

#define MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN 8
#define MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN 8
#define MAVLINK_MSG_ID_49001_LEN 8
#define MAVLINK_MSG_ID_49001_MIN_LEN 8

#define MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC 29
#define MAVLINK_MSG_ID_49001_CRC 29



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROTOR_FREQUENCY { \
    49001, \
    "ROTOR_FREQUENCY", \
    2, \
    {  { "measured_frequency_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rotor_frequency_t, measured_frequency_rpm) }, \
         { "estimated_accurancy_rpm ", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rotor_frequency_t, estimated_accurancy_rpm ) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROTOR_FREQUENCY { \
    "ROTOR_FREQUENCY", \
    2, \
    {  { "measured_frequency_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rotor_frequency_t, measured_frequency_rpm) }, \
         { "estimated_accurancy_rpm ", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rotor_frequency_t, estimated_accurancy_rpm ) }, \
         } \
}
#endif

/**
 * @brief Pack a rotor_frequency message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param measured_frequency_rpm  Measured Frequency
 * @param estimated_accurancy_rpm   Estimated Accurancy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rotor_frequency_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float measured_frequency_rpm, float estimated_accurancy_rpm )
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN];
    _mav_put_float(buf, 0, measured_frequency_rpm);
    _mav_put_float(buf, 4, estimated_accurancy_rpm );

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN);
#else
    mavlink_rotor_frequency_t packet;
    packet.measured_frequency_rpm = measured_frequency_rpm;
    packet.estimated_accurancy_rpm  = estimated_accurancy_rpm ;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROTOR_FREQUENCY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC);
}

/**
 * @brief Pack a rotor_frequency message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param measured_frequency_rpm  Measured Frequency
 * @param estimated_accurancy_rpm   Estimated Accurancy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rotor_frequency_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float measured_frequency_rpm,float estimated_accurancy_rpm )
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN];
    _mav_put_float(buf, 0, measured_frequency_rpm);
    _mav_put_float(buf, 4, estimated_accurancy_rpm );

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN);
#else
    mavlink_rotor_frequency_t packet;
    packet.measured_frequency_rpm = measured_frequency_rpm;
    packet.estimated_accurancy_rpm  = estimated_accurancy_rpm ;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROTOR_FREQUENCY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC);
}

/**
 * @brief Encode a rotor_frequency struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rotor_frequency C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rotor_frequency_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rotor_frequency_t* rotor_frequency)
{
    return mavlink_msg_rotor_frequency_pack(system_id, component_id, msg, rotor_frequency->measured_frequency_rpm, rotor_frequency->estimated_accurancy_rpm );
}

/**
 * @brief Encode a rotor_frequency struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rotor_frequency C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rotor_frequency_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rotor_frequency_t* rotor_frequency)
{
    return mavlink_msg_rotor_frequency_pack_chan(system_id, component_id, chan, msg, rotor_frequency->measured_frequency_rpm, rotor_frequency->estimated_accurancy_rpm );
}

/**
 * @brief Send a rotor_frequency message
 * @param chan MAVLink channel to send the message
 *
 * @param measured_frequency_rpm  Measured Frequency
 * @param estimated_accurancy_rpm   Estimated Accurancy
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rotor_frequency_send(mavlink_channel_t chan, float measured_frequency_rpm, float estimated_accurancy_rpm )
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN];
    _mav_put_float(buf, 0, measured_frequency_rpm);
    _mav_put_float(buf, 4, estimated_accurancy_rpm );

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_FREQUENCY, buf, MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC);
#else
    mavlink_rotor_frequency_t packet;
    packet.measured_frequency_rpm = measured_frequency_rpm;
    packet.estimated_accurancy_rpm  = estimated_accurancy_rpm ;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_FREQUENCY, (const char *)&packet, MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC);
#endif
}

/**
 * @brief Send a rotor_frequency message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rotor_frequency_send_struct(mavlink_channel_t chan, const mavlink_rotor_frequency_t* rotor_frequency)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rotor_frequency_send(chan, rotor_frequency->measured_frequency_rpm, rotor_frequency->estimated_accurancy_rpm );
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_FREQUENCY, (const char *)rotor_frequency, MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rotor_frequency_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float measured_frequency_rpm, float estimated_accurancy_rpm )
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, measured_frequency_rpm);
    _mav_put_float(buf, 4, estimated_accurancy_rpm );

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_FREQUENCY, buf, MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC);
#else
    mavlink_rotor_frequency_t *packet = (mavlink_rotor_frequency_t *)msgbuf;
    packet->measured_frequency_rpm = measured_frequency_rpm;
    packet->estimated_accurancy_rpm  = estimated_accurancy_rpm ;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_FREQUENCY, (const char *)packet, MAVLINK_MSG_ID_ROTOR_FREQUENCY_MIN_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN, MAVLINK_MSG_ID_ROTOR_FREQUENCY_CRC);
#endif
}
#endif

#endif

// MESSAGE ROTOR_FREQUENCY UNPACKING


/**
 * @brief Get field measured_frequency_rpm from rotor_frequency message
 *
 * @return  Measured Frequency
 */
static inline float mavlink_msg_rotor_frequency_get_measured_frequency_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field estimated_accurancy_rpm  from rotor_frequency message
 *
 * @return  Estimated Accurancy
 */
static inline float mavlink_msg_rotor_frequency_get_estimated_accurancy_rpm (const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a rotor_frequency message into a struct
 *
 * @param msg The message to decode
 * @param rotor_frequency C-struct to decode the message contents into
 */
static inline void mavlink_msg_rotor_frequency_decode(const mavlink_message_t* msg, mavlink_rotor_frequency_t* rotor_frequency)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rotor_frequency->measured_frequency_rpm = mavlink_msg_rotor_frequency_get_measured_frequency_rpm(msg);
    rotor_frequency->estimated_accurancy_rpm  = mavlink_msg_rotor_frequency_get_estimated_accurancy_rpm (msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN? msg->len : MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN;
        memset(rotor_frequency, 0, MAVLINK_MSG_ID_ROTOR_FREQUENCY_LEN);
    memcpy(rotor_frequency, _MAV_PAYLOAD(msg), len);
#endif
}
