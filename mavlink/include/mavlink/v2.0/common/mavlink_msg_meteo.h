#pragma once
// MESSAGE METEO PACKING

#define MAVLINK_MSG_ID_METEO 333

MAVPACKED(
typedef struct __mavlink_meteo_t {
 uint64_t time_usec; /*< [us] Timestamp (microseconds since system boot or since UNIX epoch).*/
 float temperature; /*<  Temperature reading*/
 float humidity; /*<  Humidity reading*/
}) mavlink_meteo_t;

#define MAVLINK_MSG_ID_METEO_LEN 16
#define MAVLINK_MSG_ID_METEO_MIN_LEN 16
#define MAVLINK_MSG_ID_333_LEN 16
#define MAVLINK_MSG_ID_333_MIN_LEN 16

#define MAVLINK_MSG_ID_METEO_CRC 119
#define MAVLINK_MSG_ID_333_CRC 119



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_METEO { \
    333, \
    "METEO", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_meteo_t, time_usec) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_meteo_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_meteo_t, humidity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_METEO { \
    "METEO", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_meteo_t, time_usec) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_meteo_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_meteo_t, humidity) }, \
         } \
}
#endif

/**
 * @brief Pack a meteo message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (microseconds since system boot or since UNIX epoch).
 * @param temperature  Temperature reading
 * @param humidity  Humidity reading
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_meteo_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float temperature, float humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_METEO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, temperature);
    _mav_put_float(buf, 12, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_METEO_LEN);
#else
    mavlink_meteo_t packet;
    packet.time_usec = time_usec;
    packet.temperature = temperature;
    packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_METEO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_METEO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_METEO_MIN_LEN, MAVLINK_MSG_ID_METEO_LEN, MAVLINK_MSG_ID_METEO_CRC);
}

/**
 * @brief Pack a meteo message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (microseconds since system boot or since UNIX epoch).
 * @param temperature  Temperature reading
 * @param humidity  Humidity reading
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_meteo_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float temperature,float humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_METEO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, temperature);
    _mav_put_float(buf, 12, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_METEO_LEN);
#else
    mavlink_meteo_t packet;
    packet.time_usec = time_usec;
    packet.temperature = temperature;
    packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_METEO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_METEO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_METEO_MIN_LEN, MAVLINK_MSG_ID_METEO_LEN, MAVLINK_MSG_ID_METEO_CRC);
}

/**
 * @brief Encode a meteo struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param meteo C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_meteo_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_meteo_t* meteo)
{
    return mavlink_msg_meteo_pack(system_id, component_id, msg, meteo->time_usec, meteo->temperature, meteo->humidity);
}

/**
 * @brief Encode a meteo struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param meteo C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_meteo_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_meteo_t* meteo)
{
    return mavlink_msg_meteo_pack_chan(system_id, component_id, chan, msg, meteo->time_usec, meteo->temperature, meteo->humidity);
}

/**
 * @brief Send a meteo message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (microseconds since system boot or since UNIX epoch).
 * @param temperature  Temperature reading
 * @param humidity  Humidity reading
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_meteo_send(mavlink_channel_t chan, uint64_t time_usec, float temperature, float humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_METEO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, temperature);
    _mav_put_float(buf, 12, humidity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_METEO, buf, MAVLINK_MSG_ID_METEO_MIN_LEN, MAVLINK_MSG_ID_METEO_LEN, MAVLINK_MSG_ID_METEO_CRC);
#else
    mavlink_meteo_t packet;
    packet.time_usec = time_usec;
    packet.temperature = temperature;
    packet.humidity = humidity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_METEO, (const char *)&packet, MAVLINK_MSG_ID_METEO_MIN_LEN, MAVLINK_MSG_ID_METEO_LEN, MAVLINK_MSG_ID_METEO_CRC);
#endif
}

/**
 * @brief Send a meteo message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_meteo_send_struct(mavlink_channel_t chan, const mavlink_meteo_t* meteo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_meteo_send(chan, meteo->time_usec, meteo->temperature, meteo->humidity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_METEO, (const char *)meteo, MAVLINK_MSG_ID_METEO_MIN_LEN, MAVLINK_MSG_ID_METEO_LEN, MAVLINK_MSG_ID_METEO_CRC);
#endif
}

#if MAVLINK_MSG_ID_METEO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_meteo_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float temperature, float humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, temperature);
    _mav_put_float(buf, 12, humidity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_METEO, buf, MAVLINK_MSG_ID_METEO_MIN_LEN, MAVLINK_MSG_ID_METEO_LEN, MAVLINK_MSG_ID_METEO_CRC);
#else
    mavlink_meteo_t *packet = (mavlink_meteo_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->temperature = temperature;
    packet->humidity = humidity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_METEO, (const char *)packet, MAVLINK_MSG_ID_METEO_MIN_LEN, MAVLINK_MSG_ID_METEO_LEN, MAVLINK_MSG_ID_METEO_CRC);
#endif
}
#endif

#endif

// MESSAGE METEO UNPACKING


/**
 * @brief Get field time_usec from meteo message
 *
 * @return [us] Timestamp (microseconds since system boot or since UNIX epoch).
 */
static inline uint64_t mavlink_msg_meteo_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field temperature from meteo message
 *
 * @return  Temperature reading
 */
static inline float mavlink_msg_meteo_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field humidity from meteo message
 *
 * @return  Humidity reading
 */
static inline float mavlink_msg_meteo_get_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a meteo message into a struct
 *
 * @param msg The message to decode
 * @param meteo C-struct to decode the message contents into
 */
static inline void mavlink_msg_meteo_decode(const mavlink_message_t* msg, mavlink_meteo_t* meteo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    meteo->time_usec = mavlink_msg_meteo_get_time_usec(msg);
    meteo->temperature = mavlink_msg_meteo_get_temperature(msg);
    meteo->humidity = mavlink_msg_meteo_get_humidity(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_METEO_LEN? msg->len : MAVLINK_MSG_ID_METEO_LEN;
        memset(meteo, 0, MAVLINK_MSG_ID_METEO_LEN);
    memcpy(meteo, _MAV_PAYLOAD(msg), len);
#endif
}
