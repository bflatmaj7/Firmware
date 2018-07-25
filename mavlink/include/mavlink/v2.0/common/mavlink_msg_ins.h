#pragma once
// MESSAGE INS PACKING

#define MAVLINK_MSG_ID_INS 334

MAVPACKED(
typedef struct __mavlink_ins_t {
 uint64_t time_usec; /*< [us] Timestamp (microseconds since system boot or since UNIX epoch).*/
 float roll; /*<  Roll angle*/
 float pitch; /*<  Pitch angle*/
 float yaw; /*<  Yaw angle*/
 float vns; /*<  North-South velocity*/
 float vew; /*<  East-West velocity*/
 float vud; /*<  Up-Down velocity*/
}) mavlink_ins_t;

#define MAVLINK_MSG_ID_INS_LEN 32
#define MAVLINK_MSG_ID_INS_MIN_LEN 32
#define MAVLINK_MSG_ID_334_LEN 32
#define MAVLINK_MSG_ID_334_MIN_LEN 32

#define MAVLINK_MSG_ID_INS_CRC 127
#define MAVLINK_MSG_ID_334_CRC 127



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INS { \
    334, \
    "INS", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ins_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ins_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ins_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ins_t, yaw) }, \
         { "vns", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ins_t, vns) }, \
         { "vew", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ins_t, vew) }, \
         { "vud", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ins_t, vud) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INS { \
    "INS", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ins_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ins_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ins_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ins_t, yaw) }, \
         { "vns", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ins_t, vns) }, \
         { "vew", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ins_t, vew) }, \
         { "vud", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ins_t, vud) }, \
         } \
}
#endif

/**
 * @brief Pack a ins message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (microseconds since system boot or since UNIX epoch).
 * @param roll  Roll angle
 * @param pitch  Pitch angle
 * @param yaw  Yaw angle
 * @param vns  North-South velocity
 * @param vew  East-West velocity
 * @param vud  Up-Down velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ins_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float roll, float pitch, float yaw, float vns, float vew, float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, vns);
    _mav_put_float(buf, 24, vew);
    _mav_put_float(buf, 28, vud);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INS_LEN);
#else
    mavlink_ins_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.vns = vns;
    packet.vew = vew;
    packet.vud = vud;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
}

/**
 * @brief Pack a ins message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (microseconds since system boot or since UNIX epoch).
 * @param roll  Roll angle
 * @param pitch  Pitch angle
 * @param yaw  Yaw angle
 * @param vns  North-South velocity
 * @param vew  East-West velocity
 * @param vud  Up-Down velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ins_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float roll,float pitch,float yaw,float vns,float vew,float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, vns);
    _mav_put_float(buf, 24, vew);
    _mav_put_float(buf, 28, vud);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INS_LEN);
#else
    mavlink_ins_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.vns = vns;
    packet.vew = vew;
    packet.vud = vud;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
}

/**
 * @brief Encode a ins struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ins C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ins_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ins_t* ins)
{
    return mavlink_msg_ins_pack(system_id, component_id, msg, ins->time_usec, ins->roll, ins->pitch, ins->yaw, ins->vns, ins->vew, ins->vud);
}

/**
 * @brief Encode a ins struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ins C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ins_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ins_t* ins)
{
    return mavlink_msg_ins_pack_chan(system_id, component_id, chan, msg, ins->time_usec, ins->roll, ins->pitch, ins->yaw, ins->vns, ins->vew, ins->vud);
}

/**
 * @brief Send a ins message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (microseconds since system boot or since UNIX epoch).
 * @param roll  Roll angle
 * @param pitch  Pitch angle
 * @param yaw  Yaw angle
 * @param vns  North-South velocity
 * @param vew  East-West velocity
 * @param vud  Up-Down velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ins_send(mavlink_channel_t chan, uint64_t time_usec, float roll, float pitch, float yaw, float vns, float vew, float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, vns);
    _mav_put_float(buf, 24, vew);
    _mav_put_float(buf, 28, vud);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INS, buf, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
#else
    mavlink_ins_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.vns = vns;
    packet.vew = vew;
    packet.vud = vud;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INS, (const char *)&packet, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
#endif
}

/**
 * @brief Send a ins message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ins_send_struct(mavlink_channel_t chan, const mavlink_ins_t* ins)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ins_send(chan, ins->time_usec, ins->roll, ins->pitch, ins->yaw, ins->vns, ins->vew, ins->vud);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INS, (const char *)ins, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
#endif
}

#if MAVLINK_MSG_ID_INS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ins_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float roll, float pitch, float yaw, float vns, float vew, float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, vns);
    _mav_put_float(buf, 24, vew);
    _mav_put_float(buf, 28, vud);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INS, buf, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
#else
    mavlink_ins_t *packet = (mavlink_ins_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->vns = vns;
    packet->vew = vew;
    packet->vud = vud;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INS, (const char *)packet, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
#endif
}
#endif

#endif

// MESSAGE INS UNPACKING


/**
 * @brief Get field time_usec from ins message
 *
 * @return [us] Timestamp (microseconds since system boot or since UNIX epoch).
 */
static inline uint64_t mavlink_msg_ins_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field roll from ins message
 *
 * @return  Roll angle
 */
static inline float mavlink_msg_ins_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from ins message
 *
 * @return  Pitch angle
 */
static inline float mavlink_msg_ins_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw from ins message
 *
 * @return  Yaw angle
 */
static inline float mavlink_msg_ins_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vns from ins message
 *
 * @return  North-South velocity
 */
static inline float mavlink_msg_ins_get_vns(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vew from ins message
 *
 * @return  East-West velocity
 */
static inline float mavlink_msg_ins_get_vew(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vud from ins message
 *
 * @return  Up-Down velocity
 */
static inline float mavlink_msg_ins_get_vud(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a ins message into a struct
 *
 * @param msg The message to decode
 * @param ins C-struct to decode the message contents into
 */
static inline void mavlink_msg_ins_decode(const mavlink_message_t* msg, mavlink_ins_t* ins)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ins->time_usec = mavlink_msg_ins_get_time_usec(msg);
    ins->roll = mavlink_msg_ins_get_roll(msg);
    ins->pitch = mavlink_msg_ins_get_pitch(msg);
    ins->yaw = mavlink_msg_ins_get_yaw(msg);
    ins->vns = mavlink_msg_ins_get_vns(msg);
    ins->vew = mavlink_msg_ins_get_vew(msg);
    ins->vud = mavlink_msg_ins_get_vud(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INS_LEN? msg->len : MAVLINK_MSG_ID_INS_LEN;
        memset(ins, 0, MAVLINK_MSG_ID_INS_LEN);
    memcpy(ins, _MAV_PAYLOAD(msg), len);
#endif
}
