#pragma once
// MESSAGE INS PACKING

#define MAVLINK_MSG_ID_INS 334

MAVPACKED(
typedef struct __mavlink_ins_t {
 uint32_t time_usec; /*< [us] Timestamp (microseconds since system boot or since UNIX epoch).*/
 float ax; /*<  Acceleration in x-axis*/
 float ay; /*<  Acceleration in y-axis*/
 float az; /*<  Acceleration in z-axis*/
 float gx; /*<  Angular rate in x-axis*/
 float gy; /*<  Angular rate in y-axis*/
 float gz; /*<  Angular rate in z-axis*/
 float mx; /*<  Magnetic field in x-axis*/
 float my; /*<  Magnetic field in y-axis*/
 float mz; /*<  Magnetic field in z-axis*/
 float roll; /*<  Roll angle*/
 float pitch; /*<  Pitch angle*/
 float yaw; /*<  Yaw angle*/
 float vns; /*<  North-South velocity*/
 float vew; /*<  East-West velocity*/
 float vud; /*<  Up-Down velocity*/
}) mavlink_ins_t;

#define MAVLINK_MSG_ID_INS_LEN 64
#define MAVLINK_MSG_ID_INS_MIN_LEN 64
#define MAVLINK_MSG_ID_334_LEN 64
#define MAVLINK_MSG_ID_334_MIN_LEN 64

#define MAVLINK_MSG_ID_INS_CRC 122
#define MAVLINK_MSG_ID_334_CRC 122



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INS { \
    334, \
    "INS", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ins_t, time_usec) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ins_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ins_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ins_t, az) }, \
         { "gx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ins_t, gx) }, \
         { "gy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ins_t, gy) }, \
         { "gz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ins_t, gz) }, \
         { "mx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ins_t, mx) }, \
         { "my", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ins_t, my) }, \
         { "mz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ins_t, mz) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ins_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_ins_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_ins_t, yaw) }, \
         { "vns", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_ins_t, vns) }, \
         { "vew", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_ins_t, vew) }, \
         { "vud", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_ins_t, vud) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INS { \
    "INS", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ins_t, time_usec) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ins_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ins_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ins_t, az) }, \
         { "gx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ins_t, gx) }, \
         { "gy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ins_t, gy) }, \
         { "gz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ins_t, gz) }, \
         { "mx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ins_t, mx) }, \
         { "my", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ins_t, my) }, \
         { "mz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ins_t, mz) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ins_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_ins_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_ins_t, yaw) }, \
         { "vns", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_ins_t, vns) }, \
         { "vew", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_ins_t, vew) }, \
         { "vud", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_ins_t, vud) }, \
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
 * @param ax  Acceleration in x-axis
 * @param ay  Acceleration in y-axis
 * @param az  Acceleration in z-axis
 * @param gx  Angular rate in x-axis
 * @param gy  Angular rate in y-axis
 * @param gz  Angular rate in z-axis
 * @param mx  Magnetic field in x-axis
 * @param my  Magnetic field in y-axis
 * @param mz  Magnetic field in z-axis
 * @param roll  Roll angle
 * @param pitch  Pitch angle
 * @param yaw  Yaw angle
 * @param vns  North-South velocity
 * @param vew  East-West velocity
 * @param vud  Up-Down velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ins_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_usec, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float roll, float pitch, float yaw, float vns, float vew, float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INS_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, ax);
    _mav_put_float(buf, 8, ay);
    _mav_put_float(buf, 12, az);
    _mav_put_float(buf, 16, gx);
    _mav_put_float(buf, 20, gy);
    _mav_put_float(buf, 24, gz);
    _mav_put_float(buf, 28, mx);
    _mav_put_float(buf, 32, my);
    _mav_put_float(buf, 36, mz);
    _mav_put_float(buf, 40, roll);
    _mav_put_float(buf, 44, pitch);
    _mav_put_float(buf, 48, yaw);
    _mav_put_float(buf, 52, vns);
    _mav_put_float(buf, 56, vew);
    _mav_put_float(buf, 60, vud);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INS_LEN);
#else
    mavlink_ins_t packet;
    packet.time_usec = time_usec;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;
    packet.mx = mx;
    packet.my = my;
    packet.mz = mz;
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
 * @param ax  Acceleration in x-axis
 * @param ay  Acceleration in y-axis
 * @param az  Acceleration in z-axis
 * @param gx  Angular rate in x-axis
 * @param gy  Angular rate in y-axis
 * @param gz  Angular rate in z-axis
 * @param mx  Magnetic field in x-axis
 * @param my  Magnetic field in y-axis
 * @param mz  Magnetic field in z-axis
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
                                   uint32_t time_usec,float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,float roll,float pitch,float yaw,float vns,float vew,float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INS_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, ax);
    _mav_put_float(buf, 8, ay);
    _mav_put_float(buf, 12, az);
    _mav_put_float(buf, 16, gx);
    _mav_put_float(buf, 20, gy);
    _mav_put_float(buf, 24, gz);
    _mav_put_float(buf, 28, mx);
    _mav_put_float(buf, 32, my);
    _mav_put_float(buf, 36, mz);
    _mav_put_float(buf, 40, roll);
    _mav_put_float(buf, 44, pitch);
    _mav_put_float(buf, 48, yaw);
    _mav_put_float(buf, 52, vns);
    _mav_put_float(buf, 56, vew);
    _mav_put_float(buf, 60, vud);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INS_LEN);
#else
    mavlink_ins_t packet;
    packet.time_usec = time_usec;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;
    packet.mx = mx;
    packet.my = my;
    packet.mz = mz;
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
    return mavlink_msg_ins_pack(system_id, component_id, msg, ins->time_usec, ins->ax, ins->ay, ins->az, ins->gx, ins->gy, ins->gz, ins->mx, ins->my, ins->mz, ins->roll, ins->pitch, ins->yaw, ins->vns, ins->vew, ins->vud);
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
    return mavlink_msg_ins_pack_chan(system_id, component_id, chan, msg, ins->time_usec, ins->ax, ins->ay, ins->az, ins->gx, ins->gy, ins->gz, ins->mx, ins->my, ins->mz, ins->roll, ins->pitch, ins->yaw, ins->vns, ins->vew, ins->vud);
}

/**
 * @brief Send a ins message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (microseconds since system boot or since UNIX epoch).
 * @param ax  Acceleration in x-axis
 * @param ay  Acceleration in y-axis
 * @param az  Acceleration in z-axis
 * @param gx  Angular rate in x-axis
 * @param gy  Angular rate in y-axis
 * @param gz  Angular rate in z-axis
 * @param mx  Magnetic field in x-axis
 * @param my  Magnetic field in y-axis
 * @param mz  Magnetic field in z-axis
 * @param roll  Roll angle
 * @param pitch  Pitch angle
 * @param yaw  Yaw angle
 * @param vns  North-South velocity
 * @param vew  East-West velocity
 * @param vud  Up-Down velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ins_send(mavlink_channel_t chan, uint32_t time_usec, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float roll, float pitch, float yaw, float vns, float vew, float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INS_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, ax);
    _mav_put_float(buf, 8, ay);
    _mav_put_float(buf, 12, az);
    _mav_put_float(buf, 16, gx);
    _mav_put_float(buf, 20, gy);
    _mav_put_float(buf, 24, gz);
    _mav_put_float(buf, 28, mx);
    _mav_put_float(buf, 32, my);
    _mav_put_float(buf, 36, mz);
    _mav_put_float(buf, 40, roll);
    _mav_put_float(buf, 44, pitch);
    _mav_put_float(buf, 48, yaw);
    _mav_put_float(buf, 52, vns);
    _mav_put_float(buf, 56, vew);
    _mav_put_float(buf, 60, vud);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INS, buf, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
#else
    mavlink_ins_t packet;
    packet.time_usec = time_usec;
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;
    packet.mx = mx;
    packet.my = my;
    packet.mz = mz;
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
    mavlink_msg_ins_send(chan, ins->time_usec, ins->ax, ins->ay, ins->az, ins->gx, ins->gy, ins->gz, ins->mx, ins->my, ins->mz, ins->roll, ins->pitch, ins->yaw, ins->vns, ins->vew, ins->vud);
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
static inline void mavlink_msg_ins_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_usec, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float roll, float pitch, float yaw, float vns, float vew, float vud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, ax);
    _mav_put_float(buf, 8, ay);
    _mav_put_float(buf, 12, az);
    _mav_put_float(buf, 16, gx);
    _mav_put_float(buf, 20, gy);
    _mav_put_float(buf, 24, gz);
    _mav_put_float(buf, 28, mx);
    _mav_put_float(buf, 32, my);
    _mav_put_float(buf, 36, mz);
    _mav_put_float(buf, 40, roll);
    _mav_put_float(buf, 44, pitch);
    _mav_put_float(buf, 48, yaw);
    _mav_put_float(buf, 52, vns);
    _mav_put_float(buf, 56, vew);
    _mav_put_float(buf, 60, vud);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INS, buf, MAVLINK_MSG_ID_INS_MIN_LEN, MAVLINK_MSG_ID_INS_LEN, MAVLINK_MSG_ID_INS_CRC);
#else
    mavlink_ins_t *packet = (mavlink_ins_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->ax = ax;
    packet->ay = ay;
    packet->az = az;
    packet->gx = gx;
    packet->gy = gy;
    packet->gz = gz;
    packet->mx = mx;
    packet->my = my;
    packet->mz = mz;
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
static inline uint32_t mavlink_msg_ins_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ax from ins message
 *
 * @return  Acceleration in x-axis
 */
static inline float mavlink_msg_ins_get_ax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field ay from ins message
 *
 * @return  Acceleration in y-axis
 */
static inline float mavlink_msg_ins_get_ay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field az from ins message
 *
 * @return  Acceleration in z-axis
 */
static inline float mavlink_msg_ins_get_az(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gx from ins message
 *
 * @return  Angular rate in x-axis
 */
static inline float mavlink_msg_ins_get_gx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gy from ins message
 *
 * @return  Angular rate in y-axis
 */
static inline float mavlink_msg_ins_get_gy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gz from ins message
 *
 * @return  Angular rate in z-axis
 */
static inline float mavlink_msg_ins_get_gz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field mx from ins message
 *
 * @return  Magnetic field in x-axis
 */
static inline float mavlink_msg_ins_get_mx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field my from ins message
 *
 * @return  Magnetic field in y-axis
 */
static inline float mavlink_msg_ins_get_my(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field mz from ins message
 *
 * @return  Magnetic field in z-axis
 */
static inline float mavlink_msg_ins_get_mz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field roll from ins message
 *
 * @return  Roll angle
 */
static inline float mavlink_msg_ins_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field pitch from ins message
 *
 * @return  Pitch angle
 */
static inline float mavlink_msg_ins_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field yaw from ins message
 *
 * @return  Yaw angle
 */
static inline float mavlink_msg_ins_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field vns from ins message
 *
 * @return  North-South velocity
 */
static inline float mavlink_msg_ins_get_vns(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field vew from ins message
 *
 * @return  East-West velocity
 */
static inline float mavlink_msg_ins_get_vew(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field vud from ins message
 *
 * @return  Up-Down velocity
 */
static inline float mavlink_msg_ins_get_vud(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
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
    ins->ax = mavlink_msg_ins_get_ax(msg);
    ins->ay = mavlink_msg_ins_get_ay(msg);
    ins->az = mavlink_msg_ins_get_az(msg);
    ins->gx = mavlink_msg_ins_get_gx(msg);
    ins->gy = mavlink_msg_ins_get_gy(msg);
    ins->gz = mavlink_msg_ins_get_gz(msg);
    ins->mx = mavlink_msg_ins_get_mx(msg);
    ins->my = mavlink_msg_ins_get_my(msg);
    ins->mz = mavlink_msg_ins_get_mz(msg);
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
