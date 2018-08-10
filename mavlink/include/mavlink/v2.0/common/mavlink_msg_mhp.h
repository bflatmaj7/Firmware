#pragma once
// MESSAGE MHP PACKING

#define MAVLINK_MSG_ID_MHP 335

MAVPACKED(
typedef struct __mavlink_mhp_t {
 uint64_t time_usec; /*< Timestamp (microseconds since system boot or since UNIX epoch).*/
 float dp0; /*< diff. pressure, port 0*/
 float dp1; /*< diff. pressure, port 1*/
 float dp2; /*< diff. pressure, port 2*/
 float dp3; /*< diff. pressure, port 3*/
 float dp4; /*< diff. pressure, port 4*/
 float dpS; /*< diff. pressure, static port*/
 float tas; /*< true airspeed*/
 float aoa; /*< angle of attack*/
 float aos; /*< angle of sideslip*/
}) mavlink_mhp_t;

#define MAVLINK_MSG_ID_MHP_LEN 44
#define MAVLINK_MSG_ID_MHP_MIN_LEN 44
#define MAVLINK_MSG_ID_335_LEN 44
#define MAVLINK_MSG_ID_335_MIN_LEN 44

#define MAVLINK_MSG_ID_MHP_CRC 193
#define MAVLINK_MSG_ID_335_CRC 193



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MHP { \
    335, \
    "MHP", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mhp_t, time_usec) }, \
         { "dp0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mhp_t, dp0) }, \
         { "dp1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mhp_t, dp1) }, \
         { "dp2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mhp_t, dp2) }, \
         { "dp3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mhp_t, dp3) }, \
         { "dp4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mhp_t, dp4) }, \
         { "dpS", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mhp_t, dpS) }, \
         { "tas", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_mhp_t, tas) }, \
         { "aoa", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_mhp_t, aoa) }, \
         { "aos", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_mhp_t, aos) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MHP { \
    "MHP", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mhp_t, time_usec) }, \
         { "dp0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mhp_t, dp0) }, \
         { "dp1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mhp_t, dp1) }, \
         { "dp2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mhp_t, dp2) }, \
         { "dp3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mhp_t, dp3) }, \
         { "dp4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mhp_t, dp4) }, \
         { "dpS", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mhp_t, dpS) }, \
         { "tas", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_mhp_t, tas) }, \
         { "aoa", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_mhp_t, aoa) }, \
         { "aos", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_mhp_t, aos) }, \
         } \
}
#endif

/**
 * @brief Pack a mhp message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since system boot or since UNIX epoch).
 * @param dp0 diff. pressure, port 0
 * @param dp1 diff. pressure, port 1
 * @param dp2 diff. pressure, port 2
 * @param dp3 diff. pressure, port 3
 * @param dp4 diff. pressure, port 4
 * @param dpS diff. pressure, static port
 * @param tas true airspeed
 * @param aoa angle of attack
 * @param aos angle of sideslip
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mhp_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float dp0, float dp1, float dp2, float dp3, float dp4, float dpS, float tas, float aoa, float aos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MHP_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, dp0);
    _mav_put_float(buf, 12, dp1);
    _mav_put_float(buf, 16, dp2);
    _mav_put_float(buf, 20, dp3);
    _mav_put_float(buf, 24, dp4);
    _mav_put_float(buf, 28, dpS);
    _mav_put_float(buf, 32, tas);
    _mav_put_float(buf, 36, aoa);
    _mav_put_float(buf, 40, aos);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MHP_LEN);
#else
    mavlink_mhp_t packet;
    packet.time_usec = time_usec;
    packet.dp0 = dp0;
    packet.dp1 = dp1;
    packet.dp2 = dp2;
    packet.dp3 = dp3;
    packet.dp4 = dp4;
    packet.dpS = dpS;
    packet.tas = tas;
    packet.aoa = aoa;
    packet.aos = aos;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MHP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MHP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MHP_MIN_LEN, MAVLINK_MSG_ID_MHP_LEN, MAVLINK_MSG_ID_MHP_CRC);
}

/**
 * @brief Pack a mhp message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since system boot or since UNIX epoch).
 * @param dp0 diff. pressure, port 0
 * @param dp1 diff. pressure, port 1
 * @param dp2 diff. pressure, port 2
 * @param dp3 diff. pressure, port 3
 * @param dp4 diff. pressure, port 4
 * @param dpS diff. pressure, static port
 * @param tas true airspeed
 * @param aoa angle of attack
 * @param aos angle of sideslip
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mhp_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float dp0,float dp1,float dp2,float dp3,float dp4,float dpS,float tas,float aoa,float aos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MHP_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, dp0);
    _mav_put_float(buf, 12, dp1);
    _mav_put_float(buf, 16, dp2);
    _mav_put_float(buf, 20, dp3);
    _mav_put_float(buf, 24, dp4);
    _mav_put_float(buf, 28, dpS);
    _mav_put_float(buf, 32, tas);
    _mav_put_float(buf, 36, aoa);
    _mav_put_float(buf, 40, aos);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MHP_LEN);
#else
    mavlink_mhp_t packet;
    packet.time_usec = time_usec;
    packet.dp0 = dp0;
    packet.dp1 = dp1;
    packet.dp2 = dp2;
    packet.dp3 = dp3;
    packet.dp4 = dp4;
    packet.dpS = dpS;
    packet.tas = tas;
    packet.aoa = aoa;
    packet.aos = aos;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MHP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MHP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MHP_MIN_LEN, MAVLINK_MSG_ID_MHP_LEN, MAVLINK_MSG_ID_MHP_CRC);
}

/**
 * @brief Encode a mhp struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mhp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mhp_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mhp_t* mhp)
{
    return mavlink_msg_mhp_pack(system_id, component_id, msg, mhp->time_usec, mhp->dp0, mhp->dp1, mhp->dp2, mhp->dp3, mhp->dp4, mhp->dpS, mhp->tas, mhp->aoa, mhp->aos);
}

/**
 * @brief Encode a mhp struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mhp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mhp_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mhp_t* mhp)
{
    return mavlink_msg_mhp_pack_chan(system_id, component_id, chan, msg, mhp->time_usec, mhp->dp0, mhp->dp1, mhp->dp2, mhp->dp3, mhp->dp4, mhp->dpS, mhp->tas, mhp->aoa, mhp->aos);
}

/**
 * @brief Send a mhp message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since system boot or since UNIX epoch).
 * @param dp0 diff. pressure, port 0
 * @param dp1 diff. pressure, port 1
 * @param dp2 diff. pressure, port 2
 * @param dp3 diff. pressure, port 3
 * @param dp4 diff. pressure, port 4
 * @param dpS diff. pressure, static port
 * @param tas true airspeed
 * @param aoa angle of attack
 * @param aos angle of sideslip
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mhp_send(mavlink_channel_t chan, uint64_t time_usec, float dp0, float dp1, float dp2, float dp3, float dp4, float dpS, float tas, float aoa, float aos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MHP_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, dp0);
    _mav_put_float(buf, 12, dp1);
    _mav_put_float(buf, 16, dp2);
    _mav_put_float(buf, 20, dp3);
    _mav_put_float(buf, 24, dp4);
    _mav_put_float(buf, 28, dpS);
    _mav_put_float(buf, 32, tas);
    _mav_put_float(buf, 36, aoa);
    _mav_put_float(buf, 40, aos);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MHP, buf, MAVLINK_MSG_ID_MHP_MIN_LEN, MAVLINK_MSG_ID_MHP_LEN, MAVLINK_MSG_ID_MHP_CRC);
#else
    mavlink_mhp_t packet;
    packet.time_usec = time_usec;
    packet.dp0 = dp0;
    packet.dp1 = dp1;
    packet.dp2 = dp2;
    packet.dp3 = dp3;
    packet.dp4 = dp4;
    packet.dpS = dpS;
    packet.tas = tas;
    packet.aoa = aoa;
    packet.aos = aos;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MHP, (const char *)&packet, MAVLINK_MSG_ID_MHP_MIN_LEN, MAVLINK_MSG_ID_MHP_LEN, MAVLINK_MSG_ID_MHP_CRC);
#endif
}

/**
 * @brief Send a mhp message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mhp_send_struct(mavlink_channel_t chan, const mavlink_mhp_t* mhp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mhp_send(chan, mhp->time_usec, mhp->dp0, mhp->dp1, mhp->dp2, mhp->dp3, mhp->dp4, mhp->dpS, mhp->tas, mhp->aoa, mhp->aos);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MHP, (const char *)mhp, MAVLINK_MSG_ID_MHP_MIN_LEN, MAVLINK_MSG_ID_MHP_LEN, MAVLINK_MSG_ID_MHP_CRC);
#endif
}

#if MAVLINK_MSG_ID_MHP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mhp_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float dp0, float dp1, float dp2, float dp3, float dp4, float dpS, float tas, float aoa, float aos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, dp0);
    _mav_put_float(buf, 12, dp1);
    _mav_put_float(buf, 16, dp2);
    _mav_put_float(buf, 20, dp3);
    _mav_put_float(buf, 24, dp4);
    _mav_put_float(buf, 28, dpS);
    _mav_put_float(buf, 32, tas);
    _mav_put_float(buf, 36, aoa);
    _mav_put_float(buf, 40, aos);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MHP, buf, MAVLINK_MSG_ID_MHP_MIN_LEN, MAVLINK_MSG_ID_MHP_LEN, MAVLINK_MSG_ID_MHP_CRC);
#else
    mavlink_mhp_t *packet = (mavlink_mhp_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->dp0 = dp0;
    packet->dp1 = dp1;
    packet->dp2 = dp2;
    packet->dp3 = dp3;
    packet->dp4 = dp4;
    packet->dpS = dpS;
    packet->tas = tas;
    packet->aoa = aoa;
    packet->aos = aos;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MHP, (const char *)packet, MAVLINK_MSG_ID_MHP_MIN_LEN, MAVLINK_MSG_ID_MHP_LEN, MAVLINK_MSG_ID_MHP_CRC);
#endif
}
#endif

#endif

// MESSAGE MHP UNPACKING


/**
 * @brief Get field time_usec from mhp message
 *
 * @return Timestamp (microseconds since system boot or since UNIX epoch).
 */
static inline uint64_t mavlink_msg_mhp_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field dp0 from mhp message
 *
 * @return diff. pressure, port 0
 */
static inline float mavlink_msg_mhp_get_dp0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field dp1 from mhp message
 *
 * @return diff. pressure, port 1
 */
static inline float mavlink_msg_mhp_get_dp1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field dp2 from mhp message
 *
 * @return diff. pressure, port 2
 */
static inline float mavlink_msg_mhp_get_dp2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field dp3 from mhp message
 *
 * @return diff. pressure, port 3
 */
static inline float mavlink_msg_mhp_get_dp3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field dp4 from mhp message
 *
 * @return diff. pressure, port 4
 */
static inline float mavlink_msg_mhp_get_dp4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field dpS from mhp message
 *
 * @return diff. pressure, static port
 */
static inline float mavlink_msg_mhp_get_dpS(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field tas from mhp message
 *
 * @return true airspeed
 */
static inline float mavlink_msg_mhp_get_tas(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field aoa from mhp message
 *
 * @return angle of attack
 */
static inline float mavlink_msg_mhp_get_aoa(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field aos from mhp message
 *
 * @return angle of sideslip
 */
static inline float mavlink_msg_mhp_get_aos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a mhp message into a struct
 *
 * @param msg The message to decode
 * @param mhp C-struct to decode the message contents into
 */
static inline void mavlink_msg_mhp_decode(const mavlink_message_t* msg, mavlink_mhp_t* mhp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mhp->time_usec = mavlink_msg_mhp_get_time_usec(msg);
    mhp->dp0 = mavlink_msg_mhp_get_dp0(msg);
    mhp->dp1 = mavlink_msg_mhp_get_dp1(msg);
    mhp->dp2 = mavlink_msg_mhp_get_dp2(msg);
    mhp->dp3 = mavlink_msg_mhp_get_dp3(msg);
    mhp->dp4 = mavlink_msg_mhp_get_dp4(msg);
    mhp->dpS = mavlink_msg_mhp_get_dpS(msg);
    mhp->tas = mavlink_msg_mhp_get_tas(msg);
    mhp->aoa = mavlink_msg_mhp_get_aoa(msg);
    mhp->aos = mavlink_msg_mhp_get_aos(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MHP_LEN? msg->len : MAVLINK_MSG_ID_MHP_LEN;
        memset(mhp, 0, MAVLINK_MSG_ID_MHP_LEN);
    memcpy(mhp, _MAV_PAYLOAD(msg), len);
#endif
}
