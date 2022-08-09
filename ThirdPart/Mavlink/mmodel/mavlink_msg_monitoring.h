#pragma once
// MESSAGE MONITORING PACKING

#define MAVLINK_MSG_ID_MONITORING 154

MAVPACKED(
typedef struct __mavlink_monitoring_t {
 uint32_t tow; /*<  Time Of Week*/
 float pos_x; /*<  current position x  */
 float pos_y; /*<  current position y  */
 float pos_z; /*<  current position z  */
 float head; /*<   current heading  */
 uint32_t status1; /*<  status #1  */
 uint32_t status2; /*<  status #2 (RESERVED)  */
 uint8_t rtk_nbase; /*<  the number of base satellite */
 uint8_t rtk_nrover; /*<  the number of rover satellite  */
 uint8_t battery; /*<  Battery (%) */
}) mavlink_monitoring_t;

#define MAVLINK_MSG_ID_MONITORING_LEN 31
#define MAVLINK_MSG_ID_MONITORING_MIN_LEN 31
#define MAVLINK_MSG_ID_154_LEN 31
#define MAVLINK_MSG_ID_154_MIN_LEN 31

#define MAVLINK_MSG_ID_MONITORING_CRC 196
#define MAVLINK_MSG_ID_154_CRC 196



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MONITORING { \
    154, \
    "MONITORING", \
    10, \
    {  { "tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_monitoring_t, tow) }, \
         { "rtk_nbase", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_monitoring_t, rtk_nbase) }, \
         { "rtk_nrover", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_monitoring_t, rtk_nrover) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_monitoring_t, battery) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_monitoring_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_monitoring_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_monitoring_t, pos_z) }, \
         { "head", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_monitoring_t, head) }, \
         { "status1", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_monitoring_t, status1) }, \
         { "status2", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_monitoring_t, status2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MONITORING { \
    "MONITORING", \
    10, \
    {  { "tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_monitoring_t, tow) }, \
         { "rtk_nbase", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_monitoring_t, rtk_nbase) }, \
         { "rtk_nrover", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_monitoring_t, rtk_nrover) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_monitoring_t, battery) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_monitoring_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_monitoring_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_monitoring_t, pos_z) }, \
         { "head", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_monitoring_t, head) }, \
         { "status1", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_monitoring_t, status1) }, \
         { "status2", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_monitoring_t, status2) }, \
         } \
}
#endif

/**
 * @brief Pack a monitoring message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tow  Time Of Week
 * @param rtk_nbase  the number of base satellite 
 * @param rtk_nrover  the number of rover satellite  
 * @param battery  Battery (%) 
 * @param pos_x  current position x  
 * @param pos_y  current position y  
 * @param pos_z  current position z  
 * @param head   current heading  
 * @param status1  status #1  
 * @param status2  status #2 (RESERVED)  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_monitoring_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t tow, uint8_t rtk_nbase, uint8_t rtk_nrover, uint8_t battery, float pos_x, float pos_y, float pos_z, float head, uint32_t status1, uint32_t status2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint32_t(buf, 20, status1);
    _mav_put_uint32_t(buf, 24, status2);
    _mav_put_uint8_t(buf, 28, rtk_nbase);
    _mav_put_uint8_t(buf, 29, rtk_nrover);
    _mav_put_uint8_t(buf, 30, battery);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MONITORING_LEN);
#else
    mavlink_monitoring_t packet;
    packet.tow = tow;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.status1 = status1;
    packet.status2 = status2;
    packet.rtk_nbase = rtk_nbase;
    packet.rtk_nrover = rtk_nrover;
    packet.battery = battery;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MONITORING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MONITORING;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
}

/**
 * @brief Pack a monitoring message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tow  Time Of Week
 * @param rtk_nbase  the number of base satellite 
 * @param rtk_nrover  the number of rover satellite  
 * @param battery  Battery (%) 
 * @param pos_x  current position x  
 * @param pos_y  current position y  
 * @param pos_z  current position z  
 * @param head   current heading  
 * @param status1  status #1  
 * @param status2  status #2 (RESERVED)  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_monitoring_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t tow,uint8_t rtk_nbase,uint8_t rtk_nrover,uint8_t battery,float pos_x,float pos_y,float pos_z,float head,uint32_t status1,uint32_t status2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint32_t(buf, 20, status1);
    _mav_put_uint32_t(buf, 24, status2);
    _mav_put_uint8_t(buf, 28, rtk_nbase);
    _mav_put_uint8_t(buf, 29, rtk_nrover);
    _mav_put_uint8_t(buf, 30, battery);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MONITORING_LEN);
#else
    mavlink_monitoring_t packet;
    packet.tow = tow;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.status1 = status1;
    packet.status2 = status2;
    packet.rtk_nbase = rtk_nbase;
    packet.rtk_nrover = rtk_nrover;
    packet.battery = battery;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MONITORING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MONITORING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
}

/**
 * @brief Encode a monitoring struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param monitoring C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_monitoring_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_monitoring_t* monitoring)
{
    return mavlink_msg_monitoring_pack(system_id, component_id, msg, monitoring->tow, monitoring->rtk_nbase, monitoring->rtk_nrover, monitoring->battery, monitoring->pos_x, monitoring->pos_y, monitoring->pos_z, monitoring->head, monitoring->status1, monitoring->status2);
}

/**
 * @brief Encode a monitoring struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param monitoring C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_monitoring_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_monitoring_t* monitoring)
{
    return mavlink_msg_monitoring_pack_chan(system_id, component_id, chan, msg, monitoring->tow, monitoring->rtk_nbase, monitoring->rtk_nrover, monitoring->battery, monitoring->pos_x, monitoring->pos_y, monitoring->pos_z, monitoring->head, monitoring->status1, monitoring->status2);
}

/**
 * @brief Send a monitoring message
 * @param chan MAVLink channel to send the message
 *
 * @param tow  Time Of Week
 * @param rtk_nbase  the number of base satellite 
 * @param rtk_nrover  the number of rover satellite  
 * @param battery  Battery (%) 
 * @param pos_x  current position x  
 * @param pos_y  current position y  
 * @param pos_z  current position z  
 * @param head   current heading  
 * @param status1  status #1  
 * @param status2  status #2 (RESERVED)  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_monitoring_send(mavlink_channel_t chan, uint32_t tow, uint8_t rtk_nbase, uint8_t rtk_nrover, uint8_t battery, float pos_x, float pos_y, float pos_z, float head, uint32_t status1, uint32_t status2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint32_t(buf, 20, status1);
    _mav_put_uint32_t(buf, 24, status2);
    _mav_put_uint8_t(buf, 28, rtk_nbase);
    _mav_put_uint8_t(buf, 29, rtk_nrover);
    _mav_put_uint8_t(buf, 30, battery);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, buf, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#else
    mavlink_monitoring_t packet;
    packet.tow = tow;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.status1 = status1;
    packet.status2 = status2;
    packet.rtk_nbase = rtk_nbase;
    packet.rtk_nrover = rtk_nrover;
    packet.battery = battery;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)&packet, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}

/**
 * @brief Send a monitoring message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_monitoring_send_struct(mavlink_channel_t chan, const mavlink_monitoring_t* monitoring)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_monitoring_send(chan, monitoring->tow, monitoring->rtk_nbase, monitoring->rtk_nrover, monitoring->battery, monitoring->pos_x, monitoring->pos_y, monitoring->pos_z, monitoring->head, monitoring->status1, monitoring->status2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)monitoring, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}

#if MAVLINK_MSG_ID_MONITORING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_monitoring_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t tow, uint8_t rtk_nbase, uint8_t rtk_nrover, uint8_t battery, float pos_x, float pos_y, float pos_z, float head, uint32_t status1, uint32_t status2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint32_t(buf, 20, status1);
    _mav_put_uint32_t(buf, 24, status2);
    _mav_put_uint8_t(buf, 28, rtk_nbase);
    _mav_put_uint8_t(buf, 29, rtk_nrover);
    _mav_put_uint8_t(buf, 30, battery);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, buf, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#else
    mavlink_monitoring_t *packet = (mavlink_monitoring_t *)msgbuf;
    packet->tow = tow;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->pos_z = pos_z;
    packet->head = head;
    packet->status1 = status1;
    packet->status2 = status2;
    packet->rtk_nbase = rtk_nbase;
    packet->rtk_nrover = rtk_nrover;
    packet->battery = battery;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)packet, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}
#endif

#endif

// MESSAGE MONITORING UNPACKING


/**
 * @brief Get field tow from monitoring message
 *
 * @return  Time Of Week
 */
static inline uint32_t mavlink_msg_monitoring_get_tow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field rtk_nbase from monitoring message
 *
 * @return  the number of base satellite 
 */
static inline uint8_t mavlink_msg_monitoring_get_rtk_nbase(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field rtk_nrover from monitoring message
 *
 * @return  the number of rover satellite  
 */
static inline uint8_t mavlink_msg_monitoring_get_rtk_nrover(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field battery from monitoring message
 *
 * @return  Battery (%) 
 */
static inline uint8_t mavlink_msg_monitoring_get_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field pos_x from monitoring message
 *
 * @return  current position x  
 */
static inline float mavlink_msg_monitoring_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pos_y from monitoring message
 *
 * @return  current position y  
 */
static inline float mavlink_msg_monitoring_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pos_z from monitoring message
 *
 * @return  current position z  
 */
static inline float mavlink_msg_monitoring_get_pos_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field head from monitoring message
 *
 * @return   current heading  
 */
static inline float mavlink_msg_monitoring_get_head(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field status1 from monitoring message
 *
 * @return  status #1  
 */
static inline uint32_t mavlink_msg_monitoring_get_status1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field status2 from monitoring message
 *
 * @return  status #2 (RESERVED)  
 */
static inline uint32_t mavlink_msg_monitoring_get_status2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Decode a monitoring message into a struct
 *
 * @param msg The message to decode
 * @param monitoring C-struct to decode the message contents into
 */
static inline void mavlink_msg_monitoring_decode(const mavlink_message_t* msg, mavlink_monitoring_t* monitoring)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    monitoring->tow = mavlink_msg_monitoring_get_tow(msg);
    monitoring->pos_x = mavlink_msg_monitoring_get_pos_x(msg);
    monitoring->pos_y = mavlink_msg_monitoring_get_pos_y(msg);
    monitoring->pos_z = mavlink_msg_monitoring_get_pos_z(msg);
    monitoring->head = mavlink_msg_monitoring_get_head(msg);
    monitoring->status1 = mavlink_msg_monitoring_get_status1(msg);
    monitoring->status2 = mavlink_msg_monitoring_get_status2(msg);
    monitoring->rtk_nbase = mavlink_msg_monitoring_get_rtk_nbase(msg);
    monitoring->rtk_nrover = mavlink_msg_monitoring_get_rtk_nrover(msg);
    monitoring->battery = mavlink_msg_monitoring_get_battery(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MONITORING_LEN? msg->len : MAVLINK_MSG_ID_MONITORING_LEN;
        memset(monitoring, 0, MAVLINK_MSG_ID_MONITORING_LEN);
    memcpy(monitoring, _MAV_PAYLOAD(msg), len);
#endif
}
