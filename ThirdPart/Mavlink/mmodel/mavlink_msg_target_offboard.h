#pragma once
// MESSAGE TARGET_OFFBOARD PACKING

#define MAVLINK_MSG_ID_TARGET_OFFBOARD 153

MAVPACKED(
typedef struct __mavlink_target_offboard_t {
 uint32_t time_boot_ms; /*< Timestamp in milliseconds since system boot*/
 float pos_x; /*<  target pos x  */
 float pos_y; /*<  target pos y  */
 float pos_z; /*<  target pos z  */
 float head; /*<  target heading  */
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t led_type; /*<  display shape */
 uint8_t led_speed; /*<  display speed */
 uint8_t led_r; /*<  red value (0~256) */
 uint8_t led_g; /*<  green value (0~256) */
 uint8_t led_b; /*<  blue value (0~256) */
 uint8_t led_bright; /*<  brightness  */
}) mavlink_target_offboard_t;

#define MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN 28
#define MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN 28
#define MAVLINK_MSG_ID_153_LEN 28
#define MAVLINK_MSG_ID_153_MIN_LEN 28

#define MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC 207
#define MAVLINK_MSG_ID_153_CRC 207



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET_OFFBOARD { \
    153, \
    "TARGET_OFFBOARD", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_target_offboard_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_target_offboard_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_target_offboard_t, target_component) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_offboard_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_offboard_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_offboard_t, pos_z) }, \
         { "head", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_offboard_t, head) }, \
         { "led_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_target_offboard_t, led_type) }, \
         { "led_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_target_offboard_t, led_speed) }, \
         { "led_r", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_target_offboard_t, led_r) }, \
         { "led_g", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_target_offboard_t, led_g) }, \
         { "led_b", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_target_offboard_t, led_b) }, \
         { "led_bright", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_target_offboard_t, led_bright) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET_OFFBOARD { \
    "TARGET_OFFBOARD", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_target_offboard_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_target_offboard_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_target_offboard_t, target_component) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_offboard_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_offboard_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_offboard_t, pos_z) }, \
         { "head", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_offboard_t, head) }, \
         { "led_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_target_offboard_t, led_type) }, \
         { "led_speed", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_target_offboard_t, led_speed) }, \
         { "led_r", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_target_offboard_t, led_r) }, \
         { "led_g", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_target_offboard_t, led_g) }, \
         { "led_b", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_target_offboard_t, led_b) }, \
         { "led_bright", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_target_offboard_t, led_bright) }, \
         } \
}
#endif

/**
 * @brief Pack a target_offboard message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param pos_x  target pos x  
 * @param pos_y  target pos y  
 * @param pos_z  target pos z  
 * @param head  target heading  
 * @param led_type  display shape 
 * @param led_speed  display speed 
 * @param led_r  red value (0~256) 
 * @param led_g  green value (0~256) 
 * @param led_b  blue value (0~256) 
 * @param led_bright  brightness  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_offboard_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float pos_x, float pos_y, float pos_z, float head, uint8_t led_type, uint8_t led_speed, uint8_t led_r, uint8_t led_g, uint8_t led_b, uint8_t led_bright)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, led_type);
    _mav_put_uint8_t(buf, 23, led_speed);
    _mav_put_uint8_t(buf, 24, led_r);
    _mav_put_uint8_t(buf, 25, led_g);
    _mav_put_uint8_t(buf, 26, led_b);
    _mav_put_uint8_t(buf, 27, led_bright);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN);
#else
    mavlink_target_offboard_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.led_type = led_type;
    packet.led_speed = led_speed;
    packet.led_r = led_r;
    packet.led_g = led_g;
    packet.led_b = led_b;
    packet.led_bright = led_bright;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_OFFBOARD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC);
}

/**
 * @brief Pack a target_offboard message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param pos_x  target pos x  
 * @param pos_y  target pos y  
 * @param pos_z  target pos z  
 * @param head  target heading  
 * @param led_type  display shape 
 * @param led_speed  display speed 
 * @param led_r  red value (0~256) 
 * @param led_g  green value (0~256) 
 * @param led_b  blue value (0~256) 
 * @param led_bright  brightness  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_offboard_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,float pos_x,float pos_y,float pos_z,float head,uint8_t led_type,uint8_t led_speed,uint8_t led_r,uint8_t led_g,uint8_t led_b,uint8_t led_bright)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, led_type);
    _mav_put_uint8_t(buf, 23, led_speed);
    _mav_put_uint8_t(buf, 24, led_r);
    _mav_put_uint8_t(buf, 25, led_g);
    _mav_put_uint8_t(buf, 26, led_b);
    _mav_put_uint8_t(buf, 27, led_bright);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN);
#else
    mavlink_target_offboard_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.led_type = led_type;
    packet.led_speed = led_speed;
    packet.led_r = led_r;
    packet.led_g = led_g;
    packet.led_b = led_b;
    packet.led_bright = led_bright;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_OFFBOARD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC);
}

/**
 * @brief Encode a target_offboard struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_offboard C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_offboard_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_offboard_t* target_offboard)
{
    return mavlink_msg_target_offboard_pack(system_id, component_id, msg, target_offboard->time_boot_ms, target_offboard->target_system, target_offboard->target_component, target_offboard->pos_x, target_offboard->pos_y, target_offboard->pos_z, target_offboard->head, target_offboard->led_type, target_offboard->led_speed, target_offboard->led_r, target_offboard->led_g, target_offboard->led_b, target_offboard->led_bright);
}

/**
 * @brief Encode a target_offboard struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_offboard C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_offboard_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_offboard_t* target_offboard)
{
    return mavlink_msg_target_offboard_pack_chan(system_id, component_id, chan, msg, target_offboard->time_boot_ms, target_offboard->target_system, target_offboard->target_component, target_offboard->pos_x, target_offboard->pos_y, target_offboard->pos_z, target_offboard->head, target_offboard->led_type, target_offboard->led_speed, target_offboard->led_r, target_offboard->led_g, target_offboard->led_b, target_offboard->led_bright);
}

/**
 * @brief Send a target_offboard message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param pos_x  target pos x  
 * @param pos_y  target pos y  
 * @param pos_z  target pos z  
 * @param head  target heading  
 * @param led_type  display shape 
 * @param led_speed  display speed 
 * @param led_r  red value (0~256) 
 * @param led_g  green value (0~256) 
 * @param led_b  blue value (0~256) 
 * @param led_bright  brightness  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_offboard_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float pos_x, float pos_y, float pos_z, float head, uint8_t led_type, uint8_t led_speed, uint8_t led_r, uint8_t led_g, uint8_t led_b, uint8_t led_bright)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, led_type);
    _mav_put_uint8_t(buf, 23, led_speed);
    _mav_put_uint8_t(buf, 24, led_r);
    _mav_put_uint8_t(buf, 25, led_g);
    _mav_put_uint8_t(buf, 26, led_b);
    _mav_put_uint8_t(buf, 27, led_bright);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_OFFBOARD, buf, MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC);
#else
    mavlink_target_offboard_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.led_type = led_type;
    packet.led_speed = led_speed;
    packet.led_r = led_r;
    packet.led_g = led_g;
    packet.led_b = led_b;
    packet.led_bright = led_bright;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_OFFBOARD, (const char *)&packet, MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC);
#endif
}

/**
 * @brief Send a target_offboard message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_offboard_send_struct(mavlink_channel_t chan, const mavlink_target_offboard_t* target_offboard)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_offboard_send(chan, target_offboard->time_boot_ms, target_offboard->target_system, target_offboard->target_component, target_offboard->pos_x, target_offboard->pos_y, target_offboard->pos_z, target_offboard->head, target_offboard->led_type, target_offboard->led_speed, target_offboard->led_r, target_offboard->led_g, target_offboard->led_b, target_offboard->led_bright);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_OFFBOARD, (const char *)target_offboard, MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_offboard_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float pos_x, float pos_y, float pos_z, float head, uint8_t led_type, uint8_t led_speed, uint8_t led_r, uint8_t led_g, uint8_t led_b, uint8_t led_bright)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, led_type);
    _mav_put_uint8_t(buf, 23, led_speed);
    _mav_put_uint8_t(buf, 24, led_r);
    _mav_put_uint8_t(buf, 25, led_g);
    _mav_put_uint8_t(buf, 26, led_b);
    _mav_put_uint8_t(buf, 27, led_bright);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_OFFBOARD, buf, MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC);
#else
    mavlink_target_offboard_t *packet = (mavlink_target_offboard_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->pos_z = pos_z;
    packet->head = head;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->led_type = led_type;
    packet->led_speed = led_speed;
    packet->led_r = led_r;
    packet->led_g = led_g;
    packet->led_b = led_b;
    packet->led_bright = led_bright;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_OFFBOARD, (const char *)packet, MAVLINK_MSG_ID_TARGET_OFFBOARD_MIN_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN, MAVLINK_MSG_ID_TARGET_OFFBOARD_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET_OFFBOARD UNPACKING


/**
 * @brief Get field time_boot_ms from target_offboard message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_target_offboard_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from target_offboard message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_target_offboard_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field target_component from target_offboard message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_target_offboard_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field pos_x from target_offboard message
 *
 * @return  target pos x  
 */
static inline float mavlink_msg_target_offboard_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pos_y from target_offboard message
 *
 * @return  target pos y  
 */
static inline float mavlink_msg_target_offboard_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pos_z from target_offboard message
 *
 * @return  target pos z  
 */
static inline float mavlink_msg_target_offboard_get_pos_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field head from target_offboard message
 *
 * @return  target heading  
 */
static inline float mavlink_msg_target_offboard_get_head(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field led_type from target_offboard message
 *
 * @return  display shape 
 */
static inline uint8_t mavlink_msg_target_offboard_get_led_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field led_speed from target_offboard message
 *
 * @return  display speed 
 */
static inline uint8_t mavlink_msg_target_offboard_get_led_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field led_r from target_offboard message
 *
 * @return  red value (0~256) 
 */
static inline uint8_t mavlink_msg_target_offboard_get_led_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field led_g from target_offboard message
 *
 * @return  green value (0~256) 
 */
static inline uint8_t mavlink_msg_target_offboard_get_led_g(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field led_b from target_offboard message
 *
 * @return  blue value (0~256) 
 */
static inline uint8_t mavlink_msg_target_offboard_get_led_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field led_bright from target_offboard message
 *
 * @return  brightness  
 */
static inline uint8_t mavlink_msg_target_offboard_get_led_bright(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Decode a target_offboard message into a struct
 *
 * @param msg The message to decode
 * @param target_offboard C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_offboard_decode(const mavlink_message_t* msg, mavlink_target_offboard_t* target_offboard)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target_offboard->time_boot_ms = mavlink_msg_target_offboard_get_time_boot_ms(msg);
    target_offboard->pos_x = mavlink_msg_target_offboard_get_pos_x(msg);
    target_offboard->pos_y = mavlink_msg_target_offboard_get_pos_y(msg);
    target_offboard->pos_z = mavlink_msg_target_offboard_get_pos_z(msg);
    target_offboard->head = mavlink_msg_target_offboard_get_head(msg);
    target_offboard->target_system = mavlink_msg_target_offboard_get_target_system(msg);
    target_offboard->target_component = mavlink_msg_target_offboard_get_target_component(msg);
    target_offboard->led_type = mavlink_msg_target_offboard_get_led_type(msg);
    target_offboard->led_speed = mavlink_msg_target_offboard_get_led_speed(msg);
    target_offboard->led_r = mavlink_msg_target_offboard_get_led_r(msg);
    target_offboard->led_g = mavlink_msg_target_offboard_get_led_g(msg);
    target_offboard->led_b = mavlink_msg_target_offboard_get_led_b(msg);
    target_offboard->led_bright = mavlink_msg_target_offboard_get_led_bright(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN? msg->len : MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN;
        memset(target_offboard, 0, MAVLINK_MSG_ID_TARGET_OFFBOARD_LEN);
    memcpy(target_offboard, _MAV_PAYLOAD(msg), len);
#endif
}
