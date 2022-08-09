#pragma once
// MESSAGE LED_CONTROL PACKING

#define MAVLINK_MSG_ID_LED_CONTROL 150

MAVPACKED(
typedef struct __mavlink_led_control_t {
 uint8_t type; /*<  display shape */
 uint8_t r; /*<  red value (0~256) */
 uint8_t g; /*<  green value (0~256) */
 uint8_t b; /*<  blue value (0~256) */
 uint8_t brightness; /*<  brightness  */
}) mavlink_led_control_t;

#define MAVLINK_MSG_ID_LED_CONTROL_LEN 5
#define MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN 5
#define MAVLINK_MSG_ID_150_LEN 5
#define MAVLINK_MSG_ID_150_MIN_LEN 5

#define MAVLINK_MSG_ID_LED_CONTROL_CRC 5
#define MAVLINK_MSG_ID_150_CRC 5



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LED_CONTROL { \
    150, \
    "LED_CONTROL", \
    5, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_led_control_t, type) }, \
         { "r", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_led_control_t, r) }, \
         { "g", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_led_control_t, g) }, \
         { "b", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_led_control_t, b) }, \
         { "brightness", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_led_control_t, brightness) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LED_CONTROL { \
    "LED_CONTROL", \
    5, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_led_control_t, type) }, \
         { "r", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_led_control_t, r) }, \
         { "g", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_led_control_t, g) }, \
         { "b", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_led_control_t, b) }, \
         { "brightness", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_led_control_t, brightness) }, \
         } \
}
#endif

/**
 * @brief Pack a led_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type  display shape 
 * @param r  red value (0~256) 
 * @param g  green value (0~256) 
 * @param b  blue value (0~256) 
 * @param brightness  brightness  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LED_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, r);
    _mav_put_uint8_t(buf, 2, g);
    _mav_put_uint8_t(buf, 3, b);
    _mav_put_uint8_t(buf, 4, brightness);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_LEN);
#else
    mavlink_led_control_t packet;
    packet.type = type;
    packet.r = r;
    packet.g = g;
    packet.b = b;
    packet.brightness = brightness;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LED_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN, MAVLINK_MSG_ID_LED_CONTROL_LEN, MAVLINK_MSG_ID_LED_CONTROL_CRC);
}

/**
 * @brief Pack a led_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type  display shape 
 * @param r  red value (0~256) 
 * @param g  green value (0~256) 
 * @param b  blue value (0~256) 
 * @param brightness  brightness  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_led_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t type,uint8_t r,uint8_t g,uint8_t b,uint8_t brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LED_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, r);
    _mav_put_uint8_t(buf, 2, g);
    _mav_put_uint8_t(buf, 3, b);
    _mav_put_uint8_t(buf, 4, brightness);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LED_CONTROL_LEN);
#else
    mavlink_led_control_t packet;
    packet.type = type;
    packet.r = r;
    packet.g = g;
    packet.b = b;
    packet.brightness = brightness;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LED_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LED_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN, MAVLINK_MSG_ID_LED_CONTROL_LEN, MAVLINK_MSG_ID_LED_CONTROL_CRC);
}

/**
 * @brief Encode a led_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param led_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_led_control_t* led_control)
{
    return mavlink_msg_led_control_pack(system_id, component_id, msg, led_control->type, led_control->r, led_control->g, led_control->b, led_control->brightness);
}

/**
 * @brief Encode a led_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param led_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_led_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_led_control_t* led_control)
{
    return mavlink_msg_led_control_pack_chan(system_id, component_id, chan, msg, led_control->type, led_control->r, led_control->g, led_control->b, led_control->brightness);
}

/**
 * @brief Send a led_control message
 * @param chan MAVLink channel to send the message
 *
 * @param type  display shape 
 * @param r  red value (0~256) 
 * @param g  green value (0~256) 
 * @param b  blue value (0~256) 
 * @param brightness  brightness  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_led_control_send(mavlink_channel_t chan, uint8_t type, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LED_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, r);
    _mav_put_uint8_t(buf, 2, g);
    _mav_put_uint8_t(buf, 3, b);
    _mav_put_uint8_t(buf, 4, brightness);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL, buf, MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN, MAVLINK_MSG_ID_LED_CONTROL_LEN, MAVLINK_MSG_ID_LED_CONTROL_CRC);
#else
    mavlink_led_control_t packet;
    packet.type = type;
    packet.r = r;
    packet.g = g;
    packet.b = b;
    packet.brightness = brightness;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN, MAVLINK_MSG_ID_LED_CONTROL_LEN, MAVLINK_MSG_ID_LED_CONTROL_CRC);
#endif
}

/**
 * @brief Send a led_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_led_control_send_struct(mavlink_channel_t chan, const mavlink_led_control_t* led_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_led_control_send(chan, led_control->type, led_control->r, led_control->g, led_control->b, led_control->brightness);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL, (const char *)led_control, MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN, MAVLINK_MSG_ID_LED_CONTROL_LEN, MAVLINK_MSG_ID_LED_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_LED_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_led_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, r);
    _mav_put_uint8_t(buf, 2, g);
    _mav_put_uint8_t(buf, 3, b);
    _mav_put_uint8_t(buf, 4, brightness);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL, buf, MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN, MAVLINK_MSG_ID_LED_CONTROL_LEN, MAVLINK_MSG_ID_LED_CONTROL_CRC);
#else
    mavlink_led_control_t *packet = (mavlink_led_control_t *)msgbuf;
    packet->type = type;
    packet->r = r;
    packet->g = g;
    packet->b = b;
    packet->brightness = brightness;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LED_CONTROL, (const char *)packet, MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN, MAVLINK_MSG_ID_LED_CONTROL_LEN, MAVLINK_MSG_ID_LED_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE LED_CONTROL UNPACKING


/**
 * @brief Get field type from led_control message
 *
 * @return  display shape 
 */
static inline uint8_t mavlink_msg_led_control_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field r from led_control message
 *
 * @return  red value (0~256) 
 */
static inline uint8_t mavlink_msg_led_control_get_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field g from led_control message
 *
 * @return  green value (0~256) 
 */
static inline uint8_t mavlink_msg_led_control_get_g(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field b from led_control message
 *
 * @return  blue value (0~256) 
 */
static inline uint8_t mavlink_msg_led_control_get_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field brightness from led_control message
 *
 * @return  brightness  
 */
static inline uint8_t mavlink_msg_led_control_get_brightness(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a led_control message into a struct
 *
 * @param msg The message to decode
 * @param led_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_led_control_decode(const mavlink_message_t* msg, mavlink_led_control_t* led_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    led_control->type = mavlink_msg_led_control_get_type(msg);
    led_control->r = mavlink_msg_led_control_get_r(msg);
    led_control->g = mavlink_msg_led_control_get_g(msg);
    led_control->b = mavlink_msg_led_control_get_b(msg);
    led_control->brightness = mavlink_msg_led_control_get_brightness(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LED_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_LED_CONTROL_LEN;
        memset(led_control, 0, MAVLINK_MSG_ID_LED_CONTROL_LEN);
    memcpy(led_control, _MAV_PAYLOAD(msg), len);
#endif
}
