#pragma once
// MESSAGE SCENARIO_CMD PACKING

#define MAVLINK_MSG_ID_SCENARIO_CMD 155

MAVPACKED(
typedef struct __mavlink_scenario_cmd_t {
 float param1; /*<  param1 */
 float param2; /*<  param2 */
 float param3; /*<  param3 */
 uint32_t param4; /*<  param4  */
 uint8_t target_system; /*<  target sysytem id */
 uint8_t cmd; /*<  command */
 uint8_t param5[32]; /*<  param5 */
}) mavlink_scenario_cmd_t;

#define MAVLINK_MSG_ID_SCENARIO_CMD_LEN 50
#define MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN 50
#define MAVLINK_MSG_ID_155_LEN 50
#define MAVLINK_MSG_ID_155_MIN_LEN 50

#define MAVLINK_MSG_ID_SCENARIO_CMD_CRC 180
#define MAVLINK_MSG_ID_155_CRC 180

#define MAVLINK_MSG_SCENARIO_CMD_FIELD_PARAM5_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SCENARIO_CMD { \
    155, \
    "SCENARIO_CMD", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_scenario_cmd_t, target_system) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_scenario_cmd_t, cmd) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_scenario_cmd_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_scenario_cmd_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_scenario_cmd_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_scenario_cmd_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_UINT8_T, 32, 18, offsetof(mavlink_scenario_cmd_t, param5) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SCENARIO_CMD { \
    "SCENARIO_CMD", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_scenario_cmd_t, target_system) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_scenario_cmd_t, cmd) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_scenario_cmd_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_scenario_cmd_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_scenario_cmd_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_scenario_cmd_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_UINT8_T, 32, 18, offsetof(mavlink_scenario_cmd_t, param5) }, \
         } \
}
#endif

/**
 * @brief Pack a scenario_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  target sysytem id 
 * @param cmd  command 
 * @param param1  param1 
 * @param param2  param2 
 * @param param3  param3 
 * @param param4  param4  
 * @param param5  param5 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scenario_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t cmd, float param1, float param2, float param3, uint32_t param4, const uint8_t *param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCENARIO_CMD_LEN];
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, cmd);
    _mav_put_uint8_t_array(buf, 18, param5, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCENARIO_CMD_LEN);
#else
    mavlink_scenario_cmd_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.target_system = target_system;
    packet.cmd = cmd;
    mav_array_memcpy(packet.param5, param5, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCENARIO_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCENARIO_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_CRC);
}

/**
 * @brief Pack a scenario_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  target sysytem id 
 * @param cmd  command 
 * @param param1  param1 
 * @param param2  param2 
 * @param param3  param3 
 * @param param4  param4  
 * @param param5  param5 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scenario_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t cmd,float param1,float param2,float param3,uint32_t param4,const uint8_t *param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCENARIO_CMD_LEN];
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, cmd);
    _mav_put_uint8_t_array(buf, 18, param5, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCENARIO_CMD_LEN);
#else
    mavlink_scenario_cmd_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.target_system = target_system;
    packet.cmd = cmd;
    mav_array_memcpy(packet.param5, param5, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCENARIO_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCENARIO_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_CRC);
}

/**
 * @brief Encode a scenario_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scenario_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scenario_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scenario_cmd_t* scenario_cmd)
{
    return mavlink_msg_scenario_cmd_pack(system_id, component_id, msg, scenario_cmd->target_system, scenario_cmd->cmd, scenario_cmd->param1, scenario_cmd->param2, scenario_cmd->param3, scenario_cmd->param4, scenario_cmd->param5);
}

/**
 * @brief Encode a scenario_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param scenario_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scenario_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_scenario_cmd_t* scenario_cmd)
{
    return mavlink_msg_scenario_cmd_pack_chan(system_id, component_id, chan, msg, scenario_cmd->target_system, scenario_cmd->cmd, scenario_cmd->param1, scenario_cmd->param2, scenario_cmd->param3, scenario_cmd->param4, scenario_cmd->param5);
}

/**
 * @brief Send a scenario_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  target sysytem id 
 * @param cmd  command 
 * @param param1  param1 
 * @param param2  param2 
 * @param param3  param3 
 * @param param4  param4  
 * @param param5  param5 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scenario_cmd_send(mavlink_channel_t chan, uint8_t target_system, uint8_t cmd, float param1, float param2, float param3, uint32_t param4, const uint8_t *param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCENARIO_CMD_LEN];
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, cmd);
    _mav_put_uint8_t_array(buf, 18, param5, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCENARIO_CMD, buf, MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_CRC);
#else
    mavlink_scenario_cmd_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.target_system = target_system;
    packet.cmd = cmd;
    mav_array_memcpy(packet.param5, param5, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCENARIO_CMD, (const char *)&packet, MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_CRC);
#endif
}

/**
 * @brief Send a scenario_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_scenario_cmd_send_struct(mavlink_channel_t chan, const mavlink_scenario_cmd_t* scenario_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_scenario_cmd_send(chan, scenario_cmd->target_system, scenario_cmd->cmd, scenario_cmd->param1, scenario_cmd->param2, scenario_cmd->param3, scenario_cmd->param4, scenario_cmd->param5);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCENARIO_CMD, (const char *)scenario_cmd, MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_SCENARIO_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_scenario_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t cmd, float param1, float param2, float param3, uint32_t param4, const uint8_t *param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_uint32_t(buf, 12, param4);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, cmd);
    _mav_put_uint8_t_array(buf, 18, param5, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCENARIO_CMD, buf, MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_CRC);
#else
    mavlink_scenario_cmd_t *packet = (mavlink_scenario_cmd_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->target_system = target_system;
    packet->cmd = cmd;
    mav_array_memcpy(packet->param5, param5, sizeof(uint8_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCENARIO_CMD, (const char *)packet, MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_LEN, MAVLINK_MSG_ID_SCENARIO_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE SCENARIO_CMD UNPACKING


/**
 * @brief Get field target_system from scenario_cmd message
 *
 * @return  target sysytem id 
 */
static inline uint8_t mavlink_msg_scenario_cmd_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field cmd from scenario_cmd message
 *
 * @return  command 
 */
static inline uint8_t mavlink_msg_scenario_cmd_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field param1 from scenario_cmd message
 *
 * @return  param1 
 */
static inline float mavlink_msg_scenario_cmd_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from scenario_cmd message
 *
 * @return  param2 
 */
static inline float mavlink_msg_scenario_cmd_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from scenario_cmd message
 *
 * @return  param3 
 */
static inline float mavlink_msg_scenario_cmd_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from scenario_cmd message
 *
 * @return  param4  
 */
static inline uint32_t mavlink_msg_scenario_cmd_get_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field param5 from scenario_cmd message
 *
 * @return  param5 
 */
static inline uint16_t mavlink_msg_scenario_cmd_get_param5(const mavlink_message_t* msg, uint8_t *param5)
{
    return _MAV_RETURN_uint8_t_array(msg, param5, 32,  18);
}

/**
 * @brief Decode a scenario_cmd message into a struct
 *
 * @param msg The message to decode
 * @param scenario_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_scenario_cmd_decode(const mavlink_message_t* msg, mavlink_scenario_cmd_t* scenario_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    scenario_cmd->param1 = mavlink_msg_scenario_cmd_get_param1(msg);
    scenario_cmd->param2 = mavlink_msg_scenario_cmd_get_param2(msg);
    scenario_cmd->param3 = mavlink_msg_scenario_cmd_get_param3(msg);
    scenario_cmd->param4 = mavlink_msg_scenario_cmd_get_param4(msg);
    scenario_cmd->target_system = mavlink_msg_scenario_cmd_get_target_system(msg);
    scenario_cmd->cmd = mavlink_msg_scenario_cmd_get_cmd(msg);
    mavlink_msg_scenario_cmd_get_param5(msg, scenario_cmd->param5);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SCENARIO_CMD_LEN? msg->len : MAVLINK_MSG_ID_SCENARIO_CMD_LEN;
        memset(scenario_cmd, 0, MAVLINK_MSG_ID_SCENARIO_CMD_LEN);
    memcpy(scenario_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
