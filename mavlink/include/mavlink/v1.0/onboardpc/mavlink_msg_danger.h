// MESSAGE DANGER PACKING

#define MAVLINK_MSG_ID_DANGER 151

typedef struct __mavlink_danger_t
{
 uint64_t time_usec; ///< Timestamp (UNIX)
 float danger_dir; ///< Direction of the danger in rad
 float avoid_dir; ///< Avoid dirrection in rad
 uint16_t danger_dist; ///< Distance to danger in mm
 uint8_t sensor_id; ///< Sensor ID
 uint8_t danger_level; ///< Level of danger
 uint8_t avoid_level; ///< Avoid level/weight
} mavlink_danger_t;

#define MAVLINK_MSG_ID_DANGER_LEN 21
#define MAVLINK_MSG_ID_151_LEN 21

#define MAVLINK_MSG_ID_DANGER_CRC 41
#define MAVLINK_MSG_ID_151_CRC 41



#define MAVLINK_MESSAGE_INFO_DANGER { \
	"DANGER", \
	7, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_danger_t, time_usec) }, \
         { "danger_dir", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_danger_t, danger_dir) }, \
         { "avoid_dir", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_danger_t, avoid_dir) }, \
         { "danger_dist", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_danger_t, danger_dist) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_danger_t, sensor_id) }, \
         { "danger_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_danger_t, danger_level) }, \
         { "avoid_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_danger_t, avoid_level) }, \
         } \
}


/**
 * @brief Pack a danger message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param danger_level Level of danger
 * @param danger_dist Distance to danger in mm
 * @param danger_dir Direction of the danger in rad
 * @param avoid_level Avoid level/weight
 * @param avoid_dir Avoid dirrection in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_danger_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t sensor_id, uint8_t danger_level, uint16_t danger_dist, float danger_dir, uint8_t avoid_level, float avoid_dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DANGER_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, danger_dir);
	_mav_put_float(buf, 12, avoid_dir);
	_mav_put_uint16_t(buf, 16, danger_dist);
	_mav_put_uint8_t(buf, 18, sensor_id);
	_mav_put_uint8_t(buf, 19, danger_level);
	_mav_put_uint8_t(buf, 20, avoid_level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DANGER_LEN);
#else
	mavlink_danger_t packet;
	packet.time_usec = time_usec;
	packet.danger_dir = danger_dir;
	packet.avoid_dir = avoid_dir;
	packet.danger_dist = danger_dist;
	packet.sensor_id = sensor_id;
	packet.danger_level = danger_level;
	packet.avoid_level = avoid_level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DANGER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DANGER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DANGER_LEN, MAVLINK_MSG_ID_DANGER_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DANGER_LEN);
#endif
}

/**
 * @brief Pack a danger message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param danger_level Level of danger
 * @param danger_dist Distance to danger in mm
 * @param danger_dir Direction of the danger in rad
 * @param avoid_level Avoid level/weight
 * @param avoid_dir Avoid dirrection in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_danger_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t sensor_id,uint8_t danger_level,uint16_t danger_dist,float danger_dir,uint8_t avoid_level,float avoid_dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DANGER_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, danger_dir);
	_mav_put_float(buf, 12, avoid_dir);
	_mav_put_uint16_t(buf, 16, danger_dist);
	_mav_put_uint8_t(buf, 18, sensor_id);
	_mav_put_uint8_t(buf, 19, danger_level);
	_mav_put_uint8_t(buf, 20, avoid_level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DANGER_LEN);
#else
	mavlink_danger_t packet;
	packet.time_usec = time_usec;
	packet.danger_dir = danger_dir;
	packet.avoid_dir = avoid_dir;
	packet.danger_dist = danger_dist;
	packet.sensor_id = sensor_id;
	packet.danger_level = danger_level;
	packet.avoid_level = avoid_level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DANGER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DANGER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DANGER_LEN, MAVLINK_MSG_ID_DANGER_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DANGER_LEN);
#endif
}

/**
 * @brief Encode a danger struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param danger C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_danger_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_danger_t* danger)
{
	return mavlink_msg_danger_pack(system_id, component_id, msg, danger->time_usec, danger->sensor_id, danger->danger_level, danger->danger_dist, danger->danger_dir, danger->avoid_level, danger->avoid_dir);
}

/**
 * @brief Encode a danger struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param danger C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_danger_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_danger_t* danger)
{
	return mavlink_msg_danger_pack_chan(system_id, component_id, chan, msg, danger->time_usec, danger->sensor_id, danger->danger_level, danger->danger_dist, danger->danger_dir, danger->avoid_level, danger->avoid_dir);
}

/**
 * @brief Send a danger message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param danger_level Level of danger
 * @param danger_dist Distance to danger in mm
 * @param danger_dir Direction of the danger in rad
 * @param avoid_level Avoid level/weight
 * @param avoid_dir Avoid dirrection in rad
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_danger_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t sensor_id, uint8_t danger_level, uint16_t danger_dist, float danger_dir, uint8_t avoid_level, float avoid_dir)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DANGER_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, danger_dir);
	_mav_put_float(buf, 12, avoid_dir);
	_mav_put_uint16_t(buf, 16, danger_dist);
	_mav_put_uint8_t(buf, 18, sensor_id);
	_mav_put_uint8_t(buf, 19, danger_level);
	_mav_put_uint8_t(buf, 20, avoid_level);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANGER, buf, MAVLINK_MSG_ID_DANGER_LEN, MAVLINK_MSG_ID_DANGER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANGER, buf, MAVLINK_MSG_ID_DANGER_LEN);
#endif
#else
	mavlink_danger_t packet;
	packet.time_usec = time_usec;
	packet.danger_dir = danger_dir;
	packet.avoid_dir = avoid_dir;
	packet.danger_dist = danger_dist;
	packet.sensor_id = sensor_id;
	packet.danger_level = danger_level;
	packet.avoid_level = avoid_level;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANGER, (const char *)&packet, MAVLINK_MSG_ID_DANGER_LEN, MAVLINK_MSG_ID_DANGER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DANGER, (const char *)&packet, MAVLINK_MSG_ID_DANGER_LEN);
#endif
#endif
}

#endif

// MESSAGE DANGER UNPACKING


/**
 * @brief Get field time_usec from danger message
 *
 * @return Timestamp (UNIX)
 */
static inline uint64_t mavlink_msg_danger_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_id from danger message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_danger_get_sensor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field danger_level from danger message
 *
 * @return Level of danger
 */
static inline uint8_t mavlink_msg_danger_get_danger_level(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field danger_dist from danger message
 *
 * @return Distance to danger in mm
 */
static inline uint16_t mavlink_msg_danger_get_danger_dist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field danger_dir from danger message
 *
 * @return Direction of the danger in rad
 */
static inline float mavlink_msg_danger_get_danger_dir(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field avoid_level from danger message
 *
 * @return Avoid level/weight
 */
static inline uint8_t mavlink_msg_danger_get_avoid_level(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field avoid_dir from danger message
 *
 * @return Avoid dirrection in rad
 */
static inline float mavlink_msg_danger_get_avoid_dir(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a danger message into a struct
 *
 * @param msg The message to decode
 * @param danger C-struct to decode the message contents into
 */
static inline void mavlink_msg_danger_decode(const mavlink_message_t* msg, mavlink_danger_t* danger)
{
#if MAVLINK_NEED_BYTE_SWAP
	danger->time_usec = mavlink_msg_danger_get_time_usec(msg);
	danger->danger_dir = mavlink_msg_danger_get_danger_dir(msg);
	danger->avoid_dir = mavlink_msg_danger_get_avoid_dir(msg);
	danger->danger_dist = mavlink_msg_danger_get_danger_dist(msg);
	danger->sensor_id = mavlink_msg_danger_get_sensor_id(msg);
	danger->danger_level = mavlink_msg_danger_get_danger_level(msg);
	danger->avoid_level = mavlink_msg_danger_get_avoid_level(msg);
#else
	memcpy(danger, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DANGER_LEN);
#endif
}
