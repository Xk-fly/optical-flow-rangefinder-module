
#include "massage_adapter.h"
uint8_t SYSTEM_ID = 1 ;
uint8_t COMPONENT_ID = MAV_COMP_ID_PERIPHERAL;

/**
 * @brief 发送optical_flow作为mavlink
 * @param huart 串口句柄指针
 * @param flow_x 光流x
 * @param flow_y 光流y
 * @param quality 光流质量
 * @param distance_mm 距离，单位毫米
 * @param distance_valid 距离有效标志
 */
void send_optical_flow(UART_HandleTypeDef *huart, int16_t flow_x, int16_t flow_y,
                       uint8_t quality, uint16_t distance_mm, uint8_t distance_valid){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_optical_flow_t packet = {0};
    // 当前系统时间微秒
    packet.time_usec = (uint64_t)HAL_GetTick() * 1000ULL;
    packet.sensor_id = 0;
    packet.flow_comp_m_x = 0;
    packet.flow_comp_m_y = 0;
    packet.flow_x = flow_x;
    packet.flow_y = flow_y;
    packet.quality = quality;
    packet.ground_distance = distance_valid ? ((float)distance_mm / 1000.0f) : -1.0f;

    // 打包消息
    mavlink_msg_optical_flow_encode(
    	SYSTEM_ID,    // system_id
		COMPONENT_ID,  // component_id
        &msg,
        &packet
    );

    // 转为字节数组
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // 发送
    HAL_UART_Transmit(huart, buf, len, 100);
}


/**
 * @brief 发送distance_sensor作为mavlink
 * @param huart 串口句柄指针
 * @param distance 距离
 */
void send_distance_sensor(UART_HandleTypeDef *huart,uint16_t distance){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_distance_sensor_t mdst = {0};

	mdst.time_boot_ms = HAL_GetTick();
	mdst.id = 1;
	//	cm
	mdst.min_distance = 5;
	//	cm
	mdst.max_distance = 360;

	mdst.current_distance = distance / 10;
	//	MAV_DISTANCE_SENSOR_LASER
	mdst.type = 0;
	//Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.
	mdst.covariance = UINT8_MAX;
	/**
	 * Direction the sensor faces.
	 * downward-facing: ROTATION_PITCH_270,
	 * upward-facing: ROTATION_PITCH_90,
	 * backward-facing: ROTATION_PITCH_180,
	 * forward-facing: ROTATION_NONE,
	 * left-facing: ROTATION_YAW_90,
	 * right-facing: ROTATION_YAW_270
	 */
	mdst.orientation = 25;

	mavlink_msg_distance_sensor_encode(
			SYSTEM_ID,
			COMPONENT_ID,
			&msg,
			&mdst
			);

	// 转为字节流
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	// UART 发送
	HAL_UART_Transmit(huart, buf, len, 100);
}

void send_status_text(UART_HandleTypeDef *huart, uint8_t severity, const char *text){
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_statustext_t statustext = {0};

	statustext.severity = severity;
	for (uint8_t i = 0; i < MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN && text[i] != '\0'; i++) {
		statustext.text[i] = text[i];
	}

	mavlink_msg_statustext_encode(
			SYSTEM_ID,
			COMPONENT_ID,
			&msg,
			&statustext
			);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(huart, buf, len, 100);
}

/**
 * @brief 发送heart_beat作为mavlink
 * @param huart 串口句柄指针
 * @param distance 距离
 */
void send_heart_beat(UART_HandleTypeDef *huart){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_heartbeat_t hb = {0};
    hb.type = MAV_TYPE_ONBOARD_CONTROLLER; // 设备类型：板载控制器（外设）
    hb.autopilot = MAV_AUTOPILOT_INVALID;  // 不是飞控
    hb.base_mode = 0;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;
    // 打包 HEARTBEAT
    mavlink_msg_heartbeat_encode(
    	SYSTEM_ID,    // system_id（你的STM32 ID）
		COMPONENT_ID,  // component_id（外设ID）
        &msg,
        &hb
    );

    // 转为字节流
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	// UART 发送
	HAL_UART_Transmit(huart, buf, len, 100);
}
