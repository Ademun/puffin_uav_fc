#ifndef COMMS_H
#define COMMS_H

#include "common/mavlink.h"
#include <esp_err.h>

#define MAVLINK_TX_CHAN MAVLINK_COMM_0
#define MAVLINK_RX_CHAN MAVLINK_COMM_1
#define MAV_SYSTEM_ID 1
#define MAV_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1
#define MAV_DRONE_TYPE MAV_TYPE_QUADROTOR
#define MAV_AUTOPILOT_TYPE MAV_AUTOPILOT_GENERIC

esp_err_t communications_start();
void mav_send(const mavlink_message_t *msg);
void mav_lock(void);
void mav_unlock(void);

extern int udp_socket;

#endif