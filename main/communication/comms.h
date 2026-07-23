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

typedef enum {
  TX_REQUEST_OK,
  TX_REQUEST_NOT_FOUND,
  TX_REQUEST_NOT_READY,
} tx_request_result_t;

esp_err_t communications_start();
void mav_send(const mavlink_message_t *msg);
void mav_lock(void);
void mav_unlock(void);

bool tx_set_message_interval(uint32_t mav_msg_id, int32_t interval_us);

tx_request_result_t tx_request_message(uint32_t mav_msg_id);

extern int udp_socket;

#endif