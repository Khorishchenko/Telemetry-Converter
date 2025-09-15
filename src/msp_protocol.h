#pragma once

// MSP command codes (Betaflight/iNav compatible)
#define MSP_API_VERSION           1
#define MSP_FC_VARIANT            2
#define MSP_FC_VERSION            3
#define MSP_BOARD_INFO            4
#define MSP_BUILD_INFO            5

#define MSP_RC                    105
#define MSP_RAW_GPS               106
#define MSP_BATTERY_STATUS        107
#define MSP_ATTITUDE              108

#define MSP_STATUS_EX             150
#define MSP_UID                   160

// MSPv2 extended codes (examples)
#define MSP_BATTERY_STATE         0x010B
#define MSP_RC_EXTENDED           0x0105

// Додай інші потрібні коди за потреби