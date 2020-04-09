#ifndef _LPP_H_
#define _LPP_H_

#include "generic_types.h"
//Implementation of the Loco Positioning Protocol for 
//data transmission of anchor locations

#define LPP_HEADER_SHORT_PACKET 0xF0
#define LPP_SHORT_ANCHORPOS 0x01

// MAC Layer contants
#define MAC802154_TYPE_BEACON 0
#define MAC802154_TYPE_DATA 1
#define MAC802154_TYPE_ACK 2
#define MAC802154_TYPE_CMD 3

#define MAC802154_HEADER_LENGTH 21

// later, if we wanted to be able to update the LPS nodes via the
// ground robots instead of just the UAVs, we need to be able to transmit
// This means the receive permanently should be set to false and after
// every receive, the start function in LocoDeck should be called

typedef struct lppPacket_s{
    uint8_t dest;
    uint8_t length;
    uint8_t data[30];
} lppShortPackt_t;

// void handleLppPacket(uint8_t dataLen, loco_packet_t* packet, void(*fnHandle)(void*));

struct lppAnchorPos_s{
    float x;
    float y;
    float z;
}__attribute__((packed));

typedef lppAnchorPos_s anchorPos_t;

#endif