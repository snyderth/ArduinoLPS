#ifndef __GEN_TYPES_H_
#define __GEN_TYPES_H_


/** 
 * Struct to receive dw1000 packets and interpret the data from the anchors.
 * @todo Understand why this can be cast to an array of 8-bit integers to be passed to the getData function of the DWM1000.
 */
typedef struct packet_s{
    union {
      uint16_t fcf;
      struct {
        uint16_t type:3;
        uint16_t security:1;
        uint16_t framePending:1;
        uint16_t ack:1;
        uint16_t ipan:1;
        uint16_t reserved:3;
        uint16_t destAddrMode:2;
        uint16_t version:2;
        uint16_t srcAddrMode:2;
      } fcf_s;
    };
    uint8_t seq;
    uint16_t pan;
    uint64_t destAddress;
    uint64_t sourceAddress;

    uint8_t payload[128];
}__attribute__((packed)) loco_packet_t;


// Always use this vector 3 struct
#ifdef vector3
#undef vector3
#endif

typedef struct vector3{
    float x;
    float y;
    float z;

    inline vector3 operator-(vector3 a){
      return {x - a.x, y - a.y, z - a.z};
    }

    inline vector3 operator+(vector3 a){
      return {x + a.x, y + a.y, z + a.z};
    }

    float norm(){
      return pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2);
    }


} vec3;

typedef vec3 point3;
void print_point3(point3);

//To queue up for the estimator
typedef struct tdoaMeasurement_s{
    point3 anchorPos[2]; // 
    float distanceDiff; // 4
    float stdDev; // 4
} tdoaMeasurement_t;

const uint16_t TDOA_SIZE = 32; 
// Empirically tested on UBUNTU 18.04 WSL

#endif