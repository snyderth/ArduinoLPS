#ifndef __TDOA2_H_
#define __TDOA2_H_

/**
 * tdoa2.h
 * This is a class that implements the tdoa2 algorithm
 * implemented by Bitcraze.
 * Heavily based on crazyflie-firmware codebase
 * @see https://github.com/bitcraze/crazyflie-firmware
 * @see http://mikehamer.info/assets/papers/Ledergerber,%20Hamer,%20DAndrea%20-%20IROS%202015.pdf
 * 
 * Author:  Thomas Snyder
 * Date:    4/9/2020
 * 
 */

#include "DW1000.h"
#include "DW1000Time.h"
#include "generic_types.h"
#include "physical_constants.h"
#include "lpp.h"
#define PACKET_TYPE_TDOA2 0x22
#define LOCODECK_NUM_ANCHORS 8
/***************************************
 *  Keep track of:
 *      - One packet history of each anchor that includes:
 *          - Packet (range packet)
 *          - RX time
 *          - clock correction 
 *          - anchor status timeout
 *      - Previous anchor number
 * 
 *  Structure of tdoa2 algorithm:
 *      1. Receive packet
 *      2. Get the packet payload and cast it to 3 arrays and a type:
 *          - sequence numbers of the current packet of each anchor
 *          - timestamps of the anchors
 *          - distances between the anchors
 *              - These are known by the anchors and provided to the
 *                robot via the UWB communication
 *      3. Get the arrival time of the packet
 *      4. Calculate the clock correction between this anchor and the previous
 *          anchor. Log the clock correction into the history for the anchor
 *      5. Calculate the distance between this anchor and the previous anchor
 *      6. Give the distance difference and anchor numbers to the estimator to 
 *          use in its triangulation algorithm
 * ******************************************/

#define LOCODECK_TS_FREQ (499.2e6 * 128)

// //Allows the data to be passed as a void pointer to the tdoaHandleLpp Function
// typedef struct lpsLppPacketWrapper_s{
//     uint8_t anchorID;
//     uint8_t* data;
// }lpsLppPacketWrapper_t;


//Keeps track of each anchor range
typedef struct range2_s{
    uint8_t type;
    uint8_t seqNums[LOCODECK_NUM_ANCHORS];
    uint32_t timestamps[LOCODECK_NUM_ANCHORS];
    uint16_t distances[LOCODECK_NUM_ANCHORS];
} __attribute__((packed)) locoRange2_t;


//Store anchor configuration
typedef struct tdoaConf_s{

    uint64_t anchorAddress[LOCODECK_NUM_ANCHORS];
    point3 anchorPos[LOCODECK_NUM_ANCHORS];
    bool anchorPosOk;

}tdoa2Conf;



// Used to store attributes of each anchor
typedef struct tdoa2Hist_s{
    locoRange2_t packet;
    DW1000Time arrival;
    //Clock correction tag (T) to anchor (A)
    double clockCorrectionTtoA;
    uint32_t anchorStatusTimeout;
} tdoa2Hist_t;


//The lpp header is located after the locorange packet
#define LPS_TDOA2_LPP_HEADER (sizeof(locoRange2_t))
#define LPS_TDOA2_LPP_TYPE (LPS_TDOA2_LPP_HEADER + 1)
#define LPS_TDOA2_LPP_PAYLOAD (LPS_TDOA2_LPP_HEADER + 2)


class Tdoa2Alg{

    public:
        //Packet that we just received, probably unnecessary
        locoRange2_t rangePacket;
        //Configuration for the tdoa algorithm
        tdoa2Conf systemConf;
        // Stores the last packet for each anchor
        tdoa2Hist_t history[LOCODECK_NUM_ANCHORS];
        uint8_t prevAnchorID;

        Tdoa2Alg(); 
        void updateHistory(const locoRange2_t* packet, int64_t arrivalTime, uint8_t anchorID);
        void printCC();
        bool calcClockCorrection(double* prevCC, uint8_t anchorID, const locoRange2_t* packet, int64_t arrivalTime);
        bool calcTDoADistDiff(float* distDiffTDoA, const uint8_t previousAnchorID, const uint8_t currAnchorID, const locoRange2_t* packet, int64_t arrival);
        void tdoaHandleLPPshort(uint8_t datalen, loco_packet_t* packet);
        void printAnchorPos();

    private:
        bool _seqNumConsecutive(uint8_t, uint8_t);
        uint64_t _truncateToLocalTimestamp(uint64_t);
        uint64_t _truncateToAnchorTimestamp(uint64_t);
};
#endif //__TDOA2_H_