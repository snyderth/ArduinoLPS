#include "include/tdoa2.h"

/**
 * Calculates the difference in distances between two anchors to the tag. Implementation
 * of equation 16 in "A Robot Self-Localization System using One-Way Ultra-Wideband Communication".
 *
 * @param distDiffTDoA The calculated distance difference number between the two anchors and the tag.
 * @param previousAnchorID The anchor ID number of the previous anchor.
 * @param currAnchorID The anchor ID number of the current anchor whose packet was just received. The anchors be in sequence.
 * @param packet The range packet that was just received.
 * @param arrival The local time (tag time) in ticks that the packet was received at.
 * @return False if packets are not in sequence or timestamps are 0. Else returns true.
 * 
 * @see <a href="http://mikehamer.info/assets/papers/Ledergerber,%20Hamer,%20DAndrea%20-%20IROS%202015.pd">
 *          A Robot Self-Localization System using One-Way Ultra-Wideband Communication
 *      </a>
 */
bool Tdoa2Alg::calcTDoADistDiff(float* distDiffTDoA, const uint8_t previousAnchorID, const uint8_t currAnchorID, const locoRange2_t* packet, int64_t arrival){
    //Check to see if the previous anchor history packet is in sequence with the received packet record of the prevous anchor. Making sure the tag and the anchor 
    //whose packet we just received are in sync and neither has missed a packet from the previous anchor
    const bool seqNumInTagGood = _seqNumConsecutive(history[currAnchorID].packet.seqNums[previousAnchorID], packet->seqNums[previousAnchorID]);
    //Check to see if the history of this anchor the tag is receiving from is in sequence with the packet that has just been received.
    //Making sure the tag hasn't dropped any packets from the anchor it just recived from.
    const bool seqNumInAnchorGood = _seqNumConsecutive(history[currAnchorID].packet.seqNums[currAnchorID], packet->seqNums[currAnchorID]);

    // If the anchor or tag has missed a packet, this algorithm won't work, return false.
    if (! (seqNumInAnchorGood && seqNumInTagGood)){
        // Serial.print("Tag sequence number: ");
        // Serial.print(seqNumInTagGood);
        // Serial.print(" Anchor sequence number: ");
        // Serial.println(seqNumInAnchorGood);

        // Serial.println("Sequence numbers are out of order.");
        return false;
    }

    // The reference anchor refers to the previous anchor
    // The time (in tag ticks) when the current packet was received by the tag from anchor N
    const int64_t tagReceptionTimeOfCurrentPacketFromCurrAnchorNInTagTime = arrival;
    // The time that the anchor N received the reference anchor packet in the clock of anchor N
    const int64_t timeWhenAnchorNRXReferenceAnchorPacketInAnchorNClock = packet->timestamps[previousAnchorID];
    // The time of flight distance betweeen the reference anchor and anchor N
    const int64_t timeOfFlightofReferenceAnchortoAnchorNinAnchorNClk = packet->distances[previousAnchorID];
    // the previously calculated clock correction
    const double clockCorrection = history[currAnchorID].clockCorrectionTtoA;

    const bool anchorDistanceOk = (timeOfFlightofReferenceAnchortoAnchorNinAnchorNClk != 0);
    const bool rxTimeInTagOk = (timeWhenAnchorNRXReferenceAnchorPacketInAnchorNClock != 0);
    const bool CCok = (clockCorrection != 0.0);

    //If the anchor distance, receive time, or clock correction are zero, return false
    if(!(anchorDistanceOk && rxTimeInTagOk && CCok)){
        Serial.println("Anchor distance or rx time in tag or CC is zero.");
        return false;
    }

    const int64_t timeWhenAnchorNTransmittedInAnchorNClock = packet->timestamps[currAnchorID];
    const int64_t timeWhenTagRecievedReferenceAnchorPacketInTagClock = history[previousAnchorID].arrival.getTimestamp();

    //Equation 16 latter part (_0T_i^TX - _0T_j^TX)
    const int64_t deltaBetweenRefAnchorTXandAnchorNTXinAnchorNClk = (
                                                                    timeOfFlightofReferenceAnchortoAnchorNinAnchorNClk + 
                                                                    _truncateToAnchorTimestamp(
                                                                        timeWhenAnchorNTransmittedInAnchorNClock - 
                                                                        timeWhenAnchorNRXReferenceAnchorPacketInAnchorNClock
                                                                        )
                                                                    );

    //Equation 16 all together
    const int64_t timeDiffOfArrivalInAnchorNClk = _truncateToAnchorTimestamp(
                                                   tagReceptionTimeOfCurrentPacketFromCurrAnchorNInTagTime - 
                                                   timeWhenTagRecievedReferenceAnchorPacketInTagClock
                                                ) * clockCorrection -
                                                deltaBetweenRefAnchorTXandAnchorNTXinAnchorNClk;

    *distDiffTDoA = SPEED_OF_LIGHT * timeDiffOfArrivalInAnchorNClk / LOCODECK_TS_FREQ;

    return true;

}



/**
 * Tdoa2 Algorithm constructor. Initializes tag addresses to the proper values
 */
Tdoa2Alg::Tdoa2Alg(){
    systemConf.anchorAddress[0] = 0xbccf000000000000;
    systemConf.anchorAddress[1] = 0xbccf000000000001;
    systemConf.anchorAddress[2] = 0xbccf000000000002;
    systemConf.anchorAddress[3] = 0xbccf000000000003;
    systemConf.anchorAddress[4] = 0xbccf000000000004;
    systemConf.anchorAddress[5] = 0xbccf000000000005;
#if LOCODECK_NUM_ANCHORS > 6
    systemConf.anchorAddress[6] = 0xbccf000000000006;  
#endif
#if LOCODECK_NUM_ANCHORS > 7
    systemConf.anchorAddress[7] = 0xbccf000000000007;
#endif
    systemConf.anchorPosOk = false; 
}


/**
 * Updates the history of the anchor whose packet was just received.
 * @param packet Range packet that was just received
 * @param arrivalTime The time, in local tag ticks, that the packet arrived
 * @param anchorID The ID of the anchor that sent the last received packet
 */
void Tdoa2Alg::updateHistory(const locoRange2_t* packet, int64_t arrivalTime, uint8_t anchorID){
    memcpy(&history[anchorID].packet, packet, sizeof(locoRange2_t));
    history[anchorID].arrival = arrivalTime;
}


/**
 * A debugging function that prints out the clock correction calculations
 */
void Tdoa2Alg::printCC(){
    for(int i = 0; i < LOCODECK_NUM_ANCHORS; i++){
        Serial.print("Anchor ");
        Serial.print(i);
        Serial.print(" CC: ");
        Serial.println(history[i].clockCorrectionTtoA, 10);
    }
}


/**
 * Determine if the current sequence number is in number order with the previously received sequence number.
 * @param prevSeq Previous sequence number
 * @param currSeq Sequence number of the most recently received value
 * @return True if the current sequence number equals the previous sequence number + 1
 */
bool Tdoa2Alg::_seqNumConsecutive(uint8_t prevSeq, uint8_t currSeq){
    // Serial.print("Previous: "); Serial.print(prevSeq); Serial.print(" Current: "); Serial.println(currSeq);
    bool good = (currSeq == (prevSeq + 1)) ? true : false;
    // Serial.print("Returning "); Serial.println(good);
    return (currSeq == (prevSeq + 1)) ? true : false;
}


uint64_t Tdoa2Alg::_truncateToAnchorTimestamp(uint64_t timeStamp){
    return timeStamp & 0x00FFFFFFFFul;
}


uint64_t Tdoa2Alg::_truncateToLocalTimestamp(uint64_t timeStamp){
    return timeStamp & 0x00FFFFFFFFul;
}




/**
 * Calculate the necessary clock correction between tag and anchor for the
 * TDoA measurement. Implementation of equation 14 in the paper listed in See Also.
 * 
 * The equation (14) implemented here is not actually the same as the equation listed in the
 * paper. Note the omission of the subtraction of one from the equation. This is because
 * equation (16) in the paper listed in See Also adds one to this term. Because the math
 * cancels out, and we are not using this value in any other equation, saving an operation
 * is worth omitting the cancelled values.
 * 
 * @param CC The history value that will store the new clock correction
 * @param anchorID The anchor ID of the anchor that sent the packet we are performing calculations on
 * @param packet The packet that we just received
 * @param arrivalTime The time, in ticks, that packet arrived at. I'm fairly sure this is the local tick and not the anchor tick.
 * @return A boolean value that indicates whether this data is valid
 * @see <a href="http://mikehamer.info/assets/papers/Ledergerber,%20Hamer,%20DAndrea%20-%20IROS%202015.pd">
 *          A Robot Self-Localization System using One-Way Ultra-Wideband Communication
 *      </a>
 */
bool Tdoa2Alg::calcClockCorrection(double* CC, uint8_t anchorID, const locoRange2_t* packet, int64_t arrivalTime){

    //Make sure the packets that are being used are in sequence. If not we cannot use this
    //As a clock synchronization calculation. That is ok, as long as we eventually get some type of
    //synchronization eventually
    if(!_seqNumConsecutive(history[anchorID].packet.seqNums[anchorID], packet->seqNums[anchorID]))
        return false;

    // For the just-received packet
    // The time that the packet from anchor N was received (tag clock ticks)
    const int64_t receivedAnchorNPacketByTaginTagClkTicks = arrivalTime;
    // The time that anchor N transmitted the packet (anchor clock ticks)
    const int64_t timeWhenAnchorNTransmitsInAnchorNClkTicks = packet->timestamps[anchorID];
    
    // For the previous packet from Anchor N:
    // The time that the previous packet from anchor N was received (tag clock ticks)
    const int64_t lastReceivedAnchorPacketByTaginTagClkTicks = history[anchorID].arrival.getTimestamp();
    // The time that anchor N tranmitted the previous packet (anchor clock ticks)
    const int64_t lastTimeWhenAnchorNTransmitsInAnchorNClkTicks = history[anchorID].packet.timestamps[anchorID];


    // The Bitcraze implementation truncates these values for some reason. Maybe optimization? I posted on the forum
    // asking why they do that, so I will change it if their reasoning is sound or it is absolutely needed
    // The numerator of the equation (14)
    const double frameTimeAnchorClk = _truncateToAnchorTimestamp(timeWhenAnchorNTransmitsInAnchorNClkTicks - lastTimeWhenAnchorNTransmitsInAnchorNClkTicks);
    // The denominator of the equation (14)
    const double frameTimeTagClk = _truncateToLocalTimestamp(receivedAnchorNPacketByTaginTagClkTicks - lastReceivedAnchorPacketByTaginTagClkTicks);

    // const double frameTimeAnchorClk = (timeWhenAnchorNTransmitsInAnchorNClkTicks - lastTimeWhenAnchorNTransmitsInAnchorNClkTicks);
    // const double frameTimeTagClk = (receivedAnchorNPacketByTaginTagClkTicks - lastReceivedAnchorPacketByTaginTagClkTicks);
    *CC = frameTimeAnchorClk / frameTimeTagClk;
    return true;
}



/**
 * A function to handle the short data after the ranging packet. Every LPS packet carries an LPP short after the ranging data.
 * This LPP short contains the x, y, z position of the anchor that sent it. This function updates the anchor positions 
 * with this information.
 * @param datalen Length of the full received packet
 * @param packet Received packet
 */
void Tdoa2Alg::tdoaHandleLPPshort(uint8_t datalen, loco_packet_t* packet){

    //Find the length of the payload data
    const int32_t payloadLength = datalen - MAC802154_HEADER_LENGTH;
    const int32_t startOfLppDataInPayload = LPS_TDOA2_LPP_HEADER;
    const int32_t lppDataLen = payloadLength - startOfLppDataInPayload;
    // Serial.print("LppDataLen: "); Serial.println(lppDataLen);
    if(lppDataLen > 0){
        //Get the lpp header in the packet we received
        const uint8_t lppPacketHeader = packet->payload[LPS_TDOA2_LPP_HEADER];
        //If the lpp packet starts with 0xF0
        // Serial.print("LppPacketHeader (should be 0xF0): ");
        // Serial.println(lppPacketHeader, HEX);
        if(lppPacketHeader == LPP_HEADER_SHORT_PACKET){
            
            int8_t srcId = -1;

            for(int8_t i = 0; i < LOCODECK_NUM_ANCHORS; i++){
               if(packet->sourceAddress == systemConf.anchorAddress[i]){
                   //Get the anchor number
                   srcId = i;
               } 

            }
            // Serial.print("SrcID (>=0): ");
            // Serial.println(srcId);
            // Serial.print("LPS TDOA Header is located at: ");
            // Serial.println(LPS_TDOA2_LPP_HEADER);
            // Serial.print("Payload: ");
            // for(int i =0; i < lppDataLen; i++){
            //     Serial.println((packet->payload)[LPS_TDOA2_LPP_HEADER + i], HEX);
            // }
            // Serial.println();
            if(srcId >= 0){
                // update the anchor position.
                const uint8_t* data = (uint8_t*)&((packet->payload)[LPS_TDOA2_LPP_TYPE]);
                // Serial.print("Data(should be 0x01): ");
                // Serial.println(data[0], HEX);
                if(data[0] == LPP_SHORT_ANCHORPOS){
                    anchorPos_t *position = (anchorPos_t*)&(data[1]);
                    systemConf.anchorPos[srcId].x = position->x;
                    systemConf.anchorPos[srcId].y = position->y;
                    systemConf.anchorPos[srcId].z = position->z;
                    systemConf.anchorPosOk = true;
                }
            }
        }
    }
}


/**
 * A function to print a 3d point to the serial monitor
 * @param point 3d point to print
 */
void print_point3(point3 point){
    Serial.print("(");
    Serial.print(point.x);
    Serial.print(", ");
    Serial.print(point.y);
    Serial.print(", ");
    Serial.print(point.z);
    Serial.println(")");
}


/**
 * A debug function to display the known anchor positions over the serial monitor
 */
void Tdoa2Alg::printAnchorPos(){
    for(int i = 0; i < LOCODECK_NUM_ANCHORS; i++){
        point3 pos = systemConf.anchorPos[i];
        Serial.print("Anchor "); Serial.print(i); Serial.print(": ");
        print_point3(pos);
    }
}