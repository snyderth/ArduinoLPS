#include <ArduinoLPS.h>

// Pinout
const uint8_t RST = 9;
const uint8_t IRQ = 2;
const uint8_t CS = 8;
const uint8_t OE = 3; // For logic-level shifter OE

TDOATag txTag;
LocoDeck lpsTag;
Tdoa2Alg tdoa2;


/**
 * Receive callback for a tdoa algorithm
 */
void receiveCBtdoa(){
     DW1000.blinkAllLEDs();
    uint16_t datalen;
    loco_packet_t data;
    lpsTag.getRXData(&datalen, &data);
    
    uint8_t anchorID = data.sourceAddress & 0xff;
    
    DW1000Time recvTime;
    DW1000.getReceiveTimestamp(recvTime);

    const locoRange2_t* range = (locoRange2_t*)(data.payload);

    if(anchorID < LOCODECK_NUM_ANCHORS){

        tdoa2.calcClockCorrection(&(tdoa2.history[anchorID].clockCorrectionTtoA),
                                  anchorID, 
                                  range, 
                                  recvTime.getTimestamp());

        if(anchorID != tdoa2.prevAnchorID){
            float tdoaDistDiff = 0.0;
            if(tdoa2.calcTDoADistDiff(&tdoaDistDiff, tdoa2.prevAnchorID, anchorID, range, recvTime.getTimestamp())){
                // Serial.print("Time difference of arrival measurement between anchor ");
                if(((tdoa2.prevAnchorID + 1) & 0x07) == anchorID){
                    // Serial.print("TDOA,");
                    // Serial.print(anchorID);
                    // Serial.print(" and anchor ");
                    // Serial.print(",");
                    // Serial.print(tdoa2.prevAnchorID);
                    // Serial.print(",");
                    // Serial.println(tdoaDistDiff, 15);
 
                }

                tdoaMeasurement_t tdoaMeas;
                memcpy(tdoaMeas.anchorPos + 0, tdoa2.systemConf.anchorPos + tdoa2.prevAnchorID, sizeof(point3));
                memcpy(tdoaMeas.anchorPos + 1, tdoa2.systemConf.anchorPos + anchorID, sizeof(point3));
                tdoaMeas.distanceDiff = tdoaDistDiff;
                tdoaMeas.stdDev = MEASUREMENT_NOISE_STD;
                // Serial.print("distanceDiff between Anchor ");
                // Serial.print(anchorID);
                // Serial.print(" and Anchor ");
                // Serial.print(tdoa2.prevAnchorID, 15);
                // Serial.print(" is ");
                // Serial.println(tdoaDistDiff);
                // Queue.enqueue((void*)&tdoaMeas);
                
                /* Send the tdoa measurement over uart */
                txTag << tdoaMeas;
            }

        }


    if(((tdoa2.prevAnchorID + 1) & 0x07) == anchorID){
        // Serial.print(anchorID);
        // Serial.print(" and anchor ");
        // Serial.print(",");
        // Serial.println(tdoa2.prevAnchorID);
        tdoa2.updateHistory(range, recvTime.getTimestamp(), anchorID);
        tdoa2.prevAnchorID = anchorID;
    }
        //For updating anchor positions
        tdoa2.tdoaHandleLPPshort(datalen, &data);
    }
}

void rxTimeout(){
    lpsTag.restart(RST, CS, IRQ, DW1000.MODE_SHORTDATA_FAST_ACCURACY, DW1000.CHANNEL_2, DW1000.PREAMBLE_CODE_64MHZ_9, true);
}

void rxFail(){
    lpsTag.restart(RST, CS, IRQ, DW1000.MODE_SHORTDATA_FAST_ACCURACY, DW1000.CHANNEL_2, DW1000.PREAMBLE_CODE_64MHZ_9, true);
}


void setup(){
    txTag.initialize();
    pinMode(OE, OUTPUT);
    digitalWrite(OE, HIGH);


    lpsTag.configure(RST, 
                     CS, 
                     IRQ, 
                     DW1000.MODE_SHORTDATA_FAST_ACCURACY, 
                     DW1000.CHANNEL_2, 
                     DW1000.PREAMBLE_CODE_64MHZ_9, 
                     true);

    lpsTag.registerRXCallback(receiveCBtdoa);
    lpsTag.registerRXTimeoutCallback(rxTimeout);
    lpsTag.registerRXFailCallback(rxFail);

    lpsTag.start();


}


long startTime = millis();
void loop(){
    lpsTag.mainLoop();

    /* NOTE: tag is interrupt driven => mainloop is free to do any other work */

    /* Hearbeat LED and freeze recovery */
    if(millis() - startTime > 100){
        // lpsTag.printRegister(dwRegister::PMSC_LEDC_REG);
        // DW1000.blinkAllLEDs();
        // tdoa2.printCC();
        // Serial.println("Running");

        // tdoa2.printAnchorPos();

        byte ledc_reg[LEN_PMSC_LEDC];
        memset(ledc_reg, 0, LEN_PMSC_LEDC);
        DW1000.readBytes(PMSC, PMSC_LEDC_SUB, ledc_reg, LEN_PMSC_LEDC);
        if((ledc_reg[1] & 0b00000001) != 1){
            //Detect when the tag has stopped and restart it.
            // lpsTag.restart();
            lpsTag.restart(RST, CS, IRQ, DW1000.MODE_SHORTDATA_FAST_ACCURACY, DW1000.CHANNEL_2, DW1000.PREAMBLE_CODE_64MHZ_9, true);
            // Serial.println("restarted");
        }
        startTime = millis();
        // Serial.println("Blink");
    }
}
