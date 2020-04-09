#ifndef _TDOATAG_H_
#define _TDOATAG_H_

#include <Arduino.h>
#include "generic_types.h"
#include <SoftwareSerial.h>

/**
 * A class to communicate with a tdoa tag over the UART interface
 */
class TDOATag{
    public:
        /* Constructors */
        TDOATag(){}
        TDOATag(void(*queueInterface)(tdoaMeasurement_t)){
            enqueueFunc_ = queueInterface;
        }
        
        /**
         * @brief Initializes the proper serial connection for the TDOA Tag.
         * NOTE: This function reinitializes the serial port!! If using Serial
         * for other things, DO NOTE INITIALIZE THE TDOATAG INTERFACE.
         * @param baud The new baudrate for serial communication.
         */
        void initialize(uint32_t baud=512000){
            Serial.end();
            Serial.begin(baud);
        }


        /**
         * @brief Initializes the serial connection with new baud and a function pointer
         * if not already set in constructor.
         * @param baud the new baudrate for serial communication.
         * @param enqueueFunc The new callback for adding to the tdoa queue
         */
        void initialize(void(*enqueueFunc)(tdoaMeasurement_t), uint32_t baud=512000){
            Serial.end();
            Serial.begin(baud);
            if(enqueueFunc != nullptr){
                enqueueFunc_ = enqueueFunc;
            }
        }


        /**
         * @brief A function to check whether the tdoaMeasurement has possibly arrived on the serial bus.
         * @return TRUE if the available bytes exceeds or equals the tdoaMeeasurement_t size.
         * @return FALSE otherwise.
         */
        bool hasMeasurement(){
            if(Serial.available() >= sizeof(tdoaMeasurement_t))
                return true;
            else
                return false;
        }

        /**
         * @brief Receives the data. The packet format for data reception follows:
         *      +----+-------------------+
         *      |'<' | tdoaMeasurement_t |
         *      +----+-------------------+
         * Byte:  0    1                X       X = sizeof(tdoaMeasurement_t) 
         * @return TRUE when data reception is complete
         * @return FALSE if data reception has failed
         */
        bool getMeasurement(){
            uint8_t byteCount = 0, loopCount = 0;
            
            bool rxComplete = false, rxInProgress = false;
            uint8_t rxByte = 0, rxBuff[sizeof(tdoaMeasurement_t)] = {0}; 

            while( Serial.available() > 0 && byteCount < sizeof(tdoaMeasurement_t) ){
                // Begin by reading the first byte
                rxByte = Serial.read();

                if(rxInProgress){
                    // Add to the rxBuffer
                    rxBuff[byteCount++] = rxByte;
                }
                else if(rxByte == '<'){/*Beginning delimiter*/
                    rxInProgress = true;
                }

                // end delim may be unnecessary if just counting
                // the bytes that should be received.
                // else if(rxByte == '>'){/*End delimiter*/
                //     rxInProgress = false;
                //     rxComplete = true;
                // }


                /* For breaking possible infinite loops */
                loopCount++;
                if(loopCount > 1000){
                    return false;
                }
            }

            tdoaMeasurement_t* rxData = (tdoaMeasurement_t*)rxBuff;

            /* Enqueue the data */
            enqueueFunc_(*rxData);
            return true;
        }


        /**
         * @brief Function for tdoa tag uC that sends the proper data over the serial bus.
         * Packet:
         *      +----+-------------------+
         *      |'<' | tdoaMeasurement_t |
         *      +----+-------------------+
         * Byte:  0    1                X       X = sizeof(tdoaMeasurement_t)
         * @param tdoa A tdoaMeasurement struct
         * @return 0 On success
         * @return -1 On failure due to write space unavailable
         */
        int8_t sendMeasurement(tdoaMeasurement_t tdoa){
            txUART((void*) &tdoa, sizeof(tdoa));
        }


        /**
         * @brief << operator overload for sending tdoaMeasurements.
         * @param tdoa Tdoa distance difference measurement.
         */
        void operator<< (const tdoaMeasurement_t tdoa){
            txUART((void*) &tdoa, sizeof(tdoa));
        }


        /**
         * @brief Transmit via UART
         * @param data A void pointer to the data to be transmitted
         * @param size Specifies the size of the data pointed to by data
         */ 
        int8_t txUART(const void* data, size_t size){
            if(Serial.availableForWrite() < size + 1){
                return -1;
            }
            unsigned char byteP;
            /* Write to serial */
            Serial.write('<'); // beginning delim
            for(byteP = 0; byteP < size; byteP++)
                Serial.write(((uint8_t*)data)[byteP]);

            return 0;
        }

    private:
        /* Function for queueing the data */
        void (*enqueueFunc_)(tdoaMeasurement_t);
};



#endif