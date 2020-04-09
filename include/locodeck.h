#ifndef __LOCO_DECK_H_
#define __LOCO_DECK_H_

#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>
#include <DW1000.h>
#include <DW1000Mac.h>
#include "lpp.h"
#include "generic_types.h"


/**
 * Register identification for the DW1000 hardware.
 * @todo Add more register names to the enum and possibly give this enumeration its
 * own file.
 */
enum dwRegister{
    PMSC_LEDC_REG,
    GPIO_CTRL_REG,
};



class LocoDeck{
    public:

        LocoDeck();
        void configure(uint8_t rst, uint8_t cs, uint8_t irq, const byte mode[]=DW1000.MODE_SHORTDATA_FAST_ACCURACY, const byte channel=DW1000.CHANNEL_2, const byte preamble=DW1000.PREAMBLE_CODE_64MHZ_9, bool smartPower=true);
        void mainLoop();
        void start();
        void printRegister(dwRegister reg);
        void registerRXCallback(void(*)(void));
        void registerRXTimeoutCallback(void(*)(void));
        void registerRXFailCallback(void(*)(void));
        void restart();
        void restart(uint8_t rst, uint8_t cs, uint8_t irq, const byte mode[]=DW1000.MODE_SHORTDATA_FAST_ACCURACY, const byte channel=DW1000.CHANNEL_2, const byte preamble=DW1000.PREAMBLE_CODE_64MHZ_9, bool smartPower=true);
        void getRXData(uint16_t* datalen, loco_packet_t* data);
    private:
        byte _mode[3];
        uint8_t _rst;
        uint8_t _cs;
        uint8_t _irq;
        byte _channel;
        byte _preamble;
        bool _smartPower;

        static void receiver();


};

#endif //__LOCO_DECK_H_