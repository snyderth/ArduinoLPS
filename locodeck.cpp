#include "include/locodeck.h"




void LocoDeck::mainLoop(){
    // Serial.println("Looping");
}





/**
 * Prints a given register. Currently only supports PMSC_LEDC_REG.
 * @todo Add more register support for easier debugging
 * @param reg Register to print. @see locodeck.h dwRegister enumeration for supported registers.
 * 
 */
void LocoDeck::printRegister(dwRegister reg){
    switch (reg)
    {
    case dwRegister::PMSC_LEDC_REG:
        byte regBuff[LEN_PMSC_LEDC];
        memset(regBuff, 0, LEN_PMSC_LEDC);
        DW1000.readBytes(PMSC, PMSC_LEDC_SUB, regBuff, LEN_PMSC_LEDC);
        for(int i = LEN_PMSC_LEDC; i  >= 0; i++){
            Serial.print(regBuff[i], HEX);
        }
        Serial.println("\n");
        break;
    
    case dwRegister::GPIO_CTRL_REG:

        break;
    default:
        Serial.println("[ERROR] No such register!");
        break;
    }
}


/**
 *  Returns a byte array that is dynamically allocated. It must
 *  be freed after use.
 *  @param datalen: Unsigned 16 bit integer length of data passed by reference
 *  @return A populated byte array whose length is specified by datalen param
 */
void LocoDeck::getRXData(uint16_t* datalen, loco_packet_t* data){
    *datalen = DW1000.getDataLength();
    // loco_packet_t* data = (loco_packet_t*)(malloc(sizeof(loco_packet_t)));
    // memset(data, 0, datalen);
    DW1000.getData((byte*)data, *datalen);
    // return data;
}


/**
 * Starts the tag in receive mode.
 * NOTE: later, if we wanted to be able to update the LPS nodes via the
 * ground robots instead of just the UAVs, we need to be able to transmit
 * This means the receive permanently should be set to false and after
 * every receive, the start function in LocoDeck should be called
 * @todo Set receive permanently to false and instead restart the receive every
 * time a packet is received so that we are able to transmit. This will allow the 
 * ground robots to change the LPS anchor positions.
 */
void LocoDeck::start(){
    //init receive code
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}


/**
 * Default constructor. Initialize all values to zero.
 */
LocoDeck::LocoDeck(){
    _channel = 0;
    _cs = 0;
    _irq = 0;
    _rst = 0;
    _preamble = 0;
    _smartPower = false;
}


/*
    Constructor with default values
*/
// LocoDeck::LocoDeck(uint8_t rst, uint8_t cs, uint8_t irq){

//     DW1000.begin(irq, rst);
//     DW1000.select(cs);
//     DW1000.enableLedBlinking();
// }

/*
    Constructor with user-defined values
*/ 
// LocoDeck::LocoDeck(const uint8_t rst, const uint8_t cs, const uint8_t irq, const byte mode[], const byte channel){
//     DW1000.begin(irq, rst);
//     DW1000.select(cs);
//     DW1000.enableLedBlinking();
//     configure(mode, channel);
// }


/**
 *  DW1000 rx callback wrapper function
 *  @param handler A function pointer to the user defined rx callback
 */
void LocoDeck::registerRXCallback(void (* handler)(void)){
    DW1000.attachReceivedHandler(handler);
}


/**
 *  DW1000 rx timout callback wrapper function
 *  @param handler A function pointer to the user defined rx timeout callback
 */
void LocoDeck::registerRXTimeoutCallback(void(*handler)(void)){
    DW1000.attachReceiveTimeoutHandler(handler);
}


/**
 * DW1000 rx fail callback wrapper function
 * @param handler Function pointer to the user defined rx fail callback
 */
void LocoDeck::registerRXFailCallback(void(*handler)(void)){
    DW1000.attachReceiveFailedHandler(handler);
}

/**
 * Configures the tag to the settings used in Bitcraze's LPS system.
 * @param rst Pin that is attached to the DWM1000 reset pin
 * @param cs SPI chip select pin
 * @param irq SPI interupt pin
 * @param mode The transmit and receive mode of the DWM1000. Mode for LPS (not long range) is MODE_SHORTDATA_FAST_ACCURACY. Modes are type byte and are as follows:
 *   - MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
 *   - MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
 *   - MODE_LONGDATA_FAST_LOWPOWER[]  = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
 *   - MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
 *   - MODE_LONGDATA_FAST_ACCURACY[]  = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
 *   - MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};
 * @param preamble Preamble code. Pre-defined code for the preamble length. Default for LPS is PREAMBLE_CODE_64MHZ_9. Available codes are of type byte and are as follows:
 *   - PREAMBLE_CODE_16MHZ_1 
 *   - PREAMBLE_CODE_16MHZ_2 
 *   - PREAMBLE_CODE_16MHZ_3 
 *   - PREAMBLE_CODE_16MHZ_4 
 *   - PREAMBLE_CODE_16MHZ_5 
 *   - PREAMBLE_CODE_16MHZ_6 
 *   - PREAMBLE_CODE_16MHZ_7 
 *   - PREAMBLE_CODE_16MHZ_8 
 *   - PREAMBLE_CODE_64MHZ_9 
 *   - PREAMBLE_CODE_64MHZ_10
 *   - PREAMBLE_CODE_64MHZ_11
 *   - PREAMBLE_CODE_64MHZ_12
 *   - PREAMBLE_CODE_64MHZ_17
 *   - PREAMBLE_CODE_64MHZ_18
 *   - PREAMBLE_CODE_64MHZ_19
 *   - PREAMBLE_CODE_64MHZ_20
 *
 * @param channel Defines the channel to listen on. Defaults to channel 2 for LPS. Available channels 1-7. Can be uint8_t of the channel number
 * @param smartPower Use smart TX power or not. See user manual system configuration register DIS_STXP bit. 
 * @see <a href="https://www.decawave.com/sites/default/files/resources/dw1000_user_manual_2.11.pdf">DW1000 User Manual</a>
 */
void LocoDeck::configure(const uint8_t rst, const uint8_t cs, const uint8_t irq, const byte mode[], const byte channel, const byte preamble, bool smartPower){
    DW1000.begin(irq, rst);
    DW1000.select(cs);
    DW1000.enableLedBlinking();
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setChannel(channel);
    DW1000.enableMode(mode);
    DW1000.setPreambleCode(preamble);
    DW1000.useSmartPower(smartPower);
    DW1000.commitConfiguration();
    
    for(int i = 0; i < 3; i++){
       _mode[0] = mode[0];
    }
        
    _preamble = preamble;
    _rst = rst;
    _cs = cs;
    _irq = irq;
    _channel = channel;
    _smartPower = smartPower;
}


/**
 * Sometimes the DWM1000 stops receiving things. The current fix is
 * to do a full system restart with this function.
 * @todo Understand why the DWM1000 intermittently seemingly resets itself (All registers reset to default. Observation based on the PMSC_LEDC register)
 */
void LocoDeck::restart(){
    DW1000.reset();
    configure(_rst, _cs, _irq, _mode, _channel, _preamble, _smartPower);
    DW1000.enableLedBlinking();
    start();
}


/**
 * Sometimes the DWM1000 stops receiving things. The current fix is
 * to do a full system restart with this function.
 * UPDATE: This behavior was caused due to the headers stretching out and 
 * periodically disconnecting due to this stretch.
 */
void LocoDeck::restart(uint8_t rst, uint8_t cs, uint8_t irq, const byte mode[], const byte channel, const byte preamble, bool smartPower){
    DW1000.reset();
    delay(1);
    configure(rst, cs, irq, mode, channel, preamble, smartPower);
    DW1000.enableLedBlinking();
    start();
}