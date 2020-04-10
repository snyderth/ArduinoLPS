# Arduino LPS

An Arduino Library to use the [Bitcraze LPS tag](https://www.bitcraze.io/products/loco-positioning-deck/) on an Arduino microcontroller.

NOTE: This library depends on the DW1000 arduino library from [thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000) with an added LED blink function. The new library is found [snyderth/DW1000](https://github.com/snyderth/DW1000)

## Usage

The LPS tag uses SPI with an external interrupt to trigger the communication to an external microcontroller. To use this library, import the `ArduinoLPS.h` header file:
```
#include <ArduinoLPS.h>
```

The SPI pins (MISO, MOSI, SCK) are hardware dedicated pins that are specific to the microcontroller. Please reference the schematic for the [Loco Deck](https://wiki.bitcraze.io/_media/projects:lps:loco_deck_revd.pdf) and a generic Arduino UNO.

![Arduino UNO schematic](images/arduino-schema.png)

The interrrupt pin is also hardware-specific. Whichever microcontroller you are using, you must use an external interrupt pin.

Other pins (reset, chip/slave select) are user-defined and should be passed to the LocoDeck object configure function.

To use the LocoDeck class:
```
LocoDeck lpsTag;

...

setup(){
    ...
    
    lpsTag.configure(
        <RESET_PIN>,
        <CS_PIN>,
        <INTERRUPT_PIN>,
        <MODE>,
        <CHANNEL>,
        <PREAMBLE>,
        <SMART_POWER bool>
    );
    ...

    lpsTag.registerRXCallback(<FUNCTION POINTER>);
    lpsTag.start();
}
```

That's it! For proper data fetching, look at the `receiveCBtdoa()` function in `examples/tdoaTag/tdoaTag.ino`. This should be in the RX callback that is registered by the `registerRXCallback()`.

