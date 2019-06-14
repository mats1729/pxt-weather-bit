/**
* Mary West @ SparkFun Electronics 
* Ryan Mortenson https://github.com/ryanjmortenson
* Harry Fairhead @ IoT-Programmer 
* June 13, 2017
* https://github.com/sparkfun/pxt-weather-bit
*
* Development environment specifics:
* Written in Microsoft PXT
* Tested with a SparkFun weather:bit for micro:bit
*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions 
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/


#include "pxt.h"
#include <cstdint>
#include <math.h>


using namespace pxt;

// v0 backward compat support
#ifndef PXT_BUFFER_DATA
#define PXT_BUFFER_DATA(buffer) buffer->payload
#endif

namespace weatherbit {
    // 1-wire protocol timings in microseconds
#define INIT_HOLD0 600 /* min 480, max 960 */
#define INIT_WAIT 60 /* min 15, max 60 */
#define INIT_READ_AFTER 10 /* when to read the presence signal. it can be read 
                              at least for 60 us and at most for 240 us */
#define INIT_RX 600 /* min 480, includes INIT_WAIT and INIT_READ_AFTER */
#define WRITE0_HOLD0 70 /* > 60, < 120 */
#define WRITE1_HOLD0  2 /* > 1, << 15 */
#define WRITE_SLOT 60 /* min 60, includes WRITE?_HOLD0 */
#define READ_HOLD0 2 /* > 1, << 15 */
#define READ_AFTER 14 /* >> 1, max 15, includes READ_HOLD0 */
#define READ_SLOT 60 /* min 60, includes READ_HOLD0 */
#define SLOT_RECOVERY 1 /* min 1 */

// Miceobit operation times in us
#define IO_TIME 3 /* The time to read or write a pin. */
#define WAIT_DELAY 7 /* The number microseconds used by the wait function */

// Delay command. The function wait_us cannot delay less than about 8 us.
#define DELAY(period) wait_us(period - WAIT_DELAY)

// Temperature conversion time in ms
#define CONVERSION_TIME 750

// main() runs in its own thread in the OS
MicroBitPin P12=uBit.io.P12;
MicroBitPin P13= uBit.io.P13;
    
    int read_time;
    Timer tictoc;
    
    uint8_t init() {
        P12.setDigitalValue(0);
        DELAY(INIT_HOLD0);
        P12.setDigitalValue(1);
        DELAY(INIT_WAIT + INIT_READ_AFTER);
        int b = P13.getDigitalValue();
        DELAY(INIT_RX - (INIT_WAIT + INIT_READ_AFTER));
        return b; /* Should be zero */
    }

    void writeBit(int b) {
        if (b == 1) {
            P12.setDigitalValue(0);
            /* WRITE1_HOLD0 is too short for wait_us  */
            P12.setDigitalValue(1);
            DELAY(WRITE_SLOT - WRITE1_HOLD0);
        } else {
            P12.setDigitalValue(0);
            DELAY(WRITE0_HOLD0);
            P12.setDigitalValue(1);
        }
    }

    void writeByte(int byte) {
        int i;
        for (i = 0; i < 8; i++) {
            if (byte & 1) {
                writeBit(1);
            } else {
                writeBit(0);
            }
            byte = byte >> 1;
        }
    }

    int readBit() {
        volatile int i;
        P12.setDigitalValue(0);
        /* READ_HOLD0 is too short for wait_us */
        P12.setDigitalValue(1);
        DELAY(READ_AFTER);
        int b = P13.getDigitalValue();
        DELAY(READ_SLOT - READ_AFTER);
        return b;
    }

    int convert() {
        writeByte(0x44);
        wait_ms(CONVERSION_TIME);
    }

    int readByte() {
        int byte = 0;
        int i;
        tictoc.start();
        for (i = 0; i < 8; i++) {
            byte = byte | readBit() << i;
        };
        tictoc.stop();
        char buff[10];
        sprintf(buff, " %d us\n", tictoc.read_us());
        uBit.display.scroll(buff, 200);
        return byte;
    }

    //%
    int16_t soilTemp() {
        init();
        writeByte(0xCC);
        convert();
        init();
        writeByte(0xCC);
        writeByte(0xBE);
        int b1 = readByte();
        int b2 = readByte();

        int16_t temp = (b2 << 8 | b1);
        return temp * 100 / 16;
    }

    /*
    * Compensates the pressure value read from the register.  This done in C++ because
    * it requires the use of 64-bit signed integers which isn't provided in TypeScript
    */
    //%
    uint32_t compensatePressure(int32_t pressRegVal, int32_t tFine, Buffer compensation) {
        // Compensation Values
        uint16_t digP1;
        int16_t digP2;
        int16_t digP3;
        int16_t digP4;
        int16_t digP5;
        int16_t digP6;
        int16_t digP7;
        int16_t digP8;
        int16_t digP9;

        // Unpack the compensation data        
        auto ptr = PXT_BUFFER_DATA(compensation);
        memcpy((uint8_t *) &digP1, ptr + 0, 2);
        memcpy((uint8_t *) &digP2, ptr + 2, 2);
        memcpy((uint8_t *) &digP3, ptr + 4, 2);
        memcpy((uint8_t *) &digP4, ptr + 6, 2);
        memcpy((uint8_t *) &digP5, ptr + 8, 2);
        memcpy((uint8_t *) &digP6, ptr + 10, 2);
        memcpy((uint8_t *) &digP7, ptr + 12, 2);
        memcpy((uint8_t *) &digP8, ptr + 14, 2);
        memcpy((uint8_t *) &digP9, ptr + 16, 2);

        // Do the compensation
        int64_t firstConv = ((int64_t) tFine) - 128000; // Changed from 12800 to 128000 to get correct result.
        int64_t secondConv = firstConv * firstConv * (int64_t)digP6;
        secondConv = secondConv + ((firstConv*(int64_t)digP5)<<17);
        secondConv = secondConv + (((int64_t)digP4)<<35);
        firstConv = ((firstConv * firstConv * (int64_t)digP3)>>8) + ((firstConv * (int64_t)digP2)<<12);
        firstConv = (((((int64_t)1)<<47)+firstConv))*((int64_t)digP1)>>33;
        if (firstConv == 0) {
            return 0; // avoid exception caused by division by zero
        }
        int64_t p = 1048576-pressRegVal;
        p = (((p<<31)-secondConv)*3125)/firstConv;
        firstConv = (((int64_t)digP9) * (p>>13) * (p>>13)) >> 25;
        secondConv = (((int64_t)digP8) * p) >> 19;
        p = ((p + firstConv + secondConv) >> 8) + (((int64_t)digP7)<<4);
        return (uint32_t)p;
    }

    /*
    * calculates the Altitude based on pressure. 
    */
    //%    
    uint32_t calcAltitude(int32_t pressRegVal, int32_t tFine, Buffer compensation) {
       
        return 44330*(1-pow(((compensatePressure(pressRegVal, tFine, compensation)/25600)/1013.25), 0.1903));
    }
}
