/**
 *  @filename   :   epdif.cpp
 *  @brief      :   Implements EPD interface functions
 *                  Users have to implement all the functions in epdif.cpp
 *  @author     :   Yehui from Waveshare
 *
 *  Copyright (C) Waveshare     August 10 2017
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <unistd.h>
#include "ti/drivers/GPIO.h"
#include "ti/drivers/SPI.h"

#include "CC1310_LAUNCHXL.h"

#include "epdif.h"
//#include <spi.h>

SPI_Handle      spi;

EpdIf::EpdIf() {
};

EpdIf::~EpdIf() {
};

void EpdIf::DigitalWrite(int pin, int value) {
//    digitalWrite(pin, value);
    GPIO_write(pin, value);
}

int EpdIf::DigitalRead(int pin) {
//    return digitalRead(pin);
    return GPIO_read(pin);
}

void EpdIf::DelayMs(unsigned int delaytime) {
//    delay(delaytime);
    useconds_t us = (useconds_t) 1000 * delaytime;
    usleep(us);
}

void EpdIf::SpiTransfer(unsigned char data) {
//    digitalWrite(CS_PIN, LOW);
//    SPI.transfer(data);
//    digitalWrite(CS_PIN, HIGH);
    GPIO_write(MY_GPIO_SPI_CS, 0);

    SPI_Transaction spiTransaction;
    uint8_t         transmitBuffer;
    uint8_t         receiverBuffer;
    bool            transferOK;

    receiverBuffer = 0;
    transmitBuffer = data;

    spiTransaction.count = 1;
    spiTransaction.txBuf = &transmitBuffer;
    spiTransaction.rxBuf = &receiverBuffer;
    transferOK = SPI_transfer(spi, &spiTransaction);

    if (!transferOK) {
        // Error in SPI or transfer already in progress.
        while(1);
    }

    GPIO_write(MY_GPIO_SPI_CS, 1);
}

int EpdIf::IfInit(void) {

    /* initialization had already been completed by board initialzation code.
    pinMode(CS_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    pinMode(DC_PIN, OUTPUT);
    pinMode(BUSY_PIN, INPUT); 

    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    SPI.begin();
    */

    SPI_Params      spiParams;
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.dataSize = 8;       // 8-bit data size

    spi = SPI_open(CC1310_LAUNCHXL_SPI0, &spiParams);
    if (spi == NULL) {
        while (1);  // SPI_open() failed
    }

    return 0;
}

