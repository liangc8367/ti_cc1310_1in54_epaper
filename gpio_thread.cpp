/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
#include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header file */
#include "Board.h"

#include "epd1in54b.h"
#include "imagedata.h"
#include "epdpaint.h"

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON0.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle an LED */
    //GPIO_toggle(Board_GPIO_LED0);

    uint_fast8_t v = GPIO_read(index);
    GPIO_write(Board_GPIO_LED0, !v);
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON1.
 *  This may not be used for all boards.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Clear the GPIO interrupt and toggle an LED */
  //  GPIO_toggle(Board_GPIO_LED1);

    uint_fast8_t v = GPIO_read(index);
    GPIO_write(Board_GPIO_LED1, !v);
}

#define MSGSIZE 16

#define COLORED      1
#define UNCOLORED    0

// liangc, workaround
unsigned char frame_black[200*200/8];
unsigned char frame_red[200*200/8];

void demo(Epd &epd)
{
//    unsigned char* frame_black = (unsigned char*)malloc(epd.width * epd.height / 8);
//    unsigned char* frame_red = (unsigned char*)malloc(epd.width * epd.height / 8);

    Paint paint_black(frame_black, epd.width, epd.height);
    Paint paint_red(frame_red, epd.width, epd.height);
    paint_black.Clear(UNCOLORED);
    paint_red.Clear(UNCOLORED);

    /* Draw something to the frame buffer */
    /* For simplicity, the arguments are explicit numerical coordinates */
    paint_black.DrawRectangle(10, 60, 50, 110, COLORED);
    paint_black.DrawLine(10, 60, 50, 110, COLORED);
    paint_black.DrawLine(50, 60, 10, 110, COLORED);
    paint_black.DrawCircle(120, 80, 30, COLORED);
    paint_red.DrawFilledRectangle(10, 130, 50, 180, COLORED);
    paint_red.DrawFilledRectangle(0, 6, 200, 26, COLORED);
    paint_red.DrawFilledCircle(120, 150, 30, COLORED);

    /*Write strings to the buffer */
    paint_black.DrawStringAt(30, 30, "e-Paper Demo", &Font16, COLORED);
    paint_red.DrawStringAt(28, 10, "Hello world!", &Font16, UNCOLORED);

    /* Display the frame buffer */
    epd.DisplayFrame(frame_black, frame_red);

    /* Display the image buffer */
    //epd.DisplayFrame(IMAGE_BLACK, IMAGE_RED);
}

/*
 *  ======== mainThread ========
 */
extern "C" void *mainThread(void *arg0)
{
    /* 1 second delay */
   // uint32_t time = 1;

    /* Call driver init functions */
    GPIO_init();
    // I2C_init();
    // SDSPI_init();

    // SPI_init();
    // UART_init();
    // Watchdog_init();

    /* install Button callback */
    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);

    // fix default config, enable both edge interrupt
    GPIO_PinConfig config;
    GPIO_getConfig(Board_GPIO_BUTTON0, &config);
    config |= GPIO_CFG_IN_INT_BOTH_EDGES;
    GPIO_setConfig(Board_GPIO_BUTTON0, config);

    /* Enable interrupts */
    GPIO_enableInt(Board_GPIO_BUTTON0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on Board_GPIO_BUTTON1.
     */
    if (Board_GPIO_BUTTON0 != Board_GPIO_BUTTON1) {

        GPIO_PinConfig config;
        GPIO_getConfig(Board_GPIO_BUTTON1, &config);
        config |= GPIO_CFG_IN_INT_BOTH_EDGES;
        GPIO_setConfig(Board_GPIO_BUTTON1, config);

        /* Install Button callback */
        GPIO_setCallback(Board_GPIO_BUTTON1, gpioButtonFxn1);
        GPIO_enableInt(Board_GPIO_BUTTON1);
    }

    /* Turn on user LED */
//    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
//
//    while (1) {
//        sleep(time);
//        GPIO_toggle(Board_GPIO_LED0);
//    }

#if 0
    // toggle MY SPI CS
    while(1) {
        sleep(1);
        GPIO_toggle(MY_GPIO_EPAPER_DC);
    }
#endif

    SPI_init();

    Epd epd;
    if (epd.Init() != 0) {
        while(1);
    }

//    epd.DisplayFrame(IMAGE_BLACK, IMAGE_RED);

    demo(epd);

#if 0
    SPI_Handle      spi;
    SPI_Params      spiParams;
    SPI_Transaction spiTransaction;
    uint8_t         transmitBuffer[MSGSIZE];
    uint8_t         receiveBuffer[MSGSIZE];
    bool            transferOK;

    SPI_init();  // Initialize the SPI driver
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.dataSize = 8;       // 8-bit data size

    spi = SPI_open(Board_SPI0, &spiParams);
    if (spi == NULL) {
        while (1);  // SPI_open() failed
    }

    // Fill in transmitBuffer
    memset(receiveBuffer, 0, sizeof(receiveBuffer));
    int i;
    for(i = 0; i< sizeof(transmitBuffer); i++){
        transmitBuffer[i] = i;
    }

    while(1) {
        spiTransaction.count = MSGSIZE;
        spiTransaction.txBuf = transmitBuffer;
        spiTransaction.rxBuf = receiveBuffer;
        transferOK = SPI_transfer(spi, &spiTransaction);


        if (!transferOK) {
            // Error in SPI or transfer already in progress.
            while(1);
        }
        sleep(1);
    }
#endif


    return (NULL);
}
