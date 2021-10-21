//Matthew Clockel
//Southern New Hampshire University
//Professor Pettit
//29 September 2021

/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <stdio.h>
/* Timer header file */
#include <ti/drivers/Timer.h>
/* Driver configuration */
#include "ti_drivers_config.h"

//The morse code for OK message.
enum MorseCode OK[11] = {DAHS, SPACES, DAHS, SPACES, DAHS, SPACE_BETWEEN, DAHS, SPACES, DITS, SPACES, DAHS};
//The morse code for SOS message
enum MorseCode SOS[17]={DITS, SPACES, DITS, SPACES, DITS, SPACE_BETWEEN, DAHS, SPACES, DAHS, SPACES, DAHS, SPACE_BETWEEN, DITS, SPACES, DITS, SPACES, DITS};
/* State machine declarations */
enum MorseCode_Cycles {START, MESSAGE_SOS, MESSAGE_OK, STOP}; // The cycle to start the message and stop.
enum MorseCode {DITS, DAHS, SPACES, SPACE_BETWEEN}; // Short burst light Dits and long burst of light Dats with space between.
enum Modes {MODE_SOS, MODE_OK}; // This switch from sos and ok modes.
enum MorseCode_Cycles MorseCode_Cycle; //Handler for cycles
enum Modes Mode; // Handler for mode.
int MorseCodeLoop = 0;

void gpioButtonFxn(uint_least8_t index)
{
  // This will change between different modes.
    Mode = Mode == MODE_SOS ? MODE_OK : MODE_SOS;
}

int i = 0;

void morseCodeMessage(enum MorseCode message[], int length){
    enum MorseCode output;
    // if else statement to stop led or output message.
    if (i > length - 1) {
        MorseCode_Cycle = STOP;
        i = 0;
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        MorseCodeLoop = 7;
        return;
    }
    else {
        output = message[i];
        i++;
    }
//Switch cases
    switch(output) {
    // The red light are DITS and led on for 500ms.
    case DITS:
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        MorseCodeLoop = 1;
        break;
    // The green light are the DAHS and led on for 1500ms.
    case DAHS:
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        MorseCodeLoop = 3;
        break;
    //  The spaces turn both lights off for 500ms
    case SPACES:
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        MorseCodeLoop = 1;
        break;
    case SPACE_BETWEEN:
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        MorseCodeLoop = 3;
        break;
    default:
        break;
    }
}

void MorseCode_Loop() {
    if (MorseCodeLoop > 0) {
        MorseCodeLoop--;
        return;
    }
    else {
        switch(MorseCode_Cycle) {
        // Starts the message
            case START:
                // Mode to determine if SOS and OK.
                switch(Mode){
                    case MODE_OK:
                        MorseCode_Cycle = MESSAGE_OK;
                        morseCodeMessage(OK, sizeof(OK));
                        break;
                    case MODE_SOS:
                        MorseCode_Cycle = MESSAGE_SOS;
                        morseCodeMessage(SOS, sizeof(SOS));
                        break;
                }
                break;
            // Continues the full message
            case MESSAGE_OK:
                morseCodeMessage(OK, sizeof(OK));
                break;
            case MESSAGE_SOS:
                morseCodeMessage(SOS, sizeof(SOS));
                break;
            case STOP:
                MorseCode_Cycle = START;
                break;
        }
    }
}

// Timer call back
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    MorseCode_Loop();
}

// start timer
void initTimer(void)
{
 Timer_Handle timer0;
 Timer_Params params;

 Timer_init();
 Timer_Params_init(&params);
 // changed the period to 500000
 params.period = 500000;
 params.periodUnits = Timer_PERIOD_US;
 params.timerMode = Timer_CONTINUOUS_CALLBACK;
 params.timerCallback = timerCallback;

 timer0 = Timer_open(CONFIG_TIMER_0, &params);

 if (timer0 == NULL) {
 /* Failed to initialized timer */
 while (1) {}
 }

 if (Timer_start(timer0) == Timer_STATUS_ERROR) {
  /* Failed to start timer */
  while (1) {}
  }

}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    // This will begin the timer
    initTimer();
    return NULL;
}
