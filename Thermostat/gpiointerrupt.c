//Matthew Clockel
//Southern New Hampshire University
//Professor Pettit
//13 October 2021

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

// Drivers
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
    DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
    DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

// Driver Handles - Global variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
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

enum States {
    Initial,
    LowerTemp,
    RaiseTemp,
} State;

unsigned char Button1;
unsigned char Button2;

// Thermostat functions
int16_t temperature;
int16_t setpoint;
int8_t heat;


void ButtonFunctions() {
    switch(State) {
    // Buttons in the not pressed state
        case Initial:
            Button1 = 0;
            Button2 = 0;
            State = LowerTemp;
            break;
    // If button1 or button2 is pressed
        case LowerTemp:
            if (Button1 || Button2) {
                State = RaiseTemp;
            }
            break;
    // If no button1 or button 2
        case RaiseTemp:
            if (!(Button1 || Button2)){
                State = LowerTemp;
            }
            break;
        default:
            State = Initial;
            break;
    }
    switch(State) {
        case Initial:
            break;
        case LowerTemp:
            break;
        case RaiseTemp:
            // Updates setpoint value
            if (Button1) {
                setpoint -= 1;  // setpoint temperature decreases by 1 degree C
                Button1 = 0;    // Lowers flag of button1
            }
            if (Button2) {
                setpoint += 1;  // setpoint temperature increase by 1 degree C
                Button2 = 0;    // Lowers flag of button2
            }
            break;
        default:
            break;
    }
}


enum SecondState {
    Initial2,
    AdjustTempOff,
    AdjustTempOn,
} State2;
// Switch case statement for led functions
void LED_Function() {
    switch(State2) {
        case Initial2:
            heat = 0;
            State2 = AdjustTempOff;
            break;
        case AdjustTempOff:
            // If statement temperature is less than the setpoint to adjust temperature turn heat on
            if (temperature < setpoint) {
                State2 = AdjustTempOn;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                heat = 1;
            }
            break;
        case AdjustTempOn:
            // If statement temperature is not less than setpoint is turned off
            if (!(temperature < setpoint)){
                State2 = AdjustTempOff;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                heat = 0;
            }
            break;
        default:
            State2 = Initial2;
            break;
    }

    switch(State2) {
        case Initial2:
            break;
        case AdjustTempOff:
            break;
        case AdjustTempOn:
            break;
        default:
            break;
    }
}

void gpioButtonFxn0(uint_least8_t index)
{
    // Button1 is equal to 1 which will mean the button has been pushed.
    Button1 = 1;
}

void gpioButtonFxn1(uint_least8_t index)
{
    // Button2 is equal to 1 which will mean the button has been pushed.
    Button2 = 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initI2C();
    initTimer();
    initUART();

    //  Timer set 1000000 microseconds
    const unsigned int timer = 100000;
    SetTimer(timer0, Timer_PERIOD_US, timer);

    // Check button set to 200ms
    unsigned long CheckButton = 200000;

    // Check temperature every 500ms
    unsigned long CheckTemperature = 500000;

    // UART output set to 1000ms
    unsigned long UART_Output = 1000000;

    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

   // Setpoint and temperature variables
    setpoint = readTemp();
    temperature = readTemp();


    // Two initial states
    State = Initial;
    State2 = Initial2;

    // Set seconds variable to zero
    int32_t seconds = 0;

    while(1){
        // If greater than or equal to 200ms to check button every 200ms
        if (CheckButton >= 200000) {
            ButtonFunctions();
            CheckButton = 0;
        }

        // If greater than or equal to 500ms statement to check temperature and update LED every 500ms
        if (CheckTemperature >= 500000) {
            temperature = readTemp();
            LED_Function();
            CheckTemperature = 0;
        }
        // If greater than or equal to 10000ms UART output to server formatted <AA,BB,S,CCC>
        if (UART_Output >= 1000000) {
            DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds))
            seconds += 1;
            UART_Output = 0;
        }


        while(!TimerFlag){} // Wait for timer period
        TimerFlag = 0;      // Lower flag raised by timer

        // Configured timers
        CheckButton += timer;
        CheckTemperature += timer;
        UART_Output += timer;
    }
}
