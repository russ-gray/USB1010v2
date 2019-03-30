/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    usb1010v2.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "usb1010v2.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

DRV_I2C_BUFFER_EVENT i2cOpStatus;

uint8_t         operationStatus;

/* I2C Driver TX buffer  */
uint8_t         TX_buffer[8] __attribute__((aligned(4)));

/* I2C successful write counter */
static uint32_t successCount = 0;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

USB1010V2_DATA usb1010v2Data;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
/* I2C master callback function - deactivated 1/6/19; seems to be deprecated
void I2CMasterOpStatusCb ( DRV_I2C_BUFFER_EVENT event,
                           DRV_I2C_BUFFER_HANDLE bufferHandle,
                           uintptr_t context)
{

    switch (event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            successCount++; // Increment success counter
            Nop();
            break;
        //case DRV_I2C_BUFFER_EVENT_ERROR:
        //    break;
        default:
            break;
    }
}
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

// Function to check I2C transfer status
DRV_I2C_BUFFER_EVENT APP_Check_Transfer_Status(DRV_HANDLE drvOpenHandle, DRV_I2C_BUFFER_HANDLE drvBufferHandle)
{
    //return (DRV_I2C_TransferStatusGet(usb1010v2Data.drvI2CHandle, drvBufferHandle));
    return (DRV_I2C_TransferStatusGet(drvOpenHandle, drvBufferHandle));
}

// Function to write I2C data to a specified address
bool writeI2Cdata(uint8_t address, uint8_t target, uint8_t payload)
{
    // Load new values to transmit buffer
    TX_buffer[0] = target; 
    TX_buffer[1] = payload;

    uint8_t errorCount = 0; // Set local error counter to zero
    
    WRITE: // This label allows the function to retry writing a set number of times
    usb1010v2Data.drvI2CTxRxBufferHandle[0] = DRV_I2C_Transmit(usb1010v2Data.drvI2CHandle,
                                                address,
                                                &TX_buffer[0],
                                                2,
                                                NULL);
    
    // Delay loop to ensure current write has finished    
    while ( (APP_Check_Transfer_Status(usb1010v2Data.drvI2CHandle, usb1010v2Data.drvI2CTxRxBufferHandle[0]) != DRV_I2C_BUFFER_EVENT_COMPLETE)
          && (APP_Check_Transfer_Status(usb1010v2Data.drvI2CHandle, usb1010v2Data.drvI2CTxRxBufferHandle[0]) != DRV_I2C_BUFFER_EVENT_ERROR) )
    { // Insert no-op
        Nop();
    }
            
    if (APP_Check_Transfer_Status(usb1010v2Data.drvI2CHandle, usb1010v2Data.drvI2CTxRxBufferHandle[0]) == DRV_I2C_BUFFER_EVENT_ERROR)
    { // Transmission error
        if (errorCount < MAX_WRITE_RETRIES)
        { // Maximum retry count not reached; increment error counter, retry write
            errorCount++;
            goto WRITE;
        }
        else
        { // Maximum retry count reached; return failure message
            return false;
        }
    }
    return true;
}

// Function to initialize a codec
bool codecInit(uint8_t codec)
{
    // Reset static success counter
    successCount = 0;
    // Configure multiplexer switch
    if (writeI2Cdata(MULT_ADDR, 0x00, codec)) successCount++;
    // Configure shutdown mode (global enable = off)
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x45, 0x00)) successCount++;
    // Configure system clock
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x1B, 0x10)) successCount++;
    // Configure interface format for 24-bit operation
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x22, 0x05)) successCount++;
    // Configure interface I/O
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x25, 0x03)) successCount++;
    // Configure left and right ADC levels
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x17, 0x03)) successCount++;
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x18, 0x03)) successCount++;
    // Configure filters for music playback - Seems unnecessary (12/15/18)
    //writeI2Cdata(CODEC_SLAVE_WR, 0x26, 0x80);
    // Configure ADC
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x44, 0x05)) successCount++;
    // Configure DAC
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x43, 0x01)) successCount++;
    // Configure input pair (default value)
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x0D, LINE_INPUT)) successCount++;
    //writeI2Cdata(CODEC_SLAVE_WR, 0x0D, PHONO_INPUT);
    // Configure left and right ADC inputs
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x15, 0x08)) successCount++;
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x16, 0x10)) successCount++;
    // Configure left line out mixer
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x37, 0x01)) successCount++;
    // Configure right line out mixer
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x3A, 0x82)) successCount++;
    // Configure input gain (default value)
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x0E, GAIN_FLAT)) successCount++;
    // Configure left line out for minimum volume
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x39, 0x00)) successCount++;
    // Configure right line out for minimum volume
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x3C, 0x00)) successCount++;
    // Configure shutdown mode (global enable = on)
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x45, 0x80)) successCount++;
    // Configure input and output enable
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x3E, 0x0F)) successCount++;
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x3F, 0x0F)) successCount++;
    // Configure left line out volume
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x39, 0x15)) successCount++;
    // Configure right line out volume
    if (writeI2Cdata(CODEC_SLAVE_WR, 0x3C, 0x15)) successCount++;
    
    if (successCount == INIT_WRITE_COUNT)
    {
        return true;
    }
    return false;
}

// Function to toggle a codec input
bool codecInputToggle(uint8_t codec, bool *currentMode)
{
    bool retVal = false;    // Return value variable
    
    if (writeI2Cdata(MULT_ADDR, 0x00, codec))
    { // Communication path to codec open
        switch (*currentMode)
        { // Write appropriate input selection to codec
            case LINE_LEVEL: 
            {
                retVal = writeI2Cdata(CODEC_SLAVE_WR, 0x0D, PHONO_INPUT);
                break;
            }
            case PHONO_LEVEL: 
            {
                retVal = writeI2Cdata(CODEC_SLAVE_WR, 0x0D, LINE_INPUT);
                break;
            }
        }
    }
    // If I2C write was successful, toggle the input mode
    if (retVal == true) *currentMode = !(*currentMode);
    return retVal;
}

// Function to calculate and return elapsed time
uint32_t getElapsedTime(uint32_t time)
{ // Modular arithmetic is used to handle timer wraparound case
    uint32_t elapsedTime = (MAIN_TIMER_PERIOD + time - usb1010v2Data.currTime) % MAIN_TIMER_PERIOD;
    return elapsedTime;
}

// Function to handle switch presses
bool switchPress(uint32_t time, bool *mode)
{ // Returns true if valid switch press is processed, false otherwise
    uint32_t swElapsedTime = getElapsedTime(time);
    bool retVal = true;
    
    if (swElapsedTime >= MIN_SWITCH_DELAY)
    { // Switch actuation de-bounced
        retVal = false;
        if (swElapsedTime >= GAIN_SWITCH_DELAY && swElapsedTime < MAX_SWITCH_DELAY)
        { // Switch actuation time-validated; handle switch press
            if (swElapsedTime >= THRU_SWITCH_DELAY)
            { // Long press for thru mode
                *mode = THRU_CHANGE;   
            }
            else 
            { // No long press implies short press for gain change
                *mode = GAIN_CHANGE;
            }
            // Set activation flag and state transition for valid switch press
            retVal = true;
            usb1010v2Data.state = USB1010V2_STATE_CHANGE_CHANNEL_MODE;
        }
    }
    return retVal;
}

// Function to implement channel gain and thru mode changes
bool channelChange(uint8_t codec, bool changeMode, bool *switchFlag, bool *thruMode, uint8_t *gainMode)
{ 
    // Configure multiplexer for correct codec
    if(writeI2Cdata(MULT_ADDR, 0x00, codec))
    {
        switch (changeMode)
        {
            bool success = false;   // Local success flag

            case THRU_CHANGE:
            { // Thru change
                if (*thruMode == THRU_ON)
                { // Thru mode currently on; return to digital audio interface
                    if (writeI2Cdata(CODEC_SLAVE_WR, 0x37, 0x01) && writeI2Cdata(CODEC_SLAVE_WR, 0x3A, 0x82))
                    { // Codec reconfigured; change mode
                        *thruMode = THRU_OFF;
                        success = true;
                    }
                }
                else if (*thruMode == THRU_OFF)
                { // Thru mode currently off; route inputs to outputs
                    if (writeI2Cdata(CODEC_SLAVE_WR, 0x37, 0x04) && writeI2Cdata(CODEC_SLAVE_WR, 0x3A, 0x88))
                    { // Codec reconfigured; change mode
                        *thruMode = THRU_ON;
                        success = true;
                    }
                }
                // If thru change was successful, turn off switch activation flag
                *switchFlag = !success;
            break;
            }

            case GAIN_CHANGE:
            { // Gain change

                // Calculate incremented gain mode
                uint8_t newGainMode = (*gainMode + 1) % 3;

                switch (newGainMode)
                { // Change gain to new selection
                    case GAIN_m3dB:
                    { // Write -3dB gain setting to codec
                        if (writeI2Cdata(CODEC_SLAVE_WR, 0x0E, GAIN_3_DN)) success = true;
                        break;
                    }
                    case GAIN_0dB:
                    { // Write 0dB gain setting to codec
                        if (writeI2Cdata(CODEC_SLAVE_WR, 0x0E, GAIN_FLAT)) success = true;
                        break;
                    }
                    case GAIN_p3dB:
                    { // Write +3dB gain setting to codec
                        if (writeI2Cdata(CODEC_SLAVE_WR, 0x0E, GAIN_3_UP)) success = true;
                        break;
                    }
                }

                if (success) 
                { // Successful change; update gain mode and turn off switch activation flag
                    *gainMode = newGainMode;
                    *switchFlag = !success;
                }
            break;
            }
        }
    }
}

// Function to test I2C communications (use for bring-up/debugging)
bool i2cTest()
{
    bool    goodComm = false,   // test  pass/fail flags
            badComm = false;
            
    // Valid communication test (to I/O expander)
    if (writeI2Cdata(EXPR_ADDR, 0x06, 0x00) && writeI2Cdata(EXPR_ADDR, 0x07, 0x00))
    { // All pins of I/O expander configured for output; turn green LEDs on
        if (writeI2Cdata(EXPR_ADDR, 0x02, 0x44) && writeI2Cdata(EXPR_ADDR, 0x03, 0x22))
        { // All green LEDs on; set good communication flag
            goodComm = true;
        }
    }
    
    // Invalid communication test (to un-assigned address)
    if (!writeI2Cdata(0x30, 0xDE, 0xAD))
    { // Write routine fails and returns after prescribed retries; set bad communication flag
        badComm = true;
    }
    
    // Return true only if both tests pass
    bool retVal = goodComm && badComm;
    return retVal;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void USB1010V2_Initialize ( void )

  Remarks:
    See prototype in usb1010v2.h.
 */

void USB1010V2_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    usb1010v2Data.state = USB1010V2_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    // Set all initialization status flags to false
    usb1010v2Data.ledInitialized = false;
    usb1010v2Data.i2cInitialized = false;
    usb1010v2Data.appInitialized = false;
    usb1010v2Data.mainTimerInitialized = false;
    usb1010v2Data.codecsInitialized = false;
    usb1010v2Data.smpsInitialized = false;
    usb1010v2Data.powerInitialized = false;
    
    // Set main timer handle to invalid
    usb1010v2Data.mainTimer = SYS_TMR_HANDLE_INVALID;
    
    // Set I2C test flag to false
    usb1010v2Data.i2cTested = false;
    
    // Set default configuration flags to false
    usb1010v2Data.codec1ok = false;
    usb1010v2Data.codec2ok = false;
    usb1010v2Data.codec3ok = false;
    usb1010v2Data.codec4ok = false;
    usb1010v2Data.ledsDefault = false;
    
    // Set input modes to default (line level), input toggle flags off
    usb1010v2Data.inputModeCh1 = LINE_LEVEL;
    usb1010v2Data.inputModeCh2 = LINE_LEVEL;
    usb1010v2Data.inputModeCh3 = LINE_LEVEL;
    usb1010v2Data.inputModeCh4 = LINE_LEVEL;
    usb1010v2Data.inputToggleCh1 = false;
    usb1010v2Data.inputToggleCh2 = false;
    usb1010v2Data.inputToggleCh3 = false;
    usb1010v2Data.inputToggleCh4 = false;
    
    // Set gain switch flags to false
    usb1010v2Data.gainSwCh1 = false;
    usb1010v2Data.gainSwCh2 = false;
    usb1010v2Data.gainSwCh3 = false;
    usb1010v2Data.gainSwCh4 = false;
    
    // Set gain states to default (0dB)
    usb1010v2Data.gainModeCh1 = GAIN_0dB;
    usb1010v2Data.gainModeCh2 = GAIN_0dB;
    usb1010v2Data.gainModeCh3 = GAIN_0dB;
    usb1010v2Data.gainModeCh4 = GAIN_0dB;
    
    // Set thru mode flags to off
    usb1010v2Data.thruModeCh1 = THRU_OFF;
    usb1010v2Data.thruModeCh2 = THRU_OFF;
    usb1010v2Data.thruModeCh3 = THRU_OFF;
    usb1010v2Data.thruModeCh4 = THRU_OFF;
}


/******************************************************************************
  Function:
    void USB1010V2_Tasks ( void )

  Remarks:
    See prototype in usb1010v2.h.
 */

void USB1010V2_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( usb1010v2Data.state )
    {
        /* Application's initial state. */
        case USB1010V2_STATE_INIT:
        {
            // Start heartbeat timer for debug mode
            if (DEBUG_MODE)
            { // Set timer period to defined heartbeat interval
                usb1010v2Data.hDelayTimer = SYS_TMR_DelayMS(HEARTBEAT_DELAY);
                if (usb1010v2Data.hDelayTimer != SYS_TMR_HANDLE_INVALID)
                { // Valid handle returned; turn LED on and set status flag
                    LED2On();
                    usb1010v2Data.ledInitialized = true;
                }
            }
            
            // Start main timer
            if (usb1010v2Data.mainTimer == SYS_TMR_HANDLE_INVALID)
            { // Timer handle invalid, start new timer
                usb1010v2Data.mainTimer = SYS_TMR_CallbackPeriodic(MAIN_TIMER_PERIOD, 0, NULL);
            }                              
            if (usb1010v2Data.mainTimer != SYS_TMR_HANDLE_INVALID)
            { // Valid handle returned; set status flag
                usb1010v2Data.mainTimerInitialized = true;
            }
            
            // Initialize I2C master driver
            usb1010v2Data.drvI2CHandle = DRV_I2C_Open(DRV_I2C_INDEX_0,DRV_IO_INTENT_READWRITE);
            if (usb1010v2Data.drvI2CHandle != (DRV_HANDLE) NULL)
            { // Valid handle returned; set status flag
                usb1010v2Data.i2cInitialized = true;
            }
            
            // Check for successful initialization conditions
            if (DEBUG_MODE)
            { // Debug mode activated; check for I2C, main timer, and heartbeat LED
                usb1010v2Data.appInitialized =  usb1010v2Data.i2cInitialized 
                                                && usb1010v2Data.mainTimerInitialized
                                                && usb1010v2Data.ledInitialized;
            }
            else
            { // Debug mode deactivated; check for I2C and main timer only
                usb1010v2Data.appInitialized =  usb1010v2Data.i2cInitialized
                                                && usb1010v2Data.mainTimerInitialized;
            }
            
            if (usb1010v2Data.appInitialized)
            { // Application successfully initialized; transition to service tasks state */
                    usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;
            }
            break;
        }

        /* Application main task servicing state */
        case USB1010V2_STATE_SERVICE_TASKS:
        {   
            // Get the current time
            usb1010v2Data.currTime = SYS_TMR_ObjectCountGet(usb1010v2Data.mainTimer, NULL);
            
            if (DEBUG_MODE)
            { // Debug mode activated; handle debug functions
                if(!usb1010v2Data.i2cTested && TEST_I2C)
                { // I2C untested; perform test
                    if (i2cTest())
                    { // I2C function validated; turn on blue success LED
                        LED1On();
                    }
                    else
                    { // I2C function not validated; turn on yellow failure LED
                        LED3On();
                    }
                    usb1010v2Data.i2cTested = true;
                }

                if (SYS_TMR_DelayStatusGet(usb1010v2Data.hDelayTimer)) 
                { // Single shot timer has now timed out; toggle green LED and transition to service tasks state
                     LED2Toggle();
                     usb1010v2Data.state = USB1010V2_STATE_RESTART_HB_TIMER;
                     break;
                }  
            }
            
            if (!usb1010v2Data.powerInitialized)
            { // Analog power not initialized; transition to analog power initialization state
                usb1010v2Data.state = USB1010V2_STATE_INITIALIZE_POWER;
                break;
            }
            
            if (!usb1010v2Data.codecsInitialized)
            { // Codecs not initialized; transition to codec initialization state
                usb1010v2Data.state = USB1010V2_STATE_INITIALIZE_CODECS;
                break;
            }
            
            // Ensure codecs are initialized before allowing switch operation
            if (usb1010v2Data.codecsInitialized)
            { 
                // Check phono/line switches for change requirements
                if (PHONO_SW1StateGet() != usb1010v2Data.inputModeCh1)
                {
                    usb1010v2Data.inputToggleCh1 = true;
                }
                if (PHONO_SW2StateGet() != usb1010v2Data.inputModeCh2)
                {
                    usb1010v2Data.inputToggleCh2 = true;
                }
                if (PHONO_SW3StateGet() != usb1010v2Data.inputModeCh3)
                {
                    usb1010v2Data.inputToggleCh3 = true;
                }
                if (PHONO_SW4StateGet() != usb1010v2Data.inputModeCh4)
                {
                    usb1010v2Data.inputToggleCh4 = true;
                }

                if (usb1010v2Data.inputToggleCh1 ||
                    usb1010v2Data.inputToggleCh2 ||
                    usb1010v2Data.inputToggleCh3 ||
                    usb1010v2Data.inputToggleCh4)
                { // Transition to input mode change state
                    usb1010v2Data.state = USB1010V2_STATE_CHANGE_INPUT_MODE;
                    break;
                }
            
                // Check for channel switch actuation
                if (!usb1010v2Data.gainSwCh1 && GAIN_SW1StateGet())
                { // Channel 1 gain switch actuated; set flag and record time
                    usb1010v2Data.gainSwCh1 = true;
                    usb1010v2Data.startTimeSW1 = usb1010v2Data.currTime;
                }

                if (!usb1010v2Data.gainSwCh2 && GAIN_SW2StateGet())
                { // Channel 2 gain switch actuated; set flag and record time
                    usb1010v2Data.gainSwCh2 = true;
                    usb1010v2Data.startTimeSW2 = usb1010v2Data.currTime;
                }

                if (!usb1010v2Data.gainSwCh3 && GAIN_SW3StateGet())
                { // Channel 3 gain switch actuated; set flag and record time
                    usb1010v2Data.gainSwCh3 = true;
                    usb1010v2Data.startTimeSW3 = usb1010v2Data.currTime;
                }

                if (!usb1010v2Data.gainSwCh4 && GAIN_SW4StateGet())
                { // Channel 4 gain switch actuated; set flag and record time
                    usb1010v2Data.gainSwCh4 = true;
                    usb1010v2Data.startTimeSW4 = usb1010v2Data.currTime;
                }
                
                // Debounce and process switch press data
                if (usb1010v2Data.gainSwCh1 && !GAIN_SW1StateGet())
                { // Channel 1 gain switch released; handle switch press
                    usb1010v2Data.gainSwCh1 = switchPress(usb1010v2Data.startTimeSW1, &usb1010v2Data.changeModeCh1);
                }
                
                if (usb1010v2Data.gainSwCh2 && !GAIN_SW2StateGet())
                { // Channel 2 gain switch released; handle switch press
                    usb1010v2Data.gainSwCh2 = switchPress(usb1010v2Data.startTimeSW2, &usb1010v2Data.changeModeCh2);
                }
                
                if (usb1010v2Data.gainSwCh3 && !GAIN_SW3StateGet())
                { // Channel 3 gain switch released; handle switch press
                    usb1010v2Data.gainSwCh3 = switchPress(usb1010v2Data.startTimeSW3, &usb1010v2Data.changeModeCh3);
                }
                
                if (usb1010v2Data.gainSwCh4 && !GAIN_SW4StateGet())
                { // Channel 4 gain switch released; handle switch press
                    usb1010v2Data.gainSwCh4 = switchPress(usb1010v2Data.startTimeSW4, &usb1010v2Data.changeModeCh4);
                }
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        case USB1010V2_STATE_INITIALIZE_POWER:
        { 
            if (!usb1010v2Data.smpsInitialized)
            { // SMPS not started; enable switching regulators
                EN_POSREGOn();
                EN_NEGREGOn();
                
                // Record SMPS start time
                usb1010v2Data.startTimeSMPS = usb1010v2Data.currTime;
                
                // Set SMPS initialization flag
                usb1010v2Data.smpsInitialized = true;
            }
            
            // Get elapsed time since SMPS startup as local variable. Modular arithmetic is used to handle timer wraparound.
            //uint32_t elapsedTime = (MAIN_TIMER_PERIOD + usb1010v2Data.startTimeSMPS - usb1010v2Data.currTime) 
            //                        % MAIN_TIMER_PERIOD;
            uint32_t elapsedSMPSTime = getElapsedTime(usb1010v2Data.startTimeSMPS);
                    
            if (usb1010v2Data.smpsInitialized && (elapsedSMPSTime >= SMPS_STARTUP_DELAY))
            { // SMPS section started; enable LDOs
                EN_POSLDOOn();
                EN_NEGLDOOn();
            
                if (DEBUG_MODE)
                { // Turn on yellow success LED
                    LED3On();
                }
                
                // Set power initialization flag
                usb1010v2Data.powerInitialized = true;
            }
            
            // Transition to task servicing state
            usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;
            break;
        }
        
        case USB1010V2_STATE_INITIALIZE_CODECS:
        {
            if (usb1010v2Data.i2cInitialized)
            { // I2C driver started; ok to initialize codecs
                if (!usb1010v2Data.codec1ok)
                { // Codec 1 not initialized yet
                    usb1010v2Data.codec1ok = codecInit(CODEC_1);
                }
                
                if (!usb1010v2Data.codec2ok)
                { // Codec 2 not initialized yet
                    usb1010v2Data.codec2ok = codecInit(CODEC_2);
                }
                
                if (!usb1010v2Data.codec3ok)
                { // Codec 2 not initialized yet
                    usb1010v2Data.codec3ok = codecInit(CODEC_3);
                }
                if (!usb1010v2Data.codec4ok)
                { // Codec 2 not initialized yet
                    usb1010v2Data.codec4ok = codecInit(CODEC_4);
                }
                
                if (writeI2Cdata(EXPR_ADDR, 0x06, 0x00) && writeI2Cdata(EXPR_ADDR, 0x07, 0x00))
                { // All pins of I/O expander configured for output; turn green LEDs on
                    if (writeI2Cdata(EXPR_ADDR, 0x02, 0x44) && writeI2Cdata(EXPR_ADDR, 0x03, 0x22))
                    { // All green LEDs on; set LED default configuration flag
                        usb1010v2Data.ledsDefault = true;
                    }
                }
                
                if( usb1010v2Data.codec1ok && 
                    usb1010v2Data.codec2ok && 
                    usb1010v2Data.codec3ok &&
                    usb1010v2Data.codec4ok &&
                    usb1010v2Data.ledsDefault)
                { // All four codecs initialized; set flag and transition to service tasks state
                    if (DEBUG_MODE)
                    { // Turn on blue success LED
                        LED1On();
                    }
                    usb1010v2Data.codecsInitialized = true;
                    usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;
                }
            }
            break;
        }
        
        case USB1010V2_STATE_CHANGE_INPUT_MODE:
        {
            if (usb1010v2Data.inputToggleCh1)
            { // Toggle input 1
                if (codecInputToggle(CODEC_1, &usb1010v2Data.inputModeCh1))
                { // Input successfully toggled, turn off flag
                    usb1010v2Data.inputToggleCh1 = false;
                }
            }
            
            if (usb1010v2Data.inputToggleCh2)
            { // Toggle input 2
                if (codecInputToggle(CODEC_2, &usb1010v2Data.inputModeCh2))
                { // Input successfully toggled, turn off flag
                    usb1010v2Data.inputToggleCh2 = false;
                }
            }
            
            if (usb1010v2Data.inputToggleCh3)
            { // Toggle input 3
                if (codecInputToggle(CODEC_3, &usb1010v2Data.inputModeCh3))
                { // Input successfully toggled, turn off flag
                    usb1010v2Data.inputToggleCh3 = false;
                }
            }
            
            if (usb1010v2Data.inputToggleCh4)
            { // Toggle input 4
                if (codecInputToggle(CODEC_4, &usb1010v2Data.inputModeCh4))
                { // Input successfully toggled, turn off flag
                    usb1010v2Data.inputToggleCh4 = false;
                }
            }
            
            // Transition to service tasks state
            usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;   
            break;
        }
        
        case USB1010V2_STATE_CHANGE_CHANNEL_MODE:
        {
            /* TODO: Finish this state 3/12/19 */
            // Process each channel for active change flags
            if (usb1010v2Data.gainSwCh1)
            {
                channelChange(  CODEC_1, 
                                usb1010v2Data.changeModeCh1, 
                                &usb1010v2Data.gainSwCh1, 
                                &usb1010v2Data.thruModeCh1, 
                                &usb1010v2Data.gainModeCh1);
            }
            
            if (usb1010v2Data.gainSwCh2)
            {
                channelChange(  CODEC_2, 
                                usb1010v2Data.changeModeCh2, 
                                &usb1010v2Data.gainSwCh2, 
                                &usb1010v2Data.thruModeCh2, 
                                &usb1010v2Data.gainModeCh2);
            }
            
            if (usb1010v2Data.gainSwCh3)
            {
                channelChange(  CODEC_3, 
                                usb1010v2Data.changeModeCh3, 
                                &usb1010v2Data.gainSwCh3, 
                                &usb1010v2Data.thruModeCh3, 
                                &usb1010v2Data.gainModeCh3);
            }
            
            if (usb1010v2Data.gainSwCh4)
            {
                channelChange(  CODEC_4, 
                                usb1010v2Data.changeModeCh4, 
                                &usb1010v2Data.gainSwCh4, 
                                &usb1010v2Data.thruModeCh4, 
                                &usb1010v2Data.gainModeCh4);
            }
            
            // Get new LED configuration
            uint8_t newCh12,
                    newCh34;
            
            /* Table mapping for LED values by channel grouping
             Color     1-2     3-4     Setting
             ----------------------------------
             Red        8       1       THRU
             Yellow     4       2       +3dB
             Green      2       4       0dB
             Blue       1       8       -3dB
             
             The I/O expander payload generation algorithms below are derived from the table above
            */
            
            newCh12 =   (usb1010v2Data.thruModeCh1 ? 0x08 : 0x01 << usb1010v2Data.gainModeCh1) + 
                        ((usb1010v2Data.thruModeCh2 ? 0x08 : 0x01 << usb1010v2Data.gainModeCh2) << 4);
            
            newCh34 =   (usb1010v2Data.thruModeCh3 ? 0x01 : 0x01 << (3 - usb1010v2Data.gainModeCh3)) + 
                        ((usb1010v2Data.thruModeCh4 ? 0x01 : 0x01 << (3 - usb1010v2Data.gainModeCh4)) << 4);
            
            // Update LEDs            
            writeI2Cdata(EXPR_ADDR, 0x03, newCh12);
            writeI2Cdata(EXPR_ADDR, 0x02, newCh34);
            
            // Transition to service tasks state
            usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;    
            break;
        }
        
        case USB1010V2_STATE_RESTART_HB_TIMER: 
        { // Create a new timer
            usb1010v2Data.hDelayTimer = SYS_TMR_DelayMS(HEARTBEAT_DELAY); 
            if (usb1010v2Data.hDelayTimer != SYS_TMR_HANDLE_INVALID)
            { // Valid handle returned; transition to service tasks state
                usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;
            }
            break; 
        }
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
