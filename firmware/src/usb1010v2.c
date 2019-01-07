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
            { // Analog power not initialized, transition to analog power initialization state
                usb1010v2Data.state = USB1010V2_STATE_INITIALIZE_POWER;
                break;
            }
            
            if (!usb1010v2Data.codecsInitialized)
            { // Codecs not initialized, transition to codec initialization state
                usb1010v2Data.state = USB1010V2_STATE_INITIALIZE_CODECS;
                break;
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
            uint32_t elapsedTime = (MAIN_TIMER_PERIOD + usb1010v2Data.startTimeSMPS - usb1010v2Data.currTime) 
                                    % MAIN_TIMER_PERIOD;
                    
            if (usb1010v2Data.smpsInitialized && (elapsedTime >= SMPS_STARTUP_DELAY))
            { // SMPS section started
                // Enable LDOs
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
        { // This state is still under construction (12/7/2018)
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
                    { // Turn on success LED
                        LED1On();
                    }
                    usb1010v2Data.codecsInitialized = true;
                    usb1010v2Data.state = USB1010V2_STATE_SERVICE_TASKS;
                }
            }
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
