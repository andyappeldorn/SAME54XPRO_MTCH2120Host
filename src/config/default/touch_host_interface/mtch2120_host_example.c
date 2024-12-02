/*******************************************************************************
  MPLAB Harmony Touch Host Interface v1.1.0 Release

  Company:
    Microchip Technology Inc.

  File Name:
    host_example.c

  Summary:
    This file contains the example "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>      // Defines NULL
#include <stdbool.h>     // Defines true
#include <stdlib.h>      // Defines EXIT_FAILURE
#include "definitions.h" // SYS function prototypes
#include"config/default/touch_host_interface/mtch2120.h"
#include"config/default/touch_host_interface/mtch2120_host_example.h"

#include"config/default/touch_host_interface/touchTune.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

static uint8_t deviceID_example = 0u;
static uint8_t version_example  = 0u;
static uint8_t threshold_example[DEF_NUM_SENSORS] = {
                                                        20u, 20u, 20u, 20u, 
                                                        20u, 20u, 20u, 20u, 
                                                        20u, 20u, 20u, 20u,
                                                        20u, 20u, 20u, 20u
                                                    };

static volatile MTCH2120_I2C_Status i2cStatus_example = SUCCESS;
static uint8_t offset_example = 0u;

void touch_init_example(void)
{
    mtch2120_touchDeviceInit(MTCH2120_ADDRESS_0);

#if ENABLE_TOUCH_TUNE==1u
    touchTuneInit();
#endif // ENABLE_TOUCH_TUNE
    
    //--------------------------------------------------------------------------
    // Note: 
    //   more examples are available in "mtch2120_examples.h"
    //--------------------------------------------------------------------------
    
    // reading Device ID from MTCH2120 device
    offset_example = 0u;    // calculated as per mtch2120 datasheet
    i2cStatus_example = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_ID | offset_example), (uint8_t*)&deviceID_example, (uint8_t)sizeof(deviceID_example));
    if(i2cStatus_example == SUCCESS)
    {
        // success
    }
    else
    {
        // failed to read
    }
    
    // reading Device Version from MTCH2120 device
    offset_example = 1u;    // calculated as per mtch2120 datasheet
    i2cStatus_example = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_ID | offset_example), (uint8_t*)&version_example, (uint8_t)sizeof(version_example));
    if(i2cStatus_example == SUCCESS)
    {
        // success
    }
    else
    {
        // failed to read
    }
    
    // writing threshold for all the channels
    offset_example = 0u;    // calculated as per mtch2120 datasheet  
    i2cStatus_example = mtch2120_writeToMemory(((uint16_t)ADDR_THRESHOLD | offset_example), (uint8_t*)threshold_example, (uint8_t)sizeof(threshold_example));
    if(i2cStatus_example == SUCCESS)
    {
        // success
    }
    else
    {
        // failed to read
    }
}

void touch_mainloop_example( void )
{
    static uint16_t buttonStatusMask = 0u;
    /* Read the touch status when interrupt pin goes low */
    if(MTCH_INT_Get() == 0u)
    {
        i2cStatus_example = mtch2120_readFromMemory(((uint16_t)ADDR_STATUS | 2u), (uint8_t*)&buttonStatusMask, (uint8_t)sizeof(buttonStatusMask));
        if(i2cStatus_example == SUCCESS)
        {
            // success
        }
        else
        {
            // failed to read
        }
        
        for(uint8_t channel = 0u; channel < DEF_NUM_SENSORS; channel++)
        {
            if((buttonStatusMask & ((uint16_t)1u << channel)) != 0u)
            {
                // channel_n detect
            }
            else
            {
                // channel_n not-detect
            }
        }
    }

#if ENABLE_TOUCH_TUNE == 1u
    static uint16_t loopCnt = 0u;
    if(touchTuneIsDebugDataSent() == 0u) 
    {
        loopCnt++;
    }
    if(loopCnt > 1000u) 
    {
        loopCnt = 0u;
        /* Touch tune related function calls */
        mtch2120_touchDeviceReadDebugData();
        touchTuneNewDataAvailable();        
    }
    touchTuneProcess();
#endif // ENABLE_TOUCH_TUNE
}

/*******************************************************************************
 End of File
*/
