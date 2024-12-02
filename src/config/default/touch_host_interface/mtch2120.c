
/*******************************************************************************
  MPLAB Harmony Touch Host Interface v1.1.0 Release

  Company:
    Microchip Technology Inc.

  File Name:
    mtch2120.c

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
 * Copyright (C) 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
 * ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************



#include <stdint.h>
#include "mtch2120.h"


// platform
#include "../src/config/default/touch_host_interface/touchI2C.h"

  
#include <stdint.h>

uint8_t mtch2120_deviceAddress = MTCH2120_ADDRESS_0;

bool mtch2120_isDS_Activated = false;

// i2c communication status flag
static volatile bool mtch2120_transmissionCompleted  = false;
static volatile bool mtch2120_receptionCompleted     = false;

// timeout status flag
static volatile bool mtch2120_communicationTimeout = false;

static bool mtch2120_isDriverInitialized = false;


/*******************************************************************************
 * This function will wait until response received / timeout happened.
 * @type     :   private function
 * @Param    :   void
 * @return   :   void
*******************************************************************************/
static void mtch2120_wait(void);

/*******************************************************************************
 * This function will set the flag once i2c transmission has been completed.
 * @type     :   private function
 * @Param    :   void
 * @return   :   void
*******************************************************************************/
static void mtch2120_txCompleted(void);

/*******************************************************************************
 * This function will set the flag once i2c reception has been completed.
 * @type     :   private function
 * @Param    :   void
 * @return   :   void
*******************************************************************************/
static void mtch2120_rxCompleted(uint8_t byte);

/*******************************************************************************
 * 
 * Private Function definitions
 * 
 ******************************************************************************/

/*******************************************************************************
 * This function will wait until transmission completed / reception completed / timeout.
 * @type     :   private function
 * @Param    :   void
 * @return   :   void
*******************************************************************************/
static void mtch2120_wait(void)
{
    uint16_t communicationTimeout = 0u;
    
    bool mtch2120_stopWaiting        = false;
    mtch2120_transmissionCompleted   = false;
    mtch2120_receptionCompleted      = false;
    mtch2120_communicationTimeout    = false;
    
    do
    {
        if(mtch2120_transmissionCompleted == true)
        {
            mtch2120_stopWaiting = true;
        }
        else if(mtch2120_receptionCompleted == true)
        {
            mtch2120_stopWaiting = true;
        }
        else
        {
            // counter based timeout
            if(communicationTimeout < 65000u)
            {
                communicationTimeout++;
            }
            else
            {
                mtch2120_stopWaiting = true;
                mtch2120_communicationTimeout = true;
            }
        }
    }while(mtch2120_stopWaiting == false);
}

/*******************************************************************************
 * This function will call automatically when the transmission completed.
 * @type     :   private function
 * @Param    :   void
 * @return   :   void
*******************************************************************************/
static void mtch2120_txCompleted(void)
{
    mtch2120_transmissionCompleted = true;
}

/*******************************************************************************
 * This function will call automatically when the reception completed.
 * @type     :   private function
 * @Param    :   void
 * @return   :   void
*******************************************************************************/
static void mtch2120_rxCompleted(uint8_t byte)
{
    (void)byte;     // added to avoid compilation warning.
    mtch2120_receptionCompleted = true;
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
*******************************************************************************/

/*******************************************************************************
 * This function initialize the touch driver...
 * @Param    :   void
 * @return   :   boolean
*******************************************************************************/
void mtch2120_touchDeviceInit(uint8_t address)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    
    switch(address)
    {
        case MTCH2120_ADDRESS_0:
        case MTCH2120_ADDRESS_1:
            mtch2120_deviceAddress = address;
            if(mtch2120_isDriverInitialized == false)
            {
                touchI2cInit(mtch2120_txCompleted, mtch2120_rxCompleted);
                mtch2120_isDriverInitialized = true;
            }
            break;
        default:
            i2cStatus = INVALIED_INPUT_PARAMETER;
            break;
    }
    
    if(i2cStatus == SUCCESS)
    {
#if ENABLE_TOUCH_TUNE == 1u        
        mtch2120_communicationStatus.initError = 0u;
#endif // ENABLE_TOUCH_TUNE
    }
    else
    {
#if ENABLE_TOUCH_TUNE == 1u
        mtch2120_communicationStatus.initError = 1u;
#endif // ENABLE_TOUCH_TUNE
    }
}

/*******************************************************************************
 * This function will read the data from the corresponding memory
 * @Param 1  :   Memory Address
 * @Param 2  :   buffer for copy the data
 * @Param 3  :   how many bytes would like to read from the memory address
 * @return   :   MTCH2120_I2C_Status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_readFromMemory(uint16_t memoryAddress, uint8_t * buffer, uint8_t length)
{
    MTCH2120_I2C_Status i2cStatus  = SUCCESS;
    SERCOM_I2C_ERROR i2cError = SERCOM_I2C_ERROR_NONE;
    
    const uint16_t address  = (memoryAddress & 0xFF00u);   // MSB
    
    uint8_t retryCount = 0u;
    if(mtch2120_isDriverInitialized)
    {
        // we haven't checked the length of the input parameter. Because it might vary based on the buffer type
        if((address < (uint16_t)END_MEMORY_ADDRESS) && (buffer != NULL))
        {
            while(retryCount < MAX_RETRY_COUNT)
            {
                touchI2cReceiveDataFrom_16bit_Address(mtch2120_deviceAddress,           // device address
                                                        memoryAddress,                  // i2c memory address (16 bit [address | offset])
                                                        buffer,                         // buffer for store the data
                                                        length);                        // how many bytes to read   (Note:- input buffer data type will be either 8 bit or 16 bit)
                // wait until receive data (or) wait until timeout occurred
                mtch2120_wait();
                i2cError = SERCOM3_I2C_ErrorGet();
                if(i2cError != SERCOM_I2C_ERROR_NONE)
                {
                    retryCount++;
                    i2cStatus = I2C_COMMN_ERROR;
                }
                else if (mtch2120_communicationTimeout == true)
                {
                    retryCount++;
                    i2cStatus = I2C_COMMN_ERROR;                    
                }
                else
                {
                    i2cStatus = SUCCESS;
                    break;
                }
            };
        }
        else
        {
            i2cStatus = INVALIED_INPUT_PARAMETER;
        }
    }
    else
    {
        i2cStatus = NOT_INITIALIZED;
    }
    return i2cStatus;
}

/*******************************************************************************
 * This function will write the data to the corresponding memory
 * @Param 1  :   Memory Address
 * @Param 2  :   data
 * @Param 3  :   how many bytes would like to write
 * @return   :   MTCH2120_I2C_Status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_writeToMemory(uint16_t memoryAddress, const uint8_t *data, uint8_t length)
{
    MTCH2120_I2C_Status i2cStatus  = SUCCESS;
    
    const uint16_t address  = (memoryAddress & 0xFF00u);   // MSB
    
    SERCOM_I2C_ERROR i2cError = SERCOM_I2C_ERROR_NONE;

    uint8_t retryCount = 0u;
    if(mtch2120_isDriverInitialized)
    {
        // we haven't checked the length of the input parameter. Because it might vary based on the buffer type
        if((address < (uint16_t)END_MEMORY_ADDRESS) && (data != NULL))
        {
            while(retryCount < MAX_RETRY_COUNT)
            {
                touchI2cSendDataTo_16bit_Address(mtch2120_deviceAddress,            // device address
                                                    memoryAddress,                  // i2c memory address (16 bit [address | offset])
                                                    data,                           // buffer for store the data
                                                    length);                        // how many bytes to read   (Note:- input buffer data type will be either 8 bit or 16 bit)
                // wait until receive data (or) wait until timeout occurred
                mtch2120_wait();
                i2cError = SERCOM3_I2C_ErrorGet();
                if(i2cError != SERCOM_I2C_ERROR_NONE)
                {
                    retryCount++;
                    i2cStatus = I2C_COMMN_ERROR;
                }
                else if(mtch2120_communicationTimeout == true)
                {
                    retryCount++;
                    i2cStatus = I2C_COMMN_ERROR;                    
                }
                else
                {
                    i2cStatus = SUCCESS;
                    break;
                }
            };
        }
        else
        {
            i2cStatus = INVALIED_INPUT_PARAMETER;
        }
    }
    else
    {
        i2cStatus = NOT_INITIALIZED;
    }
    return i2cStatus;
}

/*******************************************************************************
 * This function will get the device information from 2120 device
 * @Param    :   mtch2120_DeviceInformation_t*
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getDeviceInformation(mtch2120_DeviceInformation_t *deviceInformation_l)
{
    return mtch2120_readFromMemory((uint16_t)ADDR_DEVICE_ID, (uint8_t*)deviceInformation_l, (uint8_t)sizeof(mtch2120_DeviceInformation_t));
}

/*******************************************************************************
 * This function get the sensor status from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data (1 for detect and 0 for no detect)
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getSensorState(uint8_t channel, uint8_t *state)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*state) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_STATE | offset), state, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the channel reference from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getChannelReference(uint8_t channel, uint16_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_CHANNEL_REFERENCE | offset), (uint8_t*)value, 2u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the channel CC from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getCC(uint8_t channel, uint16_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_NODE_CC | offset), (uint8_t*)value, 2u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the node acquisition signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getNodeAcqisitionSignal(uint8_t channel, uint16_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_NODE_ACQ_SIGNALS | offset), (uint8_t*)value, 2u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the Sensor Control signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getSensorControl(uint8_t channel, uint16_t *sensorControl_l)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*sensorControl_l) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)sensorControl_l, 2u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the Sensor Control signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setSensorControl(uint8_t channel, uint16_t *sensorControl_l)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*sensorControl_l) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)sensorControl_l, 2u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the Threshold signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getThreshold(uint8_t channel, uint8_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_THRESHOLD | offset), value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the Threshold signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setThreshold(uint8_t channel, uint8_t value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(value) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_THRESHOLD | offset), (uint8_t*)&value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the Oversampling signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getOversampling(uint8_t channel, uint8_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)(sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_OVERSAMPLING | offset), value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the Oversampling signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setOversampling(uint8_t channel, uint8_t value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(value) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_OVERSAMPLING | offset), (uint8_t*)&value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the Gain value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getGain(uint8_t channel, uint8_t *value)
{ 
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_GAIN | offset), value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the Gain value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setGain(uint8_t channel, uint8_t value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(value) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_GAIN | offset), (uint8_t*)&value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the Measurement Clock Freqeuncy value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getMeasurementClkFreq(uint8_t channel, uint8_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_MEASUREMENT_CLK_FREQ | offset), value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the Measurement Clock Freqeuncy value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setMeasurementClkFreq(uint8_t channel, uint8_t value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(value) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_MEASUREMENT_CLK_FREQ | offset), (uint8_t*)&value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the CSD value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getCSD(uint8_t channel, uint8_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_CSD | offset), value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the CSD value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setCSD(uint8_t channel, uint8_t value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(value) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_CSD | offset), (uint8_t*)&value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the hysteresis value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getHysteresis(uint8_t channel, uint8_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_HYSTERESIS | offset), value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the hysteresis value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setHysteresis(uint8_t channel, uint8_t value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(value) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_HYSTERESIS | offset), (uint8_t*)&value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function get the AKS value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getAKS(uint8_t channel, uint8_t *value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(*value) * (uint8_t)channel);
        i2c_status = mtch2120_readFromMemory(((uint16_t)ADDR_AKS | offset), value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

/*******************************************************************************
 * This function set the AKS value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setAKS(uint8_t channel, uint8_t value)
{
    uint8_t offset = 0u; 
    MTCH2120_I2C_Status i2c_status = SUCCESS;
    if(channel < DEF_NUM_SENSORS)
    {
        offset = (uint8_t)((uint8_t)sizeof(value) * (uint8_t)channel);
        i2c_status = mtch2120_writeToMemory(((uint16_t)ADDR_AKS | offset), (uint8_t*)&value, 1u);
    }
    else
    {
        i2c_status = INVALIED_INPUT_PARAMETER;
    }
    return i2c_status;
}

#if ENABLE_TOUCH_TUNE==1u

void mtch2120_getAllSensorState(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_STATE | FROM_OFFSET_0), (uint8_t*)mtch2120_sensorState, (uint8_t)sizeof(mtch2120_sensorState));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}
 
void mtch2120_setDeviceControl_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_DEVICE_CONTROL, (uint8_t*)&mtch2120_deviceControl, (uint8_t)sizeof(mtch2120_DeviceControl_t));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
    
    if(mtch2120_deviceControl.deviceReset == 1u)
    {
        mtch2120_updateShieldStatus();
    }
}

void mtch2120_setGroupConfiguration_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_GROUP_CONFIGURATION, (uint8_t*)&mtch2120_groupConfiguration, (uint8_t)sizeof(mtch2120_GroupConfiguration_t));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setSensorControl_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_SENSOR_CONTROL, (uint8_t*)&mtch2120_sensorControl[0].byte, (uint8_t)(sizeof(mtch2120_SensorControl_t) * DEF_NUM_SENSORS));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setGpioPin(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_GPIO_CONFIG, (uint8_t*)&mtch2120_gpioPins, (uint8_t)(sizeof(mtch2120_gpioPins)));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setGpioDirection(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GPIO_CONFIG | 0x08u), (uint8_t*)&mtch2120_gpioDirection, (uint8_t)(sizeof(mtch2120_gpioDirection)));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setGpioOutValue(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GPIO_CONFIG | 0x10u), (uint8_t*)&mtch2120_gpioOutputValue, (uint8_t)(sizeof(mtch2120_gpioOutputValue)));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setLumpConfiguration_0(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_LUMP_CONFIG, (uint8_t*)&mtch2120_lumpConfiguration_0, (uint8_t)sizeof(mtch2120_lumpConfiguration_0));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setLumpConfiguration_1(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_LUMP_CONFIG | 0x08u), (uint8_t*)&mtch2120_lumpConfiguration_1, (uint8_t)sizeof(mtch2120_lumpConfiguration_1));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setLumpConfiguration_2(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_LUMP_CONFIG | 0x10u), (uint8_t*)&mtch2120_lumpConfiguration_2, (uint8_t)sizeof(mtch2120_lumpConfiguration_2));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setLumpConfiguration_3(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_LUMP_CONFIG | 0x18u), (uint8_t*)&mtch2120_lumpConfiguration_3, (uint8_t)sizeof(mtch2120_lumpConfiguration_3));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setThreshold_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_THRESHOLD | FROM_OFFSET_0), (uint8_t*)mtch2120_threshold, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setOversampling_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_OVERSAMPLING | FROM_OFFSET_0), (uint8_t*)mtch2120_filterlevel, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setGain_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GAIN | FROM_OFFSET_0), (uint8_t*)mtch2120_gain, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setMeasurClkFreq_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_MEASUREMENT_CLK_FREQ | FROM_OFFSET_0), (uint8_t*)mtch2120_measurementClkFreq, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setCSD_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_CSD | FROM_OFFSET_0), (uint8_t*)mtch2120_CSD, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setHysteresis_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_HYSTERESIS | FROM_OFFSET_0), (uint8_t*)mtch2120_hysteresis, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_setAKS_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_AKS | FROM_OFFSET_0), (uint8_t*)mtch2120_AKSgroup, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getDeviceControl_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory((uint16_t)ADDR_DEVICE_CONTROL, (uint8_t*)&mtch2120_deviceControl, (uint8_t)sizeof(mtch2120_DeviceControl_t));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getGroupConfiguration_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory((uint16_t)ADDR_GROUP_CONFIGURATION, (uint8_t*)&mtch2120_groupConfiguration, (uint8_t)sizeof(mtch2120_GroupConfiguration_t));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getSensorControl_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory((uint16_t)ADDR_SENSOR_CONTROL, (uint8_t*)&mtch2120_sensorControl[0].byte, (uint8_t)(sizeof(mtch2120_SensorControl_t) * DEF_NUM_SENSORS));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getGpioPin(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory((uint16_t)ADDR_GPIO_CONFIG, (uint8_t*)&mtch2120_gpioPins, (uint8_t)(sizeof(mtch2120_gpioPins)));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getGpioDirection(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | 0x08u), (uint8_t*)&mtch2120_gpioDirection, (uint8_t)(sizeof(mtch2120_gpioDirection)));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getGpioOutValue(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | 0x10u), (uint8_t*)&mtch2120_gpioOutputValue, (uint8_t)(sizeof(mtch2120_gpioOutputValue)));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getGpioInputValue(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | GPIO_IN_OFFSET), (uint8_t*)&mtch2120_gpioInputValue, (uint8_t)(sizeof(mtch2120_gpioInputValue)));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getLumpConfiguration_0(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory((uint16_t)ADDR_LUMP_CONFIG, (uint8_t*)&mtch2120_lumpConfiguration_0, (uint8_t)sizeof(mtch2120_lumpConfiguration_0));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getLumpConfiguration_1(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_LUMP_CONFIG | 0x08u), (uint8_t*)&mtch2120_lumpConfiguration_1, (uint8_t)sizeof(mtch2120_lumpConfiguration_1));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getLumpConfiguration_2(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_LUMP_CONFIG | 0x10u), (uint8_t*)&mtch2120_lumpConfiguration_2, (uint8_t)sizeof(mtch2120_lumpConfiguration_2));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getLumpConfiguration_3(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_LUMP_CONFIG | 0x18u), (uint8_t*)&mtch2120_lumpConfiguration_3, (uint8_t)sizeof(mtch2120_lumpConfiguration_3));
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getThreshold_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_THRESHOLD | FROM_OFFSET_0), (uint8_t*)mtch2120_threshold, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getOversampling_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_OVERSAMPLING | FROM_OFFSET_0), (uint8_t*)mtch2120_filterlevel, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getGain_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GAIN | FROM_OFFSET_0), (uint8_t*)mtch2120_gain, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getMeasurClkFreq_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_MEASUREMENT_CLK_FREQ | FROM_OFFSET_0), (uint8_t*)mtch2120_measurementClkFreq, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getCSD_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_CSD | FROM_OFFSET_0), (uint8_t*)mtch2120_CSD, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getHysteresis_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_HYSTERESIS | FROM_OFFSET_0), (uint8_t*)mtch2120_hysteresis, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_getAKS_Config(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_AKS | FROM_OFFSET_0), (uint8_t*)mtch2120_AKSgroup, DEF_NUM_SENSORS);
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}

void mtch2120_updateShieldStatus(void)
{
    mtch2120_getDeviceControl_Config();
    mtch2120_isDS_Activated = (mtch2120_deviceControl.drivenShield == 1u) ? true : false;
}

void mtch2120_getAllConfigurations(void)
{
    mtch2120_getDeviceControl_Config();
    mtch2120_getGroupConfiguration_Config();
    mtch2120_getSensorControl_Config();
    mtch2120_getGpioPin();
    mtch2120_getGpioDirection();
    mtch2120_getGpioOutValue();
    mtch2120_getGpioInputValue();
    mtch2120_getLumpConfiguration_0();
    mtch2120_getLumpConfiguration_1();
    mtch2120_getLumpConfiguration_2();
    mtch2120_getLumpConfiguration_3();
    mtch2120_getThreshold_Config();
    mtch2120_getOversampling_Config();
    mtch2120_getGain_Config();
    mtch2120_getMeasurClkFreq_Config();
    mtch2120_getCSD_Config();
    mtch2120_getHysteresis_Config();
    mtch2120_getAKS_Config();
}

void mtch2120_touchDeviceReadDebugData(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.DebugDataError = 0u;
    
    if(mtch2120_isDriverInitialized)
    {
        if(isStopToReadDebugData() == 0u)
        {
            do
            {
                i2cStatus = mtch2120_getDeviceInformation(&mtch2120_deviceInformation);
                if(i2cStatus != SUCCESS)
                {
                    break;
                }

                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | GPIO_IN_OFFSET), (uint8_t*)&mtch2120_gpioInputValue, (uint8_t)sizeof(mtch2120_gpioInputValue));
                if(i2cStatus != SUCCESS)
                {
                    break;
                }

                // reading touch reset status
                i2cStatus = mtch2120_readFromMemory((uint16_t)ADDR_STATUS, (uint8_t*)&mtch2120_Status, (uint8_t)sizeof(MTCH2120_Status_t));
                if(i2cStatus != SUCCESS)
                {
                    break;
                }

                // reading button state
                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_STATE | FROM_OFFSET_0), (uint8_t*)mtch2120_sensorState, (uint8_t)sizeof(mtch2120_sensorState));
                if(i2cStatus != SUCCESS)
                {
                    break;
                }        

                // reading channel reference
                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_CHANNEL_REFERENCE | FROM_OFFSET_0), (uint8_t*)mtch2120_reference, (uint8_t)sizeof(mtch2120_reference));
                if(i2cStatus != SUCCESS)
                {
                    break;
                }

                // reading node AcqSignals
                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_NODE_ACQ_SIGNALS | FROM_OFFSET_0), (uint8_t*)mtch2120_signal, (uint8_t)sizeof(mtch2120_signal));
                if(i2cStatus != SUCCESS)
                {
                    break;
                }

                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_NODE_CC | FROM_OFFSET_0), (uint8_t*)mtch2120_compensationCapacitance, (uint8_t)sizeof(mtch2120_compensationCapacitance));
                if(i2cStatus != SUCCESS)
                {
                    break;
                }

                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_THRESHOLD | FROM_OFFSET_0), (uint8_t*)mtch2120_threshold, DEF_NUM_SENSORS);
                if(i2cStatus != SUCCESS)
                {
                    break;
                }

                for(uint8_t i = 0; i < DEF_NUM_SENSORS; i++)
                {
                    mtch2120_delta[i] = ((int16_t)mtch2120_signal[i] - (int16_t)mtch2120_reference[i]);
                }
            }while(false);
        }
        if(i2cStatus != SUCCESS)
        {
            mtch2120_communicationStatus.DebugDataError = 1u;
        }
    }
}

// following functions will helps to update mtch2120 parameters of the slave module through i2c communication.
void mtch2120_setDeviceConfiguration(void)
{
    MTCH2120_I2C_Status i2cStatus = SUCCESS;
    mtch2120_communicationStatus.ConfigDataError = 0u;
    if(mtch2120_isDriverInitialized)
    {
        do
        {
            // write device control values from the touch module
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_DEVICE_CONTROL, (uint8_t*)&mtch2120_deviceControl, (uint8_t)sizeof(mtch2120_DeviceControl_t));
            if(i2cStatus != SUCCESS)
            {
                break;
            }

            // write device group configuration from the touch module
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_GROUP_CONFIGURATION, (uint8_t*)&mtch2120_groupConfiguration, (uint8_t)sizeof(mtch2120_GroupConfiguration_t));
            if(i2cStatus != SUCCESS)
            {
                break;
            }

            // write sensor control configuration from the touch module
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_SENSOR_CONTROL, (uint8_t*)&mtch2120_sensorControl[0].byte, (uint8_t)(sizeof(mtch2120_SensorControl_t) * DEF_NUM_SENSORS));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
                
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_GPIO_CONFIG, (uint8_t*)&mtch2120_gpioPins, (uint8_t)(sizeof(mtch2120_gpioPins)));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_GPIO_CONFIG, (uint8_t*)&mtch2120_gpioDirection, (uint8_t)(sizeof(mtch2120_gpioDirection)));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_GPIO_CONFIG, (uint8_t*)&mtch2120_gpioOutputValue, (uint8_t)(sizeof(mtch2120_gpioOutputValue)));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write lump configuration from the touch module
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_LUMP_CONFIG, (uint8_t*)&mtch2120_lumpConfiguration_0, (uint8_t)sizeof(mtch2120_lumpConfiguration_0));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write lump configuration from the touch module
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_LUMP_CONFIG, (uint8_t*)&mtch2120_lumpConfiguration_1, (uint8_t)sizeof(mtch2120_lumpConfiguration_1));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write lump configuration from the touch module
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_LUMP_CONFIG, (uint8_t*)&mtch2120_lumpConfiguration_2, (uint8_t)sizeof(mtch2120_lumpConfiguration_2));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write lump configuration from the touch module
            i2cStatus = mtch2120_writeToMemory((uint16_t)ADDR_LUMP_CONFIG, (uint8_t*)&mtch2120_lumpConfiguration_3, (uint8_t)sizeof(mtch2120_lumpConfiguration_3));
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write threshold from the touch module
            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_THRESHOLD | FROM_OFFSET_0), (uint8_t*)mtch2120_threshold, DEF_NUM_SENSORS);
            if(i2cStatus != SUCCESS)
            {
                break;
            }

            // write oversampling from the touch module
            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_OVERSAMPLING | FROM_OFFSET_0), (uint8_t*)mtch2120_filterlevel, DEF_NUM_SENSORS);
            if(i2cStatus != SUCCESS)
            {
                break;
            }

            // write gain from the touch module
            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GAIN | FROM_OFFSET_0), (uint8_t*)mtch2120_gain, DEF_NUM_SENSORS);
            if(i2cStatus != SUCCESS)
            {
                break;
            }

            // write PTC pre-scalar value from the touch module
            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_MEASUREMENT_CLK_FREQ | FROM_OFFSET_0), (uint8_t*)mtch2120_measurementClkFreq, DEF_NUM_SENSORS);
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write CSD from the touch module
            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_CSD | FROM_OFFSET_0), (uint8_t*)mtch2120_CSD, DEF_NUM_SENSORS);
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write hysteresis from the touch module
            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_HYSTERESIS | FROM_OFFSET_0), (uint8_t*)mtch2120_hysteresis, DEF_NUM_SENSORS);
            if(i2cStatus != SUCCESS)
            {
                break;
            }
            
            // write AKS from the touch module
            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_AKS | FROM_OFFSET_0), (uint8_t*)mtch2120_AKSgroup, DEF_NUM_SENSORS);
            if(i2cStatus != SUCCESS)
            {
                break;
            }
        }while(false);
    }
    if(i2cStatus != SUCCESS)
    {
        mtch2120_communicationStatus.ConfigDataError = 1u;
    }
}
#endif
