/*******************************************************************************
  MPLAB Harmony Touch Host Interface Release
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mtch2120.h

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
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
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

#ifndef MTCH2120_API_H
#define MTCH2120_API_H

#include <stdint.h>
#include <stdbool.h>

#define ENABLE_TOUCH_TUNE       1u

#define MAX_RETRY_COUNT         5u

#define SHIFT_LEFT(n)           ((n)<<8u)
#define RIGHT_SHIFT(n)          ((n)>>1u)

// slave address details                                // Right Shifted values          Static value     -   Assigned by PIN
#define MTCH2120_ADDRESS_0               RIGHT_SHIFT(0x40u)      //      0x20u                       00010         -     0000 
#define MTCH2120_ADDRESS_1               RIGHT_SHIFT(0x42u)      //      0x21u                       00010         -     0100

#define MAX_SLAVE_ADDR          3U

#define XFER_TIMEOUT            50u         //50 ms

#define CHANNEL_0               0u
#define FROM_OFFSET_0           0u

//-----------------------------------
// GPIO OFFSETS
//-----------------------------------
#define GPIO_PIN_OFFSET         0u
#define GPIO_PIN_LEN            8u

#define GPIO_DIR_OFFSET         8u
#define GPIO_DIR_LEN            8u

#define GPIO_OUT_OFFSET         16u
#define GPIO_OUT_LEN            8u

#define GPIO_IN_OFFSET          24u
#define GPIO_IN_LEN             8u
//-----------------------------------

#define FACTORY_CONFIGURATION       0x00B0u

#define NUM_FREQ_STEPS              3u
#define NUMBER_OF_KEYS              12u
#define DEF_NUM_LUMP_CHANNELS       4u
#define DEF_NUM_SENSORS    (NUMBER_OF_KEYS  + DEF_NUM_LUMP_CHANNELS)

//-------------------------------------
// Configuration bit masks
//------------------------------------- 
// sensor control
#define SENCTRL_BTTN_EN_MASK            ((uint8_t)1u << (uint8_t)0u)
#define SENCTRL_BTTN_CAL_MASK           ((uint8_t)1u << (uint8_t)1u)
#define SENCTRL_BTTN_SUSPEND_MASK       ((uint8_t)1u << (uint8_t)2u)
#define SENCTRL_BTTN_LP_MASK            ((uint8_t)1u << (uint8_t)7u)

// device control
#define DEVCTRL_CAL_MASK                ((uint16_t)1u << (uint16_t)0u)
#define DEVCTRL_LP_MASK                 ((uint16_t)1u << (uint16_t)1u)
#define DEVCTRL_DLPLB_MASK              ((uint16_t)1u << (uint16_t)2u)
#define DEVCTRL_DS_MASK                 ((uint16_t)1u << (uint16_t)3u)
#define DEVCTRL_DSP_MASK                ((uint16_t)1u << (uint16_t)4u)
#define DEVCTRL_DRIFTGAIN_MASK          ((uint16_t)1u << (uint16_t)5u)
#define DEVCTRL_FREQHOP_MASK            ((uint16_t)1u << (uint16_t)6u)
#define DEVCTRL_AT_MASK                 ((uint16_t)1u << (uint16_t)7u)
#define DEVCTRL_ET_MASK                 ((uint16_t)1u << (uint16_t)8u)
#define DEVCTRL_WDT_MASK                ((uint16_t)1u << (uint16_t)9u)
#define DEVCTRL_BOD_MASK                ((uint16_t)1u << (uint16_t)10u)
#define DEVCTRL_SMCFG_MASK              ((uint16_t)1u << (uint16_t)11u)
#define DEVCTRL_SAVE_MASK               ((uint16_t)1u << (uint16_t)12u)
#define DEVCTRL_RESET_MASK              ((uint16_t)1u << (uint16_t)13u)

typedef struct __attribute__((packed))
{
    // Device Identification Number
    uint8_t deviceID;
    
    // Device Version
    unsigned int minorVersionNumber:4;
    unsigned int majorVersionNumber:4;
    
    // Device Features
    unsigned int keys:1;
    unsigned int reserved_1:6;
    unsigned int freqHop:1;
    unsigned int lowPower:1;
    unsigned int lump:1;
    unsigned int shield:2;
    unsigned int reserved_2:3;
    unsigned int save_config:1;
    unsigned int reserved_3:2;
    unsigned int error_detection:2;
    unsigned int reserved_4:3;
    
    // CRC
    uint16_t crc;
}mtch2120_DeviceInformation_t;

typedef union
{
    struct __attribute__((packed))
    {
        // Device Status
        unsigned int  por:1;                          // Power-on Reset
        unsigned int  bod:1;                          // Brown-out Detection
        unsigned int  externalReset:1;                // External Reset
        unsigned int  wdt:1;                          // Watchdog Timer
        unsigned int  softwareReset:1;                // Software Reset
        unsigned int  reserved_1:3;
        unsigned int  memoryCorruption:1;             // No Configuration to load.
        unsigned int  saveFailure:1;                  // EEPROM module not working.          
        unsigned int  loadFailure:1;                  // default configuration loaded. Default configuration "deviceID -> crc" updated.
        unsigned int  reserved_2:5;                   
    }bits;
    uint16_t byte;
}mtch2120_DeviceStatus_t;

typedef struct
{
    mtch2120_DeviceStatus_t deviceStatus;
    uint16_t buttonStatus;
}MTCH2120_Status_t;

typedef union
{
    struct __attribute__((packed))
    {
        unsigned int  sensorEnable:1;
        unsigned int  sensorCalibration:1;  
        unsigned int  suspend:1;
        unsigned int  reserved:4;
        unsigned int  lowPowerSensor:1;
    }bit;
    uint8_t byte;
}mtch2120_SensorControl_t;

typedef struct
{
    uint16_t touchMeasurementPeriod;
    uint16_t lowPowerMeasurementPeriod;
    uint16_t timeoutConfig;
    uint8_t sensorReburstMode;
    uint8_t detectIntegration;
    uint8_t sensorAntiTouchIntegration;
    uint8_t sensorMaxOnTime;
    uint8_t sensorDriftHoldTime;
    uint8_t sensorTouchDriftRate;
    uint8_t sensorAntiTouchDriftRate;
    uint8_t sensorAntiTouchRecalThr;    // Sensor Anti-Touch Recalibration Threshold
    uint16_t noiseThreshold;
    uint8_t noiseIntegration;
    uint8_t hopFreqency[NUM_FREQ_STEPS];
}mtch2120_GroupConfiguration_t;

typedef struct __attribute__((packed))
{
    unsigned int calibrate:1;
    unsigned int lpEnable:1;
    unsigned int driftLowPowerLumpButtons:1;
    unsigned int drivenShield:1;
    unsigned int drivenShieldPlus:1;
    unsigned int driftGain:1;
    unsigned int freqencyHopping:1;
    unsigned int autoTune:1;
    unsigned int easyTune:1;
    unsigned int watchDogTimer:1;
    unsigned int brownOutDetect:1;
    unsigned int saveManufacturerConfig:1;
    unsigned int saveConfiguration:1;
    unsigned int deviceReset:1;
    unsigned int reserved:2;
}mtch2120_DeviceControl_t;

// I2C memory map
enum MemoryAddress
{
    ADDR_DEVICE_ID                   = SHIFT_LEFT(0),
    ADDR_STATUS                      = SHIFT_LEFT(1),
    ADDR_NODE_ACQ_SIGNALS            = SHIFT_LEFT(2),
    ADDR_CHANNEL_REFERENCE           = SHIFT_LEFT(3),
    ADDR_SENSOR_STATE                = SHIFT_LEFT(4),
    ADDR_NODE_CC                     = SHIFT_LEFT(5),
    ADDR_SENSOR_CONTROL              = SHIFT_LEFT(14),
    ADDR_CSD                         = SHIFT_LEFT(15),
    ADDR_MEASUREMENT_CLK_FREQ        = SHIFT_LEFT(16),
    ADDR_OVERSAMPLING                = SHIFT_LEFT(17),
    ADDR_THRESHOLD                   = SHIFT_LEFT(18),
    ADDR_GAIN                        = SHIFT_LEFT(19),
    ADDR_HYSTERESIS                  = SHIFT_LEFT(20),
    ADDR_AKS                         = SHIFT_LEFT(21),
    ADDR_GROUP_CONFIGURATION         = SHIFT_LEFT(22),
    ADDR_DEVICE_CONTROL              = SHIFT_LEFT(31),
    ADDR_LUMP_CONFIG                 = SHIFT_LEFT(32),
    ADDR_GPIO_CONFIG                 = SHIFT_LEFT(33),
    END_MEMORY_ADDRESS               = SHIFT_LEFT(34)
};

/* API status bits:
 *  - User should have to validate each bits instead of enumeration values.
 * 
 * SUCCESS                  :   Operation success.
 * NOT_INITIALIZED          :   User should initialize the API first before read and write operation.
 * INVALIED_INPUT_PARAMETER :   Please check the following possibilities.
 *                                  1. Slave is not powered-up.
 *                                  2. Slave is not properly connected to master.
 *                                  3. Slave has sent "NAK" signal to master.
 * I2C_COMMN_ERROR                :   It might be no response or NAK from 2120 device.
 **/
typedef enum
{
    SUCCESS                     = 0u,        // 0000 0000
    NOT_INITIALIZED             = 1u,        // 0000 0001
    INVALIED_INPUT_PARAMETER    = 2u,        // 0000 0010
    I2C_COMMN_ERROR             = 4u         // 0000 0100
}MTCH2120_I2C_Status;

extern uint8_t mtch2120_deviceAddress;
extern bool mtch2120_isDS_Activated;

// this function will initialize the API and setting a way to communicate with the slave module
void mtch2120_touchDeviceInit(uint8_t address);

// following functions will helps to gather mtch2120 parameters of the slave module through i2c communication.
void mtch2120_touchDeviceReadDebugData(void);               // check with DD

#if ENABLE_TOUCH_TUNE ==1u
// following functions will helps to update mtch2120 parameters of the slave module through i2c communication.
void mtch2120_setDeviceConfiguration(void);

void mtch2120_getAllSensorState(void);
void mtch2120_setDeviceControl_Config(void);
void mtch2120_setGroupConfiguration_Config(void);
void mtch2120_setSensorControl_Config(void);
void mtch2120_setThreshold_Config(void);
void mtch2120_setOversampling_Config(void);
void mtch2120_setGain_Config(void);
void mtch2120_setMeasurClkFreq_Config(void);
void mtch2120_setCSD_Config(void);
void mtch2120_setHysteresis_Config(void);
void mtch2120_setAKS_Config(void);
void mtch2120_setLumpConfiguration_0(void);
void mtch2120_setLumpConfiguration_1(void);
void mtch2120_setLumpConfiguration_2(void);
void mtch2120_setLumpConfiguration_3(void);
void mtch2120_setGpioPin(void);
void mtch2120_setGpioDirection(void);
void mtch2120_setGpioOutValue(void);

void mtch2120_getDeviceControl_Config(void);
void mtch2120_getGroupConfiguration_Config(void);
void mtch2120_getSensorControl_Config(void);
void mtch2120_getThreshold_Config(void);
void mtch2120_getOversampling_Config(void);
void mtch2120_getGain_Config(void);
void mtch2120_getMeasurClkFreq_Config(void);
void mtch2120_getCSD_Config(void);
void mtch2120_getHysteresis_Config(void);
void mtch2120_getAKS_Config(void);
void mtch2120_getLumpConfiguration_0(void);
void mtch2120_getLumpConfiguration_1(void);
void mtch2120_getLumpConfiguration_2(void);
void mtch2120_getLumpConfiguration_3(void);
void mtch2120_getGpioPin(void);
void mtch2120_getGpioDirection(void);
void mtch2120_getGpioOutValue(void);
void mtch2120_getGpioInputValue(void);

void mtch2120_updateShieldStatus(void);

/*******************************************************************************
 * Reading all the configuration from the touch module.
 * @Param 1  :   void
 * @return   :   void
*******************************************************************************/
void mtch2120_getAllConfigurations(void);
#endif

/*******************************************************************************
 * This function will read the data from the corresponding memory
 * @Param 1  :   Memory Address
 * @Param 2  :   buffer for copy the data
 * @Param 3  :   how many bytes would like to read from the memory address
 * @return   :   MTCH2120_I2C_Status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_readFromMemory(uint16_t memoryAddress, uint8_t *buffer, uint8_t length);

/*******************************************************************************
 * This function will write the data to the corresponding memory
 * @Param 1  :   Memory Address
 * @Param 2  :   data
 * @Param 3  :   how many bytes would like to write
 * @return   :   MTCH2120_I2C_Status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_writeToMemory(uint16_t memoryAddress, const uint8_t *data, uint8_t length);

/*******************************************************************************
 * This function will get the device id from the slave chip
 * @Param    :   mtch2120_deviceInformation buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getDeviceInformation(mtch2120_DeviceInformation_t *deviceInformation_l);

/*******************************************************************************
 * This function get the sensor status from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getSensorState(uint8_t channel, uint8_t *state);

/*******************************************************************************
 * This function get the channel reference from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getChannelReference(uint8_t channel, uint16_t *value);

/*******************************************************************************
 * This function get the channel CC from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getCC(uint8_t channel, uint16_t *value);

/*******************************************************************************
 * This function get the node acquisition signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer for store the received data
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getNodeAcqisitionSignal(uint8_t channel, uint16_t *value);

/*******************************************************************************
 * This function get the Sensor Control signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getSensorControl(uint8_t channel, uint16_t *sensorControl_l);

/*******************************************************************************
 * This function set the Sensor Control signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setSensorControl(uint8_t channel, uint16_t *sensorControl_l);

/*******************************************************************************
 * This function get the Threshold signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getThreshold(uint8_t channel, uint8_t *value);

/*******************************************************************************
 * This function set the Threshold signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setThreshold(uint8_t channel, uint8_t value);

/*******************************************************************************
 * This function get the Oversampling signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getOversampling(uint8_t channel, uint8_t *value);

/*******************************************************************************
 * This function set the Oversampling signal from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setOversampling(uint8_t channel, uint8_t value);

/*******************************************************************************
 * This function get the Gain value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getGain(uint8_t channel, uint8_t *value);

/*******************************************************************************
 * This function set the Gain value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setGain(uint8_t channel, uint8_t value);

/*******************************************************************************
 * This function get the Measurement Clock Frequency value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getMeasurementClkFreq(uint8_t channel, uint8_t *value);

/*******************************************************************************
 * This function set the Measurement Clock Frequency value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setMeasurementClkFreq(uint8_t channel, uint8_t value);

/*******************************************************************************
 * This function get the CSD value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getCSD(uint8_t channel, uint8_t *value);

/*******************************************************************************
 * This function set the CSD value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setCSD(uint8_t channel, uint8_t value);

/*******************************************************************************
 * This function get the hysteresis value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getHysteresis(uint8_t channel, uint8_t *value);

/*******************************************************************************
 * This function set the hysteresis value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setHysteresis(uint8_t channel, uint8_t value);

/*******************************************************************************
 * This function get the AKS value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_getAKS(uint8_t channel, uint8_t *value);

/*******************************************************************************
 * This function set the AKS value from the touch module.
 * @Param 1  :   Channel number
 * @Param 2  :   buffer to read the status
 * @return   :   status
*******************************************************************************/
MTCH2120_I2C_Status mtch2120_setAKS(uint8_t channel, uint8_t value);



#endif // MTCH2120_API_H
