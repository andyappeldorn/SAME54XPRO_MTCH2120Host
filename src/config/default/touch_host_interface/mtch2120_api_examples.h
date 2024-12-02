/*******************************************************************************
  MPLAB Harmony Touch Host Interface ${REL_VER} Release

  Company:
	Microchip Technology Inc.

  File Name:
	mtch2120_api_examples.h

  Summary:
	mtch2120 api examples

  Description:
    This file contains various API usage examples for MTCH2120.
    Copy the required section to application's ".c" file and uncomment 
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 * Copyright (C) ${REL_YEAR} Microchip Technology Inc. and its subsidiaries.
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


#ifndef MTCH2120_API_EXAMPLES_H
#define MTCH2120_API_EXAMPLES_H

/////////////// Get Device Id ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t deviceID_l = 0u;
//uint8_t offset = 0u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_ID | offset), (uint8_t*)&deviceID_l, (uint8_t)sizeof(deviceID_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Device Version ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t version_l = 0u;
//uint8_t offset = 1u;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_ID | offset), (uint8_t*)&version_l, (uint8_t)sizeof(version_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Device Feature ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t deviceFeature_l[3u] = {0u , 0u, 0u};
//uint8_t offset = 2u;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_ID | offset), (uint8_t*)deviceFeature_l, (uint8_t)sizeof(deviceFeature_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get CRC ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t crc_l = 0u;
//uint8_t offset = 5u;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_ID | offset), (uint8_t*)&crc_l, (uint8_t)sizeof(crc_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Device Status ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t deviceStatus_l = 0u;
//uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_STATUS | offset), (uint8_t*)&deviceStatus_l, (uint8_t)sizeof(deviceStatus_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Button Status Mask ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t buttonStatusMask_l = 0u;
//uint8_t offset = 2;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_STATUS | offset), (uint8_t*)&buttonStatusMask_l, (uint8_t)sizeof(buttonStatusMask_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Signals from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t signal_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_NODE_ACQ_SIGNALS | offset), (uint8_t*)signal_l, (uint8_t)sizeof(signal_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Signal from Channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t signal_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getNodeAcqisitionSignal(channelNumber, (uint16_t*)&signal_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Reference from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t reference_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_CHANNEL_REFERENCE | offset), (uint8_t*)reference_l, (uint8_t)sizeof(reference_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Reference from Channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t reference_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getChannelReference(channelNumber, (uint16_t*)&reference_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}

/////////////// Get Sensor State from all the channels////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorState_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_STATE | offset), (uint8_t*)sensorState_l, (uint8_t)sizeof(sensorState_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get Sensor State from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorState_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getSensorState(channelNumber, (uint8_t*)&sensorState_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get CC value from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t CC_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_NODE_CC | offset), (uint8_t*)CC_l, (uint8_t)sizeof(CC_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}

/////////////// Get CC value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t CC_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getCC(channelNumber, (uint16_t*)&CC_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}

/////////////// Disable Button 0 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorControl_l = 0u;
//const uint8_t channelNumber = 0u;
//const uint8_t offset = (sizeof(sensorControl_l) * channelNumber);
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
////read configuration from 2120 device
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//if(i2cStatus == SUCCESS)
//{
//    // disabling button 0
//    sensorControl_l &= ~(SENCTRL_BTTN_EN_MASK);
//
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        saveConfig();
//        resetDevice();
//    }
//    else
//    {
//        // failed to write
//    }
//}
//else
//{
//    // failed to read
//}

/////////////// Enable Button 0 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorControl_l = 0u;
//const uint8_t channelNumber = 0u;
//const uint8_t offset = (sizeof(sensorControl_l) * channelNumber);
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
////read configuration from 2120 device
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//if(i2cStatus == SUCCESS)
//{
//    // enabling button 0
//    sensorControl_l |= SENCTRL_BTTN_EN_MASK;
//
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        saveConfig();
//        resetDevice();
//    }
//    else
//    {
//        // failed to write
//    }
//}
//else
//{
//    // failed to read
//}

/////////////// Suspend Button 0 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorControl_l = 0u;
//const uint8_t channelNumber = 0u;
//const uint8_t offset = (sizeof(sensorControl_l) * channelNumber);
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
////read configuration from 2120 device
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//if(i2cStatus == SUCCESS)
//{
//    // suspending button 0
//    sensorControl_l |= SENCTRL_BTTN_SUSPEND_MASK;
//
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // save and reset not required
//    }
//    else
//    {
//        // failed to write
//    }
//}
//else
//{
//    // failed to read
//}


/////////////// Clear Suspend Button 0 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorControl_l = 0u;
//const uint8_t channelNumber = 0u;
//const uint8_t offset = (sizeof(sensorControl_l) * channelNumber);
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
////read configuration from 2120 device
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//if(i2cStatus == SUCCESS)
//{
//    // clear suspend button 0
//    sensorControl_l &= ~SENCTRL_BTTN_SUSPEND_MASK;
//
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // save and reset not required
//    }
//    else
//    {
//        // failed to write
//    }
//}
//else
//{
//    // failed to read
//}


/////////////// Calibrate Button 0 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorControl_l = 0u;
//const uint8_t channelNumber = 0u;
//const uint8_t offset = (sizeof(sensorControl_l) * channelNumber);
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
////read configuration from 2120 device
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//if(i2cStatus == SUCCESS)
//{
//    // clear suspend button 0
//    sensorControl_l |= SENCTRL_BTTN_CAL_MASK;
//
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // save and reset not required
//    }
//    else
//    {
//        // failed to write
//    }
//}
//else
//{
//    // failed to read
//}


/////////////// Set Button 0 as Low Power Button ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorControl_l = 0u;
//const uint8_t channelNumber = 0u;
//const uint8_t offset = (sizeof(sensorControl_l) * channelNumber);
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
////read configuration from 2120 device
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//if(i2cStatus == SUCCESS)
//{
//    // button 0 as low power
//    sensorControl_l |= SENCTRL_BTTN_LP_MASK;
//
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // save and reset not required
//    }
//    else
//    {
//        // failed to write
//    }
//}
//else
//{
//    // failed to read
//}

/////////////// Clear Low Power Button (Button 0) ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sensorControl_l = 0u;
//const uint8_t channelNumber = 0u;
//const uint8_t offset = (sizeof(sensorControl_l) * channelNumber);
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
////read configuration from 2120 device
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//if(i2cStatus == SUCCESS)
//{
//    // button 0 as low power
//    sensorControl_l &= ~SENCTRL_BTTN_LP_MASK;
//
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | offset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // save and reset not required
//    }
//    else
//    {
//        // failed to write
//    }
//}
//else
//{
//    // failed to read
//}


/////////////// Get CSD value from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t CSD_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_CSD | offset), (uint8_t*)CSD_l, (uint8_t)sizeof(CSD_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Get CSD value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t CSD_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getCSD(channelNumber, (uint8_t*)&CSD_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// Set CSD value to all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t CSD_l[DEF_NUM_SENSORS] = 
//{
//    1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u
//};
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_CSD | offset), (uint8_t*)CSD_l, (uint8_t)sizeof(CSD_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}

/////////////// Set CSD value to channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t CSD_l = 2u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_setCSD(channelNumber, CSD_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Measurement Clock Frequency value from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t measurementClkFreq_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_MEASUREMENT_CLK_FREQ | offset), (uint8_t*)measurementClkFreq_l, (uint8_t)sizeof(measurementClkFreq_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}

/////////////// get Measurement Clock Frequency value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t measurementClkFreq_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getMeasurementClock(channelNumber, (uint8_t*)&measurementClkFreq_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Measurement Clock Frequency value to all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t measurementClkFreq_l[DEF_NUM_SENSORS] = 
//{
//    3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u
//};
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_MEASUREMENT_CLK_FREQ | offset), (uint8_t*)measurementClkFreq_l, (uint8_t)sizeof(measurementClkFreq_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Measurement Clock Frequency value to channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t measurementClkFreq_l = 1u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_setMeasurementClock(channelNumber, measurementClkFreq_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Filterlevel from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t filterLevel_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_OVERSAMPLING | offset), (uint8_t*)filterLevel_l, (uint8_t)sizeof(filterLevel_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Filterlevel from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t filterLevel_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getOversampling(channelNumber, (uint8_t*)&filterLevel_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Filterlevel to all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t filterLevel_l[DEF_NUM_SENSORS] = 
//{
//    5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u
//};
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_OVERSAMPLING | offset), (uint8_t*)filterLevel_l, (uint8_t)sizeof(filterLevel_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}

/////////////// set Filterlevel to channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t filterLevel_l = 3u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_setOversampling(channelNumber, filterLevel_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Threshold value from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t threshold_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_THRESHOLD | offset), (uint8_t*)threshold_l, (uint8_t)sizeof(threshold_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Threshold value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t threshold_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getThreshold(channelNumber, (uint8_t*)&threshold_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Threshold to all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t threshold_l[DEF_NUM_SENSORS] = 
//{
//    50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u, 50u
//};
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_THRESHOLD | offset), (uint8_t*)threshold_l, (uint8_t)sizeof(threshold_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Threshold value to channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t threshold_l = 10u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_setThreshold(channelNumber, threshold_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Gain value from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t gain_l[DEF_NUM_SENSORS];
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GAIN | offset), (uint8_t*)gain_l, (uint8_t)sizeof(gain_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Gain value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t gain_l = 0u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_getGain(channelNumber, (uint8_t*)&gain_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Gain value to all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t gain_l[DEF_NUM_SENSORS] = 
//{
//    1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u
//};
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GAIN | offset), (uint8_t*)gain_l, (uint8_t)sizeof(gain_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Gain value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t gain_l = 3u;
//const uint8_t channelNumber = 1u;
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_setGain(channelNumber, gain_l);
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}

/////////////// get Hysteresis value from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t hysteresis_l[DEF_NUM_SENSORS];
//    uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_HYSTERESIS | offset), (uint8_t*)hysteresis_l, (uint8_t)sizeof(hysteresis_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get Hysteresis value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t hysteresis_l = 0u;
//    const uint8_t channelNumber = 1u;
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_getHysteresis(channelNumber, (uint8_t*)&hysteresis_l);
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// set Hysteresis value to all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t hysteresis_l[DEF_NUM_SENSORS] = 
//    {
//        3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u, 3u
//    };
//    uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_HYSTERESIS | offset), (uint8_t*)hysteresis_l, (uint8_t)sizeof(hysteresis_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set Hysteresis value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t hysteresis_l = 1u;
//    const uint8_t channelNumber = 1u;
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_setHysteresis(channelNumber, hysteresis_l);
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }



/////////////// get AKS value from all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t aks_l[DEF_NUM_SENSORS];
//    uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_AKS | offset), (uint8_t*)aks_l, (uint8_t)sizeof(aks_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }



/////////////// get AKS value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t aks_l = 0u;
//    const uint8_t channelNumber = 1u;
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_getAKS(channelNumber, (uint8_t*)&aks_l);
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set AKS value to all the channels ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t aks_l[DEF_NUM_SENSORS] = 
//    {
//        5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u, 5u
//    };
//    uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_AKS | offset), (uint8_t*)aks_l, (uint8_t)sizeof(aks_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set AKS value from channel 1 ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t aks_l = 2u;
//    const uint8_t channelNumber = 1u;
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_setAKS(channelNumber, aks_l);
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get Measurement Period ////////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t measurementPeriod_l = 0u;
//uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&measurementPeriod_l, (uint8_t)sizeof(measurementPeriod_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// set Measurement Period ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t measurementPeriod_l = 50u;
//    uint8_t offset = 0;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&measurementPeriod_l, (uint8_t)sizeof(measurementPeriod_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get Low power Measurement Period ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t lowPowerMeasurementPeriod_l = 0u;
//    uint8_t offset = 2u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&lowPowerMeasurementPeriod_l, (uint8_t)sizeof(lowPowerMeasurementPeriod_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set Low power Measurement Period ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t lowPowerMeasurementPeriod_l = 200u;
//    uint8_t offset = 2u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&lowPowerMeasurementPeriod_l, (uint8_t)sizeof(lowPowerMeasurementPeriod_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get TimeoutConfig ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t timeoutConfig_l = 0u;
//    uint8_t offset = 4u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&timeoutConfig_l, (uint8_t)sizeof(timeoutConfig_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set TimeoutConfig ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t timeoutConfig_l = 10000u;
//    uint8_t offset = 4u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&timeoutConfig_l, (uint8_t)sizeof(timeoutConfig_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// get ReburstMode ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t reburstMode_l = 0u;
//    uint8_t offset = 6u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&reburstMode_l, (uint8_t)sizeof(reburstMode_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set ReburstMode ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t reburstMode_l = 2u;
//    uint8_t offset = 6u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&reburstMode_l, (uint8_t)sizeof(reburstMode_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// get Detect Integration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t di_l = 0u;
//    uint8_t offset = 7u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&di_l, (uint8_t)sizeof(di_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// set Detect Integration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t di_l = 4u;
//    uint8_t offset = 7u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&di_l, (uint8_t)sizeof(di_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get AntiTouchIntegration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t antiTouchIntegration_l = 0u;
//    uint8_t offset = 8u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&antiTouchIntegration_l, (uint8_t)sizeof(antiTouchIntegration_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set AntiTouchIntegration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t antiTouchIntegration_l = 5u;
//    uint8_t offset = 8u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&antiTouchIntegration_l, (uint8_t)sizeof(antiTouchIntegration_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// get MaxOnTime ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t maxOnTime_l = 0u;
//    uint8_t offset = 9u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&maxOnTime_l, (uint8_t)sizeof(maxOnTime_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set MaxOnTime ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t maxOnTime_l = 10u;
//    uint8_t offset = 9u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&maxOnTime_l, (uint8_t)sizeof(maxOnTime_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// get Drift Hold Time ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t driftHoldTime_l = 0u;
//    uint8_t offset = 0x0Au;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&driftHoldTime_l, (uint8_t)sizeof(driftHoldTime_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set Drift Hold Time ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t driftHoldTime_l = 0u;
//    uint8_t offset = 0x0Au;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&driftHoldTime_l, (uint8_t)sizeof(driftHoldTime_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get TouchDrift ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t touchDrift_l = 0u;
//    uint8_t offset = 0x0Bu;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&touchDrift_l, (uint8_t)sizeof(touchDrift_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// set TouchDrift ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t touchDrift_l = 6u;
//    uint8_t offset = 0x0Bu;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&touchDrift_l, (uint8_t)sizeof(touchDrift_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get AntiTouchDrift ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t antiTouchDrift_l = 0u;
//    uint8_t offset = 0x0Cu;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&antiTouchDrift_l, (uint8_t)sizeof(antiTouchDrift_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set AntiTouchDrift ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t antiTouchDrift_l = 10u;
//    uint8_t offset = 0x0Cu;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&antiTouchDrift_l, (uint8_t)sizeof(antiTouchDrift_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get AntiTouchRecalThreshold ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t antiTouchRecalThreshold_l = 0u;
//    uint8_t offset = 0x0Du;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&antiTouchRecalThreshold_l, (uint8_t)sizeof(antiTouchRecalThreshold_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set AntiTouchRecalThreshold ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t antiTouchRecalThreshold_l = 0u;
//    uint8_t offset = 0x0Du;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&antiTouchRecalThreshold_l, (uint8_t)sizeof(antiTouchRecalThreshold_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get Noise Threshold ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t noiseThreshold_l = 0u;
//    uint8_t offset = 0x0Eu;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&noiseThreshold_l, (uint8_t)sizeof(noiseThreshold_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set Noise Threshold ////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t noiseThreshold_l = 25u;
//uint8_t offset = 0x0Eu;    // calculated as per mtch2120 datasheet
//MTCH2120_I2C_Status i2cStatus = SUCCESS;
//
//i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&noiseThreshold_l, (uint8_t)sizeof(noiseThreshold_l));
//if(i2cStatus == SUCCESS)
//{
//    // success
//}
//else
//{
//    // failed to read
//}


/////////////// get Noise Integration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t noiseIntegration_l = 0u;
//    uint8_t offset = 0x10u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&noiseIntegration_l, (uint8_t)sizeof(noiseIntegration_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set Noise Integration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t noiseIntegration_l = 7u;
//    uint8_t offset = 0x10u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&noiseIntegration_l, (uint8_t)sizeof(noiseIntegration_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// get Frequencies ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t frequencies_l[3u];
//    uint8_t offset = 0x11u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)frequencies_l, (uint8_t)sizeof(frequencies_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// set Frequencies ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint8_t frequencies_l[3u] = {1u, 2u, 3u};
//    uint8_t offset = 0x11u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GROUP_CONFIGURATION | offset), (uint8_t*)&frequencies_l, (uint8_t)sizeof(frequencies_l));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Calibrate All the Channels ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_CAL_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Low Power ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_LP_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Low Power ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_LP_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Drift LowPower LumpButtons ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_DLPLB_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Drift LowPower LumpButtons ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_DLPLB_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Driven Shield ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_DS_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Driven Shield ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_DS_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Driven Shield+ ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_DSP_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Driven Shield+ ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_DSP_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Drift Gain ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_DRIFTGAIN_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Drift Gain ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_DRIFTGAIN_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Frequency Hop ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_FREQHOP_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Frequency Hop ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_FREQHOP_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Auto Tune ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_AT_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Auto Tune ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_AT_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable Easy Tune ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_ET_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable Easy Tune ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_ET_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable WatchDog ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_WDT_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable WatchDog ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_WDT_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Enable BrownOutDetect ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_BOD_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Disable BrownOutDetect ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l &= ~DEVCTRL_BOD_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Reset to Default Configuration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_SMCFG_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Save Configuration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_SAVE_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Reset Device (Software reset) ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t deviceControl_l = 0u;
//    uint8_t offset = 0u;    // calculated as per mtch2120 datasheet
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        deviceControl_l |= DEVCTRL_RESET_MASK;
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | offset), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // success
//        }
//        else
//        {
//            // write failure
//        }
//    }
//    else
//    {
//        // failed to read
//    }

/////////////// Set Lump (Lump 1) Configuration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t mtch2120_lumpConfig1 = 24u;                                    // button 3 and 4 are part of lump 1
//    const uint8_t lumpChannel = 1u;
//    uint8_t lumpOffset = (lumpChannel * 0x08u);                             // calculated as per mtch2120 datasheet
//    
//    uint8_t sensorControl_l = 0u;
//    const uint8_t sensorControlChannel = 14u;                               // channel 14 is lump 1
//    uint8_t sensorControlOffset = (sensorControlChannel * 0x01u);           // calculated as per mtch2120 datasheet
//    
//    uint16_t deviceControl_l = 0u;
//    unsigned int delay = 0u;
//    
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    // enabling button 12 / Lump 1
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | sensorControlOffset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        sensorControl_l |= SENCTRL_BTTN_EN_MASK;  // enable channel 12
//        
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | sensorControlOffset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // configuring lump
//            i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_LUMP_CONFIG | lumpOffset), (uint8_t*)&mtch2120_lumpConfig1, (uint8_t)sizeof(mtch2120_lumpConfig1));
//            if(i2cStatus == SUCCESS)
//            {
//                // saving the configuration to kit 
//                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                if(i2cStatus == SUCCESS)
//                {
//                    deviceControl_l |= DEVCTRL_SAVE_MASK;
//                    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                    if(i2cStatus == SUCCESS)
//                    {
//                        // success
//                    }
//                    else
//                    {
//                        // write failure
//                    }
//                }
//                else
//                {
//                    // failed to read
//                }
//
//                // give some time delay to save the configuration
//                delay = 0u;
//                do
//                {
//                    delay++;
//                }while(delay < 65000u);
//
//                // Reset the device
//                deviceControl_l = 0u;
//                i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                if(i2cStatus == SUCCESS)
//                {
//                    deviceControl_l |= DEVCTRL_RESET_MASK;
//                    i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                    if(i2cStatus == SUCCESS)
//                    {
//                        // success
//                    }
//                    else
//                    {
//                        // write failure
//                    }
//                }
//                else
//                {
//                    // failed to read
//                }
//                
//                // give some time delay to reboot the device
//                delay = 0u;
//                do
//                {
//                    delay++;
//                }while(delay < 65000u);
//            }
//            else
//            {
//                // write lump config failed
//            }
//        }
//        else
//        {
//            // write sensorControl_l failed
//        }
//        
//    }
//    else
//    {
//        // read sensorControl_l failed
//    }


/////////////// get Lump (Lump 1) Configuration ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t mtch2120_lumpConfig1 = 0u;
//    
//    const uint8_t lumpChannel = 1u;
//    uint8_t offset = (lumpChannel * 0x08u);    // calculated as per mtch2120 datasheet
//    
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_LUMP_CONFIG | offset), (uint8_t*)&mtch2120_lumpConfig1, (uint8_t)sizeof(mtch2120_lumpConfig1));
//    if(i2cStatus == SUCCESS)
//    {
//        // success
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Set GPIO 0 As Output with High state ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t mtch2120_gpio = 0u;                                                     // Configuring Button 0 As Gpio 0
//    const uint8_t gpioPinOffset  = 0u;                                               // calculated as per mtch2120 datasheet
//    const uint8_t gpioDirOffset  = 0x08u;                                            // calculated as per mtch2120 datasheet
//    const uint8_t gpioOutOffset  = 0x10u;                                            // calculated as per mtch2120 datasheet
//    
//    uint8_t sensorControl_l = 0u;
//    const uint8_t sensorControlChannel = 0u;                                        // Button 0
//    uint8_t sensorControlOffset = (sensorControlChannel * 0x01u);                   // calculated as per mtch2120 datasheet
//    
//    uint16_t deviceControl_l = 0u;
//    
//    unsigned int delay = 0u;
//    
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | sensorControlOffset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        sensorControl_l &= ~SENCTRL_BTTN_EN_MASK;  // disable button 0
//        
//        // disabling button 0
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | sensorControlOffset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // configuring GPIO pin mask
//            i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioPinOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//            if(i2cStatus == SUCCESS)
//            {
//                mtch2120_gpio |= ((uint16_t)1u << (uint16_t)0u);    // GPIO pin bit-mask
//                
//                // configuring gpio 0
//                i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioPinOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                if(i2cStatus == SUCCESS)
//                {
//                    // saving the configuration to kit 
//                    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                    if(i2cStatus == SUCCESS)
//                    {
//                        deviceControl_l |= DEVCTRL_SAVE_MASK;
//                        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                        if(i2cStatus == SUCCESS)
//                        {
//                            // give some time delay to save the configuration
//                            delay = 0u;
//                            do
//                            {
//                                delay++;
//                            }while(delay < 65000u);
//                            
//                            // Reset the device
//                            deviceControl_l = 0u;
//                            i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                            if(i2cStatus == SUCCESS)
//                            {
//                                deviceControl_l |= DEVCTRL_RESET_MASK;
//                                i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                                if(i2cStatus == SUCCESS)
//                                {
//                                    // give some time delay to reboot the device
//                                    delay = 0u;
//                                    do
//                                    {
//                                        delay++;
//                                    }while(delay < 65000u);
//                                    
//                                    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioDirOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                                    if(i2cStatus == SUCCESS)
//                                    {
//                                        mtch2120_gpio |= ((uint16_t)1u << (uint16_t)0u);    // GPIO direction bit-mask - Configuring as output
//
//                                        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioDirOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                                        if(i2cStatus == SUCCESS)
//                                        {
//                                            i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioOutOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                                            if(i2cStatus == SUCCESS)
//                                            {
//                                                mtch2120_gpio |= ((uint16_t)1u << (uint16_t)0u);    // GPIO out bit-mask - GPIO 0 as High
//
//                                                i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioOutOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                                                if(i2cStatus == SUCCESS)
//                                                {
//                                                    // success
//                                                }
//                                                else
//                                                {
//                                                    // failed to write out register
//                                                }
//                                            }
//                                            else
//                                            {
//                                                // failed to read out register
//                                            }
//                                        }
//                                        else
//                                        {
//                                            // failed to write GPIO direction
//                                        }
//                                    }
//                                    else
//                                    {
//                                        // failed to read GPIO direction
//                                    }
//                                }
//                                else
//                                {
//                                    // write failure
//                                }
//                            }
//                            else
//                            {
//                                // failed to read
//                            }
//                        }
//                        else
//                        {
//                            // write failure
//                        }
//                    }
//                    else
//                    {
//                        // failed to read
//                    } 
//                }
//                else
//                {
//                    // failed to write gpio pin register
//                }
//            }
//            else
//            {
//                // failed to read gpio pin register
//            }
//        }
//        else
//        {
//            // failed to write sensor control
//        }
//    }
//    else
//    {
//        // failed to read
//    }


/////////////// Set GPIO 0 As Input with reading the input state ////////////////////////////////////////////////////////////////////////////////////////////////
//    uint16_t mtch2120_gpio = 0u;                                                     // Configuring Button 0 As Gpio 0
//    const uint8_t gpioPinOffset  = 0u;                                               // calculated as per mtch2120 datasheet
//    const uint8_t gpioDirOffset  = 0x08u;                                            // calculated as per mtch2120 datasheet
//    const uint8_t gpioInOffset   = 0x18u;                                            // calculated as per mtch2120 datasheet
//    
//    uint16_t inputBuffer = 0u;
//    
//    uint8_t sensorControl_l = 0u;
//    const uint8_t sensorControlChannel = 0u;                                        // Button 0
//    uint8_t sensorControlOffset = (sensorControlChannel * 0x01u);                   // calculated as per mtch2120 datasheet
//    
//    uint16_t deviceControl_l = 0u;
//    
//    unsigned int delay = 0u;
//    
//    MTCH2120_I2C_Status i2cStatus = SUCCESS;
//    
//    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_SENSOR_CONTROL | sensorControlOffset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//    if(i2cStatus == SUCCESS)
//    {
//        sensorControl_l &= ~SENCTRL_BTTN_EN_MASK;  // disable button 0
//        
//        // disabling button 0
//        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_SENSOR_CONTROL | sensorControlOffset), (uint8_t*)&sensorControl_l, (uint8_t)sizeof(sensorControl_l));
//        if(i2cStatus == SUCCESS)
//        {
//            // configuring GPIO pin mask
//            i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioPinOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//            if(i2cStatus == SUCCESS)
//            {
//                mtch2120_gpio &= ~((uint16_t)1u << (uint16_t)0u);    // GPIO pin bit-mask
//                
//                // configuring gpio 0
//                i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioPinOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                if(i2cStatus == SUCCESS)
//                {
//                    // saving the configuration to kit 
//                    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                    if(i2cStatus == SUCCESS)
//                    {
//                        deviceControl_l |= DEVCTRL_SAVE_MASK;
//                        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                        if(i2cStatus == SUCCESS)
//                        {
//                            // give some time delay to save the configuration
//                            delay = 0u;
//                            do
//                            {
//                                delay++;
//                            }while(delay < 65000u);
//                            
//                            // Reset the device
//                            deviceControl_l = 0u;
//                            i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                            if(i2cStatus == SUCCESS)
//                            {
//                                deviceControl_l |= DEVCTRL_RESET_MASK;
//                                i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_DEVICE_CONTROL | 0u), (uint8_t*)&deviceControl_l, (uint8_t)sizeof(deviceControl_l));
//                                if(i2cStatus == SUCCESS)
//                                {
//                                    // give some time delay to reboot the device
//                                    delay = 0u;
//                                    do
//                                    {
//                                        delay++;
//                                    }while(delay < 65000u);
//                                    
//                                    i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioDirOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                                    if(i2cStatus == SUCCESS)
//                                    {
//                                        mtch2120_gpio |= ((uint16_t)1u << (uint16_t)0u);    // GPIO direction bit-mask - Configuring as output
//
//                                        i2cStatus = mtch2120_writeToMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioDirOffset), (uint8_t*)&mtch2120_gpio, (uint8_t)sizeof(mtch2120_gpio));
//                                        if(i2cStatus == SUCCESS)
//                                        {
//                                            i2cStatus = mtch2120_readFromMemory(((uint16_t)ADDR_GPIO_CONFIG | gpioInOffset), (uint8_t*)&inputBuffer, (uint8_t)sizeof(inputBuffer));
//                                            if(i2cStatus == SUCCESS)
//                                            {
//                                                // SUCCESS
//                                                // read the input pin status from "inputBuffer" buffer...
//                                            }
//                                            else
//                                            {
//                                                // failed to read out register
//                                            }
//                                        }
//                                        else
//                                        {
//                                            // failed to write GPIO direction
//                                        }
//                                    }
//                                    else
//                                    {
//                                        // failed to read GPIO direction
//                                    }
//                                }
//                                else
//                                {
//                                    // write failure
//                                }
//                            }
//                            else
//                            {
//                                // failed to read
//                            }
//                        }
//                        else
//                        {
//                            // write failure
//                        }
//                    }
//                    else
//                    {
//                        // failed to read
//                    } 
//                }
//                else
//                {
//                    // failed to write gpio pin register
//                }
//            }
//            else
//            {
//                // failed to read gpio pin register
//            }
//        }
//        else
//        {
//            // failed to write sensor control
//        }
//    }
//    else
//    {
//        // failed to read
//    }
#endif // MTCH2120_API_EXAMPLES_H