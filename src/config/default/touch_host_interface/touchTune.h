/*******************************************************************************
  MPLAB Harmony Touch Host Interface Release
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    touchTune.h

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

#ifndef DATASTREAMER_H_
#define DATASTREAMER_H_

#include "mtch2120.h"

#include "touch_host_driver.h"

#define MTCH2120_DEVICE_TYPE    0x0Bu

#define DEF_TOUCH_DATA_STREAMER_ENABLE 1u

#if DEF_TOUCH_DATA_STREAMER_ENABLE == 1U

#define DV_HEADER    0x55u
#define DV_FOOTER    0xAAu

#define UART_RX_BUF_LENGTH      160u

#define HEADER_AWAITING     0u
#define HEADER_RECEIVED     160u
#define DATA_AWAITING       2u
#define DATA_RECEIVED       3u


#define SEND_DEBUG_DATA		0x8000u

#define ZERO                0x00u

typedef enum
{
	CONFIG_INFO =  0x00,
            
    // Configurations
    CHANNEL_CONFIGURATION   = 1u,
    SENSOR_CONTROL          = 2u,
    GROUP_CONFIGRUATION     = 3u,
    DEVICE_CONTROL          = 4u,
    LUMP_CONFIGURATION      = 5u,
    SLAVE_ADDRESS           = 6u,
    GPIO_CONFIG             = 7u,
    
    // Debug
    COMM_ERROR              = 128u,
    DEBUG_DATA              = 129u,
    DEVICE_ID               = 130u,
    DEVICE_STATUS           = 131u,
    GPIO_INPUT_UPDATE       = 132u,
    AUTOTUNE_FREQUENCIES    = 133u,
	EASYTUNE_THRESHOLD      = 134u
}FRAME_ID_VALUES;

typedef enum
{
	PC_REQUEST_CONFIG_DATA_FROM_MCU		= 0x01,		// sw read PC_REQUEST_CONFIG_DATA_FROM_MCU
	PC_SEND_CONFIG_DATA_TO_MCU			= 0x02,		// sw write	PC_SEND_CONFIG_DATA_TO_MCU
	MCU_SEND_TUNE_DATA_TO_PC			= 0x03,		// send debug data MCU_SEND_TUNE_DATA_TO_PC
	MCU_RESPOND_CONFIG_DATA_TO_PC		= 0x04, 	// sw read MCU_RESPOND_CONFIG_DATA_TO_PC
    PC_SEND_COMMAND_DATA_TO_MCU         = 0x05
}TYPE_ID_VALUES;



typedef enum
{
	AT42QT1110			   = 0x61,
	AT42QT2120			   = 0x62,
    MTCH2120               = 0x66
}DEVICE_TYPE;

typedef enum
{
	KEYS_MODULE               = 0x01,
    ERROR                     = 0x02,
    KEY_DEBUG_DATA_ID		  = 0x80,
    DEV_ID_DEBUG_DATA_ID      = 0x81,     
    NO_OF_BUTTONS_DEBUG_DATA  = 0x82,
    KEY_STATUS_DEBUG_DATA = 0x83
}DEBUG_CONFIG_FRAME_ID;

typedef enum
{
	SELF_CAP = 0x00,
	MUTUAL_CAP = 0x01
}ACQ_METHOD;

typedef enum
{
	PROTOCOL_VERSION = 0x02		// 0x00000010b - lsb 5 bits - Minor version, msb first 3 bits - Major version
}ROW_5;

void touchTuneProcess(void);
void touchUartTxComplete(uintptr_t lTouchUart);
void touchUartRxComplete(uintptr_t lTouchUart);
void touchTuneNewDataAvailable(void);
uint16_t touchTuneIsDebugDataSent(void);
uint8_t isStopToReadDebugData(void);

void touchTuneInit(void);

typedef void(*Function_Ptr)(void);

typedef struct
{
    uint8_t *memory;
    uint16_t size;
}MessageHandler;

#if ENABLE_TOUCH_TUNE ==1u

typedef struct
{
    uint8_t initError;
    uint8_t ConfigDataError;
    uint8_t DebugDataError;
    uint8_t StatusDataError;
}CommunicationError_t;

/* ======================== Touch Configurations ======================== */
extern mtch2120_DeviceInformation_t mtch2120_deviceInformation;
extern MTCH2120_Status_t mtch2120_Status;
extern uint16_t mtch2120_signal[DEF_NUM_SENSORS];
extern uint16_t mtch2120_reference[DEF_NUM_SENSORS];
extern uint8_t mtch2120_sensorState[DEF_NUM_SENSORS];
extern uint16_t mtch2120_compensationCapacitance[DEF_NUM_SENSORS]; // CC
 
extern mtch2120_SensorControl_t mtch2120_sensorControl[DEF_NUM_SENSORS];
extern uint8_t mtch2120_CSD[DEF_NUM_SENSORS];
extern uint8_t mtch2120_measurementClkFreq[DEF_NUM_SENSORS];
extern uint8_t mtch2120_filterlevel[DEF_NUM_SENSORS];
extern uint8_t mtch2120_threshold[DEF_NUM_SENSORS];  
extern uint8_t mtch2120_gain[DEF_NUM_SENSORS];
extern uint8_t mtch2120_hysteresis[DEF_NUM_SENSORS];
extern uint8_t mtch2120_AKSgroup[DEF_NUM_SENSORS];
extern mtch2120_GroupConfiguration_t mtch2120_groupConfiguration;
extern mtch2120_DeviceControl_t mtch2120_deviceControl;
 
// Lump Configuration
extern uint16_t mtch2120_lumpConfiguration_0;
extern uint16_t mtch2120_lumpConfiguration_1;
extern uint16_t mtch2120_lumpConfiguration_2;
extern uint16_t mtch2120_lumpConfiguration_3;
 
//GPIO Configuration
extern uint16_t mtch2120_gpioPins;
extern uint16_t mtch2120_gpioDirection;
extern uint16_t mtch2120_gpioOutputValue;
extern uint16_t mtch2120_gpioInputValue;
 
// computational parameter
extern int16_t mtch2120_delta[DEF_NUM_SENSORS];

/* ======================== End Touch Configurations ======================== */

extern CommunicationError_t mtch2120_communicationStatus;
#endif /* ENABLE_TOUCH_TUNE */

#endif

#endif /* DATASTREAMER_H_ */