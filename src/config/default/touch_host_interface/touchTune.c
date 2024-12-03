/*******************************************************************************
  MPLAB Harmony Touch Host Interface v1.1.0 Release

  @Company
    Microchip Technology Inc.

  @File Name
    TouchTune.c

  @Summary
    QTouch Modular Library

  @Description
    Provides the two way datastreamer protocol implementation, transmission of
          module data to data visualizer software using UART port.

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


/*----------------------------------------------------------------------------
  include files
----------------------------------------------------------------------------*/
#include "../touch_host_interface/touchTune.h"

#if ENABLE_TOUCH_TUNE == 1u

mtch2120_DeviceInformation_t mtch2120_deviceInformation;
MTCH2120_Status_t mtch2120_Status;
uint16_t mtch2120_signal[DEF_NUM_SENSORS];
uint16_t mtch2120_reference[DEF_NUM_SENSORS];
uint8_t mtch2120_sensorState[DEF_NUM_SENSORS];
uint16_t mtch2120_compensationCapacitance[DEF_NUM_SENSORS]; // CC

mtch2120_SensorControl_t mtch2120_sensorControl[DEF_NUM_SENSORS];
uint8_t mtch2120_CSD[DEF_NUM_SENSORS];
uint8_t mtch2120_measurementClkFreq[DEF_NUM_SENSORS];
uint8_t mtch2120_filterlevel[DEF_NUM_SENSORS];
uint8_t mtch2120_threshold[DEF_NUM_SENSORS];  
uint8_t mtch2120_gain[DEF_NUM_SENSORS];
uint8_t mtch2120_hysteresis[DEF_NUM_SENSORS];
uint8_t mtch2120_AKSgroup[DEF_NUM_SENSORS];
mtch2120_GroupConfiguration_t mtch2120_groupConfiguration;
mtch2120_DeviceControl_t mtch2120_deviceControl;

// Lump Configuration
uint16_t mtch2120_lumpConfiguration_0;
uint16_t mtch2120_lumpConfiguration_1;
uint16_t mtch2120_lumpConfiguration_2;
uint16_t mtch2120_lumpConfiguration_3;

//GPIO Configuration
uint16_t mtch2120_gpioPins;
uint16_t mtch2120_gpioDirection;
uint16_t mtch2120_gpioOutputValue;
uint16_t mtch2120_gpioInputValue;

// computational parameter
int16_t mtch2120_delta[DEF_NUM_SENSORS];

// communication status
CommunicationError_t mtch2120_communicationStatus;

static uint8_t stopQueryDebugData = 0u;

typedef struct
{
    uint8_t threshold_dv;
    uint8_t oversampling_dv;
    uint8_t gain_dv;
    uint8_t csd_dv;
    uint8_t measurementClkFreq;
    uint8_t channel_hysteresis_dv;
    uint8_t channel_aks_group_dv;
}ChannelConfiguration_t;

typedef struct __attribute__((packed))
{
    uint8_t sensor_state_dv;
    uint16_t channelReference_dv;
    uint16_t nodeAcqSignals_dv;
    int16_t delta_dv;
    uint16_t cc_dv;
}DebugData_t;

typedef struct __attribute__((packed))
{
    unsigned int lump0:1;
    unsigned int lump1:1;
    unsigned int lump2:1;
    unsigned int lump3:1;
    unsigned int reserved:4;
}DV_LumpConfig_t;

typedef struct __attribute__((packed))
{
    unsigned int pin:1;
    unsigned int direction:1;
    unsigned int out_buffer:1;
    unsigned int reserved:5;
}DV_gpio_t;

#if DEF_TOUCH_DATA_STREAMER_ENABLE == 1U
void uart_send_frame_header(uint8_t trans_type, uint8_t frame, uint16_t frame_len);
void uart_recv_frame_data(uint8_t frame_id, uint16_t len);
void UART_Write(uint8_t data);
void uart_send_data_wait(uint8_t data);
void uart_send_data(uint8_t con_or_debug, uint8_t *data_ptr, uint8_t data_len);

uint8_t uart_get_char(void);
void uart_get_string(uint8_t *data_recv, uint16_t len);

// button functionality
void uart_execute_command(uint8_t channel, uint8_t command, uint8_t onOff);
void uart_execute_command_allchannel(uint8_t command, uint8_t onOff);
void uart_process_command_received(uint8_t frame_id);

// following functions will handle configuration memory regions
void copy_slaveAddress(uint8_t channel_num);
void copy_channel_configuration(uint8_t channel_num);
void copy_sensorControl_configuration(uint8_t channel_num);
void copy_group_configuration(uint8_t channel_num);
void copy_deviceControl_configuration(uint8_t channel_num);
void copy_lump_configuration(uint8_t channel_num);
void copy_gpio_configuration(uint8_t channel_num);
void copy_channel_config_data(uint8_t id, uint8_t channel);

// following functions will handle debug memory regions 
void copy_communicationStatus(uint8_t channel_num);
void copy_debugData(uint8_t channel_num);
void copy_deviceInformation(uint8_t channel_num);
void copy_status(uint8_t channel_num);
void copy_gpioInputStatus(uint8_t channel_num);
void copy_autoTuneFreqencies(uint8_t channel_num);
void read_threshold(uint8_t channel_num);

// following functions will handle configuration memory regions
// DV mem update to MCU mem update.
void update_channel_configuration(uint8_t channel_num);
void update_sensorControl_configuration(uint8_t channel_num);
void update_group_configuration(uint8_t channel_num);
void update_deviceControl_configuration(uint8_t channel_num);
void update_lump_configuration(uint8_t channel_num);
void update_slaveAddress(uint8_t channel_num);
void update_gpioConfig(uint8_t channel_num);

#define NO_OF_CONFIG_FRAME_ID       (8U)
#define STREAMING_DEBUG_DATA        (1U)
#define STREAMING_CONFIG_DATA       (2U)
#define PROJECT_CONFIG_DATA_LEN     (10U)
#define OUTPUT_MODULE_CNT           (7U)

#define DEF_NUM_KEYS                DEF_NUM_SENSORS
#define DEF_NUM_EXCESS_PIN          1u      // user configure the unused pin as GPIO

#define DEBUG_DATA_PER_CH_LEN       sizeof(sensorData_t)
#define TOTAL_RUN_TIME_DATA_LEN     (DEBUG_DATA_PER_CH_LEN * DEF_NUM_KEYS)

enum buttonCommand
{
    BT_CALIBRATE_INDIVIDUAL_BUTTON = 1u,
    BT_SUSPEND_BUTTON                   = 2u,
    BT_ENABLE_BUTTON                    = 3u,
    BT_LOWPOWER_BUTTON                  = 4u,
    BT_CALIBRATE_ALL                    = 5u,
    BT_ENABLE_LOWPOWER                  = 6u,
    BT_LUMP_BTTN_DRIFT_ENABLE           = 7u,
    BT_DRIVEN_SHIELD                    = 8u,
    BT_DRIVEN_SHIELD_PLUS               = 9u,
    BT_DRIFT_GAIN                       = 10u,
    BT_FREQ_HOP                         = 11u,
    BT_AUTOTUNE                         = 12u,
    BT_EASYTUNE                         = 13u,
    BT_WDT                              = 14u,
    BT_BOD                              = 15u,
    BT_SAVE_DEFAULT                     = 16u,
    BT_SAVE                             = 17u,
    BT_RESET                            = 18u,
    BT_LUMP_0                           = 19u,
    BT_LUMP_1                           = 20u,
    BT_LUMP_2                           = 21u,
    BT_LUMP_3                           = 22u,
    STOP_QUERY_DEBUG_DATA               = 23u
};

// ----------------------------------------------------------------------------
// configuration - memory
// ----------------------------------------------------------------------------
static ChannelConfiguration_t dv_channelConfiguration;                          // MAPPED TO ID 1
static mtch2120_SensorControl_t dv_sensorControl;                               // MAPPED TO ID 2
static mtch2120_GroupConfiguration_t dv_gropuConfiguration;                     // MAPPED TO ID 3
static mtch2120_DeviceControl_t dv_deviceControl;                               // MAPPED TO ID 4
static DV_LumpConfig_t dv_lumpConfiguration;                                    // MAPPED TO ID 5
static uint8_t dv_slaveAddress;                                                 // MAPPED TO ID 6
static DV_gpio_t dv_gpioConfiguration;                                          // MAPPED TO ID 7

// ----------------------------------------------------------------------------
// debug - memory
// ----------------------------------------------------------------------------
static CommunicationError_t dv_commError;                                       // MAPPED TO ID 128
static DebugData_t dv_debugData;                                                // MAPPED TO ID 129
static mtch2120_DeviceInformation_t dv_deviceInformation;                       // MAPPED TO ID 130
static MTCH2120_Status_t dv_status;                                             // MAPPED TO ID 131
static uint16_t dv_gpioInputValue;                                              // MAPPED TO ID 132
static uint8_t dv_autoTuneFrequencies[NUM_FREQ_STEPS];                          // MAPPED TO ID 133

// configuration mapping
#define CONFIG_0_PTR        ((uint8_t *)&proj_config)
#define CONFIG_0_LEN        ((uint8_t)PROJECT_CONFIG_DATA_LEN)

#define CONFIG_1_PTR        ((uint8_t *)&dv_channelConfiguration) 
#define CONFIG_1_LEN        ((uint8_t)sizeof(ChannelConfiguration_t))
#define CONFIG_1_FRAME_LEN  (CONFIG_1_LEN * DEF_NUM_KEYS)

#define CONFIG_2_PTR        ((uint8_t*)&dv_sensorControl)
#define CONFIG_2_LEN        ((uint8_t)sizeof(mtch2120_SensorControl_t))
#define CONFIG_2_FRAME_LEN  (CONFIG_2_LEN * DEF_NUM_KEYS)

#define CONFIG_3_PTR        ((uint8_t*)&dv_gropuConfiguration)
#define CONFIG_3_LEN        ((uint8_t)sizeof(mtch2120_GroupConfiguration_t))
#define CONFIG_3_FRAME_LEN  (CONFIG_3_LEN * 1u)

#define CONFIG_4_PTR        ((uint8_t*)&dv_deviceControl)
#define CONFIG_4_LEN        ((uint8_t)sizeof(mtch2120_DeviceControl_t))
#define CONFIG_4_FRAME_LEN  (CONFIG_4_LEN * 1u)

#define CONFIG_5_PTR        ((uint8_t*)&dv_lumpConfiguration)
#define CONFIG_5_LEN        ((uint8_t)sizeof(DV_LumpConfig_t))
#define CONFIG_5_FRAME_LEN  (CONFIG_5_LEN * NUMBER_OF_KEYS)

#define CONFIG_6_PTR        ((uint8_t*)&dv_slaveAddress)  
#define CONFIG_6_LEN        ((uint8_t)sizeof(dv_slaveAddress))
#define CONFIG_6_FRAME_LEN  (CONFIG_6_LEN * 1u)

#define CONFIG_7_PTR        ((uint8_t*)&dv_gpioConfiguration)  
#define CONFIG_7_LEN        ((uint8_t)sizeof(DV_gpio_t))
#define CONFIG_7_FRAME_LEN  (NUMBER_OF_KEYS + DEF_NUM_EXCESS_PIN)

// output mapping
#define DATA_0_PTR          ((uint8_t*)&dv_commError)
#define DATA_0_ID           (uint8_t)COMM_ERROR
#define DATA_0_LEN          (uint8_t)sizeof(CommunicationError_t)
#define DATA_0_REPEAT       1u
#define DATA_0_FRAME_LEN    (DATA_0_LEN * DATA_0_REPEAT)

#define DATA_1_PTR          ((uint8_t*)&dv_debugData)
#define DATA_1_ID           (uint8_t)DEBUG_DATA
#define DATA_1_LEN          (uint8_t)sizeof(DebugData_t)
#define DATA_1_REPEAT       DEF_NUM_KEYS
#define DATA_1_FRAME_LEN    (DATA_1_LEN * DATA_1_REPEAT)

#define DATA_2_PTR          ((uint8_t*)&dv_deviceInformation)
#define DATA_2_ID           (uint8_t)DEVICE_ID
#define DATA_2_LEN          (uint8_t)sizeof(mtch2120_DeviceInformation_t)
#define DATA_2_REPEAT       1u
#define DATA_2_FRAME_LEN    (DATA_2_LEN * DATA_2_REPEAT)

#define DATA_3_PTR          ((uint8_t*)&dv_status)
#define DATA_3_ID           (uint8_t)DEVICE_STATUS
#define DATA_3_LEN          (uint8_t)sizeof(MTCH2120_Status_t)
#define DATA_3_REPEAT       1u
#define DATA_3_FRAME_LEN    (DATA_3_LEN * DATA_3_REPEAT)

#define DATA_4_PTR          ((uint8_t*)&dv_gpioInputValue)
#define DATA_4_ID           (uint8_t)GPIO_INPUT_UPDATE
#define DATA_4_LEN          (uint8_t)sizeof(dv_gpioInputValue)
#define DATA_4_REPEAT       1u
#define DATA_4_FRAME_LEN    (DATA_4_LEN * DATA_4_REPEAT)

#define DATA_5_PTR          ((uint8_t*)dv_autoTuneFrequencies)
#define DATA_5_ID           (uint8_t)AUTOTUNE_FREQUENCIES
#define DATA_5_LEN          (uint8_t)sizeof(mtch2120_groupConfiguration.hopFreqency)
#define DATA_5_REPEAT       1u
#define DATA_5_FRAME_LEN    (DATA_5_LEN * DATA_5_REPEAT)

#define DATA_6_PTR          ((uint8_t*)&mtch2120_threshold)
#define DATA_6_ID           (uint8_t)EASYTUNE_THRESHOLD
#define DATA_6_LEN          (uint8_t)sizeof(mtch2120_threshold)
#define DATA_6_REPEAT       1u
#define DATA_6_FRAME_LEN    (DATA_6_LEN * DATA_6_REPEAT)

/* configuration details */
static uint8_t proj_config[PROJECT_CONFIG_DATA_LEN] = 
{
    (uint8_t)PROTOCOL_VERSION,      // protocol version
    MTCH2120_DEVICE_TYPE,           // device type [bit-0]
    (uint8_t)SELF_CAP,              // acqisition type
    DEF_NUM_SENSORS,                // number of channels
    
    // configuration
    (0x7Fu),                        // configuration 1 to 7      (0 is project configuration)             
    (0U), 
    (0U),
    
    // output
    (0x7Fu),                        // debug 128 to 134
    (0U), 
    (0U)
};       

static uint8_t frame_len_lookup[NO_OF_CONFIG_FRAME_ID] = 
{
    CONFIG_0_LEN, 
    CONFIG_1_FRAME_LEN, 
    CONFIG_2_FRAME_LEN, 
    CONFIG_3_LEN, 
    CONFIG_4_LEN,
    CONFIG_5_FRAME_LEN,
    CONFIG_6_LEN,
    CONFIG_7_FRAME_LEN
};

static uint8_t *ptr_arr[NO_OF_CONFIG_FRAME_ID] = 
{
    CONFIG_0_PTR, 
    CONFIG_1_PTR, 
    CONFIG_2_PTR, 
    CONFIG_3_PTR, 
    CONFIG_4_PTR,
    CONFIG_5_PTR,
    CONFIG_6_PTR,
    CONFIG_7_PTR
};


static uint8_t *debug_frame_ptr_arr[OUTPUT_MODULE_CNT] = 
{
    DATA_0_PTR, 
    DATA_1_PTR,
    DATA_2_PTR,
    DATA_3_PTR,
    DATA_4_PTR,
    DATA_5_PTR,
    DATA_6_PTR
};

static uint8_t debug_frame_id[OUTPUT_MODULE_CNT] = 
{
    DATA_0_ID, 
    DATA_1_ID,
    DATA_2_ID,
    DATA_3_ID,
    DATA_4_ID,
    DATA_5_ID,
    DATA_6_ID
};

static uint8_t debug_frame_data_len[OUTPUT_MODULE_CNT] = 
{
    DATA_0_LEN, 
    DATA_1_LEN,
    DATA_2_LEN,
    DATA_3_LEN,
    DATA_4_LEN,
    DATA_5_LEN,
    DATA_6_LEN
};

static uint8_t debug_frame_total_len[OUTPUT_MODULE_CNT] = 
{
    DATA_0_FRAME_LEN, 
    DATA_1_FRAME_LEN,
    DATA_2_FRAME_LEN,
    DATA_3_FRAME_LEN,
    DATA_4_FRAME_LEN,
    DATA_5_FRAME_LEN,
    DATA_6_FRAME_LEN
};

static uint8_t debug_num_of_keys[OUTPUT_MODULE_CNT] = 
{
    DATA_0_REPEAT, 
    DATA_1_REPEAT,
    DATA_2_REPEAT,
    DATA_3_REPEAT,
    DATA_4_REPEAT,
    DATA_5_REPEAT,
    DATA_6_REPEAT
};

static void (*debug_func_ptr[OUTPUT_MODULE_CNT])(uint8_t ch) = 
{
    copy_communicationStatus,
    copy_debugData,
    copy_deviceInformation,
    copy_status,
    copy_gpioInputStatus,
    copy_autoTuneFreqencies,
    read_threshold 
};

typedef struct tag_uart_command_info_t
{
    uint8_t transaction_type;
    uint8_t frame_id;
    uint16_t num_of_bytes;
    uint8_t header_status;
} uart_command_info_t;

static uart_command_info_t uart_command_info;

static bool readOneTime = false;

static uint8_t tx_data_len = 0u;
static uint8_t *tx_data_ptr;

static volatile uint8_t current_debug_data;
static volatile uint8_t uart_tx_in_progress = 0u;
static volatile uint8_t uart_frame_header_flag = 1u;
static volatile uint8_t config_or_debug = 0u;

static volatile uint8_t write_buf_read_ptr;
static volatile uint16_t command_flags = 0x0000u;

static uint8_t write_buf_channel_num;
static uint16_t max_number_of_keys;

#if UART_RX_BUF_LENGTH <= 255u
typedef uint8_t rx_buff_ptr_t;
#else
typedef uint16_t rx_buff_ptr_t;
#endif
static volatile rx_buff_ptr_t read_buf_read_ptr = 0u;
static volatile rx_buff_ptr_t read_buf_write_ptr = 0u;

rx_buff_ptr_t uart_min_num_bytes_received(void);

static uint8_t rxData;
static uintptr_t touchUart;
static uint8_t read_buffer[UART_RX_BUF_LENGTH];

// following functions will handle configuration memory regions
// MCU mem to DV mem update.
void copy_slaveAddress(uint8_t channel_num)
{
    (void)channel_num;
    dv_slaveAddress = mtch2120_deviceAddress;
    
    max_number_of_keys = 1u;
    uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, CONFIG_6_FRAME_LEN);
    uart_send_data(STREAMING_CONFIG_DATA, CONFIG_6_PTR, CONFIG_6_LEN);
}

void copy_channel_configuration(uint8_t channel_num)
{
	max_number_of_keys = DEF_NUM_KEYS;
    
    if(readOneTime)
    {
        readOneTime = false;
        mtch2120_getThreshold_Config();
        mtch2120_getOversampling_Config();
        mtch2120_getGain_Config();
        mtch2120_getCSD_Config();
        mtch2120_getMeasurClkFreq_Config();
        mtch2120_getHysteresis_Config();
        mtch2120_getAKS_Config();
    }
    
    dv_channelConfiguration.threshold_dv           = mtch2120_threshold[channel_num];
    dv_channelConfiguration.oversampling_dv        = mtch2120_filterlevel[channel_num];
    dv_channelConfiguration.gain_dv                = mtch2120_gain[channel_num];
    dv_channelConfiguration.csd_dv                 = mtch2120_CSD[channel_num];
    dv_channelConfiguration.measurementClkFreq   = mtch2120_measurementClkFreq[channel_num];
    dv_channelConfiguration.channel_hysteresis_dv  = mtch2120_hysteresis[channel_num];
    dv_channelConfiguration.channel_aks_group_dv   = mtch2120_AKSgroup[channel_num];

	if (channel_num == 0u)
    {
        uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, CONFIG_1_FRAME_LEN);
        uart_send_data(STREAMING_CONFIG_DATA, CONFIG_1_PTR, CONFIG_1_LEN);
    }
    else
    {
        tx_data_ptr = CONFIG_1_PTR;
        tx_data_len = CONFIG_1_LEN;
    }
}

void copy_sensorControl_configuration(uint8_t channel_num)
{
	max_number_of_keys = DEF_NUM_KEYS;
    
    if(readOneTime)
    {
        readOneTime = false;
        mtch2120_getSensorControl_Config();
    }
    
    dv_sensorControl = mtch2120_sensorControl[channel_num];
	if(channel_num == 0u)
    {
        uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, CONFIG_2_FRAME_LEN);
        uart_send_data(STREAMING_CONFIG_DATA, CONFIG_2_PTR, CONFIG_2_LEN);
    }
    else
    {
        tx_data_ptr = CONFIG_2_PTR;
        tx_data_len = CONFIG_2_LEN;
    }
}

void copy_group_configuration(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    
	max_number_of_keys = 1u;
    
    mtch2120_getGroupConfiguration_Config();
    
    dv_gropuConfiguration = mtch2120_groupConfiguration;

	uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, CONFIG_3_FRAME_LEN);
    uart_send_data(STREAMING_CONFIG_DATA, CONFIG_3_PTR, CONFIG_3_LEN);
}

void copy_deviceControl_configuration(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning

	max_number_of_keys = 1u;
    
    mtch2120_getDeviceControl_Config();
    
    dv_deviceControl = mtch2120_deviceControl;

	uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, CONFIG_4_FRAME_LEN);
    uart_send_data(STREAMING_CONFIG_DATA, CONFIG_4_PTR, CONFIG_4_LEN);
}

void copy_lump_configuration(uint8_t channel_num)
{
	max_number_of_keys = CONFIG_5_FRAME_LEN;
    
    if(readOneTime)
    {
        readOneTime = false;
        mtch2120_getLumpConfiguration_0();
        mtch2120_getLumpConfiguration_1();
        mtch2120_getLumpConfiguration_2();
        mtch2120_getLumpConfiguration_3();
    }
    
    dv_lumpConfiguration.lump0 = (uint8_t)(mtch2120_lumpConfiguration_0 >> channel_num);
    dv_lumpConfiguration.lump1 = (uint8_t)(mtch2120_lumpConfiguration_1 >> channel_num);
    dv_lumpConfiguration.lump2 = (uint8_t)(mtch2120_lumpConfiguration_2 >> channel_num);
    dv_lumpConfiguration.lump3 = (uint8_t)(mtch2120_lumpConfiguration_3 >> channel_num);
    
    if(channel_num == 0u)
    {
        uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, CONFIG_5_FRAME_LEN);
        uart_send_data(STREAMING_CONFIG_DATA, CONFIG_5_PTR, CONFIG_5_LEN);
    }
    else
    {
        tx_data_ptr = CONFIG_5_PTR;
        tx_data_len = CONFIG_5_LEN;
    }
}

void copy_gpio_configuration(uint8_t channel_num)
{
    max_number_of_keys = CONFIG_7_FRAME_LEN;
    
    if(readOneTime)
    {
        readOneTime = false;
        mtch2120_getGpioPin();
        mtch2120_getGpioDirection();
        mtch2120_getGpioOutValue();
    }
    
    dv_gpioConfiguration.pin         = (uint8_t)(mtch2120_gpioPins >> channel_num);
    dv_gpioConfiguration.direction   = (uint8_t)(mtch2120_gpioDirection >> channel_num);
    dv_gpioConfiguration.out_buffer  = (uint8_t)(mtch2120_gpioOutputValue >> channel_num);

    if(channel_num == 0u)
    {
        uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, CONFIG_7_FRAME_LEN);
        uart_send_data(STREAMING_CONFIG_DATA, CONFIG_7_PTR, CONFIG_7_LEN);
    }
    else
    {
        tx_data_ptr = CONFIG_7_PTR;
        tx_data_len = CONFIG_7_LEN;
    }
}

// following functions will handle debug memory regions
// MCU mem to DV mem update.
void copy_communicationStatus(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    dv_commError = mtch2120_communicationStatus;
}

void copy_debugData(uint8_t channel_num)
{
    dv_debugData.sensor_state_dv       = mtch2120_sensorState[channel_num];    
    dv_debugData.channelReference_dv   = mtch2120_reference[channel_num];
    dv_debugData.nodeAcqSignals_dv     = mtch2120_signal[channel_num];
    dv_debugData.delta_dv              = mtch2120_delta[channel_num];
    dv_debugData.cc_dv                 = mtch2120_compensationCapacitance[channel_num];
}

void copy_deviceInformation(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    dv_deviceInformation = mtch2120_deviceInformation;
}

void copy_status(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    dv_status = mtch2120_Status;
}

void copy_gpioInputStatus(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    dv_gpioInputValue = mtch2120_gpioInputValue;
}

void copy_autoTuneFreqencies(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    if(stopQueryDebugData == 0u)
    {
        mtch2120_getGroupConfiguration_Config();
    }
    dv_autoTuneFrequencies[0u] = mtch2120_groupConfiguration.hopFreqency[0u];
    dv_autoTuneFrequencies[1u] = mtch2120_groupConfiguration.hopFreqency[1u];
    dv_autoTuneFrequencies[2u] = mtch2120_groupConfiguration.hopFreqency[2u];
}

void read_threshold(uint8_t channel_num)
{
    (void)channel_num;
    // do nothing...
}

// following functions will handle configuration memory regions
// DV mem update to MCU mem update.
void update_channel_configuration(uint8_t channel_num)
{
    mtch2120_threshold[channel_num]      = dv_channelConfiguration.threshold_dv;
    mtch2120_filterlevel[channel_num]      = dv_channelConfiguration.oversampling_dv;
    mtch2120_gain[channel_num]              = dv_channelConfiguration.gain_dv;
    mtch2120_CSD[channel_num]               = dv_channelConfiguration.csd_dv;
    mtch2120_measurementClkFreq[channel_num]      = dv_channelConfiguration.measurementClkFreq;
    mtch2120_hysteresis[channel_num]     = dv_channelConfiguration.channel_hysteresis_dv;
    mtch2120_AKSgroup[channel_num]       = dv_channelConfiguration.channel_aks_group_dv;
}

void update_sensorControl_configuration(uint8_t channel_num)
{
    mtch2120_sensorControl[channel_num]  = dv_sensorControl;
}

void update_group_configuration(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    mtch2120_groupConfiguration  =   dv_gropuConfiguration;
}

void update_deviceControl_configuration(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    mtch2120_deviceControl   =   dv_deviceControl;
}

void update_lump_configuration(uint8_t channel_num)
{
    mtch2120_lumpConfiguration_0 = (dv_lumpConfiguration.lump0 == 1u) ? (mtch2120_lumpConfiguration_0 | ((uint16_t)1u << channel_num))
                                                                      : (mtch2120_lumpConfiguration_0 & ~((uint16_t)1u << channel_num));

    mtch2120_lumpConfiguration_1 = (dv_lumpConfiguration.lump1 == 1u) ? (mtch2120_lumpConfiguration_1 | ((uint16_t)1u << channel_num))
                                                                      : (mtch2120_lumpConfiguration_1 & ~((uint16_t)1u << channel_num));
    
    mtch2120_lumpConfiguration_2 = (dv_lumpConfiguration.lump2 == 1u) ? (mtch2120_lumpConfiguration_2 | ((uint16_t)1u << channel_num))
                                                                      : (mtch2120_lumpConfiguration_2 & ~((uint16_t)1u << channel_num));
    
    mtch2120_lumpConfiguration_3 = (dv_lumpConfiguration.lump3 == 1u) ? (mtch2120_lumpConfiguration_3 | ((uint16_t)1u << channel_num))
                                                                      : (mtch2120_lumpConfiguration_3 & ~((uint16_t)1u << channel_num));
}

void update_slaveAddress(uint8_t channel_num)
{
    (void)channel_num;  // added to avoid compilation warning
    mtch2120_deviceAddress = dv_slaveAddress;
}

void update_gpioConfig(uint8_t channel_num)
{
    mtch2120_gpioPins           = (dv_gpioConfiguration.pin == 1u)          ? (mtch2120_gpioPins | ((uint16_t)1u << channel_num))
                                                                            : (mtch2120_gpioPins & ~((uint16_t)1u << channel_num));
    
    mtch2120_gpioDirection      = (dv_gpioConfiguration.direction == 1u)    ? (mtch2120_gpioDirection | ((uint16_t)1u << channel_num))
                                                                            : (mtch2120_gpioDirection & ~((uint16_t)1u << channel_num));
    
    mtch2120_gpioOutputValue    = (dv_gpioConfiguration.out_buffer == 1u)   ? (mtch2120_gpioOutputValue | ((uint16_t)1u << channel_num))
                                                                            : (mtch2120_gpioOutputValue & ~((uint16_t)1u << channel_num));
}

void copy_channel_config_data(uint8_t id, uint8_t channel)
{
    FRAME_ID_VALUES frameID = (FRAME_ID_VALUES)id;
    switch (frameID)
    {
        case CHANNEL_CONFIGURATION:
            copy_channel_configuration(channel);
            break;
        case SENSOR_CONTROL:
            copy_sensorControl_configuration(channel);
            break;
        case GROUP_CONFIGRUATION:
            copy_group_configuration(channel);
            break;
        case DEVICE_CONTROL:
            copy_deviceControl_configuration(channel);
            break;
        case LUMP_CONFIGURATION:
            copy_lump_configuration(channel);
            break;
        case SLAVE_ADDRESS:
            copy_slaveAddress(channel);
            break;
        case GPIO_CONFIG:
            copy_gpio_configuration(channel);
            break;
        default:
            max_number_of_keys = 1u;
            if(frameID == CONFIG_INFO)
            {
                stopQueryDebugData = 0u;
            }
            uart_send_frame_header((uint8_t)MCU_RESPOND_CONFIG_DATA_TO_PC, uart_command_info.frame_id, frame_len_lookup[uart_command_info.frame_id]);
            uart_send_data(STREAMING_CONFIG_DATA, ptr_arr[uart_command_info.frame_id], frame_len_lookup[uart_command_info.frame_id]);
            break;
    }
}

uint8_t uart_get_char(void)
{
    uint8_t data = read_buffer[read_buf_read_ptr];
    read_buf_read_ptr++;
    if (read_buf_read_ptr == UART_RX_BUF_LENGTH)
    {
        read_buf_read_ptr = 0;
    }
    return data;
}

void uart_get_string(uint8_t *data_recv, uint16_t len)
{
    for (uint16_t idx = 0u; idx < len; idx++)
    {
        *data_recv = uart_get_char();
        data_recv++;
    }
}

uint16_t touchTuneIsDebugDataSent(void) 
{
    return (command_flags & (uint16_t)SEND_DEBUG_DATA);
}

void touchTuneNewDataAvailable(void)
{
    command_flags |= (uint16_t)SEND_DEBUG_DATA;
}

void UART_Write(uint8_t data)
{
    static uint8_t txData;
    txData = data;
    (void)SERCOM2_USART_Write(&txData, 1);
}

void uart_send_data_wait(uint8_t data)
{
    uart_tx_in_progress = 1u;
    UART_Write(data);
    do
    {
        // wait until transmission completes
    }while (uart_tx_in_progress == 1u);
}

void uart_send_data(uint8_t con_or_debug, uint8_t *data_ptr, uint8_t data_len)
{
    if (uart_tx_in_progress == 0u)
    {
        config_or_debug = con_or_debug;
        uart_tx_in_progress = 1u;
        write_buf_channel_num = 1u;
        write_buf_read_ptr = 1u;
        tx_data_ptr = data_ptr;
        tx_data_len = data_len;
        
        UART_Write(tx_data_ptr[0]);
    }
}

rx_buff_ptr_t uart_min_num_bytes_received(void)
{
    int16_t retvar = (int16_t) read_buf_write_ptr;
	retvar -= (int16_t) read_buf_read_ptr;
	if (retvar < 0)
    {
		retvar = retvar + (int16_t) UART_RX_BUF_LENGTH;
    }
    return (rx_buff_ptr_t)(retvar);
}

void uart_send_frame_header(uint8_t trans_type, uint8_t frame, uint16_t frame_len)
{
    uart_frame_header_flag = 0u;
    uart_send_data_wait(DV_HEADER);
    uart_send_data_wait(trans_type);
    uart_send_data_wait(frame);
    uart_send_data_wait((uint8_t)(frame_len & 0x00FFu));
    uart_send_data_wait((uint8_t)((frame_len & 0xFF00u) >> 8u));
    uart_frame_header_flag = 1u; 
}

void uart_recv_frame_data(uint8_t frame_id, uint16_t len)
{
    static uint8_t ch_num = 0u;
    uint16_t num_data = 0u;
    
    FRAME_ID_VALUES frameID = (FRAME_ID_VALUES)frame_id;
    
    num_data = uart_min_num_bytes_received();
    
    switch (frameID)
    {
        case CHANNEL_CONFIGURATION: 
            while (num_data > CONFIG_1_LEN)
            {
                uint8_t *ptr = CONFIG_1_PTR;
                for (uint8_t cnt = 0u; cnt < CONFIG_1_LEN; cnt++)
                {
                    ptr[cnt] = uart_get_char();
                }

                update_channel_configuration(ch_num);
                ch_num++;
                num_data -= CONFIG_1_LEN;

                if (ch_num == DEF_NUM_KEYS)
                {
                    ch_num = 0u;
                    uart_command_info.header_status = HEADER_AWAITING;
                    command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                    (void)uart_get_char(); // reading footer
                    break;
                }
            }
            if(mtch2120_deviceControl.autoTune == 0u)
            {
                mtch2120_setThreshold_Config();
            }
            mtch2120_setOversampling_Config();
            mtch2120_setGain_Config();
            mtch2120_setCSD_Config();
            mtch2120_setMeasurClkFreq_Config();
            mtch2120_setHysteresis_Config();
            mtch2120_setAKS_Config();
            
            break;
        
        case SENSOR_CONTROL:
            while (num_data > CONFIG_2_LEN)
            {
                uint8_t *ptr = CONFIG_2_PTR;
                for (uint8_t cnt = 0u; cnt < CONFIG_2_LEN; cnt++)
                {
                    ptr[cnt] = uart_get_char();
                }

                update_sensorControl_configuration(ch_num);
                ch_num++;
                num_data -= CONFIG_2_LEN;

                if (ch_num == DEF_NUM_KEYS)
                {
                    ch_num = 0u;
                    uart_command_info.header_status = HEADER_AWAITING;
                    command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                    (void)uart_get_char(); // reading footer
                    break;
                }
            }
            mtch2120_setSensorControl_Config();
            break;

        case GROUP_CONFIGRUATION:
            {
                uint8_t *ptr = CONFIG_3_PTR;
                for (uint8_t cnt = 0u; cnt < CONFIG_3_LEN; cnt++)
                {
                    ptr[cnt] = uart_get_char();
                }

                update_group_configuration(ch_num);
                ch_num++;

                ch_num = 0u;
                uart_command_info.header_status = HEADER_AWAITING;
                command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                (void)uart_get_char(); // reading footer
            }
            mtch2120_setGroupConfiguration_Config();
            break;
            
        case DEVICE_CONTROL:
            {
                uint8_t *ptr = CONFIG_4_PTR;
                for (uint8_t cnt = 0u; cnt < CONFIG_4_LEN; cnt++)
                {
                    ptr[cnt] = uart_get_char();
                }

                update_deviceControl_configuration(ch_num);
                ch_num++;

                ch_num = 0u;
                uart_command_info.header_status = HEADER_AWAITING;
                command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                (void)uart_get_char(); // reading footer
            }
            mtch2120_setDeviceControl_Config();
            break;
            
        case LUMP_CONFIGURATION:
            while (num_data > CONFIG_5_LEN)
            {
                uint8_t *ptr = CONFIG_5_PTR;
                for (uint8_t cnt = 0u; cnt < CONFIG_5_LEN; cnt++)
                {
                    ptr[cnt] = uart_get_char();
                }

                update_lump_configuration(ch_num);
                ch_num++;
                num_data -= CONFIG_5_LEN;

              if (ch_num == NUMBER_OF_KEYS)
                {
                    ch_num = 0;
                    uart_command_info.header_status = HEADER_AWAITING;
                    command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                    (void)uart_get_char(); // reading footer
                    break;
                }
            }
            mtch2120_setLumpConfiguration_0();
            mtch2120_setLumpConfiguration_1();
            mtch2120_setLumpConfiguration_2();
            mtch2120_setLumpConfiguration_3();
            break;
            
        case SLAVE_ADDRESS:
            {
                uint8_t *ptr = CONFIG_6_PTR;
                for (uint8_t cnt = 0u; cnt < CONFIG_6_LEN; cnt++)
                {
                    ptr[cnt] = uart_get_char();
                }

                update_slaveAddress(ch_num);
                ch_num++;

                ch_num = 0u;
                uart_command_info.header_status = HEADER_AWAITING;
                command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                (void)uart_get_char(); // reading footer

            }
            break;
            
        case GPIO_CONFIG:
            while (num_data > CONFIG_7_LEN)
            {
                uint8_t *ptr = CONFIG_7_PTR;
                for (uint8_t cnt = 0u; cnt < CONFIG_7_LEN; cnt++)
                {
                    ptr[cnt] = uart_get_char();
                }

                update_gpioConfig(ch_num);
                ch_num++;
                num_data -= CONFIG_7_LEN;

                if (ch_num == CONFIG_7_FRAME_LEN)
                {
                    ch_num = 0u;
                    uart_command_info.header_status = HEADER_AWAITING;
                    command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                    (void)uart_get_char(); // reading footer
                    break;
                }
            }
            mtch2120_setGpioPin();
            mtch2120_setGpioDirection();
            mtch2120_setGpioOutValue();
            
            break;
            
        default:
            uart_get_string(ptr_arr[uart_command_info.frame_id], uart_command_info.num_of_bytes); // frame_len_lookup[uart_command_info.frame_id]);
            (void)uart_get_char();                                                                      // receiving footer
            break;
    }
}

void uart_execute_command(uint8_t channel, uint8_t command, uint8_t onOff)
{
    if(channel < DEF_NUM_KEYS)
    {
        switch((enum buttonCommand)command)
        {
            case BT_CALIBRATE_INDIVIDUAL_BUTTON:
                mtch2120_sensorControl[channel].bit.sensorCalibration = onOff;
                mtch2120_setSensorControl_Config();
                break;
            case BT_SUSPEND_BUTTON:
                mtch2120_sensorControl[channel].bit.suspend = onOff;
                mtch2120_setSensorControl_Config();
                break;
            case BT_ENABLE_BUTTON:
                mtch2120_sensorControl[channel].bit.sensorEnable = onOff;
                mtch2120_setSensorControl_Config();
                break;
            case BT_LOWPOWER_BUTTON:
                mtch2120_sensorControl[channel].bit.lowPowerSensor = onOff;
                mtch2120_setSensorControl_Config();
                break;
            case BT_LUMP_0:    // button 12
                mtch2120_lumpConfiguration_0 = (onOff == 1u) ? (mtch2120_lumpConfiguration_0 |  ((uint16_t)1u << channel))
                                                             : (mtch2120_lumpConfiguration_0 & ~((uint16_t)1u << channel));
                mtch2120_getLumpConfiguration_0();
                break;
            case BT_LUMP_1:    // button 13
                mtch2120_lumpConfiguration_1 = (onOff == 1u) ? (mtch2120_lumpConfiguration_1 |  ((uint16_t)1u << channel))
                                                             : (mtch2120_lumpConfiguration_1 & ~((uint16_t)1u << channel));
                mtch2120_getLumpConfiguration_1();
                break;
            case BT_LUMP_2:    // button 14
                mtch2120_lumpConfiguration_2 = (onOff == 1u) ? (mtch2120_lumpConfiguration_2 |  ((uint16_t)1u << channel))
                                                             : (mtch2120_lumpConfiguration_2 & ~((uint16_t)1u << channel));
                mtch2120_getLumpConfiguration_2();
                break;
            case BT_LUMP_3:    // button 15
                mtch2120_lumpConfiguration_3 = (onOff == 1u) ? (mtch2120_lumpConfiguration_3 |  ((uint16_t)1u << channel))
                                                             : (mtch2120_lumpConfiguration_3 & ~((uint16_t)1u << channel));
                mtch2120_getLumpConfiguration_3();
                break;
            default:
                // do nothing...
                ;
                break;
        }
    }
}

void uart_execute_command_allchannel(uint8_t command, uint8_t onOff)
{
    bool setConfig = true;
    enum buttonCommand buttonSwitch = (enum buttonCommand)command;
    switch(buttonSwitch)
    {
        case BT_CALIBRATE_ALL:
            mtch2120_deviceControl.calibrate = 1u;
            break;
        case BT_SAVE_DEFAULT:
            mtch2120_deviceControl.saveManufacturerConfig = 1u;
            break;
        case BT_SAVE:
            mtch2120_deviceControl.saveConfiguration = 1u;
            break;
        case BT_RESET:
            mtch2120_deviceControl.deviceReset = 1u;
            break;
        case BT_ENABLE_LOWPOWER:
            mtch2120_deviceControl.lpEnable = onOff;
            break;
        case BT_LUMP_BTTN_DRIFT_ENABLE:
            mtch2120_deviceControl.driftLowPowerLumpButtons = onOff;
            break;
        case BT_DRIVEN_SHIELD:
            mtch2120_deviceControl.drivenShield = onOff;
            break;
        case BT_DRIVEN_SHIELD_PLUS:
            mtch2120_deviceControl.drivenShieldPlus = onOff;
            break;
        case BT_DRIFT_GAIN:
            mtch2120_deviceControl.driftGain = onOff;
            break;
        case BT_FREQ_HOP:
            mtch2120_deviceControl.freqencyHopping = onOff;
            break;
        case BT_AUTOTUNE:
            mtch2120_deviceControl.autoTune = onOff;
            break;
        case BT_EASYTUNE:
            mtch2120_deviceControl.easyTune = onOff;
            break;
        case BT_WDT:
            mtch2120_deviceControl.watchDogTimer = onOff;
            break;
        case BT_BOD:
            mtch2120_deviceControl.brownOutDetect = onOff;
            break;
        case STOP_QUERY_DEBUG_DATA:
            stopQueryDebugData = onOff;
            setConfig = false;
            break;
        default:
            setConfig = false;
            break;
    }
    
    // device write only for valid command...
    if(setConfig)
    {
        mtch2120_setDeviceControl_Config();
        if((buttonSwitch == BT_RESET) || (buttonSwitch == BT_SAVE_DEFAULT))
        {
            // added delay before reading the data...to avoid sync-issue !!!
            unsigned int i = 0u;
            do
            {
                i++;
            }while(i < 1000000u);
            mtch2120_getAllConfigurations();
        }
    }
}

void uart_process_command_received(uint8_t frame_id)
{
    uint8_t channel = 0, command, onOff;
    switch(frame_id)
    {
        case 0u:
            /*common command */
            command = uart_get_char();
            onOff = uart_get_char();
            (void)uart_get_char(); // reading footer
            uart_execute_command_allchannel(command, onOff);
            break;
            
        case 1u:
            /*channel basis command */
            channel = uart_get_char();
            command = uart_get_char();
            onOff = uart_get_char();
            uart_execute_command(channel, command, onOff);
            (void)uart_get_char(); // reading footer
            break;
            
        default:
            // do nothing...
            ;
            break;
    }
}

void touchTuneProcess(void)
{
    static uint8_t debug_index = 0u;

    switch (uart_command_info.header_status)
    {
        case HEADER_AWAITING:
            if(uart_tx_in_progress == 1u)
            {
                // don't do anything while on transfer in progress...
            }
            else 
            {
                if (uart_min_num_bytes_received() > 5u)
                {
                    if (uart_get_char() == DV_HEADER)
                    {
                        readOneTime = true;
                        uart_get_string((uint8_t *)&uart_command_info   , 4); // uart_command_info.transaction_type ,uart_command_info.frame_id,uart_command_info.num_of_bytes
                        uart_command_info.header_status = DATA_AWAITING;
                    }
                }
            }
            break;
        case DATA_AWAITING:
            if (uart_command_info.transaction_type == (uint8_t)PC_SEND_CONFIG_DATA_TO_MCU) // user has pressed write to kit
            {
                if (uart_command_info.num_of_bytes >= UART_RX_BUF_LENGTH)
                {
                    uart_recv_frame_data(uart_command_info.frame_id, uart_command_info.num_of_bytes);
                }
                else if (uart_min_num_bytes_received() > uart_command_info.num_of_bytes) // total length of bytes + footer
                {
                    command_flags |= ((uint16_t)1u << (uart_command_info.frame_id)); // (uart_command_info.frame_id - CONFIG_INFO)
                    uart_command_info.header_status = DATA_RECEIVED;
                }
                else
                {
                    // do nothing...
            }
            }
            else if (uart_command_info.transaction_type == (uint8_t)PC_REQUEST_CONFIG_DATA_FROM_MCU) // read from kit
            {
                if (uart_min_num_bytes_received() > 1u) // Data length = 1 + footer
                {
                    uint8_t data1 = uart_get_char();
					uint8_t data2 = uart_get_char();
                    if((data1 == ZERO) && (data2 == DV_FOOTER))     // requesting configuration
                    {
                        command_flags |= ((uint16_t)1u << (uart_command_info.frame_id)); // (uart_command_info.frame_id - CONFIG_INFO)
                        uart_command_info.header_status = DATA_RECEIVED;
                    }
                }
            }
            else if(uart_command_info.transaction_type == (uint8_t)PC_SEND_COMMAND_DATA_TO_MCU)
            {
                if (uart_min_num_bytes_received() > uart_command_info.num_of_bytes)
                {
                    uart_command_info.header_status = DATA_RECEIVED;
                }
            }
            else
            {
                // do nothing...
            }
            break;
        case DATA_RECEIVED:
            if(uart_tx_in_progress == 0u)
            {
                if ((command_flags & 0x0FFFu) != 0u)
            {
                    if (uart_command_info.transaction_type == (uint8_t)PC_REQUEST_CONFIG_DATA_FROM_MCU) // requesting configuration
                {
                    copy_channel_config_data(uart_command_info.frame_id, 0u);
                    uart_command_info.header_status = HEADER_AWAITING;
                }
                    else if (uart_command_info.transaction_type == (uint8_t)PC_SEND_CONFIG_DATA_TO_MCU) // PC Updating parameters.
                {
                    uart_recv_frame_data(uart_command_info.frame_id, uart_command_info.num_of_bytes);
                    uart_command_info.header_status = HEADER_AWAITING;
                        command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                    }
                    else
                    {
                        // do nothing...
                    }
                }
            }
            else if(uart_command_info.transaction_type == (uint8_t)PC_SEND_COMMAND_DATA_TO_MCU)
            {
                uart_process_command_received(uart_command_info.frame_id);
                uart_command_info.header_status = HEADER_AWAITING;
            }
            else
            {
                // do nothing...
            }
            break;
        default:
            uart_command_info.header_status = HEADER_AWAITING;
            break;
    }

    if(uart_tx_in_progress == 0u)
    {
    /* to send periodic data */
        if ((command_flags & SEND_DEBUG_DATA) == SEND_DEBUG_DATA)
    {
        while (debug_func_ptr[debug_index] == NULL)
        {
            debug_index++;
            if (debug_index == OUTPUT_MODULE_CNT)
            {
                debug_index = 0u;
            }
        }
        current_debug_data = debug_frame_id[debug_index];

            uart_send_frame_header((uint8_t)MCU_SEND_TUNE_DATA_TO_PC, current_debug_data, debug_frame_total_len[debug_index]);

        (debug_func_ptr[debug_index])(0u);

        max_number_of_keys = debug_num_of_keys[debug_index];

        uart_send_data(STREAMING_DEBUG_DATA, (uint8_t *)debug_frame_ptr_arr[debug_index], debug_frame_data_len[debug_index]);

        debug_index++;

        if (debug_index == OUTPUT_MODULE_CNT)
        {
            debug_index = 0u;
        }
    }
}
}

#endif

void touchUartTxComplete(uintptr_t lTouchUart)
{
    (void)lTouchUart; // added for MISRA compliance.
#if (DEF_TOUCH_DATA_STREAMER_ENABLE == 1u)

    if (uart_frame_header_flag != 1u)
    {
        uart_tx_in_progress = 0u;
    }
    else
    {
        if (write_buf_read_ptr < tx_data_len)
        {
            UART_Write(tx_data_ptr[write_buf_read_ptr]);
            write_buf_read_ptr++;
        }
        else
        {
            if (config_or_debug == STREAMING_CONFIG_DATA)
            {
                /* per channel data are sent channel by channel to reduce RAM requirements */
                if ((write_buf_channel_num < max_number_of_keys))
                {
                    copy_channel_config_data(uart_command_info.frame_id, write_buf_channel_num);
                    write_buf_read_ptr = 1u;
                    write_buf_channel_num++;
                    UART_Write(tx_data_ptr[0u]);
                }
                
                else if (write_buf_channel_num == max_number_of_keys)
                {
                    write_buf_channel_num++;
                    command_flags &= ~((uint16_t)1u << uart_command_info.frame_id);
                    UART_Write(DV_FOOTER);
                }
                else
                {
                    uart_tx_in_progress = 0u;
                }
            }
            else if (config_or_debug == STREAMING_DEBUG_DATA)
            {
                /* per channel data are sent channel by channel to reduce RAM requirements */
                if (write_buf_channel_num < max_number_of_keys)
                {
                    (*debug_func_ptr[current_debug_data & 0x0Fu])(write_buf_channel_num);
                    write_buf_read_ptr = 1u;
                    write_buf_channel_num++;
                    UART_Write(tx_data_ptr[0u]);   
                }
                else if (write_buf_channel_num == max_number_of_keys)
                {
                    write_buf_channel_num++;
                    command_flags &= (uint16_t)~(SEND_DEBUG_DATA); // clearing off debug data
                    UART_Write(DV_FOOTER);
                }
                else
                {
                    uart_tx_in_progress = 0u;
                }
            }
            else
            {
                // do nothing...
            }
        }
    }
#endif
}



/**
 * @brief UART Tx complete call back function.
 * If user callback is not NULL, call the user registered callback function
 *
 */

/**
 * @brief UART Rx complete call back function.
 * If user callback is not NULL, call the user registered callback function
 *
 */
void touchUartRxComplete(uintptr_t lTouchUart)
{
    (void)lTouchUart;
    read_buffer[read_buf_write_ptr] = rxData;
    read_buf_write_ptr++;
    if (read_buf_write_ptr == UART_RX_BUF_LENGTH)
    {
        read_buf_write_ptr = 0u;
    }
    (void)SERCOM2_USART_Read((void *)&rxData, 1);
}

void touchTuneInit(void)
{
    SERCOM2_USART_ReadCallbackRegister(touchUartRxComplete, touchUart);
    SERCOM2_USART_WriteCallbackRegister(touchUartTxComplete, touchUart);

    (void)SERCOM2_USART_Read((void *)&rxData, 1);
}

uint8_t isStopToReadDebugData(void)
{
    return stopQueryDebugData;
}

#endif  /* ENABLE_TOUCH_TUNE */
