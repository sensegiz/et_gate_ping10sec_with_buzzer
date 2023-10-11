/******************************************************************************

@file  multi_role.c

@brief This file contains the multi_role sample application for use
with the CC2650 Bluetooth Low Energy Protocol Stack.

Group: WCS, BTS
Target Device: cc2640r2

******************************************************************************

 Copyright (c) 2013-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************


*****************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

//#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "multi.h"

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

#include "board.h"
#include "ExtFlash.h"
//#include "multi_role.h"
#include "utc_clock.h"
#include "time_clock.h"
#include "bletime.h"
#include "Watchdog.h"

#include "multi_role.h"

#include <profiles/oad/cc26xx/oad.h>
#include <profiles/oad/cc26xx/oad_image_header.h>
// Needed for HAL_SYSTEM_RESET()
#include "hal_mcu.h"
#include "ble_user_config.h"

/*********************************************************************
* CONSTANTS
*/
/************Settings Marcors ***************************/

#define   WATCHDOG_ENABLE
#define   COIN_AS_GATEWAY
//#define   ENABLE_MM
//#define ENABLE_MP
//#define ENABLE_PM
//#define ENABLE_TS
#define ENABLE_PF
//// Enable/Disable Unlimited Scanning Feature
//#define ENABLE_UNLIMITED_SCAN_RES             FALSE

// Maximum number of scan responses
// this can only be set to 15 because that is the maximum
// amount of item actions the menu module supports
#define DEFAULT_MAX_SCAN_RES                  15

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Connection parameters
#define DEFAULT_CONN_INT                      42//200
#define DEFAULT_CONN_TIMEOUT                  200//1000
#define DEFAULT_CONN_LATENCY                  0

//// Default service discovery timer delay in ms
//#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define DEFAULT_SCAN_DURATION                 110//4000
#define DEFAULT_SCAN_WIND                     200//80
#define DEFAULT_SCAN_INT                      200//80

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_GENERAL//DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// Set desired policy to use during discovery (use values from GAP_Disc_Filter_Policies)
#define DEFAULT_DISCOVERY_WHITE_LIST          GAP_DISC_FILTER_POLICY_ALL

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link`
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   1250//1050//950//1125//610
#endif

// Internal Events for RTOS application
#define MR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define MR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define MR_STATE_CHANGE_EVT                  Event_Id_00
#define MR_CHAR_CHANGE_EVT                   Event_Id_04
#define MR_CONN_EVT_END_EVT                  Event_Id_05
#define UART_WRITE_EVT                       Event_Id_06
#define READ_UART_DATA_EVT                   Event_Id_07
#define SEND_BUFFER_DATA                     Event_Id_08
#define ACK_TIMEOUT_EVT                      Event_Id_09
#define RESET_EVT                            Event_Id_10
#define TIME_RQT_EVT                         Event_Id_11
#define BLE_STAT_EVT                         Event_Id_12

#define MR_ALL_EVENTS                        (MR_ICALL_EVT           | \
                                             MR_QUEUE_EVT            | \
                                             MR_STATE_CHANGE_EVT     | \
                                             MR_CHAR_CHANGE_EVT      | \
                                             MR_CONN_EVT_END_EVT     | \
                                             UART_WRITE_EVT          | \
                                             READ_UART_DATA_EVT      | \
                                             SEND_BUFFER_DATA        | \
                                             ACK_TIMEOUT_EVT         | \
                                             RESET_EVT               | \
                                             TIME_RQT_EVT            | \
                                             BLE_STAT_EVT            | \
                                             OAD_QUEUE_EVT           | \
                                             OAD_DL_COMPLETE_EVT     | \
                                             OAD_OUT_OF_MEM_EVT)

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

//// Row numbers
//#define MR_ROW_DEV_ADDR      (TBM_ROW_APP)
//#define MR_ROW_CONN_STATUS   (TBM_ROW_APP + 1)
//#define MR_ROW_ADV           (TBM_ROW_APP + 2)
//#define MR_ROW_SECURITY      (TBM_ROW_APP + 3)
//#define MR_ROW_STATUS1       (TBM_ROW_APP + 4)
//#define MR_ROW_STATUS2       (TBM_ROW_APP + 5)

// address string length is an ascii character for each digit +
// an initial 0x + an ending null character
#define B_STR_ADDR_LEN       ((B_ADDR_LEN*2) + 3)

// How often to perform periodic event (in msec)
int READ_PERIOD = 103;//149;//23;//103;//103;//53;//331;//1001;//30-------------------------------------
#define SEND_PERIOD                     1013//1009------------------------------
int WRITE_UART  = 79;//179;//79;//47;//199;//303;//20---------------------------------------
#define ACK_PERIOD                      1109//2013//1109
#define RESET_PERIOD                    80147//60009//40009//60009-----------------------------
#define TIME_RQT_PERIOD                 30011 //1009//
#define BLE_STAT_PERIOD                 997//1009//30011//60007//30011//59981

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))

#define GATEWAY_DATA                    6
#define DEV_LIST_SIZE                   10
#define UART_WRITE_SIZE                 9//8//7
#define flashbufsize                    5500//5500//5500////3000//5000//-------------------------

#define write_count_store               0x76015//0x090000
#define read_count_store                0x76060

#define extbuffer0                      0x61500//61000 72000
#define extbuffer1                      0x62400
#define extbuffer2                      0x63300
#define extbuffer3                      0x64200
#define extbuffer4                      0x65100
#define extbuffer5                      0x66000
#define extbuffer6                      0x66F00
#define extbuffer7                      0x67E00
#define extbuffer8                      0x68D00
#define extbuffer9                      0x69C00
#define extbuffer10                     0x6AB00
#define extbuffer11                     0x6BA00
#define extbuffer12                     0x6C900
#define extbuffer13                     0x6D800//till 0x6C4C8

#define Mesh_id_SETTING                 0x7D150
#define EXT_FLASH_SETTINGS              0x7E225
/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint16_t event;  // event type
  uint8_t *pData;  // event data pointer
} mrEvt_t;

//// pairing callback event
//typedef struct
//{
//  uint16_t connectionHandle; // connection Handle
//  uint8_t state;             // state returned from GAPBondMgr
//  uint8_t status;            // status of state
//} gapPairStateEvent_t;

// discovery information
typedef struct
{
  discState_t discState;   // discovery state
  uint16_t svcStartHdl;    // service start handle
  uint16_t svcEndHdl;      // service end handle
  uint16_t charHdl;        // characteristic handle
} discInfo_t;

//// device discovery information with room for string address
//typedef struct
//{
//  uint8_t eventType;                // Indicates advertising event type used by the advertiser: @ref GAP_ADVERTISEMENT_REPORT_TYPE_DEFINES
//  uint8_t addrType;                 // Address Type: @ref ADDRTYPE_DEFINES
//  uint8_t addr[B_ADDR_LEN];         // Device's Address
//  uint8_t strAddr[B_STR_ADDR_LEN];  // Device Address as String
//} mrDevRec_t;

// entry to map index to connection handle and store address string for menu module
typedef struct
{
  uint16_t connHandle;              // connection handle of an active connection
  uint8_t strAddr[B_STR_ADDR_LEN];  // memory location for menu module to store address string
} connHandleMapEntry_t;

typedef struct
{
   uint8 static_id[2];
   int8 RSSI;
   uint8 DevList_index;
   bool in_range;
   uint8 addr1[6];
   uint8 dyn_id[2];
   uint16 dyn_id_16;
   //   uint8 dyn_id;
}dev_list;
/*********************************************************************
* GLOBAL VARIABLES
*/
uint8_t connected_device_address[6];
int8 RSSI;
const int8 RSSI_LIMIT = -90;
uint8_t status;
Watchdog_Handle WatchdogHandle;
uint8_t oad_in_progress =0;
/*********************************************************************
* LOCAL VARIABLES
*/

unsigned char *BLE0 = (unsigned char*) 0x500012ED;
unsigned char *BLE1 = (unsigned char*) 0x500012EC;
unsigned char *BLE2 = (unsigned char*) 0x500012EB;
unsigned char *BLE3 = (unsigned char*) 0x500012EA;
unsigned char *BLE4 = (unsigned char*) 0x500012E9;
unsigned char *BLE5 = (unsigned char*) 0x500012E8;

unsigned char BLEADDRESS[6];
unsigned char BLEADDRESS_ASCII[12];
unsigned char BLEADDRESS_ASCII1[12];

/******************************** MM *********************************************/
#if defined  ENABLE_MM

uint8 moving_device_name[]="CMMM";//  COIN CoIn cOIN VCIn  CMv6 CZ6k
uint8 gateway_name[]="GATWYMM";//CZtewAy   VCtewAy   GATEWAY
uint8_t BLE_version[24]="NECMM_VBLE0002_21JUL2022";//BLEVERSION
static uint8_t scanRspData[] =
{
  // complete name
  8,//11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'G', 'A', 'T', 'W', 'Y', 'M', 'M',  //'M', 'u', 'l', 't', 'i', ' ', 'R', 'o', 'l', 'e',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec


/*
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
*/
};

static uint8_t scanRspData1[] =
{
  // complete name
  8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'g','a','t','w','y','m','m',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,//hh
  0x00,//mm
  0x00,//ss
  0x00,//dd
  0x00,//ld
  0x00,//md
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec

};
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "GATWYMM";//"Multi Role :)";


/******************************** MP *********************************************/
#elif defined  ENABLE_MP

uint8 moving_device_name[]="CMMP";//  COIN CoIn cOIN VCIn  CMv6 CZ6k
uint8 gateway_name[]="GATWYMP";//CZtewAy   VCtewAy   GATEWAY
uint8_t BLE_version[24]="NECMP_VBLE0002_21JUL2022";//BLEVERSION
static uint8_t scanRspData[] =
{
  // complete name
  8,//11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'G', 'A', 'T', 'W', 'Y', 'M', 'P',  //'M', 'u', 'l', 't', 'i', ' ', 'R', 'o', 'l', 'e',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec

/*
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
*/
};

static uint8_t scanRspData1[] =
{
  // complete name
  8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'g','a','t','w','y','m','p',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,//hh
  0x00,//mm
  0x00,//ss
  0x00,//dd
  0x00,//ld
  0x00,//md
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec

};
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "GATWYMP";//"Multi Role :)";

/**************************** PM ***********************************************/
#elif defined   ENABLE_PM

uint8 moving_device_name[]="CMPM";//  COIN CoIn cOIN VCIn  CMv6 CZ6k
uint8 gateway_name[]="GATWYPM";//CZtewAy   VCtewAy   GATEWAY
uint8_t BLE_version[24]="NECPM_VBLE0002_21JUL2022";//BLEVERSION
static uint8_t scanRspData[] =
{
  // complete name
  8,//11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'G', 'A', 'T', 'W', 'Y', 'P', 'M',  //'M', 'u', 'l', 't', 'i', ' ', 'R', 'o', 'l', 'e',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec

/*
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
*/
};

static uint8_t scanRspData1[] =
{
  // complete name
  8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'g','a','t','w','y','p','m',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,//hh
  0x00,//mm
  0x00,//ss
  0x00,//dd
  0x00,//ld
  0x00,//md
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec

};
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "GATWYPM";//"Multi Role :)";

/******************************** TS *********************************************/
#elif defined   ENABLE_TS

uint8 moving_device_name[]="CMTS";//  COIN CoIn cOIN VCIn  CMv6 CZ6k
uint8 gateway_name[]="GATWYTS";//CZtewAy   VCtewAy   GATEWAY
uint8_t BLE_version[24]="NECTS_VBLE0002_21JUL2022";//BLEVERSION
static uint8_t scanRspData[] =
{
  // complete name
  8,//11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'G', 'A', 'T', 'W', 'Y', 'T', 'S',  //'M', 'u', 'l', 't', 'i', ' ', 'R', 'o', 'l', 'e',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec

/*
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
*/
};

static uint8_t scanRspData1[] =
{
  // complete name
  8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'g','a','t','w','y','t','s',

  0x0B,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,//hh
  0x00,//mm
  0x00,//ss
  0x00,//dd
  0x00,//ld
  0x00,//md
  0x00,//Buzz settings - on/off
  0x00,//Buzz settings - time in sec

};
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "GATWYTS";//"Multi Role :)";

/******************************** PF *********************************************/
#elif defined   ENABLE_PF

uint8 moving_device_name[]="CMPF";//  COIN CoIn cOIN VCIn  CMv6 CZ6k
uint8 gateway_name[]="GATWYPF";//CZtewAy   VCtewAy   GATEWAY
uint8_t BLE_version[24]="NECPF_VBLE0002_25SEP2023";//BLEVERSION
static uint8_t scanRspData[] =
{
  // complete name
  8,//11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'G', 'A', 'T', 'W', 'Y', 'P', 'F',  //'M', 'u', 'l', 't', 'i', ' ', 'R', 'o', 'l', 'e',

  0x09,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
 // 0x00,//Buzz settings - on/off
 // 0x00,//Buzz settings - time in sec

/*
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
*/
};

static uint8_t scanRspData1[] =
{
  // complete name
  8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'g','a','t','w','y','p','f',

  0x09,//0x04,//0x03,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  0xFF,
  0xFF,
  0x00,//hh
  0x00,//mm
  0x00,//ss
  0x00,//dd
  0x00,//ld
  0x00,//md
 // 0x00,//Buzz settings - on/off
 // 0x00,//Buzz settings - time in sec

};
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "GATWYPF";//"Multi Role :)";

#endif


static UART_Handle uart;
UART_Params uartParams;
uint32_t wantedRxBytes;
uint8_t rxBuf[26];
uint8_t *startaddress,first_data=1;
int rxBytes = 0;
uint8_t coindata[COIN_DATA],last_received_packet[COIN_DATA];
uint8_t utc_time_data[UART_WRITE_SIZE];
uint8_t extdata[UART_WRITE_SIZE];
uint8_t onuart[31], onuart1[31];//,LAST_DATA_onuart[27];
uint8_t inarray[14];
uint8_t outarray[14];
uint8_t utc_time_server[10];
uint8_t inoutarray[14];
uint8_t uartdata[2*UART_WRITE_SIZE];
uint8_t ble_stat_count = 0;
uint8_t test_count=0;
static uint8_t name_change=0,status_write=0;
uint8_t received_count=0,sent_count=0;
//nt n=0;
int ix;
int n=0;
uint8_t buffer_counter=0,scan_count=0;//,low_rate_scan=0;
uint8_t coin_buffer[1][COIN_DATA_2];
uint8 packet_to_send[COIN_DATA_2];
uint8_t ack=0xAC;
uint8_t connRole;
uint8_t ack_packet[2];
int jx;
bool readreturn1,writeretrn1;
uint8_t device_connected=0,server_comm_first=0,ble_status=0,loc10=0,test_name_change=1,reset_flash_command_received=0,reset_flash_command_received_count=0;
uint8_t dataToSend1[14];
//uint8_t invalidcommand[10];
uint8 sensor_id_index;
uint8_t simple1handle = 26;//45;//34;
uint8_t simple3handle = 32;//51;//40;
uint8_t devices_discovered = 0,cindex,addrType;
uint8_t dev_counter = 0,res/*,oad_in_progress=0*/;
uint8_t dev_list_index=0;
uint8_t adv_data[24];
uint8 *ptr,dest_index,wait_for_data_send=0;
uint8_t device_name[24];
//uint8 moving_device_name[]="CMPM";//  COIN CoIn cOIN VCIn  CMv6 CZ6k
//uint8 gateway_name[]="GATWYPM";//CZtewAy   VCtewAy   GATEWAY
uint8 device_length=sizeof(gateway_name)-1;
uint8 moving_device_length=sizeof(moving_device_name)-1;
uint8_t curr_addr[GATEWAY_DATA];
uint8_t prev_addr[GATEWAY_DATA];
uint8_t dest_addr[GATEWAY_DATA];

PIN_Handle hGpioPin;
dev_list scanned_coin_info[DEV_LIST_SIZE];

uint32_t time_hr,time_min,time_sec,time_total,time_total_dup,time_remainder,time_remainder_dup;
uint32_t utc_time_server_min,utc_time_server_hr,utc_time_server_sec,utc_time_server_input;

uint8_t reset_the_device=0;
int writecount = 0, writecount2 = 0, store_count = 0;
int readcount = 0;
bool readreturn;
bool writereturn;
//static uint8_t RST_count=0;
static uint8_t flag_X_Y_Z = 0, flag_X = 0, flag_Y = 0, flag_Z = 0;
uint8_t X_Y_Z[39];
uint8_t accel_X[18];
uint8_t accel_Y[18];
uint8_t accel_Z[18];
uint16_t no_res_10_sec_count=0;


uint8_t onuart2[37];
int version= 0,ping_count=0,mem_zero_flag=0;
uint8_t loc15=0,loc_mesh1=0,loc_mesh2=0,LOC0=0;
static uint8_t Mesh_id = 0, input_date=1, last_date=0x1E, last_date_value=30, new_settings_received=0;
uint8_t mesh_packet[3];

uint8_t stop_rec_data=0;
uint16_t scanned_dyn_id_16=0, dynamic_id_16=0xFFFF, dyn_id_previous_16=0xFFFF, getset_dynid_16=0;
uint8_t scanned_dyn_id[2]={0xFF,0xFF};
uint8_t same_dyn_id=0;

static uint8_t update_time_in_advt_packet=0;
uint8_t ALL_SETTINGS[4];
uint8_t setting_ack = 1,new_stored_buf=0;
//uint16_t buffer_cnt=0;//,buffer_cnt111=0;
// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;

static uint8_t u8_buzz_on_off=0xAA,u8_buzz_time=0x14,buzz_count=0,buzz_cmd_rx=0,buzz_cmd_rx1=0,buzz_count1=0,u8_buzz_flag=0;

/*********************************************************************
* LOCAL VARIABLES
*/
PIN_Handle hGpioPin;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct read_clock;
static Clock_Struct write_uart_clock;
static Clock_Struct ack_clock;
static Clock_Struct buffer_clock;
static Clock_Struct reset_clock;
static Clock_Struct time_rqt_clock;
static Clock_Struct ble_stat_clock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];


// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x09,//0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID),
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
};

// pointer to allocate the connection handle map
static connHandleMapEntry_t *connHandleMap;

// GAP GATT Attributes
//static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "GATWYPM";//"Multi Role :)";

// Number of scan results
static uint8_t scanRes = 0;

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0, count_stored_buf=0;//-------------------------------------
static uint8_t write_high = 0, write_low = 0, read_high = 0, read_low = 0;//-------------

// Pointer to per connection discovery info
discInfo_t *discInfo;

// Maximim PDU size (default = 27 octets)
//static uint16 maxPduSize;

// Scanning started flag
static bool scanningStarted = FALSE;

// Scan result list
//static mrDevRec_t devList[DEFAULT_MAX_SCAN_RES];
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Dummy parameters to use for connection updates
gapRole_updateConnParams_t updateParams =
{
  .connHandle = INVALID_CONNHANDLE,
  .minConnInterval = 80,
  .maxConnInterval = 150,
  .slaveLatency = 0,
  .timeoutMultiplier = 200
};

// Connection index for mapping connection handles
static uint16_t connIndex = INVALID_CONNHANDLE;

// Maximum number of connected devices
static uint8_t maxNumBleConns = MAX_NUM_BLE_CONNS;

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void multi_role_init( void );
static void multi_role_taskFxn(UArg a0, UArg a1);
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_charValueChangeCB(uint8_t paramID);
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData);

static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent);
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp);
//static void multi_role_sendAttRsp(void);
//static void multi_role_freeAttRsp(uint8_t status);
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle);

static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr);

//static void multi_role_performPeriodicTask(void);
static void multi_role_clockHandler(UArg arg);
static void multi_role_connEvtCB(Gap_ConnEventRpt_t *pReport);
//static void multi_role_addDeviceInfo(uint8 *pAddr, uint8 addrType);

static void multi_role_processOadWriteCB(uint8_t event, uint16_t arg);
static void multi_role_processL2CAPMsg(l2capSignalEvent_t *pMsg);

//all function
void DataSend(uint8*,uint8,uint8_t);
void Connect_to_coin();
void GetAddress(void);
void UART_parameter(void);
void uart_read_function(void);
void convert(void);
void converttoascii(void);
void write_on_uart(void);
void process_adv_packet(void);
void disconnect_and_update_buffer(uint8 next_index);
uint8 find_dest_addr(void);
void disconnect_after_10_scans(void);
uint8 compare_string(uint8 *str1,uint8 *str2,uint8 size);
void count_restore(void);
void count_store(void);
void erase_data_from_flash();
void init_whatch(void);
void erase_external_flash(void);
void clock_funtion_update(void);
void read_all_setting(void);
void store_all_settings(void);
void write_to_flash(uint8_t *buf,int buf_size);
void read_from_flash(uint8_t *buf,int buf_size);

void mesh_id_update(void);
void restart_erase_flash_all_paramerter(void);

void clock_funtion_update(void)
{
    time_total=UTC_getClock();
    if(time_total>=86400)
    {
        UTC_setClock(0);
        time_hr=0;
        time_min=0;
        time_sec=0;
        if(input_date<last_date_value)
          input_date=input_date+1;
        else if(input_date>=last_date_value)
          input_date=1;
        store_all_settings();
    }
    else
    {
        time_hr=time_total/3600;
        time_total_dup=time_total%3600;
        time_min=time_total_dup/60;
        time_sec=time_total_dup%60;
    }
}

void erase_external_flash(void)
{
    DELAY_MS(5);
    ExtFlash_erase(extbuffer0,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer1,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer2,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer3,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer4,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer5,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer6,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer7,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer8,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer9,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer10,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer11,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer12,1);
    DELAY_MS(5);
    ExtFlash_erase(extbuffer13,1);
    DELAY_MS(5);
    ExtFlash_erase(write_count_store,1);
}

void DataSend(uint8* dataToSend,uint8 size,uint8_t chandle)
{
    attPrepareWriteReq_t req;
    req.pValue = GATT_bm_alloc(connHandleMap[cindex].connHandle, ATT_WRITE_REQ, size, NULL);
    if ( req.pValue != NULL )
    {
        req.handle = chandle;
        req.offset = 0;
        req.len = size;
        memcpy(req.pValue, dataToSend,size);
        status = GATT_WriteLongCharValue(connHandleMap[cindex].connHandle, &req, selfEntity);
        if ( status != SUCCESS )
            GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
}

void init_whatch()
{
    Watchdog_Params params;
    Watchdog_init();
    Watchdog_Params_init(&params);
    params.debugStallMode = Watchdog_DEBUG_STALL_ON;
    params.resetMode = Watchdog_RESET_ON;

    #ifdef COIN_AS_GATEWAY
    WatchdogHandle = Watchdog_open(COIN_v4_LAUNCHPAD_WATCHDOG0, &params);
    #else
        WatchdogHandle = Watchdog_open(CC2640R2_LAUNCHXL_WATCHDOG0, &params);
    #endif
   // WatchdogHandle = Watchdog_open(CC2640R2_LAUNCHXL_WATCHDOG0, &params);//COIN_v4_LAUNCHPAD_WATCHDOG0   CC2640R2_LAUNCHXL_WATCHDOG0
    if (WatchdogHandle == NULL) {
        /* Error opening Watchdog */
        while (1);
    }
}

void count_store(void)
{
    write_low = writecount%100;
    write_high = writecount/100;

    read_low = readcount%100;
    read_high = readcount/100;

    ExtFlash_open();
    ExtFlash_erase(write_count_store,1);
    //ExtFlash_erase(read_count_store,2);
    ptr = &write_high;
    writeretrn1=ExtFlash_write(write_count_store,1,ptr);
    ptr = &write_low;
    writeretrn1= ExtFlash_write(write_count_store+1,1,ptr);
    ptr = &read_high;
    writeretrn1=ExtFlash_write(read_count_store,1,ptr);
    ptr = &read_low;
    writeretrn1= ExtFlash_write(read_count_store+1,1,ptr);
    ExtFlash_close();
    count_stored_buf=1;
}

void erase_data_from_flash()
{
    if((writecount==readcount && writecount>=4500) && (device_connected==0 || connRole==GAP_PROFILE_PERIPHERAL))
    {
        new_stored_buf=0;
        count_stored_buf=0;
        write_high = 0;
        write_low = 0;
        read_high = 0;
        read_low = 0;
        test_name_change=0;
        n=0;
        readcount=0;
        writecount=0;

        DELAY_MS(100);
        ExtFlash_open();
        erase_external_flash();
        ExtFlash_close();
        DELAY_MS(100);

        stop_rec_data= 0;

        if(server_comm_first==1)
        {
            scanRspData[13]=time_hr;
            scanRspData[14]=time_min;
            scanRspData[15]=time_sec;
            scanRspData[16]=input_date;
            scanRspData[17]=last_date;
            scanRspData[18]=Mesh_id;
            //scanRspData[19]=u8_buzz_on_off;
           // scanRspData[20]=u8_buzz_time;

            GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData, NULL);
        }
    }
}

void count_restore()
{
    ExtFlash_open();
    ptr=&write_high;
    readreturn1= ExtFlash_read(write_count_store,1,ptr);

    if(write_high==0xFF)
    {
        write_high=0;
    }
    ptr=&write_low;
    readreturn1=ExtFlash_read(write_count_store+1,1,ptr);
    if(write_low==0xFF)
    {
        write_low=0;
    }
    ptr=&read_high;
    readreturn1= ExtFlash_read(read_count_store,1,ptr);
    if(read_high==0xFF)
    {
        read_high=0;
    }
    ptr=&read_low;
    readreturn1= ExtFlash_read(read_count_store+1,1,ptr);
    if(read_low==0xFF)
    {
        read_low=0;
    }
    if((writecount!=0xFF)&&(readcount !=0xFF))
    {
        writecount = write_high * 100 + write_low;
        readcount = read_high * 100 + read_low;
        if(writecount > readcount)
            n=writecount-readcount;
            if(n>0)//if(n>=100)
            {
                count_stored_buf=1;
                new_stored_buf=1;
            }
    }
    ptr = &Mesh_id;//////////////////////////////////////////////////////////
    readreturn1= ExtFlash_read(loc15+Mesh_id_SETTING,1,ptr);
    if(Mesh_id==0xFF)
    {
        Mesh_id=0;
    }

    if((n<=20) )//if(n<=2)    n==0
    {
        new_stored_buf=0;
        count_stored_buf=0;
        write_high = 0;
        write_low = 0;
        read_high = 0;
        read_low = 0;
        test_name_change=0;
        n=0;
        readcount=0;
        writecount=0;

        erase_external_flash();
    }

//    ExtFlash_erase(write_count_store,1);

//    DELAY_MS(1000);
    ExtFlash_close();
}

void write_to_flash(uint8_t *buf,int buf_size)
{
     if((writecount>=flashbufsize) && (connRole==GAP_PROFILE_PERIPHERAL) )// || ((writecount >= ((flashbufsize * 90) / 100)) && (0 == store_count)))
     {
         if(n>0)
         {
             GAPRole_TerminateConnection(connHandleMap[cindex].connHandle);
             //GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData1),scanRspData1, NULL);
             count_stored_buf=1;
             new_stored_buf=1;
             test_name_change=1;
             if(ble_stat_count==0)
                 name_change=0;//-----------------------------

             device_connected=0;
         }
         else if(n==0)
         {
            new_stored_buf=0;
            n=0;
            writecount=0;
            readcount=0;
            stop_rec_data=0;

            DELAY_MS(100);
            ExtFlash_open();
            erase_external_flash();
            ExtFlash_close();
            DELAY_MS(100);

             if(server_comm_first==1)
             {
                 scanRspData[13]=time_hr;
                 scanRspData[14]=time_min;
                 scanRspData[15]=time_sec;
                 scanRspData[16]=input_date;
                 scanRspData[17]=last_date;
                 scanRspData[18]=Mesh_id;
                // scanRspData[19]=u8_buzz_on_off;
                // scanRspData[20]=u8_buzz_time;

                 GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),scanRspData, NULL);
             }

             count_stored_buf=0;
             test_name_change=0;
             if(ble_stat_count<2)
                 name_change=0;
         }
     }
     ExtFlash_open();
     writereturn = ExtFlash_write(extbuffer0+(writecount*buf_size),buf_size,buf);
     ExtFlash_close();
     writecount++;

//     buffer_cnt++;
     //////////////////////////////////////////////////
     if(writecount >= flashbufsize)
     {
         stop_rec_data= 1;
     }
     else if(writecount < flashbufsize)
     {
         stop_rec_data= 0;

         if(server_comm_first == 1)
         {
             scanRspData[13]=time_hr;
             scanRspData[14]=time_min;
             scanRspData[15]=time_sec;
             scanRspData[16]=input_date;
             scanRspData[17]=last_date;
             scanRspData[18]=Mesh_id;
            // scanRspData[19]=u8_buzz_on_off;
            // scanRspData[20]=u8_buzz_time;
             GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),scanRspData, NULL);
         }
     }
     /////////////////////////////////////////////////////////////////////
    // store_count++;
     n++;

     if(n == 21 || n == 50 || n == 100 || n == 200 || n == 300 || n == 400 || n == 500 || n == 600 || n == 700 || n==800 || n==900 || n==1000 ||  n==1200 || n==1400)
     {
         new_stored_buf=1;
         count_store();
     }
     else if(n==1600 || n==1800 || n==2000 || n == 2250 || n==2500 || n==2750 || n==3000 || n==3250 || n == 3500 || n == 3750 || n == 4000 )
     {
         new_stored_buf=1;
         count_store();
     }
     else if(n == 4200 || n == 4350 || n == 4500 || n == 4650 || n == 4850 || n == 4950 || n == 5000 )
     {
         new_stored_buf=1;
         count_store();
     }
     else if( n == 5100 || n == 5200 || n == 5300 || n == 5400 || n == 5495 || n == 5500)
     {
         new_stored_buf=1;
         count_store();
     }
}

void read_from_flash(uint8_t *buf,int buf_size)
{
    if(readcount<writecount)
    {
        ExtFlash_open();
        readreturn = ExtFlash_read(extbuffer0+(readcount*buf_size),buf_size,buf);
        ExtFlash_close();
        readcount++;
    }
    else if(readcount>writecount && n<writecount)
    {
        readcount=writecount-n;
    }
}

static void readCallback(UART_Handle uart, void *rxBuf, size_t size)
{
    return;
}

//  UART Parameters
void UART_parameter(void)
{
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.writeDataMode =  UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readCallback = readCallback;
   // uartParams.writeCallback = writeCallback;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.dataLength = UART_LEN_8;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.baudRate = 115200;//57600;
    // startaddress = &rxBuf[0];
}
/*********************************************************************
 * EXTERN FUNCTIONS
*/
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t multi_role_gapRoleCBs =
{
  multi_role_eventCB,                   // Events to be handled by the app are passed through the GAP Role here
  multi_role_paramUpdateDecisionCB      // Callback for application to decide whether to accept a param update
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

static oadTargetCBs_t multi_role_oadCBs =
{
  .pfnOadWrite = multi_role_processOadWriteCB // Write Callback.
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NOT_REGISTER       = 0,
   FOR_AOA_SCAN       = 1,
   FOR_ATT_RSP        = 2,
   FOR_AOA_SEND       = 4,
   FOR_TOF_SEND       = 8,
   FOR_OAD_SEND       = 0x10,
}connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t       connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

/*********************************************************************
 * @fn      multi_role_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegister represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t multi_role_RegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // in case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(multi_role_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  if(status == SUCCESS)
  {
    //add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      multi_role_UnRegistertToAllConnectionEvent()
 *
 * @brief   Un register  connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t multi_role_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  // in case  there is no more registration for the connection event than unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(multi_role_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for multi_role.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;

  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void multi_role_init(void)
{
  uint8_t i;
  for (i=0;i<3;i++)
  {
      PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_ON);
      DELAY_MS(100);
      PIN_setOutputValue(hGpioPin, Board_RLED,Board_LED_OFF);
      DELAY_MS(100);
  }
  read_all_setting();
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

    #ifdef COIN_AS_GATEWAY
      IOCPortConfigureSet(IOID_13, IOC_PORT_RFC_GPO0, IOC_IOMODE_NORMAL);
        // Map RFC_GPO1 to DIO6
      IOCPortConfigureSet(IOID_6, IOC_PORT_RFC_GPO1, IOC_IOMODE_NORMAL);
    #endif

    #ifdef WATCHDOG_ENABLE
      init_whatch();
    #endif

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&read_clock, multi_role_clockHandler, READ_PERIOD, 0, false,READ_UART_DATA_EVT);
  Util_constructClock(&buffer_clock, multi_role_clockHandler, SEND_PERIOD, 0, false,SEND_BUFFER_DATA);
  Util_constructClock(&write_uart_clock, multi_role_clockHandler, WRITE_UART, 0, false,UART_WRITE_EVT);
  Util_constructClock(&ack_clock, multi_role_clockHandler, ACK_PERIOD, 0, false, ACK_TIMEOUT_EVT);
  Util_constructClock(&reset_clock, multi_role_clockHandler, RESET_PERIOD, 0, false,RESET_EVT);
  Util_constructClock(&time_rqt_clock, multi_role_clockHandler, TIME_RQT_PERIOD, 0, false,TIME_RQT_EVT);
  Util_constructClock(&ble_stat_clock, multi_role_clockHandler, BLE_STAT_PERIOD, 0, false,BLE_STAT_EVT);

  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  // Setup the GAP
  {
    // Set advertising interval the same for all scenarios
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);

    // Set scan duration
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);

    // Scan interval and window the same for all scenarios
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);

    // Set connection parameters
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, 8);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);

    // Register to receive GAP and HCI messages
    GAP_RegisterForMsgs(selfEntity);
  }

  // Setup the GAP Role Profile
  {
    /*--------PERIPHERAL-------------*/
    uint8_t initialAdvertEnable = TRUE;
    uint16_t advertOffTime = 0;

    // device starts advertising upon initialization
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable, NULL);

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime, NULL);

    // Set scan response data
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData1),
                         scanRspData1, NULL);

    advertData[7]=BLEADDRESS[1];
    advertData[8]=BLEADDRESS[0];
    advertData[9]=BLEADDRESS[3];
    advertData[10]=BLEADDRESS[2];
    advertData[11]=BLEADDRESS[5];
    advertData[12]=BLEADDRESS[4];
    // Set advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);

    // set max amount of scan responses
//    uint8_t scanRes = 0;
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
    // In case that the Unlimited Scanning feature is disabled
    // send the number of scan results to the GAP
//    if(ENABLE_UNLIMITED_SCAN_RES == FALSE)
//    {
//        scanRes = DEFAULT_MAX_SCAN_RES;
//    }

    // Set the max amount of scan responses
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t),
                         &scanRes, NULL);

    // Start the GAPRole and negotiate max number of connections
    VOID GAPRole_StartDevice(&multi_role_gapRoleCBs, &maxNumBleConns);

    // Allocate memory for index to connection handle map
    if (connHandleMap = ICall_malloc(sizeof(connHandleMapEntry_t) * maxNumBleConns))
    {
      // Init index to connection handle map
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        connHandleMap[i].connHandle = INVALID_CONNHANDLE;
      }
    }

    // Allocate memory for per connection discovery information
    if (discInfo = ICall_malloc(sizeof(discInfo_t) * maxNumBleConns))
    {
      // Init index to connection handle map to 0's
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        discInfo[i].charHdl = 0;
        discInfo[i].discState = BLE_DISC_STATE_IDLE;
        discInfo[i].svcEndHdl = 0;
        discInfo[i].svcStartHdl = 0;
      }
    }
  }

  // Open the OAD module and add the OAD service to the application
  if(OAD_SUCCESS != OAD_open(OAD_DEFAULT_INACTIVITY_TIME))
  {
      /*
       *  OAD cannot be opened, steps must be taken in the application to
       *  handle this gracefully, this can mean an error, assert,
       *  or print statement.
       */
  }
  else
  {
      // Register the OAD callback with the application
      OAD_register(&multi_role_oadCBs);
  }

  // GATT
  {
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Initialize GATT Server Services
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    //DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);

    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);
  }

  count_restore();


#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

  Util_startClock(&time_rqt_clock);
  Util_startClock(&read_clock);
}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the multi_role.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
    uint8_t i;
    GetAddress();
    UART_init();
    UART_parameter();
    uart = UART_open(Board_UART0, &uartParams);
    wantedRxBytes = 26;
    rxBytes = UART_read(uart,rxBuf, sizeof(rxBuf));
    PIN_setOutputValue(hGpioPin, Board_GLED, Board_LED_ON);
    multi_role_init();
    UTC_init();


  // Application main loop
  for (;;)
  {
    uint32_t events;
           /*
            if(buzz_cmd_rx1==1)
             {
                 if(u8_buzz_time!=0)
                 buzz_count1++;

                 if(buzz_count1 == u8_buzz_time)
                 {
                   buzz_cmd_rx1=0;
                   buzz_count1=0;
                   u8_buzz_flag=0x0F;
                 }

             }

             if((u8_buzz_flag==0x01) && (u8_buzz_time!=0))
             {
                u8_buzz_flag = 0;
                PIN_setOutputValue(hGpioPin, Board_GLED, Board_LED_OFF);//Board_LED_ON
                DELAY_MS(5);
             }
             else if(u8_buzz_flag==0x0F)
             {
                u8_buzz_flag = 0;
                PIN_setOutputValue(hGpioPin, Board_GLED, Board_LED_ON);//Board_LED_OFF
                DELAY_MS(5);
             }
             */
    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);
    #ifdef WATCHDOG_ENABLE
        Watchdog_clear(WatchdogHandle);
    #endif

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // OAD events
      if(events & OAD_OUT_OF_MEM_EVT)
      {
        // The OAD module is unable to allocate memory, cancel OAD
        OAD_cancel();
      }

      if(events & OAD_QUEUE_EVT)
      {
          // Process the OAD Message Queue
          uint8_t status = OAD_processQueue();

          // If the OAD state machine encountered an error, print it
          // Return codes can be found in oad_constants.h
          if(status == OAD_DL_COMPLETE)
          {
              // Report status
          }
          else if(status == OAD_IMG_ID_TIMEOUT)
          {
              // This may be an attack, terminate the link
              GAPRole_TerminateConnection(OAD_getactiveCxnHandle());
          }
          else if(status != OAD_SUCCESS)
          {
              // Report Error
          }
      }

      if(events & OAD_DL_COMPLETE_EVT)
      {
          // Register for L2CAP Flow Control Events
          L2CAP_RegisterFlowCtrlTask(selfEntity);
      }

      // If RTOS queue is not empty, process app message.
      if (events & MR_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            multi_role_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }


      if(oad_in_progress==0)
      {
          if(server_comm_first==1)
          {
              if (events & UART_WRITE_EVT)
              {
                  if(WRITE_UART)
                      Util_startClock(&write_uart_clock);

                  write_on_uart();

                  if(reset_flash_command_received==1)//------------------------------------
                    {
                        reset_flash_command_received_count++;
                            if(reset_flash_command_received_count==283)
                            {
                                reset_flash_command_received_count=0;
                                reset_flash_command_received=0;
                            }
                    }
              }
          }

          if (events & READ_UART_DATA_EVT)
          {
              if(READ_PERIOD)
                  Util_startClock(&read_clock);

              uart_read_function();

              if(server_comm_first==0)
                no_res_10_sec_count++;

              if((no_res_10_sec_count==4300)&&(server_comm_first==0))
              {
                  //HAL_SYSTEM_RESET();
                   while(1);
              }
              else if((server_comm_first==1)&&(no_res_10_sec_count!=0))
              {
                  no_res_10_sec_count=0;
//                  counter_for30sec = 0;
//                  temp=0,temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0,temp7=0,temp8=0,point=0;
              }
          }

          if (events & SEND_BUFFER_DATA)
          {
              if(SEND_PERIOD)
                  Util_startClock(&buffer_clock);

              if (device_connected==0 && buffer_counter!=0)
              {
                  PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
                  Connect_to_coin();
              }
          }

          if (events & ACK_TIMEOUT_EVT)
          {
              disconnect_and_update_buffer(ack_packet[1]);
          }

          if(events & RESET_EVT)
          {
              GAPRole_TerminateConnection(connHandleMap[cindex].connHandle);
          }

          if(server_comm_first==0)
          {
              if(events & TIME_RQT_EVT)
              {
                  if(TIME_RQT_PERIOD)
                      Util_startClock(&time_rqt_clock);
                  Util_stopClock(&ble_stat_clock);
                  ble_status=1;

                 /* counter_for30sec++;

                  wait_for_35_sec++;
                  if(wait_for_35_sec >= 35)
                  {
                      wait_for_35_sec = 0;
                      write_start_flag = 1;
                  }

                  if(write_start_flag == 1)
                  {
                      wait_for_35_sec=0;
                      write_on_uart();
                  }*/
                  write_on_uart();
             }
          }

          if(ble_status==0)
          {
              if(events & BLE_STAT_EVT)
              {
                  if(BLE_STAT_PERIOD)
                      Util_startClock(&ble_stat_clock);

                       clock_funtion_update();

                        ping_count++;
                        if(ping_count == 10)//if(ping_count == 60)
                        {
                            ping_count=0;
                            status_write = 1;
                            extdata[0]=0;
                            extdata[1]=0x22;
                            extdata[2]=0x22;
                            extdata[3]=0x22;
                            extdata[4]=0x22;
                            extdata[5]=time_hr;
                            extdata[6]=time_min;
                            extdata[7]=time_sec;
                            extdata[8]=input_date;
                            converttoascii();

                              // Optimization 1
                                for(i = 0; i<12; i++)
                                {
                                    onuart1[i] = BLEADDRESS_ASCII[i];
                                }

                                for(i = 0; i<18; i++)
                                {
                                    onuart1[i+12] = uartdata[i];
                                }

                                    onuart1[30] = '\n';


                            UART_write(uart,onuart1,sizeof(onuart1));
                            ble_stat_count++;
                            //ble_stat_count=0;

                            if((ble_stat_count==2)&&(wait_for_data_send==0))
                            {
                                name_change=1;
                            }
                            else if(ble_stat_count == 5 )
                            {
                                count_store();
                                //HAL_SYSTEM_RESET();
                                while(1);
                            }

                            //                            else if(ble_stat_count==3)
                            //                            {
                            //                                temp=0,temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0,temp7=0,temp8=0,point =0,Last=0,count2=0;
                            //                            }
                        }
                      if(buzz_cmd_rx==1||u8_buzz_on_off==0x0F||u8_buzz_on_off==0x01)
                        {
                            buzz_count++;
                        }

                        if(buzz_count == 30)
                        {
                            buzz_count=0;
                            buzz_cmd_rx=0;
                            u8_buzz_on_off=0xAA;
                            new_settings_received=1;
                            store_all_settings();

                        }


                      if(buzz_cmd_rx1==1)
                      {
                          if(u8_buzz_time!=0)
                          buzz_count1++;

                          if(buzz_count1 == u8_buzz_time)
                          {
                            buzz_cmd_rx1=0;
                            buzz_count1=0;
                            u8_buzz_flag=0x0F;
                          }

                      }

                      if((u8_buzz_flag==0x01) && (u8_buzz_time!=0))
                      {
                         u8_buzz_flag = 0;
                         PIN_setOutputValue(hGpioPin, Board_GLED, Board_LED_OFF);//Board_LED_ON
                         DELAY_MS(5);
                      }
                      else if(u8_buzz_flag==0x0F)
                      {
                         u8_buzz_flag = 0;
                         PIN_setOutputValue(hGpioPin, Board_GLED, Board_LED_ON);//Board_LED_OFF
                         DELAY_MS(5);
                      }


                        update_time_in_advt_packet++;
                        if(update_time_in_advt_packet==1)
                        {
                            scanRspData[13]=time_hr;
                            scanRspData[14]=time_min;
                            scanRspData[15]=time_sec;
                            scanRspData[16]=input_date;
                            scanRspData[17]=last_date;
                            scanRspData[18]=Mesh_id;
                            //scanRspData[19]=u8_buzz_on_off;
                           // scanRspData[20]=u8_buzz_time;
                            GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),scanRspData, NULL);

                            update_time_in_advt_packet=0;
                        }
                        status_write=0;
              }
          }
      }
    }
  }
}

/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {
  #if !defined (USE_LL_CONN_PARAM_UPDATE)

              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
  #endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    case GAP_MSG_EVENT:
      multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
      break;

    case L2CAP_SIGNAL_EVENT:
    {
       // Process L2CAP free buffer notification
       multi_role_processL2CAPMsg((l2capSignalEvent_t *)pMsg);
       break;
    }

    default:
      // Do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if( multi_role_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      multi_role_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
      OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
  }

  // Messages from GATT server
  if (linkDB_NumActive() > 0)
  {
    // Find index from connection handle
    connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
  } // Else - in case a GATT message came after a connection has dropped, ignore it.

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      multi_role_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void multi_role_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      multi_role_UnRegistertToAllConnectionEvent (FOR_ATT_RSP);

      // We're done with the response message
      multi_role_freeAttRsp(status);
    }
  }
}

/*********************************************************************
* @fn      multi_role_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void multi_role_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      //Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp sent, retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      //Display_print1(dispHandle, MR_ROW_STATUS2, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case MR_STATE_CHANGE_EVT:
    multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
    // Free the stack message
    ICall_freeMsg(pMsg->pData);
    break;

  case MR_CHAR_CHANGE_EVT:
    multi_role_processCharValueChangeEvt(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;

  case MR_CONN_EVT_END_EVT:
      {
        if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
        {
          // The GATT server might have returned a blePending as it was trying
          // to process an ATT Response. Now that we finished with this
          // connection event, let's try sending any remaining ATT Responses
          // on the next connection event.
          // Try to retransmit pending ATT Response (if any)
          multi_role_sendAttRsp();
        }

        if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_OAD_SEND))
        {
          // Wait until all pending messages are sent
          if(numPendingMsgs == 0)
          {
            // Reset the system
            HAL_SYSTEM_RESET();
          }
          numPendingMsgs--;

        }

          ICall_free(pMsg->pData);
          break;
      }

  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      multi_role_eventCB
*
* @brief   Multi GAPRole event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (multi_role_enqueueMsg(MR_STATE_CHANGE_EVT, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
* @fn      multi_role_paramUpdateDecisionCB
*
* @brief   Callback for application to decide whether or not to accept
*          a parameter update request and, if accepted, what parameters
*          to use
*
* @param   pReq - pointer to param update request
* @param   pRsp - pointer to param update response
*
* @return  none
*/
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp)
{
  // Make some decision based on desired parameters. Here is an example
  // where only parameter update requests with 0 slave latency are accepted
  if (pReq->connLatency == 0)
  {
    // Accept and respond with remote's desired parameters
    pRsp->accepted = TRUE;
    pRsp->connLatency = pReq->connLatency;
    pRsp->connTimeout = pReq->connTimeout;
    pRsp->intervalMax = pReq->intervalMax;
    pRsp->intervalMin = pReq->intervalMin;
  }

  // Don't accept param update requests with slave latency other than 0
  else
  {
    pRsp->accepted = FALSE;
  }
}

/*********************************************************************
* @fn      multi_role_processRoleEvent
*
* @brief   Multi role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    // GAPRole started
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      // Store max pdu size
//      maxPduSize = pEvent->initDone.dataPktLen;

      // Set device info characteristic
      //DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pEvent->initDone.devAddr);
    }
    break;

    // Advertising started
    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    {

    }
    break;

    // Advertising ended
    case GAP_END_DISCOVERABLE_DONE_EVENT:
    {

    }
    break;

    // A discovered device report
    case GAP_DEVICE_INFO_EVENT:
    {
//      if(ENABLE_UNLIMITED_SCAN_RES == TRUE)
//      {
//        multi_role_addDeviceInfo(pEvent->deviceInfo.addr,
//                                        pEvent->deviceInfo.addrType);
//      }

         memset(adv_data,0,sizeof(adv_data));//----------------------------------------------
         memcpy(adv_data,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen);
         ptr=&adv_data[2];
         memcpy(device_name,ptr,device_length);
         memcpy(curr_addr,pEvent->deviceInfo.addr,GATEWAY_DATA);
         devices_discovered++;
         if (devices_discovered!=1)
         {
             res=compare_string(curr_addr,prev_addr,GATEWAY_DATA);
             if (res==0)
             {
                 memcpy(prev_addr,curr_addr,GATEWAY_DATA);
                 dev_counter++;
                 if(pEvent->deviceInfo.eventType==GAP_ADRPT_SCAN_RSP)
                 {
                     if(compare_string(device_name,moving_device_name,moving_device_length)==1)
                         RSSI=pEvent->deviceInfo.rssi;

                     process_adv_packet();
                 }
             }
             else if(res==1 && pEvent->deviceInfo.eventType==GAP_ADRPT_SCAN_RSP)
             {
                 if(compare_string(device_name,moving_device_name,moving_device_length)==1)
                     RSSI=pEvent->deviceInfo.rssi;

                 process_adv_packet();
             }
         }
         else
         {
             memcpy(prev_addr,curr_addr,GATEWAY_DATA);
             dev_counter++;
         }
    }
    break;

    // End of discovery report
    case GAP_DEVICE_DISCOVERY_EVENT:
    {
        uint8 dev_index;
        // Discovery complete
        scanningStarted = FALSE;
        scan_count++;

        if(scan_count==255)
            scan_count=1;

        // Copy all of the results
        scanRes = pEvent->discCmpl.numDevs;
        memcpy(devList, pEvent->discCmpl.pDevList, (sizeof(gapDevRec_t) * scanRes));

        if(buffer_counter!=0 && scan_count==20)
            disconnect_after_10_scans();

        if (dev_list_index>0)
        {
            dest_index=find_dest_addr();

            if(dest_index!=0xFF)
            {
                dev_index=scanned_coin_info[dest_index].DevList_index;
                memcpy(dest_addr,devList[dev_index].addr,GATEWAY_DATA);
                addrType=devList[dev_index].addrType;

                PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);
                // initialize scan index to last device
                GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE, DEFAULT_LINK_WHITE_LIST, addrType, dest_addr);
            }
        }

        /*if(pEvent->gap.hdr.status == SUCCESS)
        {

          // Discovery complete
          scanningStarted = FALSE;
        }*/
    }
    break;

    // Connection has been established
    case GAP_LINK_ESTABLISHED_EVENT:
    {
        scan_count=0;
      // If succesfully established
      if (pEvent->gap.hdr.status == SUCCESS  && stop_rec_data == 0)
      {
          ix=0;jx=0;
          device_connected=1;
          Util_startClock(&reset_clock);
          PIN_setOutputValue(hGpioPin, Board_BLED, Board_LED_ON);

        // Add index-to-connHandle mapping entry and update menus
          cindex = multi_role_addMappingEntry(pEvent->linkCmpl.connectionHandle, pEvent->linkCmpl.devAddr);

          memcpy(connected_device_address,pEvent->linkCmpl.devAddr,GATEWAY_DATA);

          connRole=pEvent->linkCmpl.connRole;
          if(connRole == GAP_PROFILE_CENTRAL)
          {
                  ptr=&coin_buffer[ix][0];
                  memcpy(packet_to_send,ptr,COIN_DATA_2);
                  DataSend(packet_to_send,COIN_DATA_2,simple3handle);

                  if (status!=SUCCESS)
                  {
                      disconnect_and_update_buffer(ix);
                  }
                  else if (status==SUCCESS)
                  {
                      Util_startClock(&ack_clock);
                  }

                  if(packet_to_send[1]==0x45 && buffer_counter==1)
                      buffer_counter=0;
          }

/*        // Add index-to-connHandle mapping entry and update menus
        uint8_t index = multi_role_addMappingEntry(pEvent->linkCmpl.connectionHandle, pEvent->linkCmpl.devAddr);

        //turn off advertising if no available links
        if (linkDB_NumActive() >= maxNumBleConns)
        {
          uint8_t advertEnabled = FALSE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
        }

        // Start periodic clock if this is the first connection
        if (linkDB_NumActive() == 1)
        {
          Util_startClock(&periodicClock);
        }*/
      }
    }
    break;

    // Connection has been terminated
    case GAP_LINK_TERMINATED_EVENT:
    {

        uint8_t advertEnabled = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
        Util_stopClock(&reset_clock);

        // read current num active so that this doesn't change before this event is processed
        uint8_t currentNumActive = linkDB_NumActive();

        // Find index from connection handle
        connIndex = multi_role_mapConnHandleToIndex(pEvent->linkTerminate.connectionHandle);

        //oad
        // Cancel the OAD if one is going on
        // A disconnect forces the peer to re-identify
        OAD_cancel();

        // Check to prevent buffer overrun
        if (connIndex < maxNumBleConns)
        {
          // Clear screen, reset discovery info, and return to main menu
          connHandleMap[connIndex].connHandle = INVALID_CONNHANDLE;

          // Reset discovery info
          discInfo[connIndex].discState = BLE_DISC_STATE_IDLE;
          discInfo[connIndex].charHdl = 0;

          device_connected=0;
          PIN_setOutputValue(hGpioPin, Board_BLED, Board_LED_OFF);

          if(connRole==GAP_PROFILE_PERIPHERAL)
              n=writecount-readcount;
              if(ack_packet[1]<=250 && n==0 && new_stored_buf==1)
              {
                  //                  n=0;
                  new_stored_buf=0;
                  write_high = 0;
                  write_low = 0;
                  read_high = 0;
                  read_low = 0;

                  ExtFlash_open();
                  //                  erase_external_flash();
                  ExtFlash_erase(write_count_store,1);
                  ExtFlash_close();
                  stop_rec_data=0;
              }
              memset(ack_packet,0,sizeof(ack_packet));

          connRole=0;
        }

      /*// read current num active so that this doesn't change before this event is processed
      uint8_t currentNumActive = linkDB_NumActive();

      // Find index from connection handle
      connIndex = multi_role_mapConnHandleToIndex(pEvent->linkTerminate.connectionHandle);

      // Cancel the OAD if one is going on
      // A disconnect forces the peer to re-identify
      OAD_cancel();

      // Check to prevent buffer overrun
      if (connIndex < maxNumBleConns)
      {
        // Clear screen, reset discovery info, and return to main menu
        connHandleMap[connIndex].connHandle = INVALID_CONNHANDLE;

        // Reset discovery info
        discInfo[connIndex].discState = BLE_DISC_STATE_IDLE;
        discInfo[connIndex].charHdl = 0;

        // If there aren't any active connections
        if (currentNumActive == 0)
        {
          // Stop periodic clock
          Util_stopClock(&periodicClock);
        }

        // If it is possible to advertise again
        if (currentNumActive == (maxNumBleConns-1))
        {

        }
      }*/
    }
    break;

    // A parameter update has occurred
    case GAP_LINK_PARAM_UPDATE_EVENT:
    {

    }
    break;

  default:
    break;
  }
}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = paramID;

    // Queue the event.
    multi_role_enqueueMsg(MR_CHAR_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_processCharValueChangeEvt(uint8_t paramID)
{
    uint8_t i;
  // Print new value depending on which characteristic was updated
  switch(paramID)
  {
  case SIMPLEPROFILE_CHAR1:
        if(connRole==GAP_PROFILE_PERIPHERAL  && stop_rec_data == 0)
        {
            ack_packet[0]=ack;
            ack_packet[1]=++jx;
            DataSend(ack_packet,2,simple1handle);

            clock_funtion_update();

            if(compare_string(coindata,last_received_packet,COIN_DATA)==0 )
            {

                 for(i=0; i<9; i++)  // Optimization 3
                    {
                        utc_time_data[i]=coindata[i];
                    }


                write_to_flash(&utc_time_data[0],UART_WRITE_SIZE);
                memcpy(last_received_packet,coindata,COIN_DATA);
//                buffer_cnt111++;
            }
        }
        else if(writecount == 0 || writecount == readcount)
        {
                stop_rec_data = 0;
        }

    break;

  case SIMPLEPROFILE_CHAR3:
      if(connRole == GAP_PROFILE_CENTRAL)
      {
          if (ack_packet[0]==ack && ack_packet[1]<buffer_counter)
          {
               Util_stopClock(&ack_clock);
               ptr=&coin_buffer[ack_packet[1]][0];
               memcpy(packet_to_send,ptr,COIN_DATA_2);
               DataSend(packet_to_send,COIN_DATA_2,simple3handle);

               if (status!=SUCCESS)
                   disconnect_and_update_buffer(ack_packet[1]);
               else
                   Util_startClock(&ack_clock);

               status=0xFF;
          }
          else if (ack_packet[0]==ack && ack_packet[1]==buffer_counter)
          {
              disconnect_and_update_buffer(ack_packet[1]);
          }

      }
      else if(mesh_packet[0]==0x59 && (mesh_packet[1]!=0 && mesh_packet[1]<0xFF) && mesh_packet[2]==0xFF)
      {
          Mesh_id=mesh_packet[1];
          mesh_id_update();
      }
      else if(mesh_packet[0]==0xFF && (mesh_packet[1]==0xFF) && mesh_packet[2]==0)
      {
          restart_erase_flash_all_paramerter();
      }
      ///////*****************Buzz on/off_cmd*************************///
     else if(mesh_packet[0]==0x51 && mesh_packet[2]==0xFF)
     {
         u8_buzz_on_off = mesh_packet[1];//threshold_value
         u8_buzz_flag = mesh_packet[1];

         PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
         DELAY_MS(100);
         PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);
         buzz_cmd_rx1=1;
     }///////*****************Buzz_set time_value*************************///
     else if(mesh_packet[0]==0x52 && mesh_packet[2]==0xFF)
     {
         u8_buzz_time=mesh_packet[1];

         PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
         DELAY_MS(100);
         PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);
         new_settings_received=1;

     }



    break;

  default:
    // Should not reach here!
    break;
  }
}


void mesh_id_update(void)
{
//    scanRspData[18]=Mesh_id;
    if(server_comm_first==1)
    {
        scanRspData[13]=time_hr;
        scanRspData[14]=time_min;
        scanRspData[15]=time_sec;
        scanRspData[16]=input_date;
        scanRspData[17]=last_date;
        scanRspData[18]=Mesh_id;
        //scanRspData[19]=u8_buzz_on_off;
        //scanRspData[20]=u8_buzz_time;
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData, NULL);
    }

    PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
    DELAY_MS(150);
    PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);

    loc_mesh1=0,loc_mesh2=0;
    ExtFlash_open();
    ExtFlash_erase(loc_mesh1+Mesh_id_SETTING,1);
    ptr = &Mesh_id;
    writeretrn1= ExtFlash_write(loc_mesh2+Mesh_id_SETTING,1,ptr);
    ExtFlash_close();
}

/*********************************************************************
* @fn      multi_role_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   pData - pointer to data to be queued
*
* @return  None.
*/
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData)
{
  // Allocate space for the message
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

  // If sucessfully allocated
  if (pMsg)
  {
    // Fill up message
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
* @fn      multi_role_mapConnHandleToIndex
*
* @brief   Translates connection handle to index
*
* @param   connHandle - the connection handle
*
* @return  index or INVALID_CONNHANDLE if connHandle isn't found
*/
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle)
{
  uint16_t index;
  // Loop through connection
  for (index = 0; index < maxNumBleConns; index ++)
  {
    // If matching connection handle found
    if (connHandleMap[index].connHandle == connHandle)
    {
      return index;
    }
  }
  // Not found if we got here
  return INVALID_CONNHANDLE;
}

/*********************************************************************
 * @fn      multi_role_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 */
static void multi_role_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}


/*********************************************************************
 * @fn      multi_role_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void multi_role_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if( multi_role_enqueueMsg(MR_CONN_EVT_END_EVT, (uint8_t *)pReport) == FALSE)
  {
    ICall_free(pReport);
  }

}

/*********************************************************************
 * @brief   Perform a periodic application task to demonstrate notification
 *          capabilities of simpleGATTProfile. This function gets called
 *          every five seconds (MR_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 */

/*
static void multi_role_performPeriodicTask(void)
{
  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called. Also note that this will
    // send a notification to each connected device.
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
}
*/

/*********************************************************************
* @fn      multi_role_addMappingEntry
*
* @brief   add a new connection to the index-to-connHandle map
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static uint8_t multi_role_addMappingEntry(uint16_t connHandle, uint8_t *addr)
{
  uint16_t index;
  // Loop though connections
  for (index = 0; index < maxNumBleConns; index++)
  {
    // If there is an open connection
    if (connHandleMap[index].connHandle == INVALID_CONNHANDLE)
    {
      // Store mapping
      connHandleMap[index].connHandle = connHandle;

      // Convert address to string
      uint8_t *pAddr = (uint8_t *) Util_convertBdAddr2Str(addr);

      // Copy converted string to persistent connection handle list
      memcpy(connHandleMap[index].strAddr, pAddr, B_STR_ADDR_LEN);

      return index;
    }
  }
  // No room if we get here
  return bleNoResources;
}


/*********************************************************************
*********************************************************************/
//application
void Connect_to_coin(void)
{
    uint8_t advreturn1,i,adv;
    memset(scanned_coin_info,0,sizeof(scanned_coin_info));//---------------
    memset(adv_data,0,sizeof(adv_data));
    dev_list_index=0;
    dev_counter=0;
    devices_discovered=0;

    for (i=0;i<DEFAULT_MAX_SCAN_RES;i++)
    {
        devList[i].addrType=0;
        devList[i].eventType=0;
        memset(devList[i].addr,0,GATEWAY_DATA);
    }

    memset(scanned_coin_info,0,sizeof(scanned_coin_info));
    GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &advreturn1, NULL);
    if(advreturn1==1)
    {
        adv=FALSE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv, NULL);
    }

    // Start or stop discovery
    if (linkDB_NumActive() < maxNumBleConns) //if we can connect to another device
    {
       if (!scanningStarted) //if we're not already scanning connecting_state
       {
           scanningStarted = TRUE;
           scanRes = 0;
           GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
       }
       else //cancel scanning
       {
           GAPRole_CancelDiscovery();
           scanningStarted = FALSE;
       }
    }
}

// Gateway Functions

// Get BLE Address of the Device
void GetAddress(void)
{
    int i,k=0;

    memcpy(&BLEADDRESS[0],BLE0,sizeof(*BLE0));
    memcpy(&BLEADDRESS[1],BLE1,sizeof(*BLE1));
    memcpy(&BLEADDRESS[2],BLE2,sizeof(*BLE2));
    memcpy(&BLEADDRESS[3],BLE3,sizeof(*BLE3));
    memcpy(&BLEADDRESS[4],BLE4,sizeof(*BLE4));
    memcpy(&BLEADDRESS[5],BLE5,sizeof(*BLE5));

    for(i=0;i<6;i++)
    {
        BLEADDRESS_ASCII1[k] = (BLEADDRESS[i]&0xF0)>>4;
        BLEADDRESS_ASCII1[k+1] = (BLEADDRESS[i]&0x0F);
        k=k+2;
    }

    for(i=0;i<12;i++)       //Optimization 5
    {
        if(BLEADDRESS_ASCII1[i] == 0x00)
            BLEADDRESS_ASCII[i] = '0';
        else if(BLEADDRESS_ASCII1[i] == 0x01)
            BLEADDRESS_ASCII[i] = '1';
        else if(BLEADDRESS_ASCII1[i] == 0x02)
            BLEADDRESS_ASCII[i] = '2';
        else if(BLEADDRESS_ASCII1[i] == 0x03)
            BLEADDRESS_ASCII[i] = '3';
        else if(BLEADDRESS_ASCII1[i] == 0x04)
            BLEADDRESS_ASCII[i] = '4';
        else if(BLEADDRESS_ASCII1[i] == 0x05)
            BLEADDRESS_ASCII[i] = '5';
        else if(BLEADDRESS_ASCII1[i] == 0x06)
            BLEADDRESS_ASCII[i] = '6';
        else if(BLEADDRESS_ASCII1[i] == 0x07)
            BLEADDRESS_ASCII[i] = '7';
        else if(BLEADDRESS_ASCII1[i] == 0x08)
            BLEADDRESS_ASCII[i] = '8';
        else if(BLEADDRESS_ASCII1[i] == 0x09)
            BLEADDRESS_ASCII[i] = '9';
        else if(BLEADDRESS_ASCII1[i] == 0x0A)
            BLEADDRESS_ASCII[i] = 'A';
        else if(BLEADDRESS_ASCII1[i] == 0x0B)
            BLEADDRESS_ASCII[i] = 'B';
        else if(BLEADDRESS_ASCII1[i] == 0x0C)
            BLEADDRESS_ASCII[i] = 'C';
        else if(BLEADDRESS_ASCII1[i] == 0x0D)
            BLEADDRESS_ASCII[i] = 'D';
        else if(BLEADDRESS_ASCII1[i] == 0x0E)
            BLEADDRESS_ASCII[i] = 'E';
        else if(BLEADDRESS_ASCII1[i] == 0x0F)
            BLEADDRESS_ASCII[i] = 'F';

    }
}


void read_all_setting(void)
{
    LOC0=0;

    ExtFlash_open();

    ptr = &ALL_SETTINGS[0];
    readreturn1= ExtFlash_read(LOC0+EXT_FLASH_SETTINGS,4,ptr);

    input_date = ALL_SETTINGS[0];
    if(input_date==0xFF || input_date==0x00)
    {
        input_date=1;
    }
    last_date = ALL_SETTINGS[1];
    if(last_date==0xFF || last_date==0x00)
    {
        last_date=0x1E;
    }
    u8_buzz_on_off = ALL_SETTINGS[2];
    if(u8_buzz_on_off==0xFF || u8_buzz_on_off==0x00)
    {
        u8_buzz_on_off=0xAA;
    }
    u8_buzz_time = ALL_SETTINGS[3];
    if(u8_buzz_time==0xFF)
    {
        u8_buzz_time=0x14;
    }

    ExtFlash_close();

    if(last_date==28 || last_date==0x1C)
        last_date_value=28;
    else if(last_date==29 || last_date==0x1D)
        last_date_value=29;
    else if(last_date==30 || last_date==0x1E)
        last_date_value=30;
    else if(last_date==31 || last_date==0x1F)
        last_date_value=31;

}

void store_all_settings(void)
{
    LOC0=0;

    ALL_SETTINGS[0]=input_date;//
    ALL_SETTINGS[1]=last_date;//

    ALL_SETTINGS[2]=u8_buzz_on_off;
    ALL_SETTINGS[3]=u8_buzz_time;


    ExtFlash_open();
    ExtFlash_erase(EXT_FLASH_SETTINGS,1);

    ptr = &ALL_SETTINGS[0];
    writeretrn1=ExtFlash_write(LOC0+EXT_FLASH_SETTINGS,4,ptr);

    ExtFlash_close();

    clock_funtion_update();

    if((server_comm_first == 1) && (new_settings_received == 1))
    {
        scanRspData[13]=time_hr;
        scanRspData[14]=time_min;
        scanRspData[15]=time_sec;
        scanRspData[16]=input_date;
        scanRspData[17]=last_date;
        scanRspData[18]=Mesh_id;
        //scanRspData[19]=u8_buzz_on_off;
       // scanRspData[20]=u8_buzz_time;

        if(input_date!=0)
        {
            GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData, NULL);
        }
    }
    new_settings_received=0;

    if(last_date==28 || last_date==0x1C)
        last_date_value=28;
    else if(last_date==29 || last_date==0x1D)
        last_date_value=29;
    else if(last_date==30 || last_date==0x1E)
        last_date_value=30;
    else if(last_date==31 || last_date==0x1F)
        last_date_value=31;
}

void restart_erase_flash_all_paramerter(void)
{
    version=0;
    DELAY_MS(500);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData1), scanRspData1, NULL);
    stop_rec_data=1;
    new_stored_buf=0;
    count_stored_buf=0;
    write_high = 0;
    write_low = 0;
    read_high = 0;
    read_low = 0;
    test_name_change=1;
    n=0;
    readcount=0;
    writecount=0;
    reset_flash_command_received=1;

    DELAY_MS(100);
    ExtFlash_open();
    erase_external_flash();
    ExtFlash_close();
    DELAY_MS(100);

    server_comm_first=0;
    Util_startClock(&time_rqt_clock);
    Util_stopClock(&write_uart_clock);
    Util_stopClock(&ble_stat_clock);
    //  Util_stopClock(&read_clock);
    ble_status=1;
    ble_stat_count = 0;
    ping_count=0;
}

void uart_read_function(void)
{
    if(strncmp(&rxBuf,&BLEADDRESS_ASCII,12)==0)
    {
        if((rxBuf[12]!=0 && rxBuf[12]!='T' && rxBuf[12]!='S') ||
                (rxBuf[13]!=0 && rxBuf[13]!='T' && rxBuf[13]!='S') ||
                    (rxBuf[14]!=0 && rxBuf[14]!='T' && rxBuf[14]!='S') ||
                        (rxBuf[15]!=0 && rxBuf[15]!='T' && rxBuf[15]!='S') ||
                            (rxBuf[16]!=0 && rxBuf[16]!='T' && rxBuf[16]!='S') ||
                                (rxBuf[17]!=0 && rxBuf[17]!='T' && rxBuf[17]!='S') ||
                                    (rxBuf[18]!=0 && rxBuf[18]!='T' && rxBuf[18]!='S') ||
                                        (rxBuf[19]!=0 && rxBuf[19]!='T' && rxBuf[19]!='S') ||
                                            (rxBuf[20]!=0 && rxBuf[20]!='T' && rxBuf[20]!='S') ||
                                                (rxBuf[21]!=0 && rxBuf[21]!='T' && rxBuf[21]!='S') ||
                                                    (rxBuf[22]!=0 && rxBuf[22]!='T' && rxBuf[22]!='S') ||
                                                        (rxBuf[23]!=0 && rxBuf[23]!='T' && rxBuf[23]!='S') ||
                                                             (rxBuf[24]!=0 && rxBuf[24]!='T' && rxBuf[24]!='S')||
                                                                 (rxBuf[25]!=0 && rxBuf[25]!='T' && rxBuf[25]!='S'))
        {

              //Optimization 6

          for(uint8_t i = 0; i<14; i++)
           {
            dataToSend1[i] = rxBuf[i+12];
           }

            convert();
            if(2 == outarray[0] && 0 == outarray[1] && 0 == outarray[2] && 0 == outarray[3] && 0 == outarray[4] && 0 == outarray[5] && 0 == outarray[6] && 0 == outarray[7] && 0 == outarray[8] && 0 == outarray[9])
            {
                ble_stat_count = 0;
                ble_status=0;
                name_change=0;
            }
            else if(outarray[0]==0 && (outarray[1]<0x18) && (outarray[2]<0x3C)
                && (outarray[3]<0x3C) && (outarray[4]>0 && outarray[4]<0x20)  &&  (outarray[5]>=28
                && outarray[5]<=31) && outarray[6]==0  )
            {
                Util_stopClock(&time_rqt_clock);//---------------------------------------------
                memcpy(utc_time_server,outarray,COIN_DATA);
                utc_time_server_hr=utc_time_server[1];
                utc_time_server_min=utc_time_server[2];
                utc_time_server_sec=utc_time_server[3];
                input_date=utc_time_server[4];
                last_date=utc_time_server[5];

                utc_time_server_input=utc_time_server_hr*60*60+utc_time_server_min*60+utc_time_server_sec;
                UTC_setClock(utc_time_server_input);

                new_settings_received=1;
                ////new addition for time update
                if(server_comm_first== 1)
                {
                    scanRspData[13]=utc_time_server_hr;
                    scanRspData[14]=utc_time_server_min;
                    scanRspData[15]=utc_time_server_sec;
                    scanRspData[16]=input_date;
                    scanRspData[17]=last_date;
                    scanRspData[18]=Mesh_id;
                    //scanRspData[19]=u8_buzz_on_off;
                   // scanRspData[20]=u8_buzz_time;

                    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData, NULL);
//                    PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
//                    DELAY_MS(150);
//                    PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);

                    if(writecount >= flashbufsize)
                    {
                        stop_rec_data= 1;
                    }
                    else if(writecount < flashbufsize)
                    {
                        stop_rec_data= 0;
                    }

                    setting_ack =0;
                }
                //////////////////////////////////////////////////////////////////////////////////

                if(server_comm_first == 0)
                {
                    if(n==0 || reset_flash_command_received==1) //-------------------------------------------------
                    {
                        scanRspData[13]=utc_time_server_hr;
                        scanRspData[14]=utc_time_server_min;
                        scanRspData[15]=utc_time_server_sec;
                        scanRspData[16]=input_date;
                        scanRspData[17]=last_date;
                        scanRspData[18]=Mesh_id;
                        //scanRspData[19]=u8_buzz_on_off;
                       // scanRspData[20]=u8_buzz_time;

                        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData, NULL);
                        test_name_change=0;
                        wait_for_data_send=0;//-------------------------------------------------------------
                    }
                    else if(n>0)//-----------------------------------
                    {
                       wait_for_data_send=1;
                       test_name_change=1;
                       count_stored_buf=1;
                       new_stored_buf=1;
                    }
                    server_comm_first=1;

                    Util_startClock(&write_uart_clock);
                    Util_startClock(&ble_stat_clock);
                    Util_startClock(&read_clock);

                    PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);
                    ble_status=0;
                    ble_stat_count = 0;//
                    name_change=0;
                    reset_flash_command_received=0;//-------------------------------------------------

                    stop_rec_data = 0;
                }
            }
            else if (outarray[0]==0xFF && outarray[1]==0x00 && outarray[2]==0xFF && outarray[3]==0xFF && reset_flash_command_received==0)
            {
                restart_erase_flash_all_paramerter();
            }//----------------------------------------------------------------------------
            else if(outarray[0]==0xB1 && (outarray[1]==0) && outarray[2]==0 && outarray[3]==0 && outarray[4]==0 && outarray[5]==0 && outarray[4]==0)
            {
                version=0;
                PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
                DELAY_MS(150);
                PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);
            }
            else if(outarray[0]==0x59 && (outarray[1]!=0 && outarray[1]<0xFF) && outarray[2]==0 && outarray[3]==0 && outarray[4]==0)
            {
                Mesh_id=outarray[1];
                mesh_id_update();
            }
            ///////*****************Buzz on/off_cmd*************************///
            else if(outarray[0]==0x51 && outarray[2]==0 && outarray[3]==0 && outarray[4]==0)
            {
               u8_buzz_on_off = outarray[1];
               u8_buzz_flag = outarray[1];

               PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
               DELAY_MS(150);
               PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);
               new_settings_received=1;
               buzz_cmd_rx=1;
               buzz_cmd_rx1=1;
            }///////*****************Buzz_set time_value*************************///
            else if(outarray[0]==0x52 && outarray[2]==0 && outarray[3]==0 && outarray[4]==0)
            {
               u8_buzz_time=outarray[1];

               PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
               DELAY_MS(150);
               PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);
               new_settings_received=1;
            }
            else
            {
                if(outarray[2]!=0)
                {
                    memcpy(coin_buffer,outarray,COIN_DATA_2);
                    buffer_counter=1;

                    getset_dynid_16=coin_buffer[0][5];
                    getset_dynid_16=(getset_dynid_16<<8) | coin_buffer[0][6];
                    Util_startClock(&buffer_clock);
                }
            }
            UART_control(uart, UARTCC26XX_CMD_RX_FIFO_FLUSH, 0);
            memset(rxBuf,0,sizeof(rxBuf));
            rxBytes = UART_read(uart,rxBuf, sizeof(rxBuf));
            ble_stat_count = 0;
            // name_change=0;


            if(new_settings_received==1)
            {
                store_all_settings();
            }
        }
    }
    else if(rxBuf[0]!=0x00)
    {
        UART_control(uart, UARTCC26XX_CMD_RX_FIFO_FLUSH, 0);
        memset(rxBuf,0,sizeof(rxBuf));
        rxBytes = UART_read(uart,rxBuf, sizeof(rxBuf));
    }

}

void convert(void)
{
    memcpy(inarray,dataToSend1,14);
    int i;
    for(i=0;i<14;i++)                                  // Optimization 7
    {
        if(inarray[i]=='1')
            inoutarray[i] = 0x01;
        else if(inarray[i]=='2')
            inoutarray[i] = 0x02;
        else if(inarray[i]=='3')
            inoutarray[i] = 0x03;
        else if(inarray[i]=='4')
            inoutarray[i] = 0x04;
        else if(inarray[i]=='5')
            inoutarray[i] = 0x05;
        else if(inarray[i]=='6')
            inoutarray[i] = 0x06;
        else if(inarray[i]=='7')
            inoutarray[i] = 0x07;
        else if(inarray[i]=='8')
            inoutarray[i] = 0x08;
        else if(inarray[i]=='9')
            inoutarray[i] = 0x09;
        else if(inarray[i]=='A')
            inoutarray[i] = 0x0A;
        else if(inarray[i]=='B')
            inoutarray[i] = 0x0B;
        else if(inarray[i]=='C')
            inoutarray[i] = 0x0C;
        else if(inarray[i]=='D')
            inoutarray[i] = 0x0D;
        else if(inarray[i]=='E')
            inoutarray[i] = 0x0E;
        else if(inarray[i]=='F')
            inoutarray[i] = 0x0F;
        else if(inarray[i]=='0')
            inoutarray[i] = 0x00;


    }
    outarray[0] = ((inoutarray[0]<<4) & 0xF0) | inoutarray[1];
    outarray[1] = ((inoutarray[2]<<4) & 0xF0) | inoutarray[3];
    outarray[2] = ((inoutarray[4]<<4) & 0xF0) | inoutarray[5];
    outarray[3] = ((inoutarray[6]<<4) & 0xF0) | inoutarray[7];
    outarray[4] = ((inoutarray[8]<<4) & 0xF0) | inoutarray[9];
    outarray[5] = ((inoutarray[10]<<4) & 0xF0) | inoutarray[11];
    outarray[6] = ((inoutarray[12]<<4) & 0xF0) | inoutarray[13];

    if((outarray[2]==0x14 && outarray[3]==0x15) && ((server_comm_first== 1)))/* || !(outarray[0]==0 && (outarray[1]<0x18) && (outarray[2]<0x3C)
                                                                            &&  (outarray[3]<0x3C) && (outarray[4]>0 && outarray[4]<0x20)
                                                                            &&  (outarray[5]>=28  && outarray[5]<=31) && outarray[6]==0 )))*/
    {
        if(outarray[4]<0x05)
            outarray[4]=0x05;
    }
    else if((outarray[2]!=0x14 && outarray[3]==0x15) && ((server_comm_first== 1)))/* || !(outarray[0]==0 && (outarray[1]<0x18) && (outarray[2]<0x3C)
                                                                                &&  (outarray[3]<0x3C) && (outarray[4]>0 && outarray[4]<0x20)
                                                                                &&  (outarray[5]>=28  && outarray[5]<=31) && outarray[6]==0 )))*/
    {
        if(outarray[4]<0x14)
            outarray[4]=0x14;
    }
}

void converttoascii(void)          // Optimization 8
{
     int i;
     uint8_t hexbuffer[UART_WRITE_SIZE];
     memcpy(hexbuffer,extdata,sizeof(extdata));
     uint8_t hexbuffer1[2*UART_WRITE_SIZE];
     uint8_t hexbuffer2[2*UART_WRITE_SIZE];

    for(uint8_t i = 0; i<9; i++)
       {
         hexbuffer1[i*2]=hexbuffer[i]&0xF0;
         hexbuffer1[i*2]=hexbuffer1[i*2]>>4;  // 0,2,...16
         hexbuffer1[i*2+1]=hexbuffer[i]&0x0F; // 1,3,...17
       }


   for(i=0;i<18;i++)           // Optimization 9
   {

     if(hexbuffer1[i] == 0x00)
       hexbuffer2[i] = '0';
     else if(hexbuffer1[i] == 0x01)
       hexbuffer2[i] = '1';
     else if(hexbuffer1[i] == 0x02)
       hexbuffer2[i] = '2';
     else if(hexbuffer1[i] == 0x03)
       hexbuffer2[i] = '3';
     else if(hexbuffer1[i] == 0x04)
       hexbuffer2[i] = '4';
     else if(hexbuffer1[i] == 0x05)
       hexbuffer2[i] = '5';
     else if(hexbuffer1[i] == 0x06)
       hexbuffer2[i] = '6';
     else if(hexbuffer1[i] == 0x07)
       hexbuffer2[i] = '7';
     else if(hexbuffer1[i] == 0x08)
       hexbuffer2[i] = '8';
     else if(hexbuffer1[i] == 0x09)
       hexbuffer2[i] = '9';
     else if(hexbuffer1[i] == 0x0A)
       hexbuffer2[i] = 'A';
     else if(hexbuffer1[i] == 0x0B)
       hexbuffer2[i] = 'B';
     else if(hexbuffer1[i] == 0x0C)
       hexbuffer2[i] = 'C';
     else if(hexbuffer1[i] == 0x0D)
       hexbuffer2[i] = 'D';
     else if(hexbuffer1[i] == 0x0E)
       hexbuffer2[i] = 'E';
     else if(hexbuffer1[i] == 0x0F)
       hexbuffer2[i] = 'F';

   }
   memcpy(uartdata,hexbuffer2,sizeof(hexbuffer2));
}

void write_on_uart(void)
{
    uint8_t i;                                  // Optimization 10
    if(version == 0 && server_comm_first==1)
    {
        for(i = 0; i<12; i++)
        {
            onuart2[i] = BLEADDRESS_ASCII[i];
        }

        for(i = 0; i<24; i++)
        {
            onuart2[i+12] = BLE_version[i];
        }
            onuart2[36] = '\n';


        UART_write(uart,onuart2,sizeof(onuart2));
        version = 1;
        setting_ack =0;
    }

    if((server_comm_first==1) && (setting_ack == 0) )
    {
        status_write = 1;
        DELAY_MS(250);

        for( i = 0; i<5; i++ )          // Optimization 11
           {
             extdata[i]=outarray[i+1];
           }
       /*
        extdata[0]=outarray[1];//hh
        extdata[1]=outarray[2];//mm
        extdata[2]=outarray[3];//ss
        extdata[3]=outarray[4];//dd
        extdata[4]=outarray[5];//ld
     */
        extdata[5]=0;
        extdata[6]=0;
        extdata[7]=0xA1;
        extdata[8]=0xA1;

        converttoascii();

        for(i = 0; i<12; i++)
        {
            onuart1[i] = BLEADDRESS_ASCII[i];
        }

        for(i = 0; i<18; i++)
        {
            onuart1[i+12] = uartdata[i];
        }
             onuart1[30] = '\n';

        UART_write(uart,onuart1,sizeof(onuart1));
        setting_ack = 1;
        status_write = 0;
    }

    if(server_comm_first==0)
    {
        extdata[0]=0;
        for( i=1; i<9; i++ )            // Optimization 12
           {
             extdata[i]=0x44;
           }

        converttoascii();

        for(i = 0; i<12; i++)
        {
            onuart1[i] = BLEADDRESS_ASCII[i];
        }

        for(i = 0; i<18; i++)
        {
            onuart1[i+12] = uartdata[i];
        }
             onuart1[30] = '\n';


        UART_write(uart,onuart1,sizeof(onuart1));
        PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_ON);
    }
    else if(n>0 /*&& device_connected==0*/ && server_comm_first==1 && name_change==0 && status_write == 0 /*&& Last==1*/)
    {
        read_from_flash(&extdata[0],UART_WRITE_SIZE);

        if((extdata[0]!=0x00 || extdata[1] != 0x00) && extdata[2] != 0x00 )
        {
            converttoascii(); //following loop is for the MM - MP - PM data 12 14 15 / 71 72 73 / 51 52 53 / x55 56 57 x58 59 60

            #if defined ENABLE_PM
                if( (uartdata[4] == 0x37 && uartdata[5] == 0x31)
                    || (uartdata[4] == 0x37 && uartdata[5] == 0x32)
                    || (uartdata[4] == 0x37 && uartdata[5] == 0x33))
            #else
               if((uartdata[4] == 0x31 && uartdata[5] == 0x35)
                  || (uartdata[4] == 0x31 && uartdata[5] == 0x34)
                  || (uartdata[4] == 0x31 && uartdata[5] == 0x32)
                  || (uartdata[4] == 0x35 && uartdata[5] == 0x31)
                  || (uartdata[4] == 0x35 && uartdata[5] == 0x32)
                  || (uartdata[4] == 0x35 && uartdata[5] == 0x33))
           #endif
            {

                        #if defined ENABLE_PM
                           if( (uartdata[4] == 0x37 && uartdata[5] == 0x31))
                        #else
                               if((uartdata[4] == 0x31 && uartdata[5] == 0x32)
                                || (uartdata[4] == 0x35 && uartdata[5] == 0x31))// ( 12 || 51)
                        #endif

                           {
                              for( i=0; i<18; i++ )         // Optimization 13
                                {
                                    accel_X[i] = uartdata[i];
                                }

                               flag_X = 1;
                           }
                        #if defined ENABLE_PM
                           else if((uartdata[4] == 0x37 && uartdata[5] == 0x32))
                        #else
                            else if((uartdata[4] == 0x31 && uartdata[5] == 0x34)
                                 || (uartdata[4] == 0x35 && uartdata[5] == 0x32) )// ( 14 || 52)
                        #endif
                           {
                               for( i=0; i<10; i++ )            // Optimization 14
                                {
                                   accel_Y[i] = uartdata[i];
                                }

                              flag_Y = 1;
                           }

                    #if defined ENABLE_PM
                           else if((uartdata[4] == 0x37 && uartdata[5] == 0x33))
                    #else
                               else if((uartdata[4] == 0x31 && uartdata[5] == 0x35)
                                    || (uartdata[4] == 0x35 && uartdata[5] == 0x33))// ( 15 || 53)
                    #endif

                           {

                                for( i=0; i<10; i++ )   // Optimization 15
                                {
                                    accel_Z[i] = uartdata[i];
                                }

                              flag_Z = 1;
                            }

                        if( flag_X == 1 && flag_Y == 1  && flag_Z == 1)
                        {

                            for( i=0; i<12; i++ )   // Optimization 16
                                {
                                     X_Y_Z[i] = BLEADDRESS_ASCII[i]; //BLE ID
                                }

                            for( i=0; i<18; i++ )
                            {
                                if(i<10)            //coin id
                                {
                                    X_Y_Z[i+12] = accel_X[i];
                                }
                                else if(i>9)        //time
                                {
                                    X_Y_Z[i+20] = accel_X[i];
                                }
                            }

                            for( i=6; i<10; i++ )
                            {
                                X_Y_Z[i+16] =accel_Y[i];
                            }

                            for( i=6; i<10; i++ )
                            {
                                X_Y_Z[i+20] = accel_Z[i];
                            }


                           X_Y_Z[38] = '\n';

                           flag_X_Y_Z = 1;
                        }

                        if(flag_X_Y_Z == 1)//coindata[8]>0 && (coindata[7]!=0 || coindata[6]!=0 || coindata[5]!=0)
                        {
                            if( (!(X_Y_Z[12]==0x30 && X_Y_Z[13]==0x30))  || (!(X_Y_Z[14]==0x30 && X_Y_Z[15]==0x30))  )
                            {
                                if((!(X_Y_Z[12]==0x46 && X_Y_Z[13]==0x46))  || (!(X_Y_Z[14]==0x46 && X_Y_Z[15]==0x46)) )
                                {
    //                               if( ( (X_Y_Z[18]!=0x30) || (X_Y_Z[19]!=0x30) || (X_Y_Z[20]!=0x30) || (X_Y_Z[21]!=0x30) ) && ( (X_Y_Z[22]!=0x30) || (X_Y_Z[23]!=0x30) || (X_Y_Z[24]!=0x30) || (X_Y_Z[25]!=0x30) ) && ( (X_Y_Z[26]!=0x30) || (X_Y_Z[27]!=0x30) || (X_Y_Z[28]!=0x30) || (X_Y_Z[29]!=0x30) ))
    //                               {
                                        if(status_write == 0)
                                        {
                                            UART_write(uart,X_Y_Z,sizeof(X_Y_Z));
                                            flag_X_Y_Z = 0;
                                            flag_X = 0;
                                            flag_Y = 0;
                                            flag_Z = 0;
                                        }
                                        else if(status_write == 1)
                                        {
                                            readcount--;
                                            n++;
                                        }
    //                               }
                                   /*else if( ( (X_Y_Z[18]==0x30) && (X_Y_Z[19]==0x30) && (X_Y_Z[20]==0x30) && (X_Y_Z[21]==0x30) ) && ( (X_Y_Z[22]==0x30) && (X_Y_Z[23]==0x30) && (X_Y_Z[24]==0x30) && (X_Y_Z[25]==0x30) ) && ( (X_Y_Z[26]==0x30) && (X_Y_Z[27]==0x30) && (X_Y_Z[28]==0x30) && (X_Y_Z[29]==0x30) ))
                                   {
                                       if(status_write == 0)
                                       {
                                           UART_write(uart,X_Y_Z,sizeof(X_Y_Z));
                                           flag_X_Y_Z = 0;
                                           flag_X = 0;
                                           flag_Y = 0;
                                           flag_Z = 0;
                                       }
                                       else if(status_write == 1)
                                       {
                                           readcount--;
                                           n++;
                                       }
                                   }*/
                                }
                            }
                        }
                }
             #if defined ENABLE_PM      // this loop is for the normal other data like TS PF
                else if(!(uartdata[4] == 0x37 && uartdata[5] == 0x31)
                        && !(uartdata[4] == 0x37 && uartdata[5] == 0x32)
                        && !(uartdata[4] == 0x37 && uartdata[5] == 0x33))
              #else
                else if(!(uartdata[4] == 0x31 && uartdata[5] == 0x34)
                        && !(uartdata[4] == 0x31 && uartdata[5] == 0x32)
                        && !(uartdata[4] == 0x31 && uartdata[5] == 0x35)
                        && !(uartdata[4] == 0x35 && uartdata[5] == 0x31)
                        && !(uartdata[4] == 0x35 && uartdata[5] == 0x32)
                        && !(uartdata[4] == 0x35 && uartdata[5] == 0x33))
              #endif
                {
                    for( i=0; i<12; i++ )   // Optimization 16
                       {
                         onuart[i] = BLEADDRESS_ASCII[i]; //BLE ID
                       }

                    for( i=0; i<18; i++ )   // Optimization 17
                       {
                         onuart[i+12] = uartdata[i];
                       }
                       onuart[30] = '\n';


                    if(((!(onuart[12]==0x30 && onuart[13]==0x30))||(!(onuart[14]==0x30 && onuart[15]==0x30)) ) )
                    {
                        if((!(onuart[12]==0x46 && onuart[13]==0x46)) || (!(onuart[14]==0x46 && onuart[15]==0x46)))
                        {
                            if(status_write == 0)
                            {
                                UART_write(uart,onuart,sizeof(onuart));
                            }
                            else if(status_write == 1)
                            {
                                readcount--;
                                n++;
                            }
                        }
                    }
                }

                n--;
        }
        else
        {
            readcount++;
            n--;

            mem_zero_flag++;
            if(mem_zero_flag == 10)
            {
                mem_zero_flag=0;
                restart_erase_flash_all_paramerter();
                while(1);
            }
        }


        if(n==0 && count_stored_buf==1 )
        {
            erase_data_from_flash();
            if(test_name_change==1)
            {
                count_stored_buf=0;
                if(server_comm_first==1)
                {
                    scanRspData[13]=time_hr;
                    scanRspData[14]=time_min;
                    scanRspData[15]=time_sec;
                    scanRspData[16]=input_date;
                    scanRspData[17]=last_date;
                    scanRspData[18]=Mesh_id;
                    //scanRspData[19]=u8_buzz_on_off;
                    //scanRspData[20]=u8_buzz_time;
                    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData, NULL);
                }
                test_name_change=0;
                wait_for_data_send=0;
            }
        }
    }
}

void process_adv_packet(void)
{
    sensor_id_index=11;
    same_dyn_id=0;

    if (compare_string(device_name,moving_device_name,moving_device_length)==1) //coin
    {
        if(adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+2]==coin_buffer[0][0] && adv_data[sensor_id_index+3]==coin_buffer[0][1] && adv_data[sensor_id_index+4]==coin_buffer[0][5] && adv_data[sensor_id_index+5]==coin_buffer[0][6] && adv_data[sensor_id_index+11] == Mesh_id)
        {
                //scanned_dyn_id_16=0;
                scanned_dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_dyn_id_16=scanned_dyn_id[0];
                scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

                //same_dyn_id=0;
//                scanned_dyn_id_16=0;
                scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
                scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
                scanned_coin_info[dev_list_index].RSSI=RSSI;
                scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
                memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
                scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

                if (RSSI >= RSSI_LIMIT)
                    scanned_coin_info[dev_list_index].in_range=TRUE;
                else
                    scanned_coin_info[dev_list_index].in_range=FALSE;

                dev_list_index++;
            }
            else if(adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+4]==coin_buffer[0][4] && adv_data[sensor_id_index+5]==coin_buffer[0][5] && adv_data[sensor_id_index+11]==Mesh_id)
            {
                //scanned_dyn_id_16=0;
                scanned_dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_dyn_id_16=scanned_dyn_id[0];
                scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

                //same_dyn_id=1;
//                scanned_dyn_id_16=0;
                scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
                scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
                scanned_coin_info[dev_list_index].RSSI=RSSI;
                scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
                memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
                scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

                if (RSSI >= RSSI_LIMIT)
                    scanned_coin_info[dev_list_index].in_range=TRUE;
                else
                    scanned_coin_info[dev_list_index].in_range=FALSE;

                dev_list_index++;
            }
            else if(adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8 && adv_data[sensor_id_index+11]==Mesh_id)
            {
                //scanned_dyn_id_16=0;
                scanned_dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_dyn_id_16=scanned_dyn_id[0];
                scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

                if((scanned_dyn_id_16<getset_dynid_16) && (abs(scanned_dyn_id_16-dynamic_id_16)>0))
                {
                    //same_dyn_id=0;
                    scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
                    scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
                    scanned_coin_info[dev_list_index].RSSI=RSSI;
                    scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
                    memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
                    scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
                    scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
                    scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

                    if (RSSI >= RSSI_LIMIT)
                        scanned_coin_info[dev_list_index].in_range=TRUE;
                    else
                        scanned_coin_info[dev_list_index].in_range=FALSE;

                    dev_list_index++;
                }
            }
            else if((adv_data[sensor_id_index]==0x0C && adv_data[sensor_id_index+1]==8) && adv_data[sensor_id_index+11]==Mesh_id)
            {
                //scanned_dyn_id_16=0;
                scanned_dyn_id[0]=adv_data[sensor_id_index+4];
                scanned_dyn_id[1]=adv_data[sensor_id_index+5];
                scanned_dyn_id_16=scanned_dyn_id[0];
                scanned_dyn_id_16=(scanned_dyn_id_16<<8) | scanned_dyn_id[1];

                if((scanned_dyn_id_16<getset_dynid_16) && scanned_dyn_id_16<0x64)
                {
                    //same_dyn_id=0;
                    scanned_coin_info[dev_list_index].static_id[0]=adv_data[sensor_id_index+2];
                    scanned_coin_info[dev_list_index].static_id[1]=adv_data[sensor_id_index+3];
                    scanned_coin_info[dev_list_index].RSSI=RSSI;
                    scanned_coin_info[dev_list_index].DevList_index=dev_counter-1;
                    memcpy(scanned_coin_info[dev_list_index].addr1,curr_addr,GATEWAY_DATA);
                    scanned_coin_info[dev_list_index].dyn_id[0]=adv_data[sensor_id_index+4];
                    scanned_coin_info[dev_list_index].dyn_id[1]=adv_data[sensor_id_index+5];
                    scanned_coin_info[dev_list_index].dyn_id_16=scanned_dyn_id_16;

                    if (RSSI >= RSSI_LIMIT)
                        scanned_coin_info[dev_list_index].in_range=TRUE;
                    else
                        scanned_coin_info[dev_list_index].in_range=FALSE;

                    dev_list_index++;
                }

            }
    }
}

void disconnect_and_update_buffer(uint8 next_index)///disconnects the buffer and updates the contents of the buffer
{
    uint8 s1;

    if(Util_isActive(&ack_clock))
        Util_stopClock(&ack_clock);

    GAPRole_TerminateConnection(connHandleMap[cindex].connHandle);

    memset(ack_packet,0,sizeof(ack_packet));

    if (next_index!=0)
    {
        if(buffer_counter==next_index)
        {
            memset(coin_buffer,0,sizeof(coin_buffer));
            buffer_counter=0;

            if(Util_isActive(&buffer_clock))
                Util_stopClock(&buffer_clock);
        }
        connRole=0;
    }
    memset(scanned_coin_info,0,sizeof(scanned_coin_info));//--------------
    dev_list_index=0;
    dev_counter=0;
    devices_discovered=0;
    memset(connected_device_address,0,sizeof(connected_device_address));
    memset(dest_addr,0,sizeof(dest_addr));
    memset(packet_to_send,0,sizeof(packet_to_send));
    memset(curr_addr,0,sizeof(curr_addr));
    memset(prev_addr,0,sizeof(prev_addr));
    memset(adv_data,0,sizeof(adv_data));

    for(s1=0;s1<DEV_LIST_SIZE;s1++)
    {
        scanned_coin_info[s1].DevList_index=0;
        scanned_coin_info[s1].RSSI=0;
        memset(scanned_coin_info[s1].addr1,0,GATEWAY_DATA);
        memset(scanned_coin_info[s1].dyn_id,0,2);
        memset(scanned_coin_info[s1].static_id,0,2);
        scanned_coin_info[s1].in_range=0;
        scanned_coin_info[s1].dyn_id_16=0;

//        scanned_coin_info[s1].dyn_id=0;
//        scanned_coin_info[s1].in_range=0;
//        scanned_coin_info[s1].static_id=0;
    }

    for (s1=0;s1<DEFAULT_MAX_SCAN_RES;s1++)
    {
        devList[s1].addrType=0;
        devList[s1].eventType=0;
        memset(devList[s1].addr,0,GATEWAY_DATA);
    }
}

void disconnect_after_10_scans(void)
{
    uint8 advt=TRUE,s1;//---------------------------------
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advt, NULL);

    PIN_setOutputValue(hGpioPin, Board_RLED, Board_LED_OFF);

    memset(coin_buffer,0,sizeof(coin_buffer));
    buffer_counter=0;

    if(Util_isActive(&buffer_clock))
        Util_stopClock(&buffer_clock);

    connRole=0;
    scan_count=0;
    memset(scanned_coin_info,0,sizeof(scanned_coin_info));//------------------------------------

    dev_list_index=0;
    dev_counter=0;
    devices_discovered=0;
    memset(connected_device_address,0,sizeof(connected_device_address));
    memset(dest_addr,0,sizeof(dest_addr));
    memset(packet_to_send,0,sizeof(packet_to_send));
    memset(curr_addr,0,sizeof(curr_addr));
    memset(prev_addr,0,sizeof(prev_addr));
    memset(adv_data,0,sizeof(adv_data));

    for(s1=0;s1<DEV_LIST_SIZE;s1++)
    {
        scanned_coin_info[s1].DevList_index=0;
        scanned_coin_info[s1].RSSI=0;
        memset(scanned_coin_info[s1].addr1,0,GATEWAY_DATA);
//        scanned_coin_info[s1].dyn_id=0;
        memset(scanned_coin_info[s1].dyn_id,0,2);
        scanned_coin_info[s1].in_range=0;
//        scanned_coin_info[s1].static_id=0;
        memset(scanned_coin_info[s1].static_id,0,2);
    }

    for (s1=0;s1<DEFAULT_MAX_SCAN_RES;s1++)
    {
        devList[s1].addrType=0;
        devList[s1].eventType=0;
        memset(devList[s1].addr,0,GATEWAY_DATA);
    }
}

uint8 find_dest_addr(void)
{
    uint8_t scenario;
    uint8_t i;
    uint16_t dest_id;
    uint8_t dest_id_index;
    uint8_t devices_in_range=0,devices_out_range=0;
    int8 highest_RSSI = -128;
    int8 lowest_RSSI = -80;

    for (i=0;i<dev_list_index;i++)
    {
         if (scanned_coin_info[i].in_range == TRUE)
             devices_in_range++;
         else
             devices_out_range++;
    }

    if (devices_out_range==0 && devices_in_range != 0 )
        scenario = 1;
    else if (devices_out_range!=0 && devices_in_range == 0 )
        scenario = 2;
    else if (devices_out_range!=0 && devices_in_range != 0)
        scenario = 3;
    else
        scenario = 0xFF;


    dest_id=0x0000;
    if (scenario!=0xFF)
    {
        if(scenario==1 || scenario==3)
        {
            if(same_dyn_id==1)
            {
                for (i=0;i<dev_list_index;i++)
                {
                      if(scanned_coin_info[i].in_range == TRUE && scanned_coin_info[i].RSSI < lowest_RSSI)
                      {
                          dest_id_index = i;
                          dest_id=scanned_coin_info[i].RSSI;
                      }
                }
            }
            else if(same_dyn_id==0)
            {
                for (i=0;i<dev_list_index;i++)
                {
                    if((scanned_coin_info[i].in_range == TRUE) && (scanned_coin_info[i].dyn_id_16 > dest_id))
                    {
                        dest_id_index = i;
                        dest_id=scanned_coin_info[i].dyn_id_16;
                    }
                }
            }
        }
        else if (scenario == 2)
        {
            for (i=0;i<dev_list_index;i++)
            {
                if(scanned_coin_info[i].in_range == FALSE && scanned_coin_info[i].RSSI > highest_RSSI)
                {
                     dest_id_index = i;
                     highest_RSSI=scanned_coin_info[i].RSSI;
                }
            }
        }
        return dest_id_index;
    }
    else
        return 0xFF;
}

uint8 compare_string(uint8 *str1,uint8 *str2,uint8 size)
{
    uint8 i,flag=1;
    for (i=0;i<size;i++)
    {
        if (*str1++ != *str2++)
        {
            flag=0;
            break;
        }
    }
    return flag;
}
/*********************************************************************
 * @fn      multi_role_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
/*
static void multi_role_addDeviceInfo(uint8 *pAddr, uint8 addrType)
{
  uint8 i;

  // If result count not at max
  if ( scanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < scanRes; i++ )
    {
      if (memcmp(pAddr, devList[i].addr, B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN );
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
}
*/

/*********************************************************************
*********************************************************************/
static void multi_role_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
static bool firstRun = TRUE;

switch(pMsg->opcode)
{
  case L2CAP_NUM_CTRL_DATA_PKT_EVT:
  {
    /*
     * We cannot reboot the device immediately after receiving
     * the enable command, we must allow the stack enough time
     * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
     * command. This command will determine the number of
     * packets currently queued up by the LE controller.
     */
    if(firstRun)
    {
      firstRun = false;

      // We only want to set the numPendingMsgs once
      numPendingMsgs = MAX_NUM_PDU - pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

      // Wait until all PDU have been sent on cxn events
      multi_role_UnRegistertToAllConnectionEvent(FOR_OAD_SEND);
      }
      break;
    }
    default:
      break;
}
}

static void multi_role_processOadWriteCB(uint8_t event, uint16_t arg)
{
  Event_post(syncEvent, event);
}
///////
