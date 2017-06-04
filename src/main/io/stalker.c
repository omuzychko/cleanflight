#define STALKER_BAUD_RATE 115200

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"

#include "drivers/dma.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"

#include "sensors/sensors.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/serial.h"
#include "io/stalker.h"

#include "flight/tracking.h"

PG_REGISTER_WITH_RESET_TEMPLATE(stalkerConfig_t, stalkerConfig, PG_STALKER_CONFIG, 0);

PG_RESET_TEMPLATE(stalkerConfig_t, stalkerConfig,
        .target_distance = 2500,
        .target_deadband = 200
);

static uint16_t crc_table [256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
    0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
    0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
    0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
    0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969,
    0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
    0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6,
    0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1,
    0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
    0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447,
    0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
    0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
    0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0,
    0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
    0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

uint16_t CRC16CCITT(uint8_t * buffer, uint32_t length, uint32_t crc)
{ 
    uint8_t * lastAddress = buffer + length;
    uint32_t tableIndex;
    
    if (length == 0) {
        return crc;
    }

    do
    {
        tableIndex = (*buffer++ ^ (crc >> 8)) & 0xff;
        crc = crc_table[tableIndex] ^ (crc << 8);
    } while(buffer < lastAddress);

    return (uint16_t)crc;
} 

typedef enum {
    STALKER_UNKNOWN,
    STALKER_INITIALIZING,
    STALKER_AWAITING_ACK,
    STALKER_RECEIVING_DATA,
    STALKER_COMMUNICATION_ERROR
} stalkerState_e;

typedef enum {
    // OK - means everything is fine
    STALKER_EROK            = 0x00,
    
    // Programmers codding error
    STALKER_ERCODE          = 0x01,
    
    // Unexpected ADC behaviour or incorrect interrupt handling
    STALKER_ERADC           = 0x02,
    
    // Unexpected DMA behaviour or incorrect interrupt handling
    STALKER_ERDMA           = 0x04,
    
    // Error selecting outgoing packet
    STALKER_ERPACKETSELECT  = 0x10,
} stalkerError_t;

// Defines STALKER packet preambulas
typedef enum {
    // Helpers for bit-mask logic
    STALKER_MASK_OUT                = 0x00FAu,
    STALKER_MASK_IN                 = 0x00AFu,
    
    // OUTGOING PACKETS
    STALKER_OUT_TARGET_RAW           = 0x10FAu,  // Target LEDs location seen by each differential sensor (calculated from RMS values of differential sensors)
    STALKER_OUT_TARGET_UAV           = 0x20FAu,  // Target information optimized for UAV navigation
    
    STALKER_OUT_ACK                 = 0x80FAu,  // Acknowledgement on system error or packet receipient
    
    // INCOMING PACKETS
    STALKER_IN_SELECT_PACKET        = 0x01AFu
} stalkerPacketID_t;

typedef struct {
    // ID of the incomming packet which has been succesfully read from UART buffer
    // DEAFULT: 0x0000 - for the case when 
    stalkerPacketID_t receiptPacketID; 
    // CRC of the incomming packet which has been succesfully read from UART buffer
    // DEAFULT: 0x0000
    uint16_t receiptPacketCRC;
    // must be EROK in case of successfull packet processing 
    // has some value if the packet processing resulted in system exception or error
    // if receiptPacketID and receiptPacketCRC are 0x0000 values - the exception occured in the system
    stalkerError_t error;  
} stalkerACK_t;

typedef struct {
    stalkerState_e state;
    uint32_t decodedPacketsCount;
    
    uint8_t             decode_data[64];
    uint8_t             decode_crc[2];
    uint8_t             decode_indexData;
    uint8_t             decode_indexCRC;
    uint16_t            decode_packetLenth;
    stalkerPacketID_t   decode_packetID;
    
    stalkerACK_t        recentACK;
    uint32_t            recentPacketTimestamp;
} stalkerStatus_t;

stalkerTargetRAW_t  STALKER_TARGET_RAW;
stalkerTargetUAV_t  STALKER_TARGET_UAV;

static serialPort_t * stalkerPort;
static stalkerStatus_t stalkerStatus;

static void stalkerSetState(stalkerState_e state){
    stalkerStatus.state = state;
}

void stalkerTurnDown(void) {
    stalkerTargetUAV_t newNavigationTarget;
    
    newNavigationTarget.azimuth  =  0;
    newNavigationTarget.elevation     = -1; // this will should decrease altitude causing the drone to land
    newNavigationTarget.headingAzimuth =  0;
    newNavigationTarget.headingElevation = 0;
    newNavigationTarget.distance           = stalkerConfig() -> target_distance;

    STALKER_TARGET_UAV = newNavigationTarget;
                
    // TODO: scream the beeppppaaah!!!
    // TODO: maybe enable GPS RTH
    
    onStalkerNewData();
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
static void stalkerDataReceive(uint16_t ch)
{
    uint8_t c = ch;
    if (stalkerStatus.decode_packetID == STALKER_MASK_OUT)
    {
        stalkerStatus.decode_packetID |= ((uint16_t)c) << 8;
        
        switch(stalkerStatus.decode_packetID) {
            
            case STALKER_OUT_TARGET_RAW:
                stalkerStatus.decode_packetLenth = sizeof(stalkerTargetRAW_t);
                stalkerStatus.decode_indexData = 0;
                stalkerStatus.decode_indexCRC = 0;
                break;
                
            case STALKER_OUT_TARGET_UAV:
                stalkerStatus.decode_packetLenth = sizeof(stalkerTargetUAV_t);
                stalkerStatus.decode_indexData = 0;
                stalkerStatus.decode_indexCRC = 0;
                break;
                
            case STALKER_OUT_ACK: 
                stalkerStatus.decode_packetLenth = sizeof(stalkerACK_t);
                stalkerStatus.decode_indexData = 0;
                stalkerStatus.decode_indexCRC = 0;
                break;
                
            default:
                // This is not a packet or unknown packet. So we ignore and continue scaning further 
                stalkerStatus.decode_packetID = 0;
                stalkerStatus.decode_packetLenth = 0;
                stalkerStatus.decode_indexCRC = 0;
                break;
        } 
        return;
    }  

    if (stalkerStatus.decode_packetID == 0 && c == STALKER_MASK_OUT)
    {
        // Looks like packet-start detected, go and read next byte right now
        stalkerStatus.decode_packetID = STALKER_MASK_OUT;
        return;
    }

    // If we got hear - we detected known packetID, and in progress of reading input data of required size.

    if (stalkerStatus.decode_indexData < stalkerStatus.decode_packetLenth)
    {
        stalkerStatus.decode_data[stalkerStatus.decode_indexData++] = c;
        return;
    }

    // If we got detected packedID - packetData readout finished already if we got to this point
    // Now compose CRC 
    if (stalkerStatus.decode_packetID) {
        uint16_t crc16;
        stalkerStatus.decode_crc[stalkerStatus.decode_indexCRC++] = c;
              
        if (stalkerStatus.decode_indexCRC < 2) 
            return; // Go for next byte of CRC
        
        // Reset CRC index for next packet calidation
        stalkerStatus.decode_indexCRC = 0;
        
        crc16 = *((uint16_t *) stalkerStatus.decode_crc);

        if (crc16 != CRC16CCITT(stalkerStatus.decode_data, stalkerStatus.decode_packetLenth, 0xFFFF)) {
                stalkerStatus.decode_packetID = 0;
                stalkerStatus.decode_packetLenth = 0;
                // Oops - no luck.
                return;
        }
        
        // we got some, remember timestamp
        // stalkerStatus.recentPacketTimestamp = millis();
        sensorsSet(SENSOR_STALKER);
        
        switch(stalkerStatus.decode_packetID) {
            
            case STALKER_OUT_TARGET_RAW:
                STALKER_TARGET_RAW = *((stalkerTargetRAW_t *) stalkerStatus.decode_data);
                break;
                
            case STALKER_OUT_TARGET_UAV:
                STALKER_TARGET_UAV = *((stalkerTargetUAV_t *) stalkerStatus.decode_data);
                // notify subscribers
                onStalkerNewData();
                break;
                
            case STALKER_OUT_ACK:
                stalkerStatus.recentACK = *((stalkerACK_t *) stalkerStatus.decode_data);
                break;
                
            default: 
                // ignore all other packets
                break;
        }

        // cleanup
        stalkerStatus.decode_packetID = 0;
        stalkerStatus.decode_packetLenth = 0;
    }
}
#pragma GCC diagnostic pop

void stalkerInit(void) {
    memset(&STALKER_TARGET_RAW, 0x00, sizeof(stalkerTargetRAW_t));
    memset(&STALKER_TARGET_UAV, 0x00, sizeof(stalkerTargetUAV_t));

    stalkerSetState(STALKER_UNKNOWN);

    serialPortConfig_t *stalkerPortConfig = findSerialPortConfig(FUNCTION_STALKER);
    if (!stalkerPortConfig) {
        featureClear(FEATURE_STALKER);
        return;
    }
    
    // Callback is NULL, reading data by Stalker Task thread.
    stalkerPort = openSerialPort(stalkerPortConfig->identifier, FUNCTION_STALKER, NULL, STALKER_BAUD_RATE, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!stalkerPort) {
        featureClear(FEATURE_STALKER);
        return;
    }

    stalkerSetState(STALKER_INITIALIZING);
}


static void updateStalkerIndicator(timeUs_t currentTimeUs)
{
    static uint32_t StalkerLEDTime;

    if (STALKER_TARGET_UAV.distance == 0) {
        LED0_OFF;
        StalkerLEDTime = 0;
    } else if ((int32_t)(currentTimeUs - StalkerLEDTime) >= 0) {
        StalkerLEDTime = currentTimeUs + STALKER_TARGET_UAV.distance*100;
        LED0_TOGGLE;
    }
}

void stalkerUpdate(timeUs_t currentTimeUs) {
    
    if (stalkerPort) {
        while (serialRxBytesWaiting(stalkerPort))
            stalkerDataReceive(serialRead(stalkerPort));
    }
        
    switch (stalkerStatus.state) {
        // case STALKER_INITIALIZING:
        //     stalkerStatus.decode_indexData = 0;
        //     stalkerStatus.decode_indexCRC = 0;
        //     stalkerStatus.decode_packetLenth = 0;
        //     stalkerStatus.decode_packetID = 0;
            
        //     stalkerStatus.recentACK.error = STALKER_EROK;
            
        //     // select packets we want to receive
        //     uint16_t packetsToReceive = STALKER_OUT_TARGET_UAV | STALKER_OUT_ACK;
        //     uint16_t crc16 = CRC16CCITT((uint8_t *) &packetsToReceive, 2,  0xFFFF);
        //     uint16_t selectPacketCMD[3] = { STALKER_IN_SELECT_PACKET, packetsToReceive, crc16}; 
        //     uint8_t * commandData = (uint8_t *) selectPacketCMD;
        //     uint8_t i;

        //     stalkerStatus.recentACK.receiptPacketID = 0x0000;
        //     stalkerSetState(STALKER_AWAITING_ACK);
            
        //     for (i = 0; i < 6; i++)
        //         serialWrite(stalkerPort, commandData[i]);
            
        //     break;
            
        // case STALKER_AWAITING_ACK:
        //     if (stalkerStatus.recentACK.receiptPacketID == STALKER_IN_SELECT_PACKET) {
        //         stalkerState_e nextState = 
        //                         (stalkerStatus.recentACK.error == STALKER_EROK) 
        //                         ? STALKER_RECEIVING_DATA 
        //                         : STALKER_COMMUNICATION_ERROR;
        //         stalkerSetState(nextState);
        //     }
        //     break;
        
        // case STALKER_RECEIVING_DATA:
        //     if (stalkerStatus.recentACK.error != STALKER_EROK) {
        //         // reinitialize
        //         stalkerSetState(STALKER_INITIALIZING);
        //         sensorsClear(SENSOR_STALKER);
        //     } else {                
        //         if (millis() - stalkerStatus.recentPacketTimestamp < 50)  {
        //             sensorsSet(SENSOR_STALKER);
        //         }
        //         else {
        //             stalkerSetState(STALKER_COMMUNICATION_ERROR);
        //         }
        //     }
        //     break;
            
        case STALKER_COMMUNICATION_ERROR:
            sensorsClear(SENSOR_STALKER); 
            break;
            
        default:
            updateStalkerIndicator(currentTimeUs);
            break;
    }
}


