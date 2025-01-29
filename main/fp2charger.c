// using some code from https://github.com/neggles/flatpack2s2
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h" // esp_random
#include "driver/twai.h"
#include "nvs.h"
#include <string.h> //memcpy memcmp

// configuration values
#include "app_main.h"
#include "fp2charger.h"

static uint32_t twaiTxTotal = 0; // statistics total transmitted packets
static uint32_t twaiRxTotal = 0; // statistics total received packets
static QueueHandle_t xTwaiTxQueue; // queue for packets to be transmitted over TWAI

// convert charger index (interval 0-15 structure seenChargers[chargerIndex]) to chargerId (interval 1-16)
#define chargerId2Index(chargerId) (((chargerId)-1) & 0x0F)
#define chargerIndex2Id(chargerIndex) ((chargerIndex)+1)

// 05024400 1638500196080000
// 05014400 1927500016391400
// 05XX4400
#define isChargerInStandaloneState(identifier) ((0xFF00FFFF & (identifier)) == 0x05004400)

// 05014004 2300001214d90033
// 05024008 2400001315d80034
// 05XX40XX
#define isStatusMessage(identifier) ((0xFF00FF00 & (identifier)) == 0x05004000)
// 0x04 - normal, 0x08 - warnings, 0x0c - alarms, 0x10 - voltage ramping up
#define FP2_CHARGER_STATUS_NORMAL   0x04
#define FP2_CHARGER_STATUS_WARNINGS 0x08
#define FP2_CHARGER_STATUS_ALARMS   0x0c
#define FP2_CHARGER_STATUS_RAMP     0x10

// 0501bc00 53 08 82 19 27 50 00 16
// 0502bc00 53 08 82 16 38 50 01 96
// 05XXbc00
#define isHardwareInformationMessage(identifier) ((0xFF00FFFF & (identifier)) == 0x0500bc00)

// 05018805 65 0c 82 34 2e 30 31 00
// 05028405 65 08 83 34 30 34 32 36
// 05XX8X05
#define isSoftwareInformationMessage(identifier) ((0xFF00F0FF & (identifier)) == 0x05008005)

volatile fp2Charger_t seenChargers[FP2_MAX_CHARGERS];
#define FP2_MAX_CHARGERS_STRUCT_SIZE sizeof(seenChargers)

extern volatile config_t config;

// Log identifier and payload of TWAI packet.
void logTWAIMessage(const char *TAG, twai_message_t *twaiMsg)
{
    if ( twaiMsg->extd != 1 )
    {
        ESP_LOGI(TAG, "Extended Frame Format (29bit ID) no set!");
    }
    char buf[40]; // ID_12345678_FF_FF_FF_FF_FF_FF_FF_FF_ - 37 max
    char *ptr_buf = buf;
    ptr_buf += sprintf(ptr_buf, "ID %08lx ", twaiMsg->identifier );
    for (int i=0; i < twaiMsg->data_length_code; i++)
    {
        ptr_buf += sprintf(ptr_buf, "%02x ", twaiMsg->data[i] );
    }
    logToFile(TAG, "Unknown packet %s", buf);
    return;
}

// MARK: TWAI requests
// Send assign ID to chargerIndex
// id-050048XX payload-YYYYYYYYYYYY0000 XX-chargerID YY-chargerSerial
static void fp2RequestAssignID(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x05004800 + chargerIndex2Id(chargerIndex) * 4; // Assign charger ID 1-16
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 8;
    memcpy(&twaiMsg.data[0], (const void *)&seenChargers[chargerIndex].serial, twaiMsg.data_length_code);
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY); // post packet to a xTwaiTxQueue for twaiTxTask
    return;
}

// Send voltage and current limits to all chargers. Individual limits command not exist
// id-05FF4004 payload-AAAABBBBCCCCDDDD AAAA-maxCurrent BBBB-maxVoltage CCCC-maxVoltage DDDD-overvoltage protection voltage
// chargerId 0xFF is broadcast address
static void fp2RequestSetLimits(uint16_t current, uint16_t voltage)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x05FF4004;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 8;
    
    twaiMsg.data[0] = current & 0xFF;
    twaiMsg.data[1] = (current >> 8) & 0xFF;
    twaiMsg.data[2] = voltage & 0xFF;
    twaiMsg.data[3] = (voltage >> 8) & 0xFF;
    twaiMsg.data[4] = voltage & 0xFF;
    twaiMsg.data[5] = (voltage >> 8) & 0xFF;
    twaiMsg.data[6] = (voltage + 200) & 0xFF; // overvoltage protection voltage +2V 
    twaiMsg.data[7] = ((voltage + 200) >> 8) & 0xFF;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send warnings request
// id-05XXbc00 payload-08YY00 XX-chargerID YY-0x04 warnings, 0x08 alarms
static void fp2RequestWarnings(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x0500bc00 | (chargerIndex2Id(chargerIndex) << 16);
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x08;
    twaiMsg.data[1] = 0x04;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send alarms request
// id-05XXbc00 payload-08YY00 XX-chargerID YY-0x04 warnings, 0x08 alarms
static void fp2RequestAlarms(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x0500bc00 | (chargerIndex2Id(chargerIndex) << 16) ;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x08;
    twaiMsg.data[1] = 0x08;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send primary software partnumber request
// id-05XX8804 payload-60YY00 XX-chargerID YY-0x08 primary software partnumber, 0x0c - primary software version
static void fp2RequestPrimarySoftwarePartnumber(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x05008804 | (chargerIndex2Id(chargerIndex) << 16);
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x60;
    twaiMsg.data[1] = 0x08;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send secondary software partnumber request
// id-05XX8404 payload-60YY00 XX-chargerID YY-0x08 secondary software partnumber, 0x0c - secondary software version
static void fp2RequestSecondarySoftwarePartnumber(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x05008404 | (chargerIndex2Id(chargerIndex) << 16);
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x60;
    twaiMsg.data[1] = 0x08;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send primary software version request
// id-05XX8804 payload-60YY00 XX-chargerID YY-0x08 primary software partnumber, 0x0c - primary software version
static void fp2RequestPrimarySoftwareVersion(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x05008804 | (chargerIndex2Id(chargerIndex) << 16);
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x60;
    twaiMsg.data[1] = 0x0c;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send secondary software version request
// id-05XX8404 payload-60YY00 XX-chargerID YY-0x08 secondary software partnumber, 0x0c - secondary software version
static void fp2RequestSecondarySoftwareVersion(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x05008404 | (chargerIndex2Id(chargerIndex) << 16) ;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x60;
    twaiMsg.data[1] = 0x0c;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send name request charger name "FLATPACK2 48/2000 HE"
// id-05XXbc00 payload-50YY00 XX-chargerID YY-0x00 name, 0x04 part number, 0x08 serial number, 0x0c hardwareVersion
static void fp2RequestName(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x0500bc00 | (chargerIndex2Id(chargerIndex) << 16) ;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x50;
    twaiMsg.data[1] = 0x00;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send part number request "241115.105"
// id-05XXbc00 payload-50YY00 XX-chargerID YY-0x00 name, 0x04 part number, 0x08 serial number, 0x0c hardwareVersion
static void fp2RequestPartNumber(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x0500bc00 | (chargerIndex2Id(chargerIndex) << 16) ;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x50;
    twaiMsg.data[1] = 0x04;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send serial number request used for check alive
// id-05XXbc00 payload-50YY00 XX-chargerID YY-0x00 name, 0x04 part number, 0x08 serial number, 0x0c hardwareVersion
static void fp2RequestSerialNumber(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x0500bc00 | (chargerIndex2Id(chargerIndex) << 16) ;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x50;
    twaiMsg.data[1] = 0x08;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send hardware version request "9"
// id-05XXbc00 payload-50YY00 XX-chargerID YY-0x00 name, 0x04 part number, 0x08 serial number, 0x0c hardwareVersion
static void fp2RequestHardwareVersion(uint8_t chargerIndex)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x0500bc00 | (chargerIndex2Id(chargerIndex) << 16) ;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 3;
    twaiMsg.data[0] = 0x50;
    twaiMsg.data[1] = 0x0c;
    twaiMsg.data[2] = 0x00;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// Send set default voltage reqest
// id-05XX9c00 payload-291500YYYY XX-chargerID YYYY-0x1815->54V->5400->0x1518
void fp2SetDefaultVoltage(uint8_t chargerIndex, uint16_t voltage)
{
    twai_message_t twaiMsg;

    twaiMsg.identifier = 0x05009c00 | (chargerIndex2Id(chargerIndex) << 16) ;
    twaiMsg.extd = 1;
    twaiMsg.data_length_code = 5;
    twaiMsg.data[0] = 0x29;
    twaiMsg.data[1] = 0x15;
    twaiMsg.data[2] = 0x00;
    twaiMsg.data[3] = voltage & 0xFF; 
    twaiMsg.data[4] = (voltage >> 8) & 0xFF;
    xQueueSendToBack(xTwaiTxQueue, &twaiMsg, portMAX_DELAY);
    return;
}

// MARK: HW process
/* process reply from charger with hardware information, status, warnings, and alarms
information messages:
0501bc00 53 00 86 46 4c 41 54 50 "FLATP" -- request Name 0501bc00 50 00 00
0501bc00 53 00 05 41 43 4b 32 20 "ACK2 "
0501bc00 53 00 04 34 38 2f 32 30 "48/20"
0501bc00 53 00 03 30 30 20 48 45 "00 HE"
0501bc00 53 00 02 00 00 00 00 00
0501bc00 53 00 01 00 00
0501bc00 53 04 83 32 34 31 31 31 "24111" -- request Part Number 0501bc00 50 04 00
0501bc00 53 04 02 35 2e 31 30 35 "5.105"
0501bc00 53 04 01 00 00
0501bc00 53 08 82 19 27 50 00 16 "1927500016" -- request serial 0501bc00 50 08 00
0501bc00 53 08 01 39             "39"
0501bc00 53 0c 82 39 00 00 00 00 "9" -- request hardwareVersion 0501bc00 50 0c 00
0501bc00 53 0c 01 00
         ^^ ^^ ^^ 86 - first packet of total six, 05 - second packet, ... 01 - last packet
         ^^ ^^reply to request
         ^^always 53
warnings and alarms
0501bc00 0e 04 00 80 00 00 00
0501bc00 0e 08 00 14 00 00 00
         ^^ ^^    ^^ bits of active warnings/alarms
         ^^ ^^reply to request 0x04-warnings 0x04-alarms
         ^^always 0x0e
*/
static void fp2ProcessHardwareInformationMessages(twai_message_t *twaiMsg)
{
    const char *TAG = __func__;
    uint8_t chargerIndex = chargerId2Index((uint8_t)(twaiMsg->identifier >> 16));
    // process warnings and alarms
    if (twaiMsg->data[0] == 0x0e)
    {
        switch (twaiMsg->data[1])
        {
        case 0x04: // warnings
            uint8_t warnings = twaiMsg->data[3];
            if (seenChargers[chargerIndex].warnings != warnings)
            {
                logToFile(TAG, "Charger ID: %d warnings changed from %02x to %02x Tin=%dC Vac=%dV Vdc=%dV Idc=%dA Tout=%dC.",
                    chargerIndex2Id(chargerIndex),
                    seenChargers[chargerIndex].warnings,
                    warnings,
                    seenChargers[chargerIndex].inletTemperature,
                    seenChargers[chargerIndex].acVoltage,
                    seenChargers[chargerIndex].dcVoltage,
                    seenChargers[chargerIndex].dcCurrent,
                    seenChargers[chargerIndex].outletTemperature);
                seenChargers[chargerIndex].warnings = warnings;
            }
            break;
        case 0x08: // alarms
            uint8_t alarms = twaiMsg->data[3];
            if (seenChargers[chargerIndex].alarms != alarms)
            {
                logToFile(TAG, "Charger ID: %d alarms changed from %02x to %02x Tin=%dC Vac=%dV Vdc=%dV Idc=%dA Tout=%dC.",
                    chargerIndex2Id(chargerIndex),
                    seenChargers[chargerIndex].alarms,
                    alarms,
                    seenChargers[chargerIndex].inletTemperature,
                    seenChargers[chargerIndex].acVoltage,
                    seenChargers[chargerIndex].dcVoltage,
                    seenChargers[chargerIndex].dcCurrent,
                    seenChargers[chargerIndex].outletTemperature);
                seenChargers[chargerIndex].alarms = alarms;
            }
            break;
        }
        return; // warnings and alarms message processed
    }

    // process information messages
    switch (twaiMsg->data[1])
    {
    case 0x00: // information.name
                switch (twaiMsg->data[2])
                {
                case 0x86: // fist part of six, just save information
                    seenChargers[chargerIndex].information.name[0] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.name[1] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.name[2] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.name[3] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.name[4] = twaiMsg->data[7];
                    break;
                case 0x05: // second part of six
                    seenChargers[chargerIndex].information.name[5] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.name[6] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.name[7] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.name[8] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.name[9] = twaiMsg->data[7];
                    break;
                case 0x04: // third part of six
                    seenChargers[chargerIndex].information.name[10] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.name[11] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.name[12] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.name[13] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.name[14] = twaiMsg->data[7];
                    break;
                case 0x03: // forth part of six
                    seenChargers[chargerIndex].information.name[15] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.name[16] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.name[17] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.name[18] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.name[19] = twaiMsg->data[7];
                    break;
                case 0x02: // fifth part of six
                    seenChargers[chargerIndex].information.name[20] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.name[21] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.name[22] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.name[23] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.name[24] = twaiMsg->data[7];
                    break;
                case 0x01: // last part
                    seenChargers[chargerIndex].information.name[25] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.name[26] = twaiMsg->data[4];
                    logToFile(TAG, "Charger ID: %d name: %s", chargerIndex2Id(chargerIndex), seenChargers[chargerIndex].information.name);
                    break;
                }
                break;
    case 0x04: // information.partNumber
                switch (twaiMsg->data[2])
                {
                case 0x83: // fist part of three, just save information
                    seenChargers[chargerIndex].information.partNumber[0] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.partNumber[1] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.partNumber[2] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.partNumber[3] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.partNumber[4] = twaiMsg->data[7];
                    break;
                case 0x02: // second part of three
                    seenChargers[chargerIndex].information.partNumber[5] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.partNumber[6] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.partNumber[7] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.partNumber[8] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.partNumber[9] = twaiMsg->data[7];
                    break;
                case 0x01: // last part
                    seenChargers[chargerIndex].information.partNumber[10] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.partNumber[11] = twaiMsg->data[4];
                    logToFile(TAG, "Charger ID: %d part number: %s", chargerIndex2Id(chargerIndex),
                        seenChargers[chargerIndex].information.partNumber);
                    break;
                }
                break;
    case 0x08: // information.serial
                switch (twaiMsg->data[2])
                {
                case 0x82: // fist part of two, just save information
                    seenChargers[chargerIndex].information.serial[0] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.serial[1] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.serial[2] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.serial[3] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.serial[4] = twaiMsg->data[7];
                    break;
                case 0x01: // last part, save and check received information update availability if check ok
                    seenChargers[chargerIndex].information.serial[5] = twaiMsg->data[3];
                    if (memcmp((const void *)&seenChargers[chargerIndex].serial, (const void *)&seenChargers[chargerIndex].information.serial,
                        sizeof(seenChargers[chargerIndex].information.serial)) == 0)
                    {
                        // stored charger serial is the same as received information.serial
                        seenChargers[chargerIndex].available = MAX_CHARGER_AVAILABILITY;
                        statusLedSet(STATUS_LED_CHARGER_AVAIL);
                    } else {
                        logToFile(TAG, "Information reply received but check failed ID: %d SN: %llx != %02x%02x%02x%02x%02x%02x.",
                            chargerIndex2Id(chargerIndex), chargerSerial2HumanReadable(seenChargers[chargerIndex].serial),
                            seenChargers[chargerIndex].information.serial[0],
                            seenChargers[chargerIndex].information.serial[1],
                            seenChargers[chargerIndex].information.serial[2],
                            seenChargers[chargerIndex].information.serial[3],
                            seenChargers[chargerIndex].information.serial[4],
                            seenChargers[chargerIndex].information.serial[5]);
                    }
                    break;
            }
            break;
    case 0x0c: // information.hardwareVersion
            switch (twaiMsg->data[2])
            {
                case 0x82: // fist part of two, just save information
                    seenChargers[chargerIndex].information.hardwareVersion[0] = twaiMsg->data[3];
                    seenChargers[chargerIndex].information.hardwareVersion[1] = twaiMsg->data[4];
                    seenChargers[chargerIndex].information.hardwareVersion[2] = twaiMsg->data[5];
                    seenChargers[chargerIndex].information.hardwareVersion[3] = twaiMsg->data[6];
                    seenChargers[chargerIndex].information.hardwareVersion[4] = twaiMsg->data[7];
                    break;
                case 0x01: // last part
                    seenChargers[chargerIndex].information.hardwareVersion[5] = twaiMsg->data[3];
                    logToFile(TAG, "Charger ID: %d hardware version: %s", chargerIndex2Id(chargerIndex),
                        seenChargers[chargerIndex].information.hardwareVersion);
                    break;
            }
            break;
    }
    return;
}

// MARK: SW process
/* reply to issued software requests 
05018805 65 08 83 34 30 34 32 36 40426 -- 05018804 60 08 00 - SW Part # Primary "404266.009"
05018805 65 08 02 35 2e 30 30 39 5.009
05018805 65 08 01 00 00
05018805 65 0c 82 33 2e 30 30 00 3.00  -- 05018804 60 0c 00 - SW Rev. Primary  "3.00"
05018805 65 0c 01 00
05018405 65 08 83 34 30 34 32 36 40426 -- 05018404 60 08 00 - SW Part # Secondary  "404266.009"
05018405 65 08 02 36 2e 30 30 39 6.009
05018405 65 08 01 00 00
05018405 65 0c 82 34 2e 30 30 00 4.00  -- 05018404 60 0c 00 - SW Rev. Secondary  "4.00"
05018405 65 0c 01 00
    ^^   ^^ ^^ ^^ 83 - first packet of total three, 02 - second packet, 01 - last packet
    ^^   ^^ ^^reply to request
    ^^   ^^always 65
    ^^88 - primary 84 - secondary
*/
static void fp2ProcessSoftwareInformationMessages(twai_message_t *twaiMsg)
{
    const char *TAG = __func__;
    uint8_t chargerIndex = chargerId2Index((uint8_t)(twaiMsg->identifier >> 16));
    uint8_t primaryOrSecondary = (uint8_t)(twaiMsg->identifier >> 8);
    if (primaryOrSecondary == 0x88) // primary
    {
        switch (twaiMsg->data[1])
        {
        case 0x08: // information.primarySoftwarePartnumber
                    switch (twaiMsg->data[2])
                    {
                    case 0x83: // fist part of three, just save information
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[0] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[1] = twaiMsg->data[4];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[2] = twaiMsg->data[5];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[3] = twaiMsg->data[6];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[4] = twaiMsg->data[7];
                        break;
                    case 0x02: // second part of three
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[5] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[6] = twaiMsg->data[4];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[7] = twaiMsg->data[5];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[8] = twaiMsg->data[6];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[9] = twaiMsg->data[7];
                        break;
                    case 0x01: // last part
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[10] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.primarySoftwarePartnumber[11] = twaiMsg->data[4];
                        logToFile(TAG, "Charger ID: %d primary software partnumber: %s", chargerIndex2Id(chargerIndex),
                            seenChargers[chargerIndex].information.primarySoftwarePartnumber);
                        break;
                    }
                    break;
        case 0x0c: // information.primarySoftwareVersion
                    switch (twaiMsg->data[2])
                    {
                    case 0x82: // fist part of two, just save information
                        seenChargers[chargerIndex].information.primarySoftwareVersion[0] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.primarySoftwareVersion[1] = twaiMsg->data[4];
                        seenChargers[chargerIndex].information.primarySoftwareVersion[2] = twaiMsg->data[5];
                        seenChargers[chargerIndex].information.primarySoftwareVersion[3] = twaiMsg->data[6];
                        seenChargers[chargerIndex].information.primarySoftwareVersion[4] = twaiMsg->data[7];
                        break;
                    case 0x01: // last part
                        seenChargers[chargerIndex].information.primarySoftwareVersion[5] = twaiMsg->data[3];
                        logToFile(TAG, "Charger ID: %d primary software version: %s", chargerIndex2Id(chargerIndex),
                            seenChargers[chargerIndex].information.primarySoftwareVersion);
                        break;
                    }
                    break;
        }
    }

    if (primaryOrSecondary == 0x84) // secondary
    {
        switch (twaiMsg->data[1])
        {
            case 0x08: // information.secondarySoftwarePartnumber
                switch (twaiMsg->data[2])
                {
                    case 0x83: // fist part of three, just save information
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[0] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[1] = twaiMsg->data[4];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[2] = twaiMsg->data[5];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[3] = twaiMsg->data[6];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[4] = twaiMsg->data[7];
                        break;
                    case 0x02: // second part of three
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[5] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[6] = twaiMsg->data[4];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[7] = twaiMsg->data[5];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[8] = twaiMsg->data[6];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[9] = twaiMsg->data[7];
                        break;
                    case 0x01: // last part
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[10] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.secondarySoftwarePartnumber[11] = twaiMsg->data[4];
                        logToFile(TAG, "Charger ID: %d secondary software partnumber: %s", chargerIndex2Id(chargerIndex),
                            seenChargers[chargerIndex].information.secondarySoftwarePartnumber);
                        break;
                }
                break;
            case 0x0c: // information.secondarySoftwareVersion
                switch (twaiMsg->data[2])
                {
                    case 0x82: // fist part of two, just save information
                        seenChargers[chargerIndex].information.secondarySoftwareVersion[0] = twaiMsg->data[3];
                        seenChargers[chargerIndex].information.secondarySoftwareVersion[1] = twaiMsg->data[4];
                        seenChargers[chargerIndex].information.secondarySoftwareVersion[2] = twaiMsg->data[5];
                        seenChargers[chargerIndex].information.secondarySoftwareVersion[3] = twaiMsg->data[6];
                        seenChargers[chargerIndex].information.secondarySoftwareVersion[4] = twaiMsg->data[7];
                        break;
                    case 0x01: // last part
                        seenChargers[chargerIndex].information.secondarySoftwareVersion[5] = twaiMsg->data[3];
                        logToFile(TAG, "Charger ID: %d secondary software version: %s", chargerIndex2Id(chargerIndex),
                            seenChargers[chargerIndex].information.secondarySoftwareVersion);
                        break;
                }
                break;
        }
    }

    return;
}

// MARK: CheckAlive
// Charger stays in active state for 64*0.2s = 12,8s without master requests.
// Every 6s assignID and request SN from available chargers. If CAN TX error happens second assignID also in active state interval.
// if charger not respond in MAX_CHARGER_AVAILABILITY time assume it not available and disconnected
// Any request (requestSN, assignID, errors, warnings, setLimits...) would extend active state to 64*0.2s = 12,8s
// If available chargers present then issue broadcast with set limits request.
static void fp2CheckAliveAvailableChargers(void *pvParameters)
{
    const char *TAG = __func__;
    vTaskDelay( pdMS_TO_TICKS(30000) ); // 30s assume all chargers have enought time to enter standalone state
    ESP_LOGI(TAG, "Task start.");

    while (true)
    {
        vTaskDelay( pdMS_TO_TICKS(6000) );
        uint8_t chargersAvailable = 0;
        for (int i=0; i<FP2_MAX_CHARGERS; i++)
        {
            if ( seenChargers[i].available > 0 )
            {
                fp2RequestAssignID(i);
                seenChargers[i].available--; // reduce availability, tracking for removed chargers
                chargersAvailable++;
                fp2RequestSerialNumber(i); // charger serial number request for extend availability
            }
            if (seenChargers[i].available == 0 && seenChargers[i].serial != 0)
            {
                logToFile(TAG, "Charger ID: %d disconnected. SN: %llx.",
                    chargerIndex2Id(i),
                    chargerSerial2HumanReadable(seenChargers[i].serial));
                seenChargers[i].serial = 0; // charger disconnected
            }
        }
        if (chargersAvailable > 0)
            if (config.limitsActive == 1)
                fp2RequestSetLimits(config.dcCurrentSet, config.dcVoltageSet);
    }
    vTaskDelete(NULL); // Task functions should never return.
}

// MARK: status process
static uint8_t skipCounter = 0;
// Process TWAI packet with charger status
// id-05XX40YY payload-AABBBBCCCCDDDDEE XX-chargerId YY-chargerStatus 0x04 - normal, 0x08 - warnings, 0x0c - alarms, 0x10 - voltage ramping up
// AA-intelTemperature, BBBB-dcCurrent, CCCC-dcVoltage, DDDD-acVoltage, EE-outletTemperature
static void fp2ProcessStatusMessages(twai_message_t *twaiMsg)
{
    const char *TAG = __func__;

    if ( (uint8_t)(twaiMsg->identifier >> 16) > 16 ) return; // chargerId not in range 1-16, do not process

    uint8_t chargerIndex = chargerId2Index((uint8_t)(twaiMsg->identifier >> 16));

    // charger ID unknown, do not process. Other controllers may present.
    if (seenChargers[chargerIndex].serial == 0 )
    {
        return;
    }

    // charger in standalone state and waits for fp2RequestAssignID
    if (seenChargers[chargerIndex].available == CHARGER_FIRST_TIME_SEEN ) return;

    uint8_t chargerStatus = (uint8_t)(twaiMsg->identifier);
    seenChargers[chargerIndex].inletTemperature = twaiMsg->data[0];
    seenChargers[chargerIndex].dcCurrent = twaiMsg->data[1] | (twaiMsg->data[2] << 8);
    seenChargers[chargerIndex].dcVoltage = twaiMsg->data[3] | (twaiMsg->data[4] << 8);
    seenChargers[chargerIndex].acVoltage = twaiMsg->data[5] | (twaiMsg->data[6] << 8);
    seenChargers[chargerIndex].outletTemperature = twaiMsg->data[7];
    if (seenChargers[chargerIndex].status != chargerStatus)
    {
        logToFile(TAG, "Charger ID: %d status changed from %d to %d Tin=%dC Vac=%dV Vdc=%dV Idc=%dA Tout=%dC.",
            chargerIndex2Id(chargerIndex),
            seenChargers[chargerIndex].status,
            chargerStatus,
            seenChargers[chargerIndex].inletTemperature,
            seenChargers[chargerIndex].acVoltage,
            seenChargers[chargerIndex].dcVoltage,
            seenChargers[chargerIndex].dcCurrent,
            seenChargers[chargerIndex].outletTemperature);
        seenChargers[chargerIndex].status = chargerStatus;
        fp2RequestWarnings(chargerIndex);
        fp2RequestAlarms(chargerIndex);
    }

    // charge is complete when status set to normal, i.e. charger switched from CC to CV mode
    // DCvoltage > config.dcVoltageSet - 0.5V, dcVoltage is close enough to battery full
    // DCcurrent in range config.dcCurrentSet * 40% - config.dcCurrentSet * 60%, 
    if (seenChargers[chargerIndex].status == FP2_CHARGER_STATUS_NORMAL )
    {
        if (seenChargers[chargerIndex].dcVoltage > config.dcVoltageSet - 50)
        {
            if ( seenChargers[chargerIndex].dcCurrent < config.dcCurrentSet * 6 / 10 &&
                 seenChargers[chargerIndex].dcCurrent > config.dcCurrentSet * 4 / 10 )
            {
                if (RELAY_GPIO_GET_LEVEL_DIRECT == 1)
                {
                    gpio_set_level(RELAY_GPIO, 0); // turn off relay
                    logToFile(TAG, "Turn off charger AC mains.");
                    logToFile(TAG, "Charger serial: %llx Tin=%dC Vac=%dV Vdc=%dV Idc=%dA Tout=%dC.",
                        chargerSerial2HumanReadable(seenChargers[chargerIndex].serial),
                        seenChargers[chargerIndex].inletTemperature,
                        seenChargers[chargerIndex].acVoltage,
                        seenChargers[chargerIndex].dcVoltage,
                        seenChargers[chargerIndex].dcCurrent,
                        seenChargers[chargerIndex].outletTemperature);
                }
            }
        }
    }

    // Status messages issued by charger every 0.2s or so, lets request warnings and alarms once every 10 status messages
    if (skipCounter++ > 10)
    {
        fp2RequestWarnings(chargerIndex);
        fp2RequestAlarms(chargerIndex);
        skipCounter = 0;
    }
    return;
}

// Task. Spawn task to request charger information, pvParameters points to uint8_t chargerIndex
static void fp2RequestChargerInformation(void *pvParameters)
{
    const char *TAG = __func__;
    uint8_t chargerIndex = *(uint8_t *)pvParameters;
    vTaskDelay( pdMS_TO_TICKS(60000) );
    ESP_LOGI(TAG, "Task start.");

    vTaskDelay( pdMS_TO_TICKS(200) ); fp2RequestName(chargerIndex);
    vTaskDelay( pdMS_TO_TICKS(200) ); fp2RequestPartNumber(chargerIndex);
    vTaskDelay( pdMS_TO_TICKS(200) ); fp2RequestHardwareVersion(chargerIndex);

    vTaskDelay( pdMS_TO_TICKS(200) ); fp2RequestPrimarySoftwarePartnumber(chargerIndex);
    vTaskDelay( pdMS_TO_TICKS(200) ); fp2RequestPrimarySoftwareVersion(chargerIndex);

    vTaskDelay( pdMS_TO_TICKS(200) ); fp2RequestSecondarySoftwarePartnumber(chargerIndex);
    vTaskDelay( pdMS_TO_TICKS(200) ); fp2RequestSecondarySoftwareVersion(chargerIndex);

    vTaskDelete(NULL); // Task functions should never return.
}

// Process TWAI packet with 'charger in standalone state and searching master'
static void fp2ProcessChargerStandaloneState(twai_message_t *twaiMsg)
{
    const char *TAG = __func__;
    uint64_t serial;
    memcpy(&serial, &twaiMsg->data[0], sizeof(serial));

    for (int i=0; i<FP2_MAX_CHARGERS; i++)
    {
        if (seenChargers[i].serial == serial)
        {
            // already in seenChargers
            return;
        }
    }

    // append new charger to a free slot
    for (int i=0; i<FP2_MAX_CHARGERS; i++)
    {
        if (seenChargers[i].serial == 0)
        {
            seenChargers[i].serial = serial;
            seenChargers[i].available = CHARGER_FIRST_TIME_SEEN;
            logToFile(TAG, "New charger detected. SN: %llx. Assigned ID: %d.", chargerSerial2HumanReadable(serial), chargerIndex2Id(i));
            // Spawn task to request information from new charger
            uint8_t chargerIndex = (uint8_t)i;
            xTaskCreate(&fp2RequestChargerInformation, "fp2RequestChargerInformation", 1024 * 3, &chargerIndex, 8, NULL);
            return;
        }
    }
    return;
};

// MARK: TWAI process
static void processTwaiMsg(twai_message_t *twaiMsg)
{
    const char *TAG = __func__;
    /* do not process hello packets
       05001639 1d19275000163900
       05001608 1c16385001960800
           ^^^^           ^^^^ - two bytes of payload & 0x3fff should be same as two bytes in identifier
    */
    if ( twaiMsg->identifier == (0x05000000 | (((twaiMsg->data[5] << 8) | twaiMsg->data[6]) & 0x3fff) ) )
        return;

    if ( isChargerInStandaloneState(twaiMsg->identifier) )
    {
        fp2ProcessChargerStandaloneState(twaiMsg);
        return;
    }

    if ( isStatusMessage(twaiMsg->identifier) )
    {
        fp2ProcessStatusMessages(twaiMsg);
        return;
    }

    if ( isHardwareInformationMessage(twaiMsg->identifier) )
    {
        fp2ProcessHardwareInformationMessages(twaiMsg);
        return;
    }

    if ( isSoftwareInformationMessage(twaiMsg->identifier) )
    {
        fp2ProcessSoftwareInformationMessages(twaiMsg);
        return;
    }

    // if packet still not processed log it!
    logTWAIMessage(TAG, twaiMsg);
    return;
}

// MARK: TWAI TX/RX
// Task. Receives TWAI message from queue xTwaiTxQueue and then send it to a charger
#define TWAI_TX_TIMEOUT_SEC 1
#define TWAI_TX_RETRIES     3
static void twaiTxTask(void *pvParameters)
{
    const char *TAG = __func__;
    ESP_LOGI(TAG, "Task start.");
    twai_message_t txMsg = {0};

    while (true)
    {
        xQueueReceive(xTwaiTxQueue, &txMsg, portMAX_DELAY);
        for (int i = 0; i < TWAI_TX_RETRIES; i++)
        {
            esp_err_t txErr = twai_transmit(&txMsg, pdMS_TO_TICKS(TWAI_TX_TIMEOUT_SEC * 1000));
            if (txErr == ESP_OK)
            {
//                logTWAIMessage(TAG, &txMsg);
                twaiTxTotal++; // statistic
                vTaskDelay(portTICK_PERIOD_MS); // to prevent packet loss
                break;
            }
        }
    }
    vTaskDelete(NULL); // Task functions should never return.
}

// Task. Receives TWAI packets and process them
static void twaiRxTask(void *pvParameters)
{
    const char *TAG = __func__;
    ESP_LOGI(TAG, "Task start.");
    twai_message_t rxMsg = {0};
    esp_err_t rxErr;

    while (true)
    {
        rxErr = twai_receive(&rxMsg, portMAX_DELAY);
        if (rxErr == ESP_OK)
        {
            twaiRxTotal++; // statistic
            processTwaiMsg(&rxMsg);
        } else {
            ESP_LOGE(TAG, "twai_receive error! %s", esp_err_to_name(rxErr));
        }
    }
    vTaskDelete(NULL); // Task functions should never return.
}

// MARK: TWAI task
// TWAI general config for ISR mode
#define TWAI_GENERAL_CONFIG_IRAM(tx_io_num, rx_io_num, op_mode) { \
        .controller_id = 0, .mode = op_mode, .tx_io = tx_io_num, .rx_io = rx_io_num, .clkout_io = TWAI_IO_UNUSED, \
        .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 5, .rx_queue_len = 5, .alerts_enabled = TWAI_ALERT_NONE, \
        .clkout_divider = 0, .intr_flags = ESP_INTR_FLAG_IRAM \
    }
// TWAI control task. Starts packet processing tasks and login request task
void twaiCtrlTask( void *pvParameters )
{
    const char *TAG = __func__;
    ESP_LOGI(TAG, "Task start.");

    // fill struct holds seen chargers with initial values
    for (int i=0; i<FP2_MAX_CHARGERS; i++)
    {
        seenChargers[i].available = 0;
        seenChargers[i].serial = 0;
    }

    // TWAI driver configuration IRAM mode
    twai_general_config_t twai_g_conf = TWAI_GENERAL_CONFIG_IRAM(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    // 125kbps
    const twai_timing_config_t twai_t_conf = TWAI_TIMING_CONFIG_125KBITS();
    // TWAI message filter. Accept only charger messages 0x05XXXXXX
    // "<<3" found here -> https://stackoverflow.com/questions/73638149/how-to-setup-my-canbus-message-filterering-acceptance-code-and-mask
    const twai_filter_config_t twai_f_conf = { .acceptance_code = 0x05000000<<3, .acceptance_mask = 0x00FFFFFF<<3, .single_filter = true };

    // Install TWAI driver
    ESP_ERROR_CHECK_WITHOUT_ABORT(twai_driver_install(&twai_g_conf, &twai_t_conf, &twai_f_conf));
    // Start TWAI
    esp_err_t err = twai_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Driver failed to start! Terminating...");
        vTaskDelete(NULL);
    };

    // enabled TWAI alerts
    const uint32_t twai_a_conf = (TWAI_ALERT_ERR_ACTIVE | TWAI_ALERT_ARB_LOST | TWAI_ALERT_RX_QUEUE_FULL |
                        TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF);
    twai_reconfigure_alerts(twai_a_conf, NULL);

    // create TWAI RX task
    TaskHandle_t twaiRxTaskHandle;
    xTaskCreate(&twaiRxTask, "twaiRxTask", 1024 * 4, NULL, tskIDLE_PRIORITY + 4, &twaiRxTaskHandle);

    // create TWAI TX task
    TaskHandle_t twaiTxTaskHandle;
    xTwaiTxQueue = xQueueCreate(10, sizeof(twai_message_t));
    xTaskCreate(&twaiTxTask, "twaiTxTask", 1024 * 4, NULL, tskIDLE_PRIORITY + 5, &twaiTxTaskHandle);

    // create task to send login command to all available chargers
    TaskHandle_t fp2CheckAliveAvailableChargersHandle;
    xTaskCreate(&fp2CheckAliveAvailableChargers, "fp2CheckAliveAvailableChargers", 1024 * 3, NULL, tskIDLE_PRIORITY + 5, &fp2CheckAliveAvailableChargersHandle);

    // handle TWAI alerts and print status
    while (true)
    {
        esp_err_t err;
        uint32_t alerts;
        err = twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAI_CTL_TIMEOUT_SEC * 1000));
        if (err == ESP_OK)
        {
            // we have some alerts
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
            {
                ESP_LOGW(TAG, "TWAI RX queue is full.");
            }
            if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
            {
                ESP_LOGW(TAG, "TWAI error! Warning limit exceeded.");
            }
            if (alerts & TWAI_ALERT_ERR_PASS)
            {
                ESP_LOGW(TAG, "TWAI error! Passive state.");
            }
            if (alerts & TWAI_ALERT_ERR_ACTIVE)
            {
                ESP_LOGW(TAG, "TWAI error! Active state.");
            }
            if (alerts & TWAI_ALERT_BUS_OFF)
            {
                ESP_LOGE(TAG, "TWAI bus-off! Suspending tasks, initiating TWAI recovery.");
                vTaskSuspend(twaiTxTaskHandle);
                vTaskSuspend(twaiRxTaskHandle);
                vTaskSuspend(fp2CheckAliveAvailableChargersHandle);
                twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
                twai_initiate_recovery();
                continue;
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED)
            {
                ESP_LOGE(TAG, "TWAI bus recovered, restarting driver.");
                for (int i = 0; i < 3; i++)
                {
                    twai_reconfigure_alerts(twai_a_conf, NULL);
                    if (twai_start() == ESP_OK)
                    {
                        ESP_LOGE(TAG, "TWAI recovery complete. Resuming tasks.");
                        vTaskResume(twaiTxTaskHandle);
                        vTaskResume(twaiRxTaskHandle);
                        vTaskResume(fp2CheckAliveAvailableChargersHandle);
                        break;
                    } else {
                        ESP_LOGE(TAG, "TWAI recovery restart attempt %d failed, retrying in 3s...", i + 1);
                        vTaskDelay(pdMS_TO_TICKS(3000));
                    }
                }
            }
        } else if (err != ESP_ERR_TIMEOUT) {
            // something else has gone wrong twai_read_alerts
            const char *alert_err_string = esp_err_to_name(err);
            ESP_LOGE(TAG, "Retrieving TWAI alerts failed! %s", alert_err_string);
        }
        // log status
        twai_status_info_t status;
        twai_get_status_info(&status);
        ESP_LOGI(TAG, "TWAI state=%d arblost=%ld buserr=%ld | TX=%ld q=%ld err=%ld fail=%ld | RX=%ld q=%ld err=%ld miss=%ld",
            (int)status.state, status.arb_lost_count, status.bus_error_count, 
            twaiTxTotal, status.msgs_to_tx, status.tx_error_counter, status.tx_failed_count, 
            twaiRxTotal, status.msgs_to_rx, status.rx_error_counter, status.rx_missed_count);
        ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes.", esp_get_minimum_free_heap_size());
/*
        ESP_LOGI(TAG, "twaiCtrlTask min available stack size: %d", uxTaskGetStackHighWaterMark(NULL) );
        ESP_LOGI(TAG, "twaiTxTaskHandle min available stack size: %d", uxTaskGetStackHighWaterMark(twaiTxTaskHandle) );
        ESP_LOGI(TAG, "twaiRxTaskHandle min available stack size: %d", uxTaskGetStackHighWaterMark(twaiRxTaskHandle) );
        ESP_LOGI(TAG, "fp2CheckAliveAvailableChargersHandle min available stack size: %d", uxTaskGetStackHighWaterMark(fp2CheckAliveAvailableChargersHandle) );
*/
    }
    vTaskDelete(NULL); // Task functions should never return.
}
