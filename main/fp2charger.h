
#include "sys/time.h"

#pragma pack(1)
typedef struct {
    uint64_t serial;    // Charger serial number, received from login inquiry packet
    uint8_t  available; // decreasing every 10s by fp2CheckAliveAvailableChargers. If == 0 struct is empty or charger removed
    uint8_t  status;    // FP2_CHARGER_STATUS_*
    uint8_t  warnings;  // bits of active warnings
    uint8_t  alarms;    // bits of active alarms
    uint8_t  inletTemperature;
    uint16_t dcCurrent; // RealValue = dcCurrent / 10
    uint16_t dcVoltage; // RealValue = dcCurrent / 100
    uint16_t acVoltage; // RealValue = acVoltage
    uint8_t  outletTemperature;
    struct {
        uint8_t name[27];           // reply from fp2RequestName()
        uint8_t partNumber[12];     // fp2RequestPartNumber()
        uint8_t serial[6];          // fp2RequestSerialNumber()
        uint8_t hardwareVersion[6]; // fp2RequestHardwareVersion()
        uint8_t primarySoftwarePartnumber[12]; // fp2RequestPrimarySoftwarePartnumber()
        uint8_t primarySoftwareVersion[6];     // fp2RequestPrimarySoftwareVersion()
        uint8_t secondarySoftwarePartnumber[12]; // fp2RequestSecondarySoftwarePartnumber()
        uint8_t secondarySoftwareVersion[6];     // fp2RequestSecondarySoftwareVersion()
    } information;
} fp2Charger_t;

typedef struct {
    uint32_t time;      // unix time
    uint8_t  inletTemperature;
    uint16_t dcCurrent; // RealValue = dcCurrent / 10
    uint16_t dcVoltage; // RealValue = dcCurrent / 100
    uint8_t  outletTemperature;
} fp2ChargerStatus_t;

#define FP2_MAX_CHARGERS 16 // do not edit

// TWAI transcever configuration
#define TWAI_TX_GPIO CONFIG_TWAI_TX_GPIO_NUM // CAN tranceiver TX pin
#define TWAI_RX_GPIO CONFIG_TWAI_RX_GPIO_NUM // CAN tranceiver RX pin
#define TWAI_CTL_TIMEOUT_SEC 4*60*60 // TWAI status print interval
#define MAX_CHARGER_AVAILABILITY 12 // extend availability for 10s*12 = 120 sec
#define CHARGER_FIRST_TIME_SEEN 4   // charger first time seen, temporary delays status messages processing

#define chargerSerial2HumanReadable(serial) (__builtin_bswap64(serial) >> 16)

void fp2SetDefaultVoltage(uint8_t chargerIndex, uint16_t voltage);
void twaiCtrlTask( void *pvParameters );
