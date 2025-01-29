// Status LED control task configuration
void statusLedSet(uint8_t newStatus);
#define STATUS_LED_CHARGER_AVAIL 1
#define STATUS_LED_ERROR 2
#define STATUS_LED_GPIO CONFIG_STATUS_LED_GPIO_NUM
// 5sec status repeat interval
// also used to feed watchdog, so STATUS_LED_IDLE_FLASH_INTERVAL should be < CONFIG_BOOTLOADER_WDT_TIME_MS
#define STATUS_LED_IDLE_FLASH_INTERVAL 5

// Relay control task configuration
#define RELAY_GPIO CONFIG_RELAY_GPIO_NUM
#include "rom/gpio.h"
// for pin configured as GPIO_MODE_OUTPUT gpio_get_level always return 0
// so get it state direct from GPIO_OUT1_REG RELAY_GPIO GPIO_NUM_33
#define RELAY_GPIO_GET_LEVEL_DIRECT ((GPIO_REG_READ(GPIO_OUT1_REG) >> (RELAY_GPIO - 32)) & 0x1)

// Power meter task configuration
// HLW8032 only send data
#define POWERMETER_UART_GPIO_TX CONFIG_POWERMETER_UART_TX_GPIO_NUM
#define POWERMETER_UART_GPIO_RX CONFIG_POWERMETER_UART_RX_GPIO_NUM
// 4*470kOhm / 1kOhm / 1000 = 1.88
#define POWERMETER_VOLTAGE_COEFFICIENT (1.88f)
// 1 / 0.002Ohm / 1000
#define POWERMETER_CURRENT_COEFFICIENT (0.5f)

// LittleFS configuration
#define LITTLEFS_PARTITION_LABEL "storage"
#define LITTLEFS_STORAGE_PATH "/littlefs"

// logToFile configuration
void logToFile(const char *TAG, const char* format, ...);
#define LOG_FILE_NAME LITTLEFS_STORAGE_PATH"/master.log"
#define LOG_FILE_NAME_TO_ROTATE LITTLEFS_STORAGE_PATH"/master.log.1"
#define LOG_TO_FILE_BUFSIZE 155
#define LOG_FILE_SIZE_TO_ROTATE 12000

// logChargerStatusToFile configuration
#define CHART_FILE_NAME LITTLEFS_STORAGE_PATH"/charger_%llx.chart"
#define CHART_FILE_NAME_S LITTLEFS_STORAGE_PATH"/charger_%s.chart"
#define CHART_FILE_NAME_TO_ROTATE LITTLEFS_STORAGE_PATH"/charger_%llx.chart.1"
#define CHART_FILE_NAME_TO_ROTATE_S LITTLEFS_STORAGE_PATH"/charger_%s.chart.1"
#define CHART_FILE_SIZE_TO_ROTATE 24*60*sizeof(fp2ChargerStatus_t) // 24hours * 60min * sizeof(fp2ChargerStatus_t)

// Load Store config
// 60 * hour + minute
#define DEFAULT_CONFIG_RELAY_TURN_ON_TIME (60*23+15)
#define DEFAULT_CONFIG_RELAY_TURN_OFF_TIME (60*06+45)
#define DEFAULT_CONFIG_APPLY_LIMITS 1
// set current to 28.5A -> 285=0x011D -- 40A-2,2kW 0x0140=32.0A-1,7kW
#define DEFAULT_CONFIG_SET_DC_CURRENT 285
// set voltage to 54.0V -> 5400 = 0x1518
#define DEFAULT_CONFIG_SET_DC_VOLTAGE 5400
#pragma pack(1)
typedef struct {
    uint8_t  limitsActive; // 1 - limits active
    uint16_t dcCurrentSet; // RealValue = dcCurrent / 10
    uint16_t dcVoltageSet; // RealValue = dcCurrent / 100
    uint16_t relayTurnOnTime; // hour * 60 + minute
    uint16_t relayTurnOffTime;
} config_t;
