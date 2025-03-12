#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h" // nvs_flash_* func + "nvs.h"
#include "time.h"
#include "sys/time.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h" // time sync
#include "esp_sntp.h" // for SNTP_SYNC_MODE_SMOOTH
#include "esp_mac.h"  // esp_read_mac
#include <dirent.h>   // opendir readdir dirent
#include "driver/gpio.h" // relay and led control
#include "driver/uart.h" // power meter
#include "rtc_wdt.h"  // watchdog reset
#include "cJSON.h"

// component esp32-wifi-provision-care https://github.com/uqfus/esp32-wifi-provision-care.git
#include "esp32-wifi-provision-care.h"
// component h2zero/esp-libtelnet: https://github.com/h2zero/esp-libtelnet.git
#include "esp-libtelnet.h"
// component espressif/mdns
#include <mdns.h>
// component joltwallet/littlefs
#include <esp_littlefs.h>
// component cmd_vfs https://github.com/uqfus/cmd_vfs_basic.git
#include "cmd_vfs_basic.h"
// component ${IDF_PATH}/examples/system/console/advanced/components/cmd_nvs
#include "cmd_nvs.h"
// component ${IDF_PATH}/examples/system/console/advanced/components/cmd_system
#include "cmd_system.h"

// module fp2charger.c
#include "fp2charger.h"

// configuration values
#include "app_main.h"

static const char *TAG = "flatpack2ctl";
extern volatile fp2Charger_t seenChargers[FP2_MAX_CHARGERS];
volatile config_t config;
float powerMeterVoltage;
float powerMeterCurrent;
float powerMeterPowerW;
float powerMeterPowerVA;
float powerMeterPowerFactor;
float powerMeterEnergy;
uint32_t powerMeterEnergyPulsesOverflows = 0;

// MARK: NVS config
static void LoadConfigFromNVS(void)
{
    const char *TAG = __func__;
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK)
    {
        size_t structSize = sizeof(config);
        nvs_get_blob(nvs, "config", (void *)&config, &structSize);
        nvs_close(nvs);
    } else {
        // apply default values
        config.limitsActive = DEFAULT_CONFIG_APPLY_LIMITS;
        config.dcCurrentSet = DEFAULT_CONFIG_SET_DC_CURRENT;
        config.dcVoltageSet = DEFAULT_CONFIG_SET_DC_VOLTAGE;
        config.relayTurnOnTime = DEFAULT_CONFIG_RELAY_TURN_ON_TIME;
        config.relayTurnOffTime = DEFAULT_CONFIG_RELAY_TURN_OFF_TIME;
    }

    if (config.limitsActive)
        logToFile(TAG, "Limits active Iset=%.1f Uset=%.2f", ((float)config.dcCurrentSet)/10, ((float)config.dcVoltageSet)/100);

    logToFile(TAG, "Relay turn on time: %02d:%02d, turn off time: %02d:%02d",
        config.relayTurnOnTime / 60, config.relayTurnOnTime % 60,
        config.relayTurnOffTime / 60, config.relayTurnOffTime % 60);
    return ;
}

static void storeConfigToNVS(void)
{
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_blob(nvs, "config", (void *)&config, sizeof(config));
        nvs_close(nvs);
    }
}

// MARK: charger chart
// Save charger status to file. Used as chart data html handle.
void logChargerStatusToFile(uint8_t chargerId)
{
    const char *TAG = __func__;
    struct timeval now;
    gettimeofday(&now, NULL);

    fp2ChargerStatus_t cs;
    cs.time = now.tv_sec;
    cs.inletTemperature = seenChargers[chargerId].inletTemperature;
    cs.dcCurrent = seenChargers[chargerId].dcCurrent;
    cs.dcVoltage = seenChargers[chargerId].dcVoltage;
    cs.outletTemperature = seenChargers[chargerId].outletTemperature;

    char chartFileName[50];
    snprintf(chartFileName, sizeof(chartFileName), CHART_FILE_NAME,
        chargerSerial2HumanReadable(seenChargers[chargerId].serial));

    FILE* fd = fopen(chartFileName, "ab"); // append binary mode
    if (fd == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file!");
        return;
    }
    fwrite(&cs, sizeof(cs), 1, fd);
    fclose(fd);

    // chart rotate
    struct stat chartFileStat;
    stat(chartFileName, &chartFileStat);
    if (chartFileStat.st_size >= CHART_FILE_SIZE_TO_ROTATE)
    {
        char chartFileNameToRotate[50];
        snprintf(chartFileNameToRotate, sizeof(chartFileNameToRotate), CHART_FILE_NAME_TO_ROTATE,
            chargerSerial2HumanReadable(seenChargers[chargerId].serial));
        if (stat(chartFileNameToRotate, &chartFileStat) == 0)  // remove old rotated chart if exist
        {
            unlink(chartFileNameToRotate);
        }
        rename(chartFileName, chartFileNameToRotate);
    }
    return;
}

// MARK: log to file
static QueueHandle_t xLogToFileQueue; // queue for log strings to be writed to file
// asyncronous log to file task
static void logToFileTask(void *pvParameters)
{
    const char *TAG = __func__;
    ESP_LOGI(TAG, "Task start.");

    char logString[LOG_TO_FILE_BUFSIZE];
    struct stat logFileStat;
    const char *logFileName = LOG_FILE_NAME;
    const char *logFileNameToRotate = LOG_FILE_NAME_TO_ROTATE;
    FILE* fd;

    while (true)
    {
        xQueueReceive(xLogToFileQueue, &logString, portMAX_DELAY);

        // log rotate
        stat(logFileName, &logFileStat);
        if (logFileStat.st_size > LOG_FILE_SIZE_TO_ROTATE)
        {
            if (stat(logFileNameToRotate, &logFileStat) == 0)  // remove old rotated logs if exist
            {
                unlink(logFileNameToRotate);
            }
            rename(logFileName, logFileNameToRotate);
        }

        fd = fopen(logFileName, "a"); // append mode, create new if not exist
        if (fd == NULL)
        {
            ESP_LOGE(TAG, "logToFile Failed to open file!");
            continue;
        }
        fprintf(fd, "%s\n", logString);
        fclose(fd);
    }
    vTaskDelete(NULL); // Task functions should never return.
}

void logToFile(const char *TAG, const char* format, ...)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    struct tm timeinfo;
    localtime_r(&now.tv_sec, &timeinfo);

    char logstr[LOG_TO_FILE_BUFSIZE];
    char *logpos = logstr;
    logpos += strftime(logpos, sizeof(logstr), "%Y.%m.%d %H:%M:%S.", &timeinfo); // 20 chars
    logpos += snprintf(logpos, sizeof(logstr)-20, "%06ld ", now.tv_usec);   // 20+7 chars
    va_list args;
    va_start(args, format);
    vsnprintf(logpos, sizeof(logstr)-27, format, args); // -27 chars
    va_end(args);

    ESP_LOGI(TAG, "%s", logstr);

    xQueueSendToBack(xLogToFileQueue, &logstr, 1); // post log string to xLogToFileQueue, minimal wait
    return;
}

// MARK: storage init
static esp_err_t storageInit(void)
{
    const char *TAG = __func__;
    esp_vfs_littlefs_conf_t littlefs_conf = {
        .base_path = LITTLEFS_STORAGE_PATH,
        .partition_label = LITTLEFS_PARTITION_LABEL,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    esp_err_t ret = esp_vfs_littlefs_register(&littlefs_conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format littlefs filesystem on partition '%s'.", LITTLEFS_PARTITION_LABEL);
        } else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find LittleFS partition.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    size_t total = 0, used = 0;
    ret = esp_littlefs_info(littlefs_conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
        esp_littlefs_format(littlefs_conf.partition_label);
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, free: %d", total, total - used);
    }

    TaskHandle_t logToFileTaskHandle;
    xLogToFileQueue = xQueueCreate(10, LOG_TO_FILE_BUFSIZE);
    xTaskCreate(&logToFileTask, "logToFileTask", 1024 * 4, NULL, tskIDLE_PRIORITY + 1, &logToFileTaskHandle);

    return ESP_OK;
}

// MARK: led task
static QueueHandle_t xStatusLedQueue; // queue for status led 
static void ledTask(void *pvParameter)
{
    const char *TAG = __func__;
    ESP_LOGI(TAG, "Task start.");

    static const gpio_config_t status_led_conf = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&status_led_conf));
    gpio_set_level(STATUS_LED_GPIO, 1); // turn on led
    uint8_t ledStatus;

    while(true)
    {
        rtc_wdt_feed(); // watchdog feed, STATUS_LED_IDLE_FLASH_INTERVAL < CONFIG_BOOTLOADER_WDT_TIME_MS
        if (xQueueReceive(xStatusLedQueue, &ledStatus, pdMS_TO_TICKS(STATUS_LED_IDLE_FLASH_INTERVAL * 1000)) == pdTRUE )
        {
            switch (ledStatus)
            {
            case STATUS_LED_CHARGER_AVAIL:
                gpio_set_level(STATUS_LED_GPIO, 1);
                vTaskDelay( pdMS_TO_TICKS(300) );
                gpio_set_level(STATUS_LED_GPIO, 0);
                vTaskDelay( pdMS_TO_TICKS(50) );
                break;
            case STATUS_LED_ERROR:
                gpio_set_level(STATUS_LED_GPIO, 1);
                vTaskDelay( pdMS_TO_TICKS(1000) );
                gpio_set_level(STATUS_LED_GPIO, 0);
                vTaskDelay( pdMS_TO_TICKS(500) );
                gpio_set_level(STATUS_LED_GPIO, 1);
                vTaskDelay( pdMS_TO_TICKS(1000) );
                gpio_set_level(STATUS_LED_GPIO, 0);
                vTaskDelay( pdMS_TO_TICKS(500) );
                gpio_set_level(STATUS_LED_GPIO, 1);
                vTaskDelay( pdMS_TO_TICKS(1000) );
                gpio_set_level(STATUS_LED_GPIO, 0);
                break;
            }
        } else {
            // no new messages, just idle flash
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay( pdMS_TO_TICKS(50) );
            gpio_set_level(STATUS_LED_GPIO, 0);
        }    
    }
    vTaskDelete(NULL); // Task functions should never return.
}

// Set new status to led
void statusLedSet(uint8_t newStatus)
{
    xQueueSendToBack(xStatusLedQueue, &newStatus, portMAX_DELAY);
    return;
}

static void statusLedInit(void)
{
    xStatusLedQueue = xQueueCreate(5, sizeof(uint8_t));
    xTaskCreate(&ledTask, "ledTask", 3*1024 , NULL, tskIDLE_PRIORITY + 1, NULL);
    return;
}

// MARK: relay task
// Relay control task, depends from sntp
static void relayTask(void *pvParameter)
{
    const char *TAG = __func__;
    ESP_LOGI(TAG, "Task start.");

    static const gpio_config_t relay_conf = {
        .pin_bit_mask = (1ULL << RELAY_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&relay_conf));
    gpio_set_level(RELAY_GPIO, 0); // startup, turn off relay

    esp_netif_sntp_sync_wait(portMAX_DELAY); // wait until time syncronized

    struct timeval now;
    gettimeofday(&now, NULL);
    struct tm timeinfo;
    localtime_r(&now.tv_sec, &timeinfo);
    // 11:59:59.200000 -> (60-59)*1000 + (500000-200000)/1000 = 1000 + 300 = 1300ms delay
    // 11:59:59.800000 -> (60-59)*1000 + (500000-800000)/1000 = 1000 - 300 = 700ms delay
    TickType_t msToSleep = (60 - timeinfo.tm_sec) * 1000 + (500000 - now.tv_usec) / 1000;
    vTaskDelay( pdMS_TO_TICKS( msToSleep ) ); // align time to a minute interval

    int minutesFromMidnight = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    // startup in charger enabled time. AC power loss?
    if ( minutesFromMidnight < config.relayTurnOffTime || config.relayTurnOnTime < minutesFromMidnight)
    {
        logToFile(TAG, "Startup in charger enabled time, turn on charger AC mains.");
        gpio_set_level(RELAY_GPIO, 1); // turn on relay
    }

    while(true)
    {
        gettimeofday(&now, NULL);
        localtime_r(&now.tv_sec, &timeinfo);
//        ESP_LOGI(TAG, "min available stack size: %d", uxTaskGetStackHighWaterMark(NULL) );
//        ESP_LOGI(TAG, "time: %02d:%02d:%02d.%ld", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, now.tv_usec );
        minutesFromMidnight = timeinfo.tm_hour * 60 + timeinfo.tm_min;
        if (config.relayTurnOffTime == minutesFromMidnight)
        {
            // if relay is on, turn it off. Could be turned off elsewhere.
            if (RELAY_GPIO_GET_LEVEL_DIRECT == 1)
            {
                gpio_set_level(RELAY_GPIO, 0);
                logToFile(TAG, "Turn charger AC mains off.");
            }
        }
        if (config.relayTurnOnTime == minutesFromMidnight)
        {
            // if relay is off turn it on. Could be turned on elsewhere.
            if (RELAY_GPIO_GET_LEVEL_DIRECT == 0)
            {
                gpio_set_level(RELAY_GPIO, 1);
                logToFile(TAG, "Turn charger AC mains on.");
            }
        }
        gettimeofday(&now, NULL);
        localtime_r(&now.tv_sec, &timeinfo);
        msToSleep = (60 - timeinfo.tm_sec) * 1000 + (500000 - now.tv_usec) / 1000;
        vTaskDelay( pdMS_TO_TICKS( msToSleep ) ); // align time to a minute interval
	}
    vTaskDelete(NULL); // Task functions should never return.
}

// MARK: power meter
// Power meter task HLW8032
static void powerMeterTask(void *pvParameter)
{
    const char *TAG = __func__;
    vTaskSuspend(NULL);
    ESP_LOGI(TAG, "Task start.");
    // HLW8032 4800bps + 8 data bit + even bit
    uart_config_t uart_config = {
        .baud_rate = 4800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, POWERMETER_UART_GPIO_TX, POWERMETER_UART_GPIO_RX,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_HW_FIFO_LEN(UART_NUM_1) * 2, 0, 0, NULL, 0));
    uint8_t uartBuf[50];
    int packetSize;
    while(true)
    {
        // From manual: The HLW8032 interval between sending data is 50mS.
        // ticks_to_wait less than 50mS should help to split uart data to individual packets,
        // but investigated from logs ticks_to_wait should be in 60ms< - <110ms range to packet split
        packetSize = uart_read_bytes(UART_NUM_1, uartBuf, sizeof(uartBuf), pdMS_TO_TICKS( 80 ) );
        if (packetSize != 24) // HLW8032 packet size == 24 bytes
        {
            continue;
        }
        uint8_t CRC = 0;
        for (uint8_t i = 2; i < 23; i++) CRC += uartBuf[i];
        if (CRC != uartBuf[23])
        {
            ESP_LOGE(TAG, "HLW8032 packet CRC error.");
            continue;
        }
        if (uartBuf[0] != 0x55) // HLW8032 not ready
        {
            ESP_LOGI(TAG, "HLW8032 State Reg != 0x55.");
            continue;
        }
        if (uartBuf[1] != 0x5A) // HLW8032 CheckREG 0x5A
        {
            ESP_LOGI(TAG, "HLW8032 Check Reg != 0x5A.");
            continue;
        }
        // process only updated values
        uint8_t flags = uartBuf[20];
        bool energyPulsesCarryFlag = flags & BIT7;
        if (energyPulsesCarryFlag)
        {
            powerMeterEnergyPulsesOverflows++;
        }
        bool voltageUpdated = flags & BIT6;
        bool currentUpdated = flags & BIT5;
        bool powerUpdated = flags & BIT4;
        if (voltageUpdated)
        {
            uint32_t VoltageCoefficientRegister = (uint32_t)(uartBuf[2]) << 16 | (uint32_t)(uartBuf[3]) << 8 | (uint32_t)(uartBuf[4]);
            uint32_t VoltageRegister =            (uint32_t)(uartBuf[5]) << 16 | (uint32_t)(uartBuf[6]) << 8 | (uint32_t)(uartBuf[7]);
            powerMeterVoltage = POWERMETER_VOLTAGE_COEFFICIENT * (float)VoltageCoefficientRegister / (float)VoltageRegister;
        }
        if (currentUpdated)
        {
            uint32_t CurrentCoefficientRegister = (uint32_t)(uartBuf[8]) << 16 | (uint32_t)(uartBuf[9]) << 8 | (uint32_t)(uartBuf[10]);
            uint32_t CurrentRegister =            (uint32_t)(uartBuf[11]) << 16 | (uint32_t)(uartBuf[12]) << 8 | (uint32_t)(uartBuf[13]);
            powerMeterCurrent = POWERMETER_CURRENT_COEFFICIENT * (float)CurrentCoefficientRegister / (float)CurrentRegister;
        }
        if (powerUpdated)
        {
            uint32_t PowerCoefficientRegister =   (uint32_t)(uartBuf[14]) << 16 | (uint32_t)(uartBuf[15]) << 8 | (uint32_t)(uartBuf[16]);
            uint32_t PowerRegister =              (uint32_t)(uartBuf[17]) << 16 | (uint32_t)(uartBuf[18]) << 8 | (uint32_t)(uartBuf[19]);
            powerMeterPowerW = POWERMETER_VOLTAGE_COEFFICIENT * POWERMETER_CURRENT_COEFFICIENT * (float)PowerCoefficientRegister / (float)PowerRegister;
            uint32_t EnergyPulses = powerMeterEnergyPulsesOverflows << 16 | (uint32_t)(uartBuf[21]) << 8 | (uint32_t)(uartBuf[22]);
            float pulsesPerkWh = 3.6e12f / (float)PowerCoefficientRegister / (POWERMETER_VOLTAGE_COEFFICIENT * POWERMETER_CURRENT_COEFFICIENT);
            powerMeterEnergy = (float)EnergyPulses / pulsesPerkWh;
        }
        if (voltageUpdated && currentUpdated && powerUpdated)
        {
            powerMeterPowerVA = powerMeterCurrent * powerMeterVoltage;
            powerMeterPowerFactor = powerMeterPowerW / powerMeterPowerVA;
        }

        ESP_LOGI(TAG, "%x Current=%.2f, Voltage=%.2f, PowerW=%.2f, PowerVA=%.2f, PowerFactor=%.2f, Energy=%.3f", flags,
            powerMeterCurrent, powerMeterVoltage, powerMeterPowerW, powerMeterPowerVA, powerMeterPowerFactor, powerMeterEnergy );
    }

    vTaskDelete(NULL); // Task functions should never return.
}

// MARK: httpd handlers
// HTTP Error (404) Handler - Redirects all requests to the /
static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "Redirect to the root uri.");
    return ESP_OK;
}

// HTTP /favicon.ico
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_test_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_test_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}

// HTTP /
static esp_err_t root_get_handler(httpd_req_t *req)
{
    extern const char root_start[] asm("_binary_index_html_gz_start");
    extern const char root_end[] asm("_binary_index_html_gz_end");
    const uint32_t root_len = root_end - root_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, root_start, root_len);
    return ESP_OK;
}

// MARK: HTTP /ctl
// HTTP /ctl - return state of controller
// /ctl?relay=1 - turn on relay, /ctl?relay=0 - turn off relay
// /ctl?limits=1 - limits active, /ctl?limits=0 - limits disabled
// /ctl?voltageLimit=5300 - set voltage limit
// /ctl?currentLimit=285 - set current limit
// /ctl?relayTurnOnTime=120 - set turn on time 02:00 -> 2*60+0
// /ctl?relayTurnOffTime=240 - set turn off time 04:00 -> 4*60+0
// /ctl?storeConfigToNVS=1 - save config
// /ctl?defaultVoltageSet=5400 - default voltage set
static esp_err_t ctl_get_handler(httpd_req_t *req)
{
    const char *TAG = __func__;
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len == 1)
    {
        // there is no query in uri, send state
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "timeNow", time(NULL));
        cJSON_AddNumberToObject(root, "relayState", RELAY_GPIO_GET_LEVEL_DIRECT);
        cJSON_AddNumberToObject(root, "limitsActive", config.limitsActive);
        cJSON_AddNumberToObject(root, "dcCurrentSet", config.dcCurrentSet);
        cJSON_AddNumberToObject(root, "dcVoltageSet", config.dcVoltageSet);
        cJSON_AddNumberToObject(root, "relayTurnOnTime", config.relayTurnOnTime);
        cJSON_AddNumberToObject(root, "relayTurnOffTime", config.relayTurnOffTime);
        cJSON *array = cJSON_AddArrayToObject(root, "chargers");
        for (size_t i=0; i<FP2_MAX_CHARGERS; i++)
        {
            if ( seenChargers[i].serial != 0 )
            {
                cJSON *row = cJSON_CreateObject();
                cJSON_AddItemToArray(array, row );
                char buf[13+1]; // "192750001639" 12 + 1
                sprintf(buf, "%llx", chargerSerial2HumanReadable(seenChargers[i].serial));
                cJSON_AddStringToObject(row, "serial", buf);
                cJSON_AddNumberToObject(row, "status", seenChargers[i].status);
                cJSON_AddNumberToObject(row, "warnings", seenChargers[i].warnings);
                cJSON_AddNumberToObject(row, "alarms", seenChargers[i].alarms);
                cJSON_AddNumberToObject(row, "inletTemperature", seenChargers[i].inletTemperature);
                cJSON_AddNumberToObject(row, "dcCurrent", seenChargers[i].dcCurrent);
                cJSON_AddNumberToObject(row, "dcVoltage", seenChargers[i].dcVoltage);
                cJSON_AddNumberToObject(row, "acVoltage", seenChargers[i].acVoltage);
                cJSON_AddNumberToObject(row, "outletTemperature", seenChargers[i].outletTemperature);
                cJSON_AddStringToObject(row, "name", (char *)seenChargers[i].information.name);
                cJSON_AddStringToObject(row, "partNumber", (char *)seenChargers[i].information.partNumber);
                cJSON_AddStringToObject(row, "hardwareVersion", (char *)seenChargers[i].information.hardwareVersion);
                cJSON_AddStringToObject(row, "primarySoftwarePartnumber", (char *)seenChargers[i].information.primarySoftwarePartnumber);
                cJSON_AddStringToObject(row, "primarySoftwareVersion", (char *)seenChargers[i].information.primarySoftwareVersion);
                cJSON_AddStringToObject(row, "secondarySoftwarePartnumber", (char *)seenChargers[i].information.secondarySoftwarePartnumber);
                cJSON_AddStringToObject(row, "secondarySoftwareVersion", (char *)seenChargers[i].information.secondarySoftwareVersion);
            }
        }

        char *status = cJSON_Print(root);
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
        httpd_resp_set_hdr(req, "Connection", "close");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, status);
        free(status);
        cJSON_Delete(root);
        return ESP_OK;
    }

    char buf[50];
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_req_get_url_query_str(req, buf, sizeof(buf)));
    char value[5];
    
    // relay state "0" "1"
    if ( httpd_query_key_value( buf, "relay", value, sizeof(value) ) == ESP_OK )
    {
        if (value[0] == '0')
        {
            // if relay is on, turn it off. Could be turned off elsewhere.
            if (RELAY_GPIO_GET_LEVEL_DIRECT == 1)
            {
                gpio_set_level(RELAY_GPIO, 0);
                logToFile(TAG, "Turn charger AC mains off.");
                httpd_resp_sendstr(req, "Turn charger AC mains off.");
            } else {
                httpd_resp_sendstr(req, "Charger AC mains already off.");
            }
        } else if (value[0] == '1')
        {
            // if relay is off turn it on. Could be turned on elsewhere.
            if (RELAY_GPIO_GET_LEVEL_DIRECT == 0)
            {
                gpio_set_level(RELAY_GPIO, 1);
                logToFile(TAG, "Turn charger AC mains on.");
                httpd_resp_sendstr(req, "Turn charger AC mains on.");
            } else {
                httpd_resp_sendstr(req, "Charger AC mains already on.");
            }
        } else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error! Bad request.");
        }
        return ESP_OK;
    }

    // limits active "0" "1", if limits disabled, charger voltage and current stays the same, not switched to default
    if ( httpd_query_key_value( buf, "limits", value, sizeof(value) ) == ESP_OK )
    {
        if (value[0] == '0')
        {
            config.limitsActive = 0;
            logToFile(TAG, "Charger limits disabled.");
            httpd_resp_sendstr(req, "Charger limits disabled.");
        } else if (value[0] == '1')
        {
            config.limitsActive = 1;
            logToFile(TAG, "Charger limits enabled.");
            httpd_resp_sendstr(req, "Charger limits enabled.");
        } else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error! Bad request.");
        }
        return ESP_OK;
    }

    // limit charger voltage, 48V charger accepts min=4150->41.5V max=5830->58.3V, change if needed
    if ( httpd_query_key_value( buf, "voltageLimit", value, sizeof(value) ) == ESP_OK )
    {
        long voltage = atol(value);
        if ( 4150 < voltage && voltage < 5830 )
        {
            config.dcVoltageSet = voltage;
            logToFile(TAG, "Voltage limit set to U=%.2fV", ((float)voltage)/100);
            httpd_resp_sendstr(req, "Voltage limit set.");
            return ESP_OK;
        }
    }

    // limit charger current, MinValue 10A, MaxValue 37.4A, change if needed
    if ( httpd_query_key_value( buf, "currentLimit", value, sizeof(value) ) == ESP_OK )
    {
        long current = atol(value);
        if ( 100 < current && current < 375 )
        {
            config.dcCurrentSet = current;
            logToFile(TAG, "Current limit set to I=%.1fA", ((float)current)/10);
            httpd_resp_sendstr(req, "Current limit set.");
            return ESP_OK;
        }
    }

    // relayTurnOnTime, atol return 0 on errors
    if ( httpd_query_key_value( buf, "relayTurnOnTime", value, sizeof(value) ) == ESP_OK )
    {
        long time = atol(value);
        if ( 0 < time && time < 24*60 )
        {
            config.relayTurnOnTime = time;
            logToFile(TAG, "Relay turn on time set to: %02d:%02d.",
                config.relayTurnOnTime / 60, config.relayTurnOnTime % 60);
            httpd_resp_sendstr(req, "Relay turn on time set.");
            return ESP_OK;
        }
    }

    // relayTurnOffTime, atol return 0 on errors
    if ( httpd_query_key_value( buf, "relayTurnOffTime", value, sizeof(value) ) == ESP_OK )
    {
        long time = atol(value);
        if ( 0 < time && time < 24*60 )
        {
            config.relayTurnOffTime = time;
            logToFile(TAG, "Relay turn off time set to: %02d:%02d.",
                config.relayTurnOffTime / 60, config.relayTurnOffTime % 60);
            httpd_resp_sendstr(req, "Relay turn off time set.");
            return ESP_OK;
        }
    }

    // storeConfigToNVS=1
    if ( httpd_query_key_value( buf, "storeConfigToNVS", value, sizeof(value) ) == ESP_OK )
    {
        if (value[0] == '1')
        {
            storeConfigToNVS();
            logToFile(TAG, "Current config saved to NVS.");
            httpd_resp_sendstr(req, "Current config saved to NVS.");
            return ESP_OK;
        }
    }

    // permanently set charger default voltage. 48V charger accepts min=4150->41.5V max=5830->58.3V
    if ( httpd_query_key_value( buf, "defaultVoltageSet", value, sizeof(value) ) == ESP_OK )
    {
        long voltage = atol(value);
        if ( 4150 < voltage && voltage < 5830 )
        {
            logToFile(TAG, "Available chargers default voltage set to Udef=%.2f", ((float)voltage)/100);
            for (size_t i=0; i<FP2_MAX_CHARGERS; i++)
            {
                if ( seenChargers[i].serial != 0 )
                {
                    fp2SetDefaultVoltage(i, voltage);
                }
            }
            httpd_resp_sendstr(req, "Available chargers default voltage set.");
            return ESP_OK;
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error! Bad request.");
    return ESP_OK;
}

// MARK: HTTP /log
static esp_err_t log_get_handler(httpd_req_t *req)
{
    const char *TAG = __func__;
    struct stat logFileStat;
    const char *logFileName = LOG_FILE_NAME;
    const char *logFileNameToRotate = LOG_FILE_NAME_TO_ROTATE;
    FILE *fd;
    char *buf;

    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "text/plain");

    // send old rotated logs, if exist
    if (stat(logFileNameToRotate, &logFileStat) == 0) // 0 if file found
    {
        fd = fopen(logFileNameToRotate, "r");
        if (!fd)
        {
            ESP_LOGE(TAG, "Failed to open log file : %s.", logFileNameToRotate);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open log file.");
            return ESP_OK;
        }
        buf = malloc(logFileStat.st_size + 1);
        if (buf == NULL)
        {
            fclose(fd);
            ESP_LOGE(TAG, "Failed to allocate memory.");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to allocate memory.");
            return ESP_OK;
        }
        if (fread(buf, 1, logFileStat.st_size, fd) != logFileStat.st_size)
        {
            free(buf);
            ESP_LOGE(TAG, "Failed to read log file : %s.", logFileNameToRotate);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read chart file.");
            return ESP_OK;
        }
        fclose(fd);
        httpd_resp_send_chunk(req, buf, logFileStat.st_size);
        free(buf);
    }

    // send logs
    if (stat(logFileName, &logFileStat) == -1) // -1 if file NOT found
    {
        ESP_LOGE(TAG, "Failed to stat log file : %s.", logFileName);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to stat log file.");
        return ESP_OK;
    }
    fd = fopen(logFileName, "r");
    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to open log file : %s.", logFileName);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open log file.");
        return ESP_OK;
    }
    buf = malloc(logFileStat.st_size + 1);
    if (buf == NULL)
    {
        fclose(fd);
        ESP_LOGE(TAG, "Failed to allocate memory.");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to allocate memory.");
        return ESP_OK;
    }
    if (fread(buf, 1, logFileStat.st_size, fd) != logFileStat.st_size)
    {
        free(buf);
        ESP_LOGE(TAG, "Failed to read log file : %s.", logFileName);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read chart file.");
        return ESP_OK;
    }
    fclose(fd);
    httpd_resp_send_chunk(req, buf, logFileStat.st_size);
    free(buf);

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// MARK: HTTP /chart
static esp_err_t chart_get_handler(httpd_req_t *req)
{
    const char *TAG = __func__;
    FILE *fd;
    struct stat chartFileStat;
    char *buf;

    if (httpd_req_get_url_query_len(req) == 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error! Bad request.");
        return ESP_OK;
    }

    char query[50];
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_req_get_url_query_str(req, query, sizeof(query)));

    char charger[13]; // "183371037950" 12+1
    if ( httpd_query_key_value( query, "charger", charger, sizeof(charger) ) != ESP_OK )
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Error! Bad request.");
        return ESP_OK;
    }

    char chartFileNameToRotate[50]; // /littlefs/charger_183371037950.chart.1
    snprintf(chartFileNameToRotate, sizeof(chartFileNameToRotate), CHART_FILE_NAME_TO_ROTATE_S, charger);

    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "application/octet-stream");

    // send old rotated chart data, if exist
    if (stat(chartFileNameToRotate, &chartFileStat) == 0) // 0 if file found
    {
        fd = fopen(chartFileNameToRotate, "rb");
        if (!fd)
        {
            ESP_LOGE(TAG, "Failed to open chart file : %s.", chartFileNameToRotate);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open chart file.");
            return ESP_OK;
        }
        buf = malloc(chartFileStat.st_size + 1);
        if (buf == NULL)
        {
            fclose(fd);
            ESP_LOGE(TAG, "Failed to allocate memory.");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to allocate memory.");
            return ESP_OK;
        }
        if (fread(buf, 1, chartFileStat.st_size, fd) != chartFileStat.st_size)
        {
            fclose(fd);
            free(buf);
            ESP_LOGE(TAG, "Failed to read chart file : %s.", chartFileNameToRotate);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read chart file.");
            return ESP_OK;
        }
        fclose(fd);
        httpd_resp_send_chunk(req, buf, chartFileStat.st_size);
        free(buf);
    }

    char chartFileName[50]; // /littlefs/charger_183371037950.chart
    snprintf(chartFileName, sizeof(chartFileName), CHART_FILE_NAME_S, charger);

    if (stat(chartFileName, &chartFileStat) == -1) // -1 if file NOT found
    {
        ESP_LOGE(TAG, "Failed to stat chart file : %s.", chartFileName);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Failed to stat chart file.");
        return ESP_OK;
    }
    fd = fopen(chartFileName, "rb");
    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to open chart file : %s.", chartFileName);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open chart file.");
        return ESP_OK;
    }
    buf = malloc(chartFileStat.st_size + 1);
    if (buf == NULL)
    {
        fclose(fd);
        ESP_LOGE(TAG, "Failed to allocate memory.");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to allocate memory.");
        return ESP_OK;
    }
    if (fread(buf, 1, chartFileStat.st_size, fd) != chartFileStat.st_size)
    {
        fclose(fd);
        free(buf);
        ESP_LOGE(TAG, "Failed to read chart file : %s.", chartFileName);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read chart file.");
        return ESP_OK;
    }
    fclose(fd);
    httpd_resp_send_chunk(req, buf, chartFileStat.st_size);
    free(buf);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// MARK: start httpd
static void start_webserver(void)
{
    const char *TAG = __func__;
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting http server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers config.max_uri_handlers = 8 default
        const httpd_uri_t favicon_uri = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler };
        httpd_register_uri_handler(server, &favicon_uri);
        // wifi_provision_care_updateota_post_handler POST handler, from component https://github.com/uqfus/esp32-wifi-provision-care.git
        const httpd_uri_t updateota_uri = { .uri = "/updateota", .method = HTTP_POST, .handler = wifi_provision_care_updateota_post_handler };
        httpd_register_uri_handler(server, &updateota_uri);

        const httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root_uri);
        const httpd_uri_t log_uri = { .uri = "/log", .method = HTTP_GET, .handler = log_get_handler };
        httpd_register_uri_handler(server, &log_uri);
        const httpd_uri_t chart_uri = { .uri = "/chart", .method = HTTP_GET, .handler = chart_get_handler };
        httpd_register_uri_handler(server, &chart_uri);
        const httpd_uri_t ctl_uri = { .uri = "/ctl", .method = HTTP_GET, .handler = ctl_get_handler };
        httpd_register_uri_handler(server, &ctl_uri);

        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    return;
}

// MARK: NVS stats
// dump NVS partition stats
/*
static void nvs_list_all(void)
{
    const char *TAG = __func__;
    nvs_stats_t nvs_stats;
    ESP_ERROR_CHECK(nvs_get_stats(NULL, &nvs_stats));
    ESP_LOGI(TAG, "NVS: UsedEntries = %u, FreeEntries = %u, AvailableEntries = %u, AllEntries = %u, AllNamespaces= (%u)",
          nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.available_entries, nvs_stats.total_entries, nvs_stats.namespace_count);
    nvs_iterator_t it = NULL;

    esp_err_t res = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
    while(res == ESP_OK)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info); // Can omit error check if parameters are guaranteed to be non-NULL
        ESP_LOGI(TAG, "NVS: namespace '%s' : key '%s', type '%d' ", info.namespace_name, info.key, info.type );
        res = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);
}
*/
// MARK: telnet console
static void telnet_rx_cb(const char *buf, size_t len)
{
    char cmdline[256+1]; // ESP_CONSOLE_CONFIG_DEFAULT() .max_cmdline_length = 256
    if (len >= sizeof(cmdline))
    {
        ESP_LOGE(TAG, "Error, command too long.");
        return;
    }
    strlcpy(cmdline, buf, len + 1); // local copy with NUL-termination
    cmdline[strcspn(cmdline, "\r\n")] = 0; // remove LF, CR, CRLF, LFCR

    int ret;
    esp_err_t err = esp_console_run(cmdline, &ret);
    if (err == ESP_ERR_NOT_FOUND) {
        printf("Unrecognized command\n");
    } else if (err == ESP_ERR_INVALID_ARG) {
        // no error, just cmdline was empty
    } else if (err == ESP_OK && ret != ESP_OK) {
        printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
    } else if (err != ESP_OK) {
        printf("Internal error: %s\n", esp_err_to_name(err));
    }
    printf(">");
    fflush(stdout);
}

// MARK: app_main
void app_main(void)
{
    ESP_LOGI(TAG, "Study Project entry point.");
    esp_log_level_set("wifi", ESP_LOG_WARN); // Only errors and warnings from WiFi stack
    esp_log_level_set("gpio", ESP_LOG_WARN); // Only errors and warnings from gpio driver
    setenv("TZ", "MSK-3", 1);                // time zone
    tzset();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_init(&console_config));
    esp_console_register_help_command(); // Default 'help' command prints the list of registered commands

    console_register_vfs_basic_func(); // cmd_vfs https://github.com/uqfus/cmd_vfs_basic.git
    register_system_common();  // examples/system/console/advanced/components/cmd_system
    register_nvs(); // examples/system/console/advanced/components/cmd_nvs

    statusLedInit();

    ESP_ERROR_CHECK(storageInit());

    esp_reset_reason_t rr = esp_reset_reason();
    logToFile(TAG, "esp_reset_reason(): %d", rr);

    wifi_provision_care((char *)TAG); // connect to wifi

    //Initialize telnet
    init_telnet(telnet_rx_cb);
    start_telnet();
    ESP_LOGI(TAG, "Telnet server up and ready for connections.");

    uint8_t ap_mac[6];
    ESP_ERROR_CHECK(esp_read_mac(ap_mac, ESP_MAC_EFUSE_FACTORY)); // burned by Espressif in production (6 bytes)
    char hostname[32];
    sprintf(hostname, "%s-%02X%02X", TAG, ap_mac[4], ap_mac[5]);
    ESP_LOGI(TAG, "Publish mDNS hostname %s.local.", hostname);
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));

    // Set time via NTP
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    sntp_config.smooth_sync = SNTP_SYNC_MODE_SMOOTH;
    esp_netif_sntp_init(&sntp_config);

    LoadConfigFromNVS();

    // Create power meter task
    xTaskCreate(&powerMeterTask, "powerMeterTask", 3 * 1024 , NULL, tskIDLE_PRIORITY + 1, NULL);

    // Create relay control task, depends from sntp
    xTaskCreate(&relayTask, "relayTask", 4 * 1024 , NULL, tskIDLE_PRIORITY + 1, NULL);

    start_webserver();

    // start CAN processing
    xTaskCreate(&twaiCtrlTask, "twaiCtrlTask", 4 * 1024, NULL, tskIDLE_PRIORITY + 5, NULL);

    while(true)
    {
        // align time to a minute interval.
        struct timeval now;
        gettimeofday(&now, NULL);
        struct tm timeinfo;
        localtime_r(&now.tv_sec, &timeinfo);
        TickType_t msToSleep = (60 - timeinfo.tm_sec) * 1000 + (500000 - now.tv_usec) / 1000;
        vTaskDelay( pdMS_TO_TICKS( msToSleep ) );

        for (int i=0; i<FP2_MAX_CHARGERS; i++)
        {
            if ( seenChargers[i].serial != 0 )
            {
                logChargerStatusToFile(i);
            }
        }

    }
    vTaskDelete(NULL); // Task functions should never return.
}
