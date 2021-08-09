#include "nvs_flash.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include "esp_ota_ops.h"

////////////OTA////////////////////
#define EXAMPLE_SERVER_IP OTA_SERVER_IP
#define EXAMPLE_SERVER_PORT OTA_SERVER_PORT
#define EXAMPLE_FILENAME OTA_EXAMPLE_FILENAME
#define BUFFSIZE 1500
#define TEXT_BUFFSIZE 1024

const int CONNECTED_BIT = BIT0;
static EventGroupHandle_t upgrade_event_group;

static const char *TAG = "APP";

typedef enum esp_ota_firm_state
{
    ESP_OTA_INIT = 0,
    ESP_OTA_PREPARE,
    ESP_OTA_START,
    ESP_OTA_RECVED,
    ESP_OTA_FINISH,
} esp_ota_firm_state_t;

typedef struct esp_ota_firm
{
    uint8_t ota_num;
    uint8_t update_ota_num;

    esp_ota_firm_state_t state;

    size_t content_len;

    size_t read_bytes;
    size_t write_bytes;

    size_t ota_size;
    size_t ota_offset;

    const char *buf;
    size_t bytes;
} esp_ota_firm_t;

static const char *TAGO = "ota";
/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = {0};
/*an packet receive buffer*/
static char text[BUFFSIZE + 1] = {0};
/* an image total length*/
static int binary_file_length = 0;
/*socket id*/
static int socket_id = -1;
////////end OTA///////////////////////////////////////
