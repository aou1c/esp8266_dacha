/* 
  NodeMCU 0.9 (ESP-12 Module)
  Контроллер для умной дачи 
  1.Двухтарифный счетчик электроэнергии (день/ночь), 
  1.1 Счетчик импульсный типа НЕВА 
    - Расчет мнгновенной мощности
  1.2. Счетчик PZE (напряжение,мощность, коэфф. мощности, частота сети)
    - Расчет мнгновенной мощности
    - Хранение изменений мгновенной мощности в кольцевом буфере RAM (~3 - 7 часов)
  2.Управление нагрузками: 5 выходов (4 - выходы на реле, 1 - вкл/выкл охраны)
  3.Измерение температуры
  4.Управление по WIFI
  5.Возможность обновления прошивки по воздуху
  6.При пропадании напряжения питания,  сохранение показаний счетчиков и состояние выходов в  
  EEPROM 
   Copyright (c) 2020, Antonov Technologies
   All rights reserved.
*/
static char Version[] = "\"1.016_12.06.2020\"";


//////////////////////////////////
// для нового SDK
//////////////////////////////////

#include "esp_netif.h"
#include "esp_event.h"
#include "protocol_examples_common.h"
#include "nvs.h"
#include <esp_http_server.h>
//////////////////////////////////

#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>

#include <time.h>
#include "nvs_flash.h"
#include "lwip/apps/sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>

//#include "esp_ota_ops.h"
//#include "include/ota/ota.c"

//#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include <esp_http_server.h>

// DS18B20 driver
 #include "include/ds18b20/ds18b20.h"
#include "include/ds18b20/ds18b20.c"
#include "onewire/onewire.c"
static float resultssensor = 0;

#include "softuart/softuart.c"

#include "PZEM004T/PZEM004Tv30.c"

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

static EventGroupHandle_t wifi_event_group;
#ifdef wifi_event_group
static EventGroupHandle_t upgrade_event_group;
#endif

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static const char *TAG = "APP";

#define GPIO_OUTPUT_IO_0 D4 // //Выход на светодиод D4
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << D0) | (1ULL << D1) | (1ULL << D2) | (1ULL << D3) | (1ULL << D5))
#define GPIO_INPUT_IO_0 13 //D7 Вход датчика сети 220
#define GPIO_INPUT_IO_1 15 // D8 вход счетчика
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_IO_0) | (1ULL << GPIO_INPUT_IO_1))

#define TEST_ONE_SHOT false // testing will be done without auto reload (one-shot)
#define TEST_RELOAD true    // testing will be done with auto reload



#define D0 16 //
#define D1 5 //
#define D2 4 //
#define D3 0 //
#define D4 2 //
#define D5 14 //
#define D6 12 //
#define D7 13 //
#define D8 15 //

char out_pin[] = {D0, D1, D2, D3, D5}; //выходы реле
unsigned int status_out[5] = {1, 1, 1, 1, 1};      // Состояние выходов (управление реле)
volatile long sum_count[6] = {0, 0, 0, 0, 0, 0};   //Итого показания счетчика в RAM день/ночь на текущий день, итого, на  предидущий день
volatile float sumZ_count[6] = {0, 0, 0, 0, 0, 0}; //Итого показания счетчика PZEM004T в RAM день/ночь на текущий день, итого, на  предидущий день
static int CountActiv = 0;

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle power_evt_queue = NULL;
static xQueueHandle ntp_evt_queue = NULL;
static xQueueHandle load_param_queue = NULL;

volatile long count = 0;   //
volatile int countPower = 450; //
int countSave = 0;
int load_day;
#define K_DEL 5 //10 - счетчик Нева  3200 имп на киловат/час

static int sensor_worked = 0;

static int WIFIDisconected = 0;

typedef struct
{
    time_t now;
    float voltage;
    float power;
} data_Z;
#define SizeBuffer 1000 //Размер кольцевого буфера

static data_Z p_buf[SizeBuffer];

static int p_dataZ = 0;

static time_t time_start;





void
    hw_timer_callback1(void *arg)
{
    count++;
    countPower--;
    if (countdelay)
        countdelay --;
    if (!countPower)
    {
        uint32_t gpio_num = 1000000;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
    if (count > (countSave * 2 * K_DEL)) {
        countSave = count / K_DEL;
    }

}

static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)count;
    count = 0;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void power_isr_handler(void *arg)
{
    countPower = 450;
}

int is_day(void)
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    return ((timeinfo.tm_hour < 23) && (timeinfo.tm_hour >= 7));
}

static void SaveParamZ(void)
{
    const char *out1 = "%d %d %d %d %d %d %f %f %f %f \0";
    char *out = NULL;
    out = malloc(100);

    time_t now;
    time(&now);

    sprintf(out, out1, now, status_out[0], status_out[1], status_out[2], status_out[3], status_out[4], sumZ_count[0], sumZ_count[1], sumZ_count[2], sumZ_count[3]);

    nvs_handle handle_1;
    ESP_ERROR_CHECK(nvs_open("paramZ", NVS_READWRITE, &handle_1));

    ESP_ERROR_CHECK(nvs_set_str(handle_1, "key", out));

    char buf[strlen(out) + 1];
    size_t buf_len = sizeof(buf);

    ESP_ERROR_CHECK(nvs_get_str(handle_1, "key", buf, &buf_len));

    nvs_close(handle_1);
    //nvs_flash_deinit();

    ESP_LOGI(TAG, "\n POWER OFF ->  SAVE: %s \n", out);
    free(out);
}

static void SaveParam(void)
{
    const char *out1 = "%d %d %d %d %d %d %d %d %d %d \0";
    char *out = NULL;
    out = malloc(100);

    time_t now;
    time(&now);

    sprintf(out, out1, now, status_out[0], status_out[1], status_out[2], status_out[3], status_out[4], sum_count[0], sum_count[1], sum_count[2], sum_count[3]);

    nvs_handle handle_1;
    ESP_ERROR_CHECK(nvs_open("param", NVS_READWRITE, &handle_1));

    ESP_ERROR_CHECK(nvs_set_str(handle_1, "key", out));

    char buf[strlen(out) + 1];
    size_t buf_len = sizeof(buf);

    ESP_ERROR_CHECK(nvs_get_str(handle_1, "key", buf, &buf_len));

    nvs_close(handle_1);
    //nvs_flash_deinit();

    ESP_LOGI(TAG, "\n POWER OFF ->  SAVE: %s \n", out);
    free(out);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    int cnt = 1;
    int enable = 0;
    int k_del = K_DEL;
    int k_power = 10/K_DEL;

    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {

            if (io_num < 1000000) {
                if (!sensor_worked)
                    countSave = io_num / k_power;
                if (enable) {
                    k_del --;
                    if (!k_del) {
                        k_del = K_DEL;
                        int is_night = !is_day();
                        for (int i = is_night; i < (4 + is_night); i += 2)
                        {
                            sum_count[i]++;
                        }
                         cnt++;
                        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
                        vTaskDelay(10 / portTICK_RATE_MS);
                        cnt++;
                        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
                     }
                }

            }
            else 
                if (io_num == 1000000)
                {
                    SaveParam();
                    SaveParamZ();
                }
                else
                {
                    enable = 1;
                    ESP_LOGI(TAG, "\n ENABLE %d \n", io_num);
                }
                        
         }

         
    }
}

/* An HTTP GET handler */
esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        //ESP_LOGI(TAG, "Request headers lost");
    }
    //WIFIDisconected = 1;
    return ESP_OK;
}

bool is_param_status_Z(httpd_req_t *req)
{
    
    char *buf;
    size_t buf_len;
    bool res = false;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1)
    {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[3];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "Z", param, sizeof(param)) == ESP_OK)
            {
                ESP_LOGI(TAG, "Found URL query parameter => Z=%s", param);
                res = true;
            }
        }
        free(buf);
    }
    return res;
}


esp_err_t  power_get_handler(httpd_req_t * req)
{
    int remaining = req->content_len;

    int dataZ = p_dataZ;

    char *buf = NULL;
    buf = malloc(100);

    const char *out1 = "{\t\"T\":\t%d,\n\t\"P\":\t%f,\n\t\"U\":\t%f\n},\0";

    for (int n = 0; n < 1000; n++)
    {
        dataZ++;
        if (dataZ > SizeBuffer)
            dataZ = 0;

        sprintf(buf, out1, p_buf[dataZ].now, p_buf[dataZ].power, p_buf[dataZ].voltage);
        httpd_resp_send_chunk(req, buf, strlen(buf));
    }
    free(buf);

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}


esp_err_t  upgrade_get_handler(httpd_req_t * req)
{
//    xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
//    esp_err_t esp_res = httpd_resp_send(req, "Start upgrade", 13);
//    ESP_LOGI(TAG, "Start upgrade: %d,  \n", esp_res);
//    xEventGroupSetBits(upgrade_event_group, CONNECTED_BIT);

    return ESP_OK;
}

esp_err_t  status_get_handler(httpd_req_t * req)
{




    time_t now;
    time(&now);

    char pin_out[8] = "\"00000\"";
    //ESP_LOGI(TAG, "%s,  %c", pin_out, '\n');
    for (int n = 0; n < 5; n++) {
        if (gpio_get_level(out_pin[n]))
            pin_out[n+1] = '1';
    }

    float sensor = resultssensor;
    if (sensor != sensor)
        sensor = 0;

    char *out = NULL;
    out = malloc(600);

    if (is_param_status_Z(req))
    {
        ESP_LOGI(TAG, "is_param_status_Z");
        const char *out1 = "{\t\"OUT\":\t%s,\n\t\"CountImp\":\t%d,\n\t\"CountDay\":\t%f, \n\t\"CountNight\":\t%f,\n\t\"CountDayPrev\":\t%f,\t\n\t\"CountNightPrev\":\t%f,\n\t\"DATE_TIME\":\t%d,\n\t\"CountDaySum\":\t%f,\n\t\"CountNightSum\":\t%f,\n\t\"Temperature\":\t%g,\n\t\"V\":\t%f,\n\t\"I\":\t%f,\n\t\"P\":\t%f,\n\t\"E\":\t%f,\n\t\"F\":\t%f,\n\t\"K\":\t%f,\n\t\"TIME_START\":\t%d,\n\t\"Ver\":\t%s\n}";
        sprintf(out, out1, pin_out, countSave, sumZ_count[0], sumZ_count[1], sumZ_count[4], sumZ_count[5], now, sumZ_count[2], sumZ_count[3], sensor, _currentValues.voltage, _currentValues.current, _currentValues.power, _currentValues.energy, _currentValues.frequeny, _currentValues.pf, time_start, Version);
    }
    else
    {
        const char *out1 = "{\t\"OUT\":\t%s,\n\t\"CountImp\":\t%d,\n\t\"CountDay\":\t%d, \n\t\"CountNight\":\t%d,\n\t\"CountDayPrev\":\t%d,\t\n\t\"CountNightPrev\":\t%d,\n\t\"DATE_TIME\":\t%d,\n\t\"CountDaySum\":\t%d,\n\t\"CountNightSum\":\t%d,\n\t\"Temperature\":\t%g,\n\t\"V\":\t%f,\n\t\"I\":\t%f,\n\t\"P\":\t%f,\n\t\"E\":\t%f,\n\t\"F\":\t%f,\n\t\"K\":\t%f\n}";
        sprintf(out, out1, pin_out, countSave, sum_count[0], sum_count[1], sum_count[4], sum_count[5], now, sum_count[2], sum_count[3], sensor, _currentValues.voltage, _currentValues.current, _currentValues.power, _currentValues.energy, _currentValues.frequeny, _currentValues.pf);
    }

    esp_err_t esp_res = httpd_resp_send(req, out, strlen(out));
    ESP_LOGI(TAG, "%d,  \n", esp_res);

    if (httpd_req_get_hdr_value_len(req, "Host") == 0)
    {
        //printf("Request headers lost %d,  %c", countSave, '\n');
        //ESP_LOGI(TAG, "Request headers lost");
    }
    free(out);

    if (esp_res != 0) {
        SaveParamZ();
        esp_restart(); //Перезагрузка

        /*         ESP_ERROR_CHECK(esp_wifi_disconnect());
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT); */
    }

    return ESP_OK;
}


static void load_PZE(void *arg);

esp_err_t init_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "init_get_handler %s", req->uri);

    while (CountActiv)   {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    };

    CountActiv = 1;

    bool Z = is_param_status_Z(req);

    char *word = NULL;
    char delim[] = "&";
    char *val = NULL;
    char name_par[2][8] = {"DCOUNT=", "NCOUNT="};

    for (int n = 0; n < 4; n++)
        sum_count[n] = 0;
    word = strtok(req->uri, delim);
    while (word)
    {
        for (int n = 0; n < 2; n += 1)
        {
            if (strstr(word, name_par[n]))
            {
                if (Z) {
                    float val = atof(strstr(word, "=") + 1);
                    sumZ_count[n + 2] = val;
                    ESP_LOGI(TAG, "sumZ_count[%d] = %f \n", n+2, val);
                    sumZ_count[0] = 0;
                    sumZ_count[1] = 0;
                }
                else
                {
                    int val = atoi(strstr(word, "=") + 1);
                    sum_count[n + 2] = val;
                }
            }
        }
        word = strtok('\0', delim);
    };
    ESP_LOGI(TAG, "sumZ_count[0]: %f", sumZ_count[0]);
    ESP_LOGI(TAG, "sumZ_count[1]: %f", sumZ_count[1]);
    ESP_LOGI(TAG, "sumZ_count[2]: %f", sumZ_count[2]);
    ESP_LOGI(TAG, "sumZ_count[3]: %f", sumZ_count[3]);

    
    CountActiv = 0;

    SaveParam();
    SaveParamZ();

    esp_err_t res = status_get_handler(req);
    return res;
}

httpd_uri_t init = {
    .uri = "/init",
    .method = HTTP_GET,
    .handler = init_get_handler,
    .user_ctx = NULL};

esp_err_t set_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "set_get_handler %s", req->uri);


    char *word = NULL;
    char delim[] = "&";
    char *val = NULL;
    char name_par[5][6] = {"OUT0=", "OUT1=", "OUT2=", "OUT3=", "OUT4="};

    word = strtok(req->uri, delim);
    ESP_LOGI(TAG, "param: %s", word);
    while (word)
    {
        for (int n=0; n<5; n+=1) {
            if (strstr(word, name_par[n]))
            {
                int val = atoi(strstr(word, "=") + 1);
                ESP_LOGI(TAG, "param: %s, %d /n", word, val);
                if (val > 1)
                {
                    ESP_LOGI(TAG, "Impuls: %d", val);
                    gpio_set_level(out_pin[n], 0);
                    vTaskDelay(val / portTICK_RATE_MS);
                    gpio_set_level(out_pin[n], 1);
                    status_out[n] = 1;
                }
                else
                {
                    status_out[n] = val;
                    gpio_set_level(out_pin[n], status_out[n]);
                }
            }
        }
        word = strtok('\0', delim);
    };

    return status_get_handler(req);
}

httpd_uri_t upgrade = {
    .uri = "/upgrade",
    .method = HTTP_GET,
    .handler = upgrade_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "Upgrade!"};

httpd_uri_t set = {
    .uri = "/set",
    .method = HTTP_GET,
    .handler = set_get_handler,
    .user_ctx = NULL};

httpd_uri_t hello = {
    .uri = "/hello",
    .method = HTTP_GET,
    .handler = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = "Hello World upgraded!"};

httpd_uri_t status = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = NULL};

httpd_uri_t power = {
    .uri = "/power",
    .method = HTTP_GET,
    .handler = power_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx = NULL};

    /* An HTTP POST handler */
esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0)
    {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                                  MIN(remaining, sizeof(buf)))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}


    httpd_uri_t echo = {
        .uri = "/echo",
        .method = HTTP_POST,
        .handler = echo_post_handler,
        .user_ctx = NULL};

    /* An HTTP PUT handler. This demonstrates realtime
 * registration and deregistration of URI handlers
 */
    esp_err_t ctrl_put_handler(httpd_req_t * req)
    {
        char buf;
        int ret;

        if ((ret = httpd_req_recv(req, &buf, 1)) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                httpd_resp_send_408(req);
            }
            return ESP_FAIL;
        }


        if (buf == '0')
        {
            /* Handler can be unregistered using the uri string */
            ESP_LOGI(TAG, "Unregistering /hello and /echo URIs");
            //httpd_unregister_uri(req->handle, "/hello");
            //httpd_unregister_uri(req->handle, "/echo");
            //httpd_unregister_uri(req->handle, "/status");
            //httpd_unregister_uri(req->handle, "/power");
            //httpd_unregister_uri(req->handle, "/set");
            httpd_unregister_uri(req->handle, "/init");
        }
        else
        {
            ESP_LOGI(TAG, "Registering /hello and /echo URIs");
            //httpd_register_uri_handler(req->handle, &hello);
            //httpd_register_uri_handler(req->handle, &echo);
            //httpd_register_uri_handler(req->handle, &status);
            //httpd_register_uri_handler(req->handle, &power);
            //httpd_register_uri_handler(req->handle, &set);
            httpd_register_uri_handler(req->handle, &init);
        }

        /* Respond with empty body */
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    httpd_uri_t ctrl = {
        .uri = "/ctrl",
        .method = HTTP_PUT,
        .handler = ctrl_put_handler,
        .user_ctx = NULL};



void Load_count_Z(void) {
    nvs_handle handle_1 = NULL;
    esp_err_t ret_ESP = nvs_open("paramZ", NVS_READONLY, &handle_1);
    if (ret_ESP == ESP_ERR_NVS_NOT_FOUND)
    {
        SaveParamZ();
        ESP_LOGI(TAG, "\n Save default parametrs: \n");
        esp_err_t ret_ESP = nvs_open("paramZ", NVS_READONLY, &handle_1);
    }
    if (handle_1 != NULL)
    {

        char buf[300];
        size_t buf_len = sizeof(buf);

        ret_ESP = nvs_get_str(handle_1, "key", buf, &buf_len);
        if (ret_ESP == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGI(TAG, "\n SaveParamZ  \n");
            SaveParamZ();
        }

        ESP_ERROR_CHECK(nvs_get_str(handle_1, "key", buf, &buf_len));
        ESP_LOGI(TAG, "\n Load parametrs: %s \n", buf);
        nvs_close(handle_1);

        time_t now;

        char format[100] = "%d %d %d %d %d %d %f %f %f %f \0";
        sscanf(&buf, &format, &now, &status_out[0], &status_out[1], &status_out[2], &status_out[3], &status_out[4], &sumZ_count[0], &sumZ_count[1], &sumZ_count[2], &sumZ_count[3]);

        ESP_LOGI(TAG, "\n");
        ESP_LOGI(TAG, "LOAD sumZ_count[0]: %f \n", sumZ_count[0]);
        ESP_LOGI(TAG, "LOAD sumZ_count[1]: %f \n", sumZ_count[1]);
        ESP_LOGI(TAG, "LOAD sumZ_count[0]: %f \n", sumZ_count[2]);
        ESP_LOGI(TAG, "LOAD sumZ_count[1]: %f \n", sumZ_count[3]);
    }
}

    void init_gpio(void)
    {
        printf("Start: init_gpio");

        gpio_config_t io_conf;
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.GPIO15/16
        io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 0;
        //configure GPIO with the given settings
        gpio_config(&io_conf);

        //interrupt of rising edge
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        //bit mask of the pins, use GPIO4/5 here
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);

        //change gpio intrrupt type for one pin
        gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

        //create a queue to handle gpio event from isr
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        power_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        ntp_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        load_param_queue = xQueueCreate(10, sizeof(uint32_t));
        //start gpio task
        xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

        //install gpio isr service
        gpio_install_isr_service(0);
        //hook isr handler for specific gpio pin
        gpio_isr_handler_add(GPIO_INPUT_IO_0, power_isr_handler, (void *)GPIO_INPUT_IO_0);
        //hook isr handler for specific gpio pin
        gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);

        time_t now;

        nvs_handle handle_1 = NULL;
        esp_err_t ret_ESP = nvs_open("param", NVS_READONLY, &handle_1);
        if (ret_ESP == ESP_ERR_NVS_NOT_FOUND) {
            SaveParamZ();
            ESP_LOGI(TAG, "\n Save default parametrs: \n");
            esp_err_t ret_ESP = nvs_open("param", NVS_READONLY, &handle_1);
        }
        if (handle_1 != NULL) {
            char buf[300];
            size_t buf_len = sizeof(buf);
            ret_ESP = nvs_get_str(handle_1, "key", buf, &buf_len);
            if (ret_ESP != ESP_OK) {
                SaveParamZ();
            }
            ESP_LOGI(TAG, "\n Load parametrs: %s \n", buf);
            nvs_close(handle_1);

            char format[100] = "%d %d %d %d %d %d %d %d %d %d \0";
            sscanf(&buf, &format, &now, &status_out[0], &status_out[1], &status_out[2], &status_out[3], &status_out[4], &sum_count[0], &sum_count[1], &sum_count[2], &sum_count[3]);

            ESP_LOGI(TAG, "\n");
            ESP_LOGI(TAG, "LOAD sum_count[0]: %d \n", (int)sum_count[0]);
            ESP_LOGI(TAG, "LOAD sum_count[1]: %d \n", (int)sum_count[1]);
        }
        Load_count_Z();

        for (size_t i = 0; i < 5; i++)
        {
            gpio_set_level(out_pin[i], status_out[i]);
            ESP_LOGI(TAG, "Set OUT: %u :%u %u \n", i, status_out[i], gpio_get_level(out_pin[i]));
        }
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        load_day = timeinfo.tm_mday;
        printf("End: init_gpio");
    }


    static void initialize_sntp(void)
    {
        ESP_LOGI(TAG, "Initializing SNTP");
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_init();
    }

    static void obtain_time(void)
    {
        ESP_LOGI(TAG, "Start obtain_time");
//        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
//                            false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Stop obtain_time");
        initialize_sntp();

        // wait for time to be set
        time_t now = 0;
        struct tm timeinfo = {0};
        int retry = 0;
        const int retry_count = 10;

        while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count)
        {
            ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            time(&now);
            localtime_r(&now, &timeinfo);
        }
        time_start = now;
    }

    static void sntp_example_task(void *arg)
    {
        time_t now;
        struct tm timeinfo;
        char strftime_buf[64];

        time(&now);
        localtime_r(&now, &timeinfo);


        ESP_LOGI(TAG, "START Time");
        ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());

        // Is time set? If not, tm_year will be (1970 - 1900).
        if (timeinfo.tm_year < (2016 - 1900))
        {
            ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
            obtain_time();
        }

        // Set timezone to Екатеринбург
        setenv("TZ", "MSK-5", 1);
        tzset();
        int num = 0;
        xQueueSendFromISR(ntp_evt_queue, &num, NULL);

        while (1)
        {
            // update 'now' variable with current time
            time(&now);
            localtime_r(&now, &timeinfo);

            if (timeinfo.tm_year < (2016 - 1900))
            {
                ESP_LOGE(TAG, "The current date/time error");
            }
            else
            {
                strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                ESP_LOGI(TAG, "The current date/time: %d, %s", timeinfo.tm_year, strftime_buf);
            }
            uint32_t num = 0;

            ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
            vTaskDelay(60000*60 / portTICK_RATE_MS);
        }
    }

      void broadcast_temperature(void *pvParameters)
    {

        uint8_t amount = 0;
        uint8_t sensors = 1;
        ds18b20_addr_t addrs[sensors];
        float results[sensors];

        uint8_t GPIO_FOR_ONE_WIRE = D6;

        char msg[100];


            for (;;)
            {
                // Search all DS18B20, return its amount and feed 't' structure with result data.
                amount = ds18b20_scan_devices(GPIO_FOR_ONE_WIRE, addrs, sensors);

                if (amount < sensors)
                {
                    printf("Something is wrong, I expect to see %d sensors \nbut just %d was detected!\n", sensors, amount);
                }
                else
                {
                    for (;;)
                    {
                        sensor_worked = 1;
                        ds18b20_measure_and_read_multi(GPIO_FOR_ONE_WIRE, addrs, sensors, results);
                        for (int i = 0; i < sensors; ++i)
                        {
                            resultssensor = results[i];
                            //printf("TEMPERATURA %f \n", resultssensor);

                        }
                        sensor_worked = 0;
                        vTaskDelay(4000 / portTICK_PERIOD_MS);
                    }
                }
                vTaskDelay(4000 / portTICK_PERIOD_MS);
            }
    }

    static void save_count(void *arg)
    {
        time_t now;
        struct tm timeinfo;
        int cur_day = 0;
        int ntp_run = 0;
        int num = 0;

        while (1)
        {

            if (xQueueReceive(ntp_evt_queue, &num, portMAX_DELAY))
            {
                ntp_run = 1;
                time(&now);
                localtime_r(&now, &timeinfo);
                cur_day = timeinfo.tm_mday;
            }
            if (ntp_run) {
                time(&now);
                localtime_r(&now, &timeinfo);

                if (cur_day != timeinfo.tm_mday)
                {
                    ESP_LOGI(TAG, "DAY: %d, %d\n", cur_day, timeinfo.tm_mday);

                    cur_day = timeinfo.tm_mday;
                    sum_count[4] = sum_count[0];
                    sum_count[5] = sum_count[1];
                    sum_count[0] = 0;
                    sum_count[1] = 0;
                    ESP_LOGI(TAG, "The save count day: %d \n", timeinfo.tm_mday);
                }

                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            else
                vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    static void load_param(void *arg)
    {
        int load_param_yes = 0;
        int num = 0;
        time_t now;
        struct tm timeinfo;

        while (!load_param_yes) {
            if (xQueueReceive(ntp_evt_queue, &num, portMAX_DELAY)) {
                ESP_LOGI(TAG, " LOAD_PARAM - > ntp_evt_queue");
                time(&now);
                localtime_r(&now, &timeinfo);

                if (timeinfo.tm_mday != load_day)
                {
                    ESP_LOGI(TAG, " LOAD_PARAM - > timeinfo.tm_mday != load_day %d %d", timeinfo.tm_mday, load_day);
                    sum_count[4] = sum_count[0];
                    sum_count[5] = sum_count[1];
                    sum_count[0] = 0;
                    sum_count[1] = 0;
                }
                load_param_yes = 1;
                int num = 1000001;
                xQueueSendFromISR(gpio_evt_queue, &num, NULL);
                ESP_LOGI(TAG, "SEND LOAD PARAM \n");
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        while (1) {
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
    }

    static void load_PZE(void *arg)
    {
        bool start = true;
        float start_count = 0;

        int cur_day_night = !is_day();
        float begin_power[4] = {0, 0, 0, 0};

        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        int cur_date = timeinfo.tm_mday;


        PZE_init(100, D4); // RX = 100 -> RX неиспользуется, только TX

        while (1)
        {
            while (CountActiv)
            {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            };

            CountActiv = 1;

            PZE_voltage();

///////////// Отладка /////////////////
/*               
           
             _currentValues.energy += 1;
            _currentValues.power += 1.01;
            if (_currentValues.energy >= 99.999)
                _currentValues.energy = 0;
 */
///////////////////////////////////////

            ESP_LOGI(TAG, "_currentValues.energy = %f", _currentValues.energy);

            if (start)
            {
                 start_count = _currentValues.energy;
                 start = false;
            }
            if (_currentValues.energy < start_count)   //Счетчик PZE переполнился 99.999 и сбросился в ноль
            {
                start_count = 0;
            }
            float inc_count = _currentValues.energy - start_count;
            start_count = _currentValues.energy;

            if (cur_day_night != !is_day())
            {
                cur_day_night = !is_day();
            }
            
            sumZ_count[cur_day_night] += inc_count;
            sumZ_count[2 + cur_day_night] += inc_count;
            

            time(&now);
            localtime_r(&now, &timeinfo);
            if (cur_date != timeinfo.tm_mday) {
                sumZ_count[0] = 0;
                sumZ_count[1] = 0;
                cur_date = timeinfo.tm_mday;                
            }

            CountActiv = 0;
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    //Сохранение мгновенный показаний в кольцевой буфер
    static void Save_PZE(void *arg)
    {


        //ESP_LOGI(TAG, "Start dataZ: %d, %f \n", (int)p_dataZ.dataZ, p_dataZ.dataZ->power);

        while (1)
        {
            if ((int)p_buf[p_dataZ].power != (int)_currentValues.power || (int)p_buf[p_dataZ].voltage != (int)_currentValues.voltage)
            {
                p_dataZ ++;
                if (p_dataZ > SizeBuffer)
                {
                    p_dataZ = 0;
                    ESP_LOGI(TAG, "begin dataZ: %d \n", p_dataZ);
                }
                time_t now;
                time(&now);
                p_buf[p_dataZ].now = now;
                p_buf[p_dataZ].power = _currentValues.power;
                p_buf[p_dataZ].voltage = _currentValues.voltage;
            }

            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
    }



/////////////////////////// HTTP    ////////////////////
    

    
    
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &echo);
        httpd_register_uri_handler(server, &ctrl);
        httpd_register_uri_handler(server, &status);
        httpd_register_uri_handler(server, &power);
        httpd_register_uri_handler(server, &set);
        httpd_register_uri_handler(server, &upgrade);
        
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static httpd_handle_t server = NULL;

static void disconnect_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, 
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
    
    
    
    
    

    void app_main()
    {

        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());


        ESP_ERROR_CHECK(example_connect());

        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

        server = start_webserver();
        
        
        
//        static httpd_handle_t server = NULL;
//        esp_err_t ret = nvs_flash_init();
//        initialise_wifi(&server, true);
        init_gpio();
        ESP_LOGI(TAG, "Start: sntp_example_task");
        xTaskCreate(sntp_example_task, "sntp_example_task", 3072, NULL, 10, NULL);
        xTaskCreate(&broadcast_temperature, "broadcast_temperature", 2048, NULL, 2, NULL);
        //xTaskCreate(power_task, "power_task", 2048, NULL, 10, NULL);
        xTaskCreate(&save_count, "save_count", 2048, NULL, 2, NULL);
        xTaskCreate(&load_param, "load_param", 2048, NULL, 10, NULL);
        xTaskCreate(&load_PZE, "load_PZE", 2048, NULL, 10, NULL);
        xTaskCreate(&Save_PZE, "Save_PZE", 2048, NULL, 10, NULL);
        //xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
        hw_timer_init(hw_timer_callback1, NULL);

        //PZE_init(100, D5); // RX = 100 -> RX неиспользуется, только TX

        while (1)
        {
            vTaskDelay(10 / portTICK_RATE_MS);
        }
        //test
    }
