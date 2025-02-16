// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "../ui.h"

static const char *TAG = "NET";

int reset_flag;
int th_update_flag;
int qwnow_update_flag;
int qair_update_flag;
int qwdaily_update_flag;

static EventGroupHandle_t my_event_group;

#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_GET_SNTP_BIT           BIT1
#define WIFI_GET_DAILYWEATHER_BIT   BIT2
#define WIFI_GET_RTWEATHER_BIT      BIT3
#define WIFI_GET_WEATHER_BIT        BIT4

#define MAX_HTTP_OUTPUT_BUFFER      2048

#define QWEATHER_DAILY_URL   "https://devapi.qweather.com/v7/weather/3d?&location=101010200&key=e56d5c380aca459c9f4a0ce876d4ad3d"
#define QWEATHER_NOW_URL     "https://devapi.qweather.com/v7/weather/now?&location=101010200&key=e56d5c380aca459c9f4a0ce876d4ad3d"
#define QAIR_NOW_URL         "https://devapi.qweather.com/v7/air/now?&location=101010200&key=e56d5c380aca459c9f4a0ce876d4ad3d"

time_t now;
struct tm timeinfo;


int qwnow_temp; // 实时天气温度
int qwnow_humi; // 实时天气湿度
int qwnow_icon; // 实时天气图标
char qwnow_text[32]; // 实时天气状态

int qwdaily_tempMax;       // 当天最高温度
int qwdaily_tempMin;       // 当天最低温度
char qwdaily_sunrise[10];  // 当天日出时间
char qwdaily_sunset[10];   // 当天日落时间

int qanow_level;       // 实时空气质量等级

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                int copy_len = 0;
                if (evt->user_data) {
                    copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                    if (copy_len) {
                        memcpy(evt->user_data + output_len, evt->data, copy_len);
                    }
                } else {
                    const int buffer_len = esp_http_client_get_content_length(evt->client);
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(buffer_len);
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    copy_len = MIN(evt->data_len, (buffer_len - output_len));
                    if (copy_len) {
                        memcpy(output_buffer + output_len, evt->data, copy_len);
                    }
                }
                output_len += copy_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}

// GZIP解压函数
int gzDecompress(char *src, int srcLen, char *dst, int* dstLen)
{
    z_stream strm;
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;

    strm.avail_in = srcLen;
    strm.avail_out = *dstLen;
    strm.next_in = (Bytef *)src;
    strm.next_out = (Bytef *)dst;

    int err = -1;
    err = inflateInit2(&strm, 31); // 初始化
    if (err == Z_OK)
    {
        printf("inflateInit2 err=Z_OK\n");
        err = inflate(&strm, Z_FINISH); // 解压gzip数据
        if (err == Z_STREAM_END) // 解压成功
        { 
            printf("inflate err=Z_OK\n");
            *dstLen = strm.total_out; 
        } 
        else // 解压失败
        {
            printf("inflate err=!Z_OK\n");
        }
        inflateEnd(&strm);
    } 
    else
    {
        printf("inflateInit2 err! err=%d\n", err);
    }

    return err;
}

// 获取每日天气预报
void get_daily_weather(void)
{
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
    int client_code = 0;
    int64_t gzip_len = 0;

    esp_http_client_config_t config = {
        .url = QWEATHER_DAILY_URL,
        .event_handler = _http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .user_data = local_response_buffer,        // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        client_code = esp_http_client_get_status_code(client);
        gzip_len = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %"PRIu64, client_code, gzip_len);
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }

    if (client_code == 200)
    {
        int buffSize = MAX_HTTP_OUTPUT_BUFFER;
        char* buffData = (char*)malloc(MAX_HTTP_OUTPUT_BUFFER);
        memset(buffData, 0, MAX_HTTP_OUTPUT_BUFFER);

        int ret = gzDecompress(local_response_buffer, gzip_len, buffData, &buffSize);

        if (Z_STREAM_END == ret) { /* 解压成功 */
            printf("daily_weather decompress success\n");
            printf("buffSize = %d\n", buffSize);
            // lv_label_set_text(label_weather, "√ 天气信息获取成功");

            cJSON *root = cJSON_Parse(buffData);
            cJSON *daily = cJSON_GetObjectItem(root,"daily");

            cJSON *daily1 = cJSON_GetArrayItem(daily, 0);

            char *temp_max = cJSON_GetObjectItem(daily1,"tempMax")->valuestring;
            char *temp_min = cJSON_GetObjectItem(daily1,"tempMin")->valuestring;
            char *sunset = cJSON_GetObjectItem(daily1,"sunset")->valuestring;
            char *sunrise = cJSON_GetObjectItem(daily1,"sunrise")->valuestring;

            qwdaily_tempMax = atoi(temp_max);
            qwdaily_tempMin = atoi(temp_min);
            strcpy(qwdaily_sunrise, sunrise);
            strcpy(qwdaily_sunset, sunset);

            ESP_LOGI(TAG, "最高气温：%d", qwdaily_tempMax);
            ESP_LOGI(TAG, "最低气温：%d", qwdaily_tempMin);
            ESP_LOGI(TAG, "日出时间：%s", qwdaily_sunrise);
            ESP_LOGI(TAG, "日落时间：%s", qwdaily_sunset);

            cJSON_Delete(root);

            qwdaily_update_flag = 1;
        }
        else {
            printf("decompress failed:%d\n", ret);
        }
        free(buffData);
    }
    esp_http_client_cleanup(client);

}

// 获取实时天气信息
void get_now_weather(void)
{
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
    int client_code = 0;
    int64_t gzip_len = 0;

    esp_http_client_config_t config = {
        .url = QWEATHER_NOW_URL,
        .event_handler = _http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .user_data = local_response_buffer,        // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        client_code = esp_http_client_get_status_code(client);
        gzip_len = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %"PRIu64, client_code, gzip_len);
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }

    if(client_code == 200)
    {
        int buffSize = MAX_HTTP_OUTPUT_BUFFER;
        char* buffData = (char*)malloc(MAX_HTTP_OUTPUT_BUFFER);
        memset(buffData, 0, MAX_HTTP_OUTPUT_BUFFER);

        int ret = gzDecompress(local_response_buffer, gzip_len, buffData, &buffSize);

        if (Z_STREAM_END == ret) { /* 解压成功 */
            printf("now weather decompress success\n");
            printf("buffSize = %d\n", buffSize);

            cJSON *root = cJSON_Parse(buffData);
            cJSON *now = cJSON_GetObjectItem(root,"now");

            char *temp = cJSON_GetObjectItem(now,"temp")->valuestring;
            char *icon = cJSON_GetObjectItem(now,"icon")->valuestring;
            char *humidity = cJSON_GetObjectItem(now,"humidity")->valuestring;

            qwnow_temp = atoi(temp);
            qwnow_humi = atoi(humidity);
            qwnow_icon = atoi(icon);

            ESP_LOGI(TAG, "地区：海淀区");
            ESP_LOGI(TAG, "温度：%d", qwnow_temp);
            ESP_LOGI(TAG, "湿度：%d", qwnow_humi);
            ESP_LOGI(TAG, "图标：%d", qwnow_icon);

            cJSON_Delete(root);

            qwnow_update_flag = 1;
        }
        else {
            printf("decompress failed:%d\n", ret);
        }
        free(buffData);
    }
    esp_http_client_cleanup(client);
}

// 获取实时空气质量
void get_air_quality(void)
{
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
    int client_code = 0;
    int64_t gzip_len = 0;

    esp_http_client_config_t config = {
        .url = QAIR_NOW_URL,
        .event_handler = _http_event_handler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .user_data = local_response_buffer,        // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        client_code = esp_http_client_get_status_code(client);
        gzip_len = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %"PRIu64, client_code, gzip_len);
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }

    if(client_code == 200)
    {
        int buffSize = MAX_HTTP_OUTPUT_BUFFER;
        char* buffData = (char*)malloc(MAX_HTTP_OUTPUT_BUFFER);
        memset(buffData, 0, MAX_HTTP_OUTPUT_BUFFER);

        int ret = gzDecompress(local_response_buffer, gzip_len, buffData, &buffSize);

        if (Z_STREAM_END == ret) { /* 解压成功 */
            printf("decompress success\n");
            printf("buffSize = %d\n", buffSize);

            cJSON *root = cJSON_Parse(buffData);
            cJSON *now = cJSON_GetObjectItem(root,"now");

            char *level = cJSON_GetObjectItem(now,"level")->valuestring;

            qanow_level = atoi(level);

            ESP_LOGI(TAG, "空气质量：%d", qanow_level);

            cJSON_Delete(root);

            qair_update_flag = 1;
        }
        else {
            printf("decompress failed:%d\n", ret);
        }
        free(buffData);
    }
    esp_http_client_cleanup(client);
}

// WIFI连接 任务函数
void wifi_connect_task(void *pvParameters)
{
    my_event_group = xEventGroupCreate();


    ESP_LOGI(TAG, "Successfully Connected to AP");

    if(reset_flag == 1) // 如果是刚开机
    {
        lv_label_set_text(label_wifi, "√ WiFi连接成功");
        lv_label_set_text(label_sntp, "正在获取网络时间");
    }

    xEventGroupSetBits(my_event_group, WIFI_CONNECTED_BIT);

    vTaskDelete(NULL);
}

// 获得日期时间 任务函数
void get_time_task(void *pvParameters)
{
    xEventGroupWaitBits(my_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    // wait for time to be set
    int retry = 0;
    const int retry_count = 6;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }

    if(retry>5)
    {
        esp_restart(); // 没有获取到时间的话 重启ESP32
    }

    esp_netif_sntp_deinit();
    // 设置时区
    setenv("TZ", "CST-8", 1); 
    tzset();
    // 获取系统时间
    time(&now);
    localtime_r(&now, &timeinfo);

    if(reset_flag == 1) // 如果是刚开机
    {
        lv_label_set_text(label_sntp, "√ 网络时间获取成功");
        lv_label_set_text(label_weather, "正在获取天气信息");
    }

    xEventGroupSetBits(my_event_group, WIFI_GET_SNTP_BIT);
    
    vTaskDelete(NULL);
}

// 获取每日天气预报的任务函数
void get_dwather_task(void *pvParameters)
{
    xEventGroupWaitBits(my_event_group, WIFI_GET_SNTP_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    //vTaskDelay(pdMS_TO_TICKS(100));

    get_daily_weather();

    if (qwdaily_update_flag == 1)
    {
        qwdaily_update_flag = 0;
        xEventGroupSetBits(my_event_group, WIFI_GET_DAILYWEATHER_BIT);
    }
    else
    {
        printf("获取3日天气信息失败\n");
    }
    
    vTaskDelete(NULL);
}

// 获取实时天气信息的任务函数
void get_rtweather_task(void *pvParameters)
{

    xEventGroupWaitBits(my_event_group, WIFI_GET_DAILYWEATHER_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //vTaskDelay(pdMS_TO_TICKS(100));
    get_now_weather();
    if (qwnow_update_flag == 1) // 获取实时天气信息成功
    {
        xEventGroupSetBits(my_event_group, WIFI_GET_RTWEATHER_BIT);
    }
    vTaskDelete(NULL);
}

// 获取实时空气质量的任务函数
void get_airq_task(void *pvParameters)
{
    // 等待获取完实时空气质量 再获取空气质量
    xEventGroupWaitBits(my_event_group, WIFI_GET_RTWEATHER_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //vTaskDelay(pdMS_TO_TICKS(100));
    get_air_quality();

    if (qair_update_flag == 1)
    {
        lv_label_set_text(label_weather, "√ 天气信息获取成功");
        xEventGroupSetBits(my_event_group, WIFI_GET_WEATHER_BIT);
    }

    vTaskDelete(NULL);
}

// 主界面各值更新函数
void value_update_cb(lv_timer_t * timer)
{
    // 更新日期 星期 时分秒
    time(&now);
    localtime_r(&now, &timeinfo);
    lv_label_set_text_fmt(led_time_label, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    lv_label_set_text_fmt(date_label, "%d年%02d月%02d日", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday);
    lv_week_show();

    // 日出日落时间交替显示 每隔5秒切换
    if (timeinfo.tm_sec%10 == 0) 
        lv_label_set_text_fmt(sunset_label, "日落 %s", qwdaily_sunset);
    else if(timeinfo.tm_sec%10 == 5)
        lv_label_set_text_fmt(sunset_label, "日出 %s", qwdaily_sunrise);

    // 更新温湿度 
    if(th_update_flag == 1)
    {
        th_update_flag = 0;
        lv_label_set_text_fmt(indoor_temp_label, "%d℃", temp_value);
        lv_label_set_text_fmt(indoor_humi_label, "%d%%", humi_value);
    }
    // 更新实时天气
    if(qwnow_update_flag == 1)
    {
        qwnow_update_flag = 0;
        lv_qweather_icon_show(); // 更新天气图标
        lv_label_set_text_fmt(qweather_text_label, "%s", qwnow_text); // 更新天气情况文字描述
        lv_label_set_text_fmt(outdoor_temp_label, "%d℃", qwnow_temp); // 更新室外温度
        lv_label_set_text_fmt(outdoor_humi_label, "%d%%", qwnow_humi); // 更新室外湿度
    }
    // 更新空气质量
    if(qair_update_flag ==1)
    {
        qair_update_flag = 0;
        lv_qair_level_show();
    }
    // 更新每日天气
    if(qwdaily_update_flag == 1)
    {
        qwdaily_update_flag = 0;
        lv_label_set_text_fmt(qweather_temp_label, "%d~%d℃", qwdaily_tempMin, qwdaily_tempMax); // 温度范围
    }
}

// 主界面 任务函数
void weathermain_page_task(void *pvParameters)
{
    int tm_cnt1 = 0;
    int tm_cnt2 = 0;

    xEventGroupWaitBits(my_event_group, WIFI_GET_WEATHER_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    example_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_clean(lv_scr_act());
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_main_page();

    th_update_flag = 0;
    qwnow_update_flag = 0;
    qair_update_flag = 0;
    qwdaily_update_flag = 0;

    my_lv_timer = lv_timer_create(value_update_cb, 1000, NULL);  // 创建一个lv_timer 每秒更新一次数据

    reset_flag = 0; // 标记开机完成

    while (1)
    {
        tm_cnt1++;
        if (tm_cnt1 > 1800) // 30分钟更新一次实时天气和实时空气质量
        {
            tm_cnt1 = 0; // 计数清零
            example_connect();  // 连接wifi
            get_now_weather();  // 获取实时天气信息
            get_air_quality();  // 获取实时空气质量
            tm_cnt2++;
            if (tm_cnt2 > 1) // 60分钟更新一次每日天气
            {
                tm_cnt2 = 0;
                get_daily_weather(); // 获取每日天气信息
            }
            example_disconnect(); // 断开wifi
            printf("weather update time:%02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(screen_state == start_page)
        {
            break;
            ESP_LOGI(TAG,"返回开始界面");
        }    
    }
    
    vTaskDelete(NULL);
}

// 开机界面
void lv_gui_start(void)
{
    // 设置背景色
    lv_obj_set_style_bg_color(ui_netapp, lv_color_hex(0x080808), 0);

    // // 显示太空人GIF图片
    // lv_obj_t *gif_start = lv_gif_create(ui_netapp);
    // lv_gif_set_src(gif_start, &image_taikong);
    // lv_obj_align(gif_start, LV_ALIGN_TOP_MID, 0, 20);

    // 连接wifi
    label_wifi = lv_label_create(ui_netapp);
    lv_label_set_text(label_wifi, "正在连接WiFi");
    lv_obj_set_style_text_font(label_wifi, &font_alipuhui, 0);
    lv_obj_set_style_text_color(ui_netapp, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_set_pos(label_wifi, 85 ,110);

    // 获取网络时间
    label_sntp = lv_label_create(ui_netapp);
    lv_label_set_text(label_sntp, "");
    lv_obj_set_style_text_font(label_sntp, &font_alipuhui, 0);
    lv_obj_set_style_text_color(ui_netapp, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_set_pos(label_sntp, 70 ,135);

    // 获取天气信息
    label_weather = lv_label_create(ui_netapp);
    lv_label_set_text(label_weather, "");
    lv_obj_set_style_text_font(label_weather, &font_alipuhui, 0);
    lv_obj_set_style_text_color(ui_netapp, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_set_pos(label_weather, 70 ,160);
}

// 显示天气图标
void lv_qweather_icon_show(void)
{
    switch (qwnow_icon)
    {
        case 100: lv_label_set_text(qweather_icon_label, "\xEF\x84\x81"); strcpy(qwnow_text, "晴"); break;
        case 101: lv_label_set_text(qweather_icon_label, "\xEF\x84\x82"); strcpy(qwnow_text, "多云"); break;
        case 102: lv_label_set_text(qweather_icon_label, "\xEF\x84\x83"); strcpy(qwnow_text, "少云"); break;
        case 103: lv_label_set_text(qweather_icon_label, "\xEF\x84\x84"); strcpy(qwnow_text, "晴间多云"); break;
        case 104: lv_label_set_text(qweather_icon_label, "\xEF\x84\x85"); strcpy(qwnow_text, "阴"); break;
        case 150: lv_label_set_text(qweather_icon_label, "\xEF\x84\x86"); strcpy(qwnow_text, "晴"); break;
        case 151: lv_label_set_text(qweather_icon_label, "\xEF\x84\x87"); strcpy(qwnow_text, "多云"); break;
        case 152: lv_label_set_text(qweather_icon_label, "\xEF\x84\x88"); strcpy(qwnow_text, "少云"); break;
        case 153: lv_label_set_text(qweather_icon_label, "\xEF\x84\x89"); strcpy(qwnow_text, "晴间多云"); break;
        case 300: lv_label_set_text(qweather_icon_label, "\xEF\x84\x8A"); strcpy(qwnow_text, "阵雨"); break;
        case 301: lv_label_set_text(qweather_icon_label, "\xEF\x84\x8B"); strcpy(qwnow_text, "强阵雨"); break;
        case 302: lv_label_set_text(qweather_icon_label, "\xEF\x84\x8C"); strcpy(qwnow_text, "雷阵雨"); break;
        case 303: lv_label_set_text(qweather_icon_label, "\xEF\x84\x8D"); strcpy(qwnow_text, "强雷阵雨"); break;
        case 304: lv_label_set_text(qweather_icon_label, "\xEF\x84\x8E"); strcpy(qwnow_text, "雷阵雨伴有冰雹"); break;
        case 305: lv_label_set_text(qweather_icon_label, "\xEF\x84\x8F"); strcpy(qwnow_text, "小雨"); break;
        case 306: lv_label_set_text(qweather_icon_label, "\xEF\x84\x90"); strcpy(qwnow_text, "中雨"); break;
        case 307: lv_label_set_text(qweather_icon_label, "\xEF\x84\x91"); strcpy(qwnow_text, "大雨"); break;
        case 308: lv_label_set_text(qweather_icon_label, "\xEF\x84\x92"); strcpy(qwnow_text, "极端降雨"); break;
        case 309: lv_label_set_text(qweather_icon_label, "\xEF\x84\x93"); strcpy(qwnow_text, "毛毛雨"); break;
        case 310: lv_label_set_text(qweather_icon_label, "\xEF\x84\x94"); strcpy(qwnow_text, "暴雨"); break;
        case 311: lv_label_set_text(qweather_icon_label, "\xEF\x84\x95"); strcpy(qwnow_text, "大暴雨"); break;
        case 312: lv_label_set_text(qweather_icon_label, "\xEF\x84\x96"); strcpy(qwnow_text, "特大暴雨"); break;
        case 313: lv_label_set_text(qweather_icon_label, "\xEF\x84\x97"); strcpy(qwnow_text, "冻雨"); break;
        case 314: lv_label_set_text(qweather_icon_label, "\xEF\x84\x98"); strcpy(qwnow_text, "小到中雨"); break;
        case 315: lv_label_set_text(qweather_icon_label, "\xEF\x84\x99"); strcpy(qwnow_text, "中到大雨"); break;
        case 316: lv_label_set_text(qweather_icon_label, "\xEF\x84\x9A"); strcpy(qwnow_text, "大到暴雨"); break;
        case 317: lv_label_set_text(qweather_icon_label, "\xEF\x84\x9B"); strcpy(qwnow_text, "暴雨到大暴雨"); break;
        case 318: lv_label_set_text(qweather_icon_label, "\xEF\x84\x9C"); strcpy(qwnow_text, "大暴雨到特大暴雨"); break;
        case 350: lv_label_set_text(qweather_icon_label, "\xEF\x84\x9D"); strcpy(qwnow_text, "阵雨"); break;
        case 351: lv_label_set_text(qweather_icon_label, "\xEF\x84\x9E"); strcpy(qwnow_text, "强阵雨"); break;
        case 399: lv_label_set_text(qweather_icon_label, "\xEF\x84\x9F"); strcpy(qwnow_text, "雨"); break;
        case 400: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA0"); strcpy(qwnow_text, "小雪"); break;
        case 401: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA1"); strcpy(qwnow_text, "中雪"); break;
        case 402: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA2"); strcpy(qwnow_text, "大雪"); break;
        case 403: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA3"); strcpy(qwnow_text, "暴雪"); break;
        case 404: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA4"); strcpy(qwnow_text, "雨夹雪"); break;
        case 405: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA5"); strcpy(qwnow_text, "雨雪天气"); break;
        case 406: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA6"); strcpy(qwnow_text, "阵雨夹雪"); break;
        case 407: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA7"); strcpy(qwnow_text, "阵雪"); break;
        case 408: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA8"); strcpy(qwnow_text, "小到中雪"); break;
        case 409: lv_label_set_text(qweather_icon_label, "\xEF\x84\xA9"); strcpy(qwnow_text, "中到大雪"); break;
        case 410: lv_label_set_text(qweather_icon_label, "\xEF\x84\xAA"); strcpy(qwnow_text, "大到暴雪"); break;
        case 456: lv_label_set_text(qweather_icon_label, "\xEF\x84\xAB"); strcpy(qwnow_text, "阵雨夹雪"); break;
        case 457: lv_label_set_text(qweather_icon_label, "\xEF\x84\xAC"); strcpy(qwnow_text, "阵雪"); break;
        case 499: lv_label_set_text(qweather_icon_label, "\xEF\x84\xAD"); strcpy(qwnow_text, "雪"); break;
        case 500: lv_label_set_text(qweather_icon_label, "\xEF\x84\xAE"); strcpy(qwnow_text, "薄雾"); break;
        case 501: lv_label_set_text(qweather_icon_label, "\xEF\x84\xAF"); strcpy(qwnow_text, "雾"); break;
        case 502: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB0"); strcpy(qwnow_text, "霾"); break;
        case 503: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB1"); strcpy(qwnow_text, "扬沙"); break;
        case 504: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB2"); strcpy(qwnow_text, "浮尘"); break;
        case 507: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB3"); strcpy(qwnow_text, "沙尘暴"); break;
        case 508: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB4"); strcpy(qwnow_text, "强沙尘暴"); break;
        case 509: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB5"); strcpy(qwnow_text, "浓雾"); break;
        case 510: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB6"); strcpy(qwnow_text, "强浓雾"); break;
        case 511: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB7"); strcpy(qwnow_text, "中度霾"); break;
        case 512: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB8"); strcpy(qwnow_text, "重度霾"); break;
        case 513: lv_label_set_text(qweather_icon_label, "\xEF\x84\xB9"); strcpy(qwnow_text, "严重霾"); break;
        case 514: lv_label_set_text(qweather_icon_label, "\xEF\x84\xBA"); strcpy(qwnow_text, "大雾"); break;
        case 515: lv_label_set_text(qweather_icon_label, "\xEF\x84\xBB"); strcpy(qwnow_text, "特强浓雾"); break;
        case 900: lv_label_set_text(qweather_icon_label, "\xEF\x85\x84"); strcpy(qwnow_text, "热"); break;
        case 901: lv_label_set_text(qweather_icon_label, "\xEF\x85\x85"); strcpy(qwnow_text, "冷"); break;
    
        default:
            printf("ICON_CODE:%d\n", qwnow_icon);
            lv_label_set_text(qweather_icon_label, "\xEF\x85\x86");
            strcpy(qwnow_text, "未知天气");
            break;
    }
}

// 显示星期几
void lv_week_show(void)
{
    switch (timeinfo.tm_wday)
    {
        case 0: lv_label_set_text(week_label, "星期日"); break;
        case 1: lv_label_set_text(week_label, "星期一"); break;
        case 2: lv_label_set_text(week_label, "星期二"); break;
        case 3: lv_label_set_text(week_label, "星期三"); break;
        case 4: lv_label_set_text(week_label, "星期四"); break; 
        case 5: lv_label_set_text(week_label, "星期五"); break;
        case 6: lv_label_set_text(week_label, "星期六"); break;
        default: lv_label_set_text(week_label, "星期日"); break;
    }
}

// 显示空气质量
void lv_qair_level_show(void)
{
    switch (qanow_level)
    {
        case 1: 
            lv_label_set_text(qair_level_label, "优"); 
            lv_obj_set_style_bg_color(qair_level_obj, lv_palette_main(LV_PALETTE_GREEN), 0); 
            lv_obj_set_style_text_color(qair_level_label, lv_color_hex(0xFFFFFF), 0);
            break;
        case 2: 
            lv_label_set_text(qair_level_label, "良"); 
            lv_obj_set_style_bg_color(qair_level_obj, lv_palette_main(LV_PALETTE_YELLOW), 0); 
            lv_obj_set_style_text_color(qair_level_label, lv_color_hex(0x000000), 0);
            break;
        case 3: 
            lv_label_set_text(qair_level_label, "轻");
            lv_obj_set_style_bg_color(qair_level_obj, lv_palette_main(LV_PALETTE_ORANGE), 0); 
            lv_obj_set_style_text_color(qair_level_label, lv_color_hex(0xFFFFFF), 0); 
            break;
        case 4: 
            lv_label_set_text(qair_level_label, "中"); 
            lv_obj_set_style_bg_color(qair_level_obj, lv_palette_main(LV_PALETTE_RED), 0); 
            lv_obj_set_style_text_color(qair_level_label, lv_color_hex(0xFFFFFF), 0);
            break; 
        case 5: 
            lv_label_set_text(qair_level_label, "重"); 
            lv_obj_set_style_bg_color(qair_level_obj, lv_palette_main(LV_PALETTE_PURPLE), 0); 
            lv_obj_set_style_text_color(qair_level_label, lv_color_hex(0xFFFFFF), 0);
            break;
        case 6: 
            lv_label_set_text(qair_level_label, "严"); 
            lv_obj_set_style_bg_color(qair_level_obj, lv_palette_main(LV_PALETTE_BROWN), 0); 
            lv_obj_set_style_text_color(qair_level_label, lv_color_hex(0xFFFFFF), 0);
            break;
        default: 
            lv_label_set_text(qair_level_label, "未"); 
            lv_obj_set_style_bg_color(qair_level_obj, lv_palette_main(LV_PALETTE_GREEN), 0); 
            lv_obj_set_style_text_color(qair_level_label, lv_color_hex(0xFFFFFF), 0);
            break;
    }
}

// 主界面
void lv_main_page(void)
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0); // 修改背景为黑色

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_opa( &style, LV_OPA_COVER );
    lv_style_set_bg_color(&style, lv_color_hex(0x00BFFF));
    lv_style_set_border_width(&style, 0);
    lv_style_set_pad_all(&style, 10);
    lv_style_set_width(&style, 320);  // 设置宽
    lv_style_set_height(&style, 240); // 设置高

    /*Create an object with the new style*/
// lv_obj_t * obj = lv_obj_create(lv_scr_act());
    lv_obj_add_style(ui_netapp, &style, 0);

    // 显示地理位置
    lv_obj_t * addr_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(addr_label, &font_alipuhui, 0);
    lv_label_set_text(addr_label, "北京市|海淀区");
    lv_obj_align_to(addr_label, ui_netapp, LV_ALIGN_TOP_LEFT, 0, 0);

    // 显示年月日
    date_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(date_label, &font_alipuhui, 0);
    lv_label_set_text_fmt(date_label, "%d年%02d月%02d日", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday);
    lv_obj_align_to(date_label, ui_netapp, LV_ALIGN_TOP_RIGHT, 0, 0);

    // 显示分割线
    lv_obj_t * above_bar = lv_bar_create(ui_netapp);
    lv_obj_set_size(above_bar, 300, 3);
    lv_obj_set_pos(above_bar, 0 , 30);
    lv_bar_set_value(above_bar, 100, LV_ANIM_OFF);

    // 显示天气图标
    qweather_icon_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(qweather_icon_label, &font_qweather, 0);
    lv_obj_set_pos(qweather_icon_label, 0 , 40);
    lv_qweather_icon_show();

    // 显示空气质量
    static lv_style_t qair_level_style;
    lv_style_init(&qair_level_style);
    lv_style_set_radius(&qair_level_style, 10);  // 设置圆角半径
    lv_style_set_bg_color(&qair_level_style, lv_palette_main(LV_PALETTE_GREEN)); // 绿色
    lv_style_set_text_color(&qair_level_style, lv_color_hex(0xffffff)); // 白色
    lv_style_set_border_width(&qair_level_style, 0);
    lv_style_set_pad_all(&qair_level_style, 0);
    lv_style_set_width(&qair_level_style, 50);  // 设置宽
    lv_style_set_height(&qair_level_style, 26); // 设置高

    qair_level_obj = lv_obj_create(ui_netapp);
    lv_obj_add_style(qair_level_obj, &qair_level_style, 0);
    lv_obj_align_to(qair_level_obj, qweather_icon_label, LV_ALIGN_OUT_RIGHT_TOP, 5, 0);

    qair_level_label = lv_label_create(qair_level_obj);
    lv_obj_set_style_text_font(qair_level_label, &font_alipuhui, 0);
    lv_obj_align(qair_level_label, LV_ALIGN_CENTER, 0, 0);
    lv_qair_level_show();

    // 显示当天室外温度范围
    qweather_temp_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(qweather_temp_label, &font_alipuhui, 0);
    lv_label_set_text_fmt(qweather_temp_label, "%d~%d℃", qwdaily_tempMin, qwdaily_tempMax);
    lv_obj_align_to(qweather_temp_label, qweather_icon_label, LV_ALIGN_OUT_RIGHT_MID, 5, 5);

    // 显示当天天气图标代表的天气状况
    qweather_text_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(qweather_text_label, &font_alipuhui, 0);
    lv_label_set_long_mode(qweather_text_label, LV_LABEL_LONG_SCROLL_CIRCULAR);     /*Circular scroll*/
    lv_obj_set_width(qweather_text_label, 80);
    lv_label_set_text_fmt(qweather_text_label, "%s", qwnow_text);
    lv_obj_align_to(qweather_text_label, qweather_icon_label, LV_ALIGN_OUT_RIGHT_BOTTOM, 5, 0);
    
    // 显示时间  小时:分钟:秒钟
    led_time_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(led_time_label, &font_led, 0);
    lv_label_set_text_fmt(led_time_label, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    lv_obj_set_pos(led_time_label, 142, 42);

    // 显示星期几
    week_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(week_label, &font_alipuhui, 0);
    lv_obj_align_to(week_label, led_time_label, LV_ALIGN_OUT_BOTTOM_RIGHT, -10, 6);
    lv_week_show();

    // 显示日落时间 
    sunset_label = lv_label_create(ui_netapp);
    lv_obj_set_style_text_font(sunset_label, &font_alipuhui, 0);
    lv_label_set_text_fmt(sunset_label, "日落 %s", qwdaily_sunset);
    lv_obj_set_pos(sunset_label, 200 , 103);

    // 显示分割线
    lv_obj_t * below_bar = lv_bar_create(ui_netapp);
    lv_obj_set_size(below_bar, 300, 3);
    lv_obj_set_pos(below_bar, 0, 130);
    lv_bar_set_value(below_bar, 100, LV_ANIM_OFF);

    // 显示室外温湿度
    static lv_style_t outdoor_style;
    lv_style_init(&outdoor_style);
    lv_style_set_radius(&outdoor_style, 10);  // 设置圆角半径
    lv_style_set_bg_color(&outdoor_style, lv_color_hex(0xd8b010)); // 
    lv_style_set_text_color(&outdoor_style, lv_color_hex(0xffffff)); // 白色
    lv_style_set_border_width(&outdoor_style, 0);
    lv_style_set_pad_all(&outdoor_style, 5);
    lv_style_set_width(&outdoor_style, 100);  // 设置宽
    lv_style_set_height(&outdoor_style, 80); // 设置高

    lv_obj_t * outdoor_obj = lv_obj_create(ui_netapp);
    lv_obj_add_style(outdoor_obj, &outdoor_style, 0);
    lv_obj_align(outdoor_obj, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    lv_obj_t *outdoor_th_label = lv_label_create(outdoor_obj);
    lv_obj_set_style_text_font(outdoor_th_label, &font_alipuhui, 0);
    lv_label_set_text(outdoor_th_label, "室外");
    lv_obj_align(outdoor_th_label, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *temp_symbol_label1 = lv_label_create(outdoor_obj);
    lv_obj_set_style_text_font(temp_symbol_label1, &font_myawesome, 0);
    lv_label_set_text(temp_symbol_label1, "\xEF\x8B\x88");  // 显示温度图标
    lv_obj_align(temp_symbol_label1, LV_ALIGN_LEFT_MID, 10, 0);

    outdoor_temp_label = lv_label_create(outdoor_obj);
    lv_obj_set_style_text_font(outdoor_temp_label, &font_alipuhui, 0);
    lv_label_set_text_fmt(outdoor_temp_label, "%d℃", qwnow_temp);
    lv_obj_align_to(outdoor_temp_label, temp_symbol_label1, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lv_obj_t *humi_symbol_label1 = lv_label_create(outdoor_obj);
    lv_obj_set_style_text_font(humi_symbol_label1, &font_myawesome, 0);
    lv_label_set_text(humi_symbol_label1, "\xEF\x81\x83");  // 显示湿度图标
    lv_obj_align(humi_symbol_label1, LV_ALIGN_BOTTOM_LEFT, 10, 0);

    outdoor_humi_label = lv_label_create(outdoor_obj);
    lv_obj_set_style_text_font(outdoor_humi_label, &font_alipuhui, 0);
    lv_label_set_text_fmt(outdoor_humi_label, "%d%%", qwnow_humi);
    lv_obj_align_to(outdoor_humi_label, humi_symbol_label1, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    // 显示室内温湿度
    static lv_style_t indoor_style;
    lv_style_init(&indoor_style);
    lv_style_set_radius(&indoor_style, 10);  // 设置圆角半径
    lv_style_set_bg_color(&indoor_style, lv_color_hex(0xfe6464)); //
    lv_style_set_text_color(&indoor_style, lv_color_hex(0xffffff)); // 白色
    lv_style_set_border_width(&indoor_style, 0);
    lv_style_set_pad_all(&indoor_style, 5);
    lv_style_set_width(&indoor_style, 100);  // 设置宽
    lv_style_set_height(&indoor_style, 80); // 设置高

    lv_obj_t * indoor_obj = lv_obj_create(ui_netapp);
    lv_obj_add_style(indoor_obj, &indoor_style, 0);
    lv_obj_align(indoor_obj, LV_ALIGN_BOTTOM_MID, 10, 0);

    lv_obj_t *indoor_th_label = lv_label_create(indoor_obj);
    lv_obj_set_style_text_font(indoor_th_label, &font_alipuhui, 0);
    lv_label_set_text(indoor_th_label, "室内");
    lv_obj_align(indoor_th_label, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *temp_symbol_label2 = lv_label_create(indoor_obj);
    lv_obj_set_style_text_font(temp_symbol_label2, &font_myawesome, 0);
    lv_label_set_text(temp_symbol_label2, "\xEF\x8B\x88");  // 温度图标
    lv_obj_align(temp_symbol_label2, LV_ALIGN_LEFT_MID, 10, 0);

    indoor_temp_label = lv_label_create(indoor_obj);
    lv_obj_set_style_text_font(indoor_temp_label, &font_alipuhui, 0);
    lv_label_set_text_fmt(indoor_temp_label, "%d℃", temp_value);
    lv_obj_align_to(indoor_temp_label, temp_symbol_label2, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lv_obj_t *humi_symbol_label2 = lv_label_create(indoor_obj);
    lv_obj_set_style_text_font(humi_symbol_label2, &font_myawesome, 0);
    lv_label_set_text(humi_symbol_label2, "\xEF\x81\x83");  // 湿度图标
    lv_obj_align(humi_symbol_label2, LV_ALIGN_BOTTOM_LEFT, 10, 0);

    indoor_humi_label = lv_label_create(indoor_obj);
    lv_obj_set_style_text_font(indoor_humi_label, &font_alipuhui, 0);
    lv_label_set_text_fmt(indoor_humi_label, "%d%%", humi_value);
    lv_obj_align_to(indoor_humi_label, humi_symbol_label2, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    // 显示太空人
    // lv_obj_t *tk_gif = lv_gif_create(ui_netapp);
    // lv_gif_set_src(tk_gif, &image_taikong);
    // lv_obj_align(tk_gif, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

}

void ui_netapp_screen_init(void)
{
    ESP_LOGI(TAG, "网络应用界面初始化");
    reset_flag = 1;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    /* 创建页面 */
    ui_netapp = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_netapp, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_netapp, lv_color_hex(0x5295B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_netapp, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_add_event_cb(ui_netapp, ui_event_netapp, LV_EVENT_ALL, NULL);
    
}
