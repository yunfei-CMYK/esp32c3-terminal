// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "../ui.h"
#include "esp_system.h"
#include <string.h>

#define DEFAULT_SCAN_LIST_SIZE 10

bool wifiswitch_flag = false;

esp_netif_t *sta_netif;

static char *ssid_list[DEFAULT_SCAN_LIST_SIZE];

static const char *TAG = "WiFi_Scan";

static void wifi_scan(void)
{

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_scan_start(NULL, true);
    ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
    // 创建一个包含所有 SSID 并以换行符分隔的字符串
    for (int i = 0; i < number; i++)
    {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        ssid_list[i] = (char *)ap_info[i].ssid;
        lv_dropdown_add_option(ui_wifiDropdown, ssid_list[i], i);
    }
}

static void nvs_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    if (lv_obj_has_state(sw, LV_STATE_CHECKED))
    {
        ESP_LOGI(TAG, "Wifi scan started");
        wifiswitch_flag = true;

        nvs_init();
        wifi_scan();
        // my_lv_timer = lv_timer_create(wifi_scan, 500, NULL);
    }
    else
    {
        ESP_LOGI(TAG, "Wifi scan stopped");
        wifiswitch_flag = false;
        lv_dropdown_clear_options(ui_wifiDropdown);
    }
}

void ui_wifipage_screen_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    ui_wifipage = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_wifipage, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_wifipage, lv_color_hex(0x5295B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_wifipage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_wifititle = lv_label_create(ui_wifipage);
    lv_obj_set_width(ui_wifititle, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_wifititle, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_wifititle, -115);
    lv_obj_set_y(ui_wifititle, 7);
    lv_obj_set_align(ui_wifititle, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_wifititle, "WiFi连接");
    lv_obj_set_style_text_color(ui_wifititle, lv_color_hex(0x353232), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_wifititle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_wifititle, &ui_font_Terminal22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_wifiDropdown = lv_dropdown_create(ui_wifipage);
    lv_obj_set_width(ui_wifiDropdown, 208);
    lv_obj_set_height(ui_wifiDropdown, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_wifiDropdown, 30);
    lv_obj_set_y(ui_wifiDropdown, -59);
    lv_obj_set_align(ui_wifiDropdown, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_wifiDropdown, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    if (!wifiswitch_flag)
    {
        lv_dropdown_clear_options(ui_wifiDropdown);
    }

    // wifi开关
    ui_wifiswitch = lv_switch_create(ui_wifipage);
    lv_obj_set_width(ui_wifiswitch, 50);
    lv_obj_set_height(ui_wifiswitch, 25);
    lv_obj_set_x(ui_wifiswitch, -47);
    lv_obj_set_y(ui_wifiswitch, -102);
    lv_obj_set_align(ui_wifiswitch, LV_ALIGN_CENTER);
    lv_obj_add_event_cb(ui_wifiswitch, switch_cb, LV_EVENT_CLICKED, NULL);

    ui_usefulwifilabel = lv_label_create(ui_wifipage);
    lv_obj_set_width(ui_usefulwifilabel, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_usefulwifilabel, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_usefulwifilabel, -114);
    lv_obj_set_y(ui_usefulwifilabel, -58);
    lv_obj_set_align(ui_usefulwifilabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_usefulwifilabel, "可用WiFi：");
    lv_obj_set_style_text_color(ui_usefulwifilabel, lv_color_hex(0x33373A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_usefulwifilabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_usefulwifilabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_passwordlabel = lv_label_create(ui_wifipage);
    lv_obj_set_width(ui_passwordlabel, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_passwordlabel, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_passwordlabel, -114);
    lv_obj_set_y(ui_passwordlabel, -22);
    lv_obj_set_align(ui_passwordlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_passwordlabel, "密        码：");
    lv_obj_set_style_text_color(ui_passwordlabel, lv_color_hex(0x313233), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_passwordlabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_passwordlabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_passwordTextArea = lv_textarea_create(ui_wifipage);
    lv_obj_set_width(ui_passwordTextArea, 209);
    lv_obj_set_height(ui_passwordTextArea, 39);
    lv_obj_set_x(ui_passwordTextArea, 29);
    lv_obj_set_y(ui_passwordTextArea, -22);
    lv_obj_set_align(ui_passwordTextArea, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_passwordTextArea, "Placeholder...");

    ui_linkbtn = lv_btn_create(ui_wifipage);
    lv_obj_set_width(ui_linkbtn, 84);
    lv_obj_set_height(ui_linkbtn, 24);
    lv_obj_set_x(ui_linkbtn, 90);
    lv_obj_set_y(ui_linkbtn, -101);
    lv_obj_set_align(ui_linkbtn, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_linkbtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_linkbtn, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_bg_color(ui_linkbtn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_linkbtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_linklabel = lv_label_create(ui_linkbtn);
    lv_obj_set_width(ui_linklabel, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_linklabel, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_linklabel, -1);
    lv_obj_set_y(ui_linklabel, 1);
    lv_obj_set_align(ui_linklabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_linklabel, "连接");
    lv_obj_set_style_text_color(ui_linklabel, lv_color_hex(0x394952), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_linklabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_linklabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_wifiKeyboard = lv_keyboard_create(ui_wifipage);
    lv_obj_set_width(ui_wifiKeyboard, 300);
    lv_obj_set_height(ui_wifiKeyboard, 120);
    lv_obj_set_x(ui_wifiKeyboard, 1);
    lv_obj_set_y(ui_wifiKeyboard, 57);
    lv_obj_set_align(ui_wifiKeyboard, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_wifiKeyboard, lv_color_hex(0xABD1E3), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_wifiKeyboard, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_wifiKeyboard, ui_event_wifiKeyboard, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_wifipage, ui_event_wifipage, LV_EVENT_ALL, NULL);
}
