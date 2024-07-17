// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#ifndef _TERMINAL_UI_H
#define _TERMINAL_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"

#include "../main/qmi8658c.h"
#include "../main/qmc5883l.h"
#include "../main/gxhtc3.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"

#include "math.h"

#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

typedef enum{
    start_page,
    function_page,
    wifi_page,
    tha_page,
    game_page,
    serialport_page,
    mpu_page,
    campass_page,
    setting_page,
}menustate;

extern menustate screen_state;


void showanim_Animation(lv_obj_t * TargetObject, int delay);
// SCREEN: ui_Startpage
void ui_Startpage_screen_init(void);
void ui_event_Startpage(lv_event_t * e);
extern lv_obj_t * ui_Startpage;
extern lv_obj_t * ui_MainTitle;
void ui_event_Entry(lv_event_t * e);
extern lv_obj_t * ui_Entry;
extern lv_obj_t * ui_Label3;
void ui_event_wifibtn(lv_event_t * e);
extern lv_obj_t * ui_wifibtn;
// SCREEN: ui_Mainpage
void ui_Mainpage_screen_init(void);
void ui_event_Mainpage(lv_event_t * e);
extern lv_obj_t * ui_Mainpage;
void ui_event_weather(lv_event_t * e);
extern lv_obj_t * ui_weather;
extern lv_obj_t * ui_weatherpng;
void ui_event_game(lv_event_t * e);
extern lv_obj_t * ui_game;
extern lv_obj_t * ui_gamepng;
void ui_event_port(lv_event_t * e);
extern lv_obj_t * ui_port;
extern lv_obj_t * ui_portpng;
void ui_event_mpu(lv_event_t * e);
extern lv_obj_t * ui_mpu;
extern lv_obj_t * ui_mpupng;
void ui_event_campass(lv_event_t * e);
extern lv_obj_t * ui_campass;
extern lv_obj_t * ui_campasspng;
void ui_event_setting(lv_event_t * e);
extern lv_obj_t * ui_setting;
extern lv_obj_t * ui_setpng;
extern lv_obj_t * ui_weatherlabel;
extern lv_obj_t * ui_settinglabel;
extern lv_obj_t * ui_campasslabel;
extern lv_obj_t * ui_mpulabel;
extern lv_obj_t * ui_gamelabel;
extern lv_obj_t * ui_voicelabel;
extern lv_obj_t * ui_functionmenutitle;
// SCREEN: ui_weatherpage
void ui_weatherpage_screen_init(void);
void ui_event_weatherpage(lv_event_t * e);
void thv_update_cb(lv_timer_t * timer);
extern lv_obj_t * ui_weatherpage;
extern lv_obj_t * ui_Label12;

extern int temp_value;
extern int humi_value;

extern float temp,humi;
void get_th_task(void *args);

// SCREEN: ui_gamepage
void ui_gamepage_screen_init(void);
void ui_event_gamepage(lv_event_t * e);
void game_update_cb(lv_timer_t *timer);
extern lv_obj_t * ui_gamepage;
extern lv_obj_t * ui_Label13;
// SCREEN: ui_portpage
void ui_portpage_screen_init(void);
void ui_event_portpage(lv_event_t * e);
void task_receive_data(void *arg);
extern lv_obj_t * ui_portpage;
extern lv_obj_t * ui_Label14;
extern lv_obj_t * ui_inputtext;
extern lv_obj_t * ui_sendbtn;
extern lv_obj_t * ui_sendlabel;
extern lv_obj_t * ui_outputtext;
extern lv_obj_t * ui_baudrateDropdown;
extern lv_obj_t * ui_clearbtn;
extern lv_obj_t * ui_clearlabel;
extern lv_obj_t * ui_inputkeyboard;
// SCREEN: ui_mpupage
void ui_mpupage_screen_init(void);
void ui_event_mpupage(lv_event_t * e);
void att_update_cb(lv_timer_t * timer);
extern lv_obj_t * ui_mpupage;
extern lv_obj_t * ui_Label15;
// SCREEN: ui_campasspage
void ui_campasspage_screen_init(void);
void ui_event_campasspage(lv_event_t * e);
void comp_update_cb(lv_timer_t * timer);
extern lv_obj_t * ui_campasspage;
extern lv_obj_t * ui_Label16;
// SCREEN: ui_settingpage
void ui_settingpage_screen_init(void);
void ui_event_settingpage(lv_event_t * e);
extern lv_obj_t * ui_settingpage;
extern lv_obj_t * ui_Label17;
extern lv_obj_t * ui_Container1;
extern lv_obj_t * ui_Label1;
// SCREEN: ui_wifipage
void ui_wifipage_screen_init(void);
void ui_event_wifipage(lv_event_t * e);
extern lv_obj_t * ui_wifipage;
extern lv_obj_t * ui_wifititle;
extern lv_obj_t * ui_wifiDropdown;
extern lv_obj_t * ui_wifiswitch;
extern lv_obj_t * ui_wifiled;
extern lv_obj_t * ui_usefulwifilabel;
extern lv_obj_t * ui_passwordlabel;
extern lv_obj_t * ui_passwordTextArea;
extern lv_obj_t * ui_linkbtn;
extern lv_obj_t * ui_linklabel;
void ui_event_wifiKeyboard(lv_event_t * e);
extern lv_obj_t * ui_wifiKeyboard;
extern lv_obj_t * ui____initial_actions0;


extern int strength;
extern float bg_duty;
extern lv_timer_t *my_lv_timer;
extern bool wifiswitch_flag;

LV_IMG_DECLARE(ui_img_wifi_png);    // assets/Wifi.png
LV_IMG_DECLARE(ui_img_944254084);    // assets/天气.png
LV_IMG_DECLARE(ui_img_227306801);    // assets/游戏手柄.png
LV_IMG_DECLARE(ui_img_2114266571);    // assets/串口监视器.png
LV_IMG_DECLARE(ui_img_613467726);    // assets/旋转.png
LV_IMG_DECLARE(ui_img_499049799);    // assets/陀螺仪完整.png
LV_IMG_DECLARE(ui_img_1887403499);    // assets/设置.png



LV_FONT_DECLARE(ui_font_Terminal);
LV_FONT_DECLARE(ui_font_Terminal22);
LV_FONT_DECLARE(font_myawesome);



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
