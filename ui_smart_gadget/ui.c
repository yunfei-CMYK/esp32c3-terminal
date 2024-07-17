// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "ui.h"
#include "ui_helpers.h"

static char *TAG = "当前执行";
static char *STATE = "当前页面";

menustate screen_state;

///////////////////// VARIABLES ////////////////////
void showanim_Animation(lv_obj_t *TargetObject, int delay);

// SCREEN: ui_Startpage
void ui_Startpage_screen_init(void);
void ui_event_Startpage(lv_event_t *e);
lv_obj_t *ui_Startpage;
lv_obj_t *ui_MainTitle;
void ui_event_Entry(lv_event_t *e);
lv_obj_t *ui_Entry;
lv_obj_t *ui_Label3;
void ui_event_wifibtn(lv_event_t *e);
lv_obj_t *ui_wifibtn;

// SCREEN: ui_Mainpage
void ui_Mainpage_screen_init(void);
void ui_event_Mainpage(lv_event_t *e);
lv_obj_t *ui_Mainpage;
void ui_event_weather(lv_event_t *e);
lv_obj_t *ui_weather;
lv_obj_t *ui_weatherpng;
void ui_event_game(lv_event_t *e);
lv_obj_t *ui_game;
lv_obj_t *ui_gamepng;
void ui_event_port(lv_event_t *e);
lv_obj_t *ui_port;
lv_obj_t *ui_portpng;
void ui_event_mpu(lv_event_t *e);
lv_obj_t *ui_mpu;
lv_obj_t *ui_mpupng;
void ui_event_campass(lv_event_t *e);
lv_obj_t *ui_campass;
lv_obj_t *ui_campasspng;
void ui_event_setting(lv_event_t *e);
lv_obj_t *ui_setting;
lv_obj_t *ui_setpng;
lv_obj_t *ui_weatherlabel;
lv_obj_t *ui_settinglabel;
lv_obj_t *ui_campasslabel;
lv_obj_t *ui_mpulabel;
lv_obj_t *ui_gamelabel;
lv_obj_t *ui_voicelabel;
lv_obj_t *ui_functionmenutitle;

// SCREEN: ui_weatherpage
void ui_weatherpage_screen_init(void);
void ui_event_weatherpage(lv_event_t *e);
lv_obj_t *ui_weatherpage;
lv_obj_t *ui_Label12;

// SCREEN: ui_gamepage
void ui_gamepage_screen_init(void);
void ui_event_gamepage(lv_event_t *e);
lv_obj_t *ui_gamepage;
lv_obj_t *ui_Label13;

// SCREEN: ui_portpage
void ui_portpage_screen_init(void);
void ui_event_portpage(lv_event_t *e);
lv_obj_t *ui_portpage;
lv_obj_t *ui_Label14;
lv_obj_t *ui_inputtext;
lv_obj_t *ui_sendbtn;
lv_obj_t *ui_sendlabel;
lv_obj_t *ui_outputtext;
lv_obj_t *ui_baudrateDropdown;
lv_obj_t *ui_clearbtn;
lv_obj_t *ui_clearlabel;
lv_obj_t *ui_inputkeyboard;

// SCREEN: ui_mpupage
void ui_mpupage_screen_init(void);
void ui_event_mpupage(lv_event_t *e);
lv_obj_t *ui_mpupage;
lv_obj_t *ui_Label15;

// SCREEN: ui_campasspage
void ui_campasspage_screen_init(void);
void ui_event_campasspage(lv_event_t *e);
lv_obj_t *ui_campasspage;
lv_obj_t *ui_Label16;

// SCREEN: ui_settingpage
void ui_settingpage_screen_init(void);
void ui_event_settingpage(lv_event_t *e);
lv_obj_t *ui_settingpage;
lv_obj_t *ui_Label17;
lv_obj_t *ui_Container1;
lv_obj_t *ui_Label1;

// SCREEN: ui_wifipage
void ui_wifipage_screen_init(void);
void ui_event_wifipage(lv_event_t *e);
lv_obj_t *ui_wifipage;
lv_obj_t *ui_wifititle;
lv_obj_t *ui_wifiDropdown;
lv_obj_t *ui_wifiswitch;
lv_obj_t *ui_wifiled;
lv_obj_t *ui_usefulwifilabel;
lv_obj_t *ui_passwordlabel;
lv_obj_t *ui_passwordTextArea;
lv_obj_t *ui_linkbtn;
lv_obj_t *ui_linklabel;
void ui_event_wifiKeyboard(lv_event_t *e);
lv_obj_t *ui_wifiKeyboard;
lv_obj_t *ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
#error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP != 1
#error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////
void showanim_Animation(lv_obj_t *TargetObject, int delay)
{
    ui_anim_user_data_t *PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_0_user_data->target = TargetObject;
    PropertyAnimation_0_user_data->val = -1;
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 1000);
    lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_y);
    lv_anim_set_values(&PropertyAnimation_0, -30, 0);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_overshoot);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_0, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_set_get_value_cb(&PropertyAnimation_0, &_ui_anim_callback_get_y);
    lv_anim_start(&PropertyAnimation_0);
    ui_anim_user_data_t *PropertyAnimation_1_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_1_user_data->target = TargetObject;
    PropertyAnimation_1_user_data->val = -1;
    lv_anim_t PropertyAnimation_1;
    lv_anim_init(&PropertyAnimation_1);
    lv_anim_set_time(&PropertyAnimation_1, 1000);
    lv_anim_set_user_data(&PropertyAnimation_1, PropertyAnimation_1_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_1, _ui_anim_callback_set_opacity);
    lv_anim_set_values(&PropertyAnimation_1, 0, 255);
    lv_anim_set_path_cb(&PropertyAnimation_1, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_1, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_1, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_1, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_1, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_1, 0);
    lv_anim_set_early_apply(&PropertyAnimation_1, true);
    lv_anim_start(&PropertyAnimation_1);
}

///////////////////// FUNCTIONS ////////////////////
void ui_event_Startpage(lv_event_t *e)
{
    screen_state = start_page;
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_SCREEN_LOAD_START)
    {
        showanim_Animation(ui_Startpage, 50);
        showanim_Animation(ui_MainTitle, 100);
        showanim_Animation(ui_Entry, 200);
        showanim_Animation(ui_wifibtn, 200);
    }
}
void ui_event_Entry(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_Mainpage, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Mainpage_screen_init);
        screen_state = function_page;
        ESP_LOGI(STATE, "%d 是功能界面", screen_state);
    }
}
void ui_event_wifibtn(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_wifipage, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_wifipage_screen_init);
        screen_state = wifi_page;
        ESP_LOGI(STATE, "%d 是WIFI界面", screen_state);
    }
}

void ui_event_Mainpage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_LONG_PRESSED)
    {
        _ui_screen_change(&ui_Startpage, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_Startpage_screen_init);
        screen_state = start_page;
        ESP_LOGI(STATE, "%d 是开始界面", screen_state);
    }
    if (event_code == LV_EVENT_SCREEN_LOAD_START)
    {
        showanim_Animation(ui_weather, 200);
        showanim_Animation(ui_game, 200);
        showanim_Animation(ui_port, 200);
        showanim_Animation(ui_mpu, 400);
        showanim_Animation(ui_campass, 400);
        showanim_Animation(ui_setting, 400);
        showanim_Animation(ui_functionmenutitle, 50);
        showanim_Animation(ui_settinglabel, 400);
        showanim_Animation(ui_campasslabel, 400);
        showanim_Animation(ui_mpulabel, 400);
        showanim_Animation(ui_voicelabel, 200);
        showanim_Animation(ui_gamelabel, 200);
        showanim_Animation(ui_weatherlabel, 200);
    }
}
void ui_event_weather(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_weatherpage, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_weatherpage_screen_init);
        screen_state = tha_page;
        ESP_LOGI(STATE, "%d 是温湿度界面", screen_state);
        // 创建一个lv_timer 定时更新数据
        ESP_LOGI(TAG, "创建温湿度传感器参数更新定时器");
        my_lv_timer = lv_timer_create(thv_update_cb, 1000, NULL);
        // 创建一个获取温湿度的任务
        ESP_LOGI(TAG, "创建获取温湿度的任务");
        xTaskCreate(get_th_task, "get_th_task", 4096, NULL, 5, NULL);
    }
}
void ui_event_game(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_gamepage, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_gamepage_screen_init);
        screen_state = game_page;
        ESP_LOGI(STATE, "%d 是游戏界面", screen_state);
        // 创建一个lv_timer 用于更新圆球的坐标
        ESP_LOGI(TAG, "创建游戏界面参数更新定时器");
        my_lv_timer = lv_timer_create(game_update_cb, 50, NULL); //
    }
}
void ui_event_port(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_portpage, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_portpage_screen_init);
        screen_state = serialport_page;
        ESP_LOGI(STATE, "%d 是串口助手界面", screen_state);
        xTaskCreate(task_receive_data, "task_receive_data", 2048, NULL, 5, NULL);
        ESP_LOGI(TAG, "创建接收串口数据的任务");
    }
}
void ui_event_mpu(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_mpupage, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_mpupage_screen_init);
        screen_state = mpu_page;
        ESP_LOGI(STATE, "%d 是陀螺仪界面", screen_state);
        ESP_LOGI(TAG, "创建姿态传感器参数更新定时器");
        my_lv_timer = lv_timer_create(att_update_cb, 100, NULL);
    }
}
void ui_event_campass(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_campasspage, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_campasspage_screen_init);
        screen_state = campass_page;
        ESP_LOGI(STATE, "%d 是指南针界面", screen_state);
        // 创建一个lv_timer 用于更新方位角的值
        ESP_LOGI(TAG, "创建地磁传感器参数更新定时器");
        my_lv_timer = lv_timer_create(comp_update_cb, 100, NULL);
    }
}
void ui_event_setting(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_screen_change(&ui_settingpage, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_settingpage_screen_init);
        screen_state = setting_page;
        ESP_LOGI(STATE, "%d 是设置界面", screen_state);
    }
}
void ui_event_weatherpage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Mainpage, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Mainpage_screen_init);
        screen_state = function_page;
        ESP_LOGI(STATE, "%d 是功能界面", screen_state);
        lv_timer_del(my_lv_timer);
        ESP_LOGI(TAG, "删除温湿度传感器参数更新定时器");
        ESP_LOGI(TAG, "删除获取温湿度的任务");
    }
}
void ui_event_gamepage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Mainpage, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Mainpage_screen_init);
        screen_state = function_page;
        ESP_LOGI(STATE, "%d 是功能界面", screen_state);
        lv_timer_del(my_lv_timer);
        ESP_LOGI(TAG, "删除游戏界面参数更新定时器");
    }
}
void ui_event_portpage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Mainpage, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Mainpage_screen_init);
        screen_state = function_page;
        ESP_LOGI(STATE, "%d 是功能界面", screen_state);
    }
}
void ui_event_mpupage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Mainpage, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Mainpage_screen_init);
        screen_state = function_page;
        ESP_LOGI(STATE, "%d 是功能界面", screen_state);
        lv_timer_del(my_lv_timer);
        ESP_LOGI(TAG, "删除姿态传感器参数更新定时器");
    }
}
void ui_event_campasspage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_LONG_PRESSED)
    {
        _ui_screen_change(&ui_Mainpage, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_Mainpage_screen_init);
        screen_state = function_page;
        ESP_LOGI(STATE, "%d 是功能界面", screen_state);
        lv_timer_del(my_lv_timer);
        ESP_LOGI(TAG, "删除地磁传感器参数更新定时器");
    }
}
void ui_event_settingpage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Mainpage, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, &ui_Mainpage_screen_init);
        screen_state = function_page;
        ESP_LOGI(STATE, "%d 是功能界面", screen_state);
    }
}
void ui_event_wifipage(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_GESTURE && lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT)
    {
        lv_indev_wait_release(lv_indev_get_act());
        _ui_screen_change(&ui_Startpage, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Startpage_screen_init);
        screen_state = start_page;
        ESP_LOGI(STATE, "%d 是开始界面", screen_state);
    }
}

void ui_event_wifiKeyboard(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        _ui_keyboard_set_target(ui_wifiKeyboard, ui_passwordTextArea);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    screen_state = start_page;
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                              false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Startpage_screen_init();
    ui_Mainpage_screen_init();
    ui_weatherpage_screen_init();
    ui_gamepage_screen_init();
    ui_portpage_screen_init();
    ui_mpupage_screen_init();
    ui_campasspage_screen_init();
    ui_settingpage_screen_init();
    ui_wifipage_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Startpage);
}
