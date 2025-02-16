// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "../ui.h"

static const char *TAG = "Mainpage";

void ui_Mainpage_screen_init(void)
{
    ESP_LOGI(TAG, "功能菜单界面初始化");
    ui_Mainpage = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Mainpage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Mainpage, lv_color_hex(0x5394B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Mainpage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_weather = lv_btn_create(ui_Mainpage);
    lv_obj_set_width(ui_weather, 58);
    lv_obj_set_height(ui_weather, 58);
    lv_obj_set_x(ui_weather, -101);
    lv_obj_set_y(ui_weather, -31);
    lv_obj_set_align(ui_weather, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_weather, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_weather, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_weather, lv_color_hex(0x5295B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_weather, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_weather, &lv_font_montserrat_8, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_weatherpng = lv_img_create(ui_weather);
    lv_img_set_src(ui_weatherpng, &ui_img_944254084);
    lv_obj_set_width(ui_weatherpng, LV_SIZE_CONTENT);   /// 64
    lv_obj_set_height(ui_weatherpng, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_x(ui_weatherpng, 2);
    lv_obj_set_y(ui_weatherpng, 2);
    lv_obj_set_align(ui_weatherpng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_weatherpng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_weatherpng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_weatherpng, lv_color_hex(0x5295B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_weatherpng, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_game = lv_btn_create(ui_Mainpage);
    lv_obj_set_width(ui_game, 58);
    lv_obj_set_height(ui_game, 58);
    lv_obj_set_x(ui_game, 1);
    lv_obj_set_y(ui_game, -29);
    lv_obj_set_align(ui_game, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_game, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_game, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_game, lv_color_hex(0x00AAFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_game, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_gamepng = lv_img_create(ui_game);
    lv_img_set_src(ui_gamepng, &ui_img_227306801);
    lv_obj_set_width(ui_gamepng, LV_SIZE_CONTENT);   /// 64
    lv_obj_set_height(ui_gamepng, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_align(ui_gamepng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_gamepng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_gamepng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_port = lv_btn_create(ui_Mainpage);
    lv_obj_set_width(ui_port, 58);
    lv_obj_set_height(ui_port, 58);
    lv_obj_set_x(ui_port, 102);
    lv_obj_set_y(ui_port, -30);
    lv_obj_set_align(ui_port, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_port, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_port, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_port, lv_color_hex(0x00AAFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_port, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_portpng = lv_img_create(ui_port);
    lv_img_set_src(ui_portpng, &ui_img_2114266571);
    lv_obj_set_width(ui_portpng, LV_SIZE_CONTENT);   /// 64
    lv_obj_set_height(ui_portpng, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_x(ui_portpng, 0);
    lv_obj_set_y(ui_portpng, -1);
    lv_obj_set_align(ui_portpng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_portpng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_portpng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_mpu = lv_btn_create(ui_Mainpage);
    lv_obj_set_width(ui_mpu, 58);
    lv_obj_set_height(ui_mpu, 58);
    lv_obj_set_x(ui_mpu, -100);
    lv_obj_set_y(ui_mpu, 64);
    lv_obj_set_align(ui_mpu, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_mpu, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_mpu, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_mpupng = lv_img_create(ui_mpu);
    lv_img_set_src(ui_mpupng, &ui_img_613467726);
    lv_obj_set_width(ui_mpupng, LV_SIZE_CONTENT);   /// 64
    lv_obj_set_height(ui_mpupng, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_align(ui_mpupng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_mpupng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_mpupng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_campass = lv_btn_create(ui_Mainpage);
    lv_obj_set_width(ui_campass, 58);
    lv_obj_set_height(ui_campass, 58);
    lv_obj_set_x(ui_campass, 2);
    lv_obj_set_y(ui_campass, 63);
    lv_obj_set_align(ui_campass, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_campass, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_campass, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_campasspng = lv_img_create(ui_campass);
    lv_img_set_src(ui_campasspng, &ui_img_499049799);
    lv_obj_set_width(ui_campasspng, LV_SIZE_CONTENT);   /// 64
    lv_obj_set_height(ui_campasspng, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_align(ui_campasspng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_campasspng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_campasspng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_setting = lv_btn_create(ui_Mainpage);
    lv_obj_set_width(ui_setting, 58);
    lv_obj_set_height(ui_setting, 58);
    lv_obj_set_x(ui_setting, 103);
    lv_obj_set_y(ui_setting, 63);
    lv_obj_set_align(ui_setting, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_setting, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_setting, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_setpng = lv_img_create(ui_setting);
    lv_img_set_src(ui_setpng, &ui_img_1887403499);
    lv_obj_set_width(ui_setpng, LV_SIZE_CONTENT);   /// 64
    lv_obj_set_height(ui_setpng, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_align(ui_setpng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_setpng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_setpng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_weatherlabel = lv_label_create(ui_Mainpage);
    lv_obj_set_width(ui_weatherlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_weatherlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_weatherlabel, -102);
    lv_obj_set_y(ui_weatherlabel, 14);
    lv_obj_set_align(ui_weatherlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_weatherlabel, "天气");
    lv_obj_set_style_text_color(ui_weatherlabel, lv_color_hex(0x1D2334), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_weatherlabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_weatherlabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_settinglabel = lv_label_create(ui_Mainpage);
    lv_obj_set_width(ui_settinglabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_settinglabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_settinglabel, 102);
    lv_obj_set_y(ui_settinglabel, 102);
    lv_obj_set_align(ui_settinglabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_settinglabel, "设置");
    lv_obj_set_style_text_font(ui_settinglabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_campasslabel = lv_label_create(ui_Mainpage);
    lv_obj_set_width(ui_campasslabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_campasslabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_campasslabel, 3);
    lv_obj_set_y(ui_campasslabel, 102);
    lv_obj_set_align(ui_campasslabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_campasslabel, "指南针");
    lv_obj_set_style_text_color(ui_campasslabel, lv_color_hex(0x1D2334), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_campasslabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_campasslabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_mpulabel = lv_label_create(ui_Mainpage);
    lv_obj_set_width(ui_mpulabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_mpulabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_mpulabel, -101);
    lv_obj_set_y(ui_mpulabel, 105);
    lv_obj_set_align(ui_mpulabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_mpulabel, "陀螺仪");
    lv_obj_set_style_text_color(ui_mpulabel, lv_color_hex(0x1D2334), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_mpulabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_mpulabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_gamelabel = lv_label_create(ui_Mainpage);
    lv_obj_set_width(ui_gamelabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_gamelabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_gamelabel, 2);
    lv_obj_set_y(ui_gamelabel, 13);
    lv_obj_set_align(ui_gamelabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_gamelabel, "游戏");
    lv_obj_set_style_text_color(ui_gamelabel, lv_color_hex(0x1D2334), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_gamelabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_gamelabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_voicelabel = lv_label_create(ui_Mainpage);
    lv_obj_set_width(ui_voicelabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_voicelabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_voicelabel, 102);
    lv_obj_set_y(ui_voicelabel, 13);
    lv_obj_set_align(ui_voicelabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_voicelabel, "串口助手");
    lv_obj_set_style_text_color(ui_voicelabel, lv_color_hex(0x1D2334), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_voicelabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_voicelabel, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_functionmenutitle = lv_label_create(ui_Mainpage);
    lv_obj_set_width(ui_functionmenutitle, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_functionmenutitle, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_functionmenutitle, -6);
    lv_obj_set_y(ui_functionmenutitle, -91);
    lv_obj_set_align(ui_functionmenutitle, LV_ALIGN_CENTER);
    lv_label_set_text(ui_functionmenutitle, "功能菜单");
    lv_obj_set_style_text_color(ui_functionmenutitle, lv_color_hex(0x1D2B3D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_functionmenutitle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_functionmenutitle, &ui_font_Terminal22, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_weather, ui_event_weather, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_game, ui_event_game, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_port, ui_event_port, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_mpu, ui_event_mpu, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_campass, ui_event_campass, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_setting, ui_event_setting, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Mainpage, ui_event_Mainpage, LV_EVENT_ALL, NULL);

}
