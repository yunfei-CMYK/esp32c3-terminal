// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "../ui.h"

lv_obj_t * comp_label;
lv_obj_t * compass_meter;
lv_meter_scale_t * compass_scale;

static const char *TAG = "Campass";

// 定时更新方位角的值
void comp_update_cb(lv_timer_t * timer)
{
    t_sQMC5883L QMC5883L;
    int comp_angle;

    qmc5883l_fetch_azimuth(&QMC5883L);
    comp_angle = round(QMC5883L.azimuth);
    comp_angle = (comp_angle + 120)%360;  // 校准角度 正北为0
    lv_label_set_text_fmt(comp_label, "%d°", comp_angle);
    comp_angle = 360 - (comp_angle+90)%360;  // 计算旋转角度
    lv_meter_set_scale_range(compass_meter, compass_scale, 0, 360, 360, comp_angle); 
}

void ui_campasspage_screen_init(void)
{
    ESP_LOGI(TAG,"指南针界面初始化");
    // 创建一个界面对象
    ui_campasspage = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_campasspage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_campasspage, lv_color_hex(0x5295B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_campasspage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // 绘制指南针仪表
    compass_meter = lv_meter_create(ui_campasspage);
    lv_obj_center(compass_meter);
    lv_obj_set_size(compass_meter, 220, 220);
    lv_obj_set_style_bg_opa(compass_meter, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(compass_meter, lv_color_hex(0xffffff), 0);
    lv_obj_remove_style(compass_meter, NULL, LV_PART_INDICATOR);

    compass_scale = lv_meter_add_scale(compass_meter);
    lv_meter_set_scale_ticks(compass_meter, compass_scale, 61, 1, 10, lv_color_hex(0xffffff)); 
    lv_meter_set_scale_major_ticks(compass_meter, compass_scale, 10, 2, 16, lv_color_hex(0xffffff), 10); 
    lv_meter_set_scale_range(compass_meter, compass_scale, 0, 360, 360, 270); 
    
    // 添加指针
    lv_obj_t * arrow_label = lv_label_create(ui_campasspage);
    lv_label_set_text(arrow_label, LV_SYMBOL_DOWN);
    lv_obj_set_style_text_color(arrow_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(arrow_label, LV_ALIGN_CENTER, 0, -100);

    // 显示方位角度
    comp_label = lv_label_create(ui_campasspage);
    lv_obj_set_style_text_font(comp_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(comp_label, lv_color_hex(0xffffff), 0);
    lv_obj_align(comp_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(comp_label, "0");

    // 创建一个lv_timer 用于更新方位角的值
    my_lv_timer = lv_timer_create(comp_update_cb, 100, NULL);  

    lv_obj_add_event_cb(ui_campasspage, ui_event_campasspage, LV_EVENT_ALL, NULL);


}
