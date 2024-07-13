// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "../ui.h"


lv_obj_t * att_x_label;
lv_obj_t * att_y_label;
lv_obj_t * att_led;

// 定时更新水平仪坐标值
void att_update_cb(lv_timer_t * timer)
{
    t_sQMI8658C QMI8658C;
    int att_led_x, att_led_y;

    qmi8658c_fetch_angleFromAcc(&QMI8658C);
    att_led_x = round(QMI8658C.AngleX);
    att_led_y = round(QMI8658C.AngleY);
    lv_obj_align(att_led, LV_ALIGN_CENTER, -att_led_x, att_led_y);
    lv_label_set_text_fmt(att_x_label, "X=%d°", -att_led_x);
    lv_label_set_text_fmt(att_y_label, "Y=%d°", att_led_y);
}

void ui_mpupage_screen_init(void)
{

        // 创建一个界面对象
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_radius(&style, 10);  
    lv_style_set_bg_opa( &style, LV_OPA_COVER );
    lv_style_set_bg_color(&style, lv_color_hex(0x00BFFF));
    lv_style_set_bg_grad_color( &style, lv_color_hex( 0x00BF00 ) );
    lv_style_set_bg_grad_dir( &style, LV_GRAD_DIR_VER );
    lv_style_set_border_width(&style, 0);
    lv_style_set_pad_all(&style, 0);
    lv_style_set_width(&style, 320);  
    lv_style_set_height(&style, 240); 

    ui_mpupage = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_mpupage, LV_OBJ_FLAG_SCROLLABLE); 
    lv_obj_add_style(ui_mpupage, &style, 0);     /// Flags

    // 画一个外圆
    lv_obj_t * arc = lv_arc_create(ui_mpupage);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE); 
    lv_obj_center(arc);
    lv_arc_set_value(arc, 360);
    lv_obj_set_size(arc, 200, 200);

    // 画一个实心圆点
    att_led  = lv_led_create(ui_mpupage);
    lv_obj_align(att_led, LV_ALIGN_CENTER, 0, 0);
    lv_led_set_brightness(att_led, 255);
    lv_led_set_color(att_led, lv_palette_main(LV_PALETTE_RED));

    // 显示X和Y的角度
    att_x_label = lv_label_create(ui_mpupage);
    lv_obj_set_style_text_font(att_x_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(att_x_label, lv_color_hex(0xffffff), 0);
    lv_obj_align(att_x_label, LV_ALIGN_TOP_LEFT, 20, 20);

    att_y_label = lv_label_create(ui_mpupage);
    lv_obj_set_style_text_font(att_y_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(att_y_label, lv_color_hex(0xffffff), 0);
    lv_obj_align(att_y_label, LV_ALIGN_TOP_RIGHT, -20, 20);

    // 创建一个lv_timer 用于更新圆的坐标
    my_lv_timer = lv_timer_create(att_update_cb, 100, NULL); 

    // ui_Label15 = lv_label_create(ui_mpupage);
    // lv_obj_set_width(ui_Label15, LV_SIZE_CONTENT);   /// 1
    // lv_obj_set_height(ui_Label15, LV_SIZE_CONTENT);    /// 1
    // lv_obj_set_align(ui_Label15, LV_ALIGN_CENTER);
    // lv_label_set_text(ui_Label15, "陀螺仪");
    // lv_obj_set_style_text_font(ui_Label15, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_mpupage, ui_event_mpupage, LV_EVENT_ALL, NULL);

}
