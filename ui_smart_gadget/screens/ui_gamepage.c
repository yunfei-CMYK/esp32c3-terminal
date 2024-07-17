// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "../ui.h"

static const char *TAG = "Game";

lv_obj_t *mat;       // 创建一个弹力球的垫子
lv_obj_t *ball;      // 创建一个弹力球
int mat_flag;        // 弹力球标志位
int ball_height = 0; // 弹力球的高度
int ball_dir = 0;    // 弹力球的方向
int mat_height = 0;  // 垫子的高度
int strength = 0;

void game_update_cb(lv_timer_t *timer)
{
    ESP_LOGI(TAG, "游戏界面更新参数");
    if (strength != 0) // 发现有手指按下屏幕
    {
        if (strength < 31) // 限制手指按下时间最大为30
        {
            mat_height = 60 - strength;                             // 计算正方体的高度
            lv_obj_set_size(mat, 80, mat_height);                   // 调整正方体的高度
            lv_obj_align_to(ball, mat, LV_ALIGN_OUT_TOP_MID, 0, 0); // 让弹力球跟随垫子一起移动
            mat_flag = 1;                                           // 表示垫子已经缩小过
        }
    }
    else if (mat_flag == 1) // 如果垫子已经缩小过
    {
        lv_obj_set_size(mat, 80, 60);                           // 垫子回弹到原始值
        lv_obj_align_to(ball, mat, LV_ALIGN_OUT_TOP_MID, 0, 0); // 弹力球跟随垫子
        mat_flag = 2;                                           // 标记垫子已经回弹
    }
    else if (mat_flag == 2) // 垫子已经回弹 小球应该向上走了
    {
        if (ball_dir == 0) // 向上运动
        {
            if (ball_height < 150) // 限制弹力球上弹高度为150像素
            {
                ball_height = ball_height + 10;                                    // 每次上升10个像素
                lv_obj_align_to(ball, mat, LV_ALIGN_OUT_TOP_MID, 0, -ball_height); // 更新弹力球位置
                if (ball_height >= (0 + (60 - mat_height) * 5))                    // 根据力度计算小球最大高度
                {
                    ball_dir = 1; // 如果达到最大高度 更改小球运动方向
                }
            }
        }
        else // 向下运动
        {
            if (ball_height > 0) // 限制小球最低高度
            {
                ball_height = ball_height - 10;                                    // 每次降低10个像素高度
                lv_obj_align_to(ball, mat, LV_ALIGN_OUT_TOP_MID, 0, -ball_height); // 更新弹力球高度
                if (ball_height == 0)                                              // 如果弹力球落到了垫子上
                {
                    mat_flag = 0;   // 垫子状态恢复
                    ball_dir = 0;   // 小球方向恢复
                    mat_height = 0; // 垫子高度恢复
                }
            }
        }
    }
    ESP_LOGI(TAG, "mat_flag = %d,ball_dir = %d,mat_height = %d,strength = %d", mat_flag, ball_dir, mat_height, strength);
}


void ui_gamepage_screen_init(void)
{
    ESP_LOGI(TAG, "游戏界面初始化");
    ui_gamepage = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_gamepage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_gamepage, lv_color_hex(0x5295B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_gamepage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // 创建一个垫子
    static lv_style_t mat_style;
    lv_style_init(&mat_style);
    lv_style_set_radius(&mat_style, 0);
    lv_style_set_border_width(&mat_style, 0);
    lv_style_set_pad_all(&mat_style, 0);
    lv_style_set_shadow_width(&mat_style, 10);
    lv_style_set_shadow_color(&mat_style, lv_color_black());
    lv_style_set_shadow_ofs_x(&mat_style, 10);
    lv_style_set_shadow_ofs_y(&mat_style, 10);

    mat = lv_obj_create(ui_gamepage);
    lv_obj_add_style(mat, &mat_style, 0);
    lv_obj_set_style_bg_color(mat, lv_color_hex(0x6B8E23), 0);
    lv_obj_align(mat, LV_ALIGN_BOTTOM_LEFT, 30, -30);
    lv_obj_set_size(mat, 80, 60);

    // 创建一个圆球
    ball = lv_led_create(ui_gamepage);
    lv_led_set_brightness(ball, 150);
    lv_led_set_color(ball, lv_palette_main(LV_PALETTE_DEEP_ORANGE));
    lv_obj_align_to(ball, mat, LV_ALIGN_OUT_TOP_MID, 0, 0);
    // ui_Label13 = lv_label_create(ui_gamepage);
    // lv_obj_set_width(ui_Label13, LV_SIZE_CONTENT);   /// 1
    // lv_obj_set_height(ui_Label13, LV_SIZE_CONTENT);    /// 1
    // lv_obj_set_align(ui_Label13, LV_ALIGN_CENTER);
    // lv_label_set_text(ui_Label13, "游戏");
    // lv_obj_set_style_text_font(ui_Label13, &ui_font_Terminal, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_gamepage, ui_event_gamepage, LV_EVENT_ALL, NULL);

}
