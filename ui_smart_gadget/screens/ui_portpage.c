// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.11
// Project name: Terminal

#include "../ui.h"

static const char *TAG = "SerialPort";

/* serial port config */

static const int RX_BUF_SIZE = 1024;
// static const int TX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_21)
#define RXD_PIN (GPIO_NUM_20)

static uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

void serialportconfig(void)
{

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char *data)
{
    serialportconfig();
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void ui_event_inputtext(lv_event_t *e)
{
    lv_event_code_t eventcode = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target(e); // 确保获取的是文本区域对象

    // 假设 ui_inputkeyboard 是我们在其他地方创建的键盘对象
    lv_obj_t *kb = ui_inputkeyboard;

    switch (eventcode)
    {
    case LV_EVENT_FOCUSED:
        // 确保 kb 是键盘对象，ta 是文本区域对象
        if (kb && ta)
        {
            lv_keyboard_set_textarea(kb, ta);
            lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
        }
        break;

    case LV_EVENT_DEFOCUSED:
        if (kb)
        {
            lv_keyboard_set_textarea(kb, NULL);
            lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        }
        break;

    default:
        break;
    }
}

void task_receive_data(void *arg)
{
    serialportconfig();
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);

    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            // 更新 LVGL 文本框控件
            lv_textarea_add_text(ui_outputtext, (const char *)data);
        }
        if(screen_state == function_page)
        {
            uart_driver_delete(UART_NUM_1);
            ESP_LOGI(TAG,"释放串口驱动资源");
            ESP_LOGI(TAG,"返回功能界面");
            break;
        }
    }
    free(data);
    vTaskDelete(NULL);
    ESP_LOGI(TAG,"发送数据任务删除");
}

static void ui_event_sendbtn(lv_event_t *e)
{
    const char *input_text = lv_textarea_get_text(ui_inputtext);

    // ESP_LOGI(TAG, "Send data: %s", input_text);

    sendData(input_text);

    // lv_textarea_set_text(ui_inputtext, "");
}

static void ui_event_clearbtn(lv_event_t *e)
{
    lv_textarea_set_text(ui_inputtext, "");
    lv_textarea_set_text(ui_outputtext, "");
}

static void ui_event_baudrate_dropdown(lv_event_t *e)
{
    lv_obj_t *dropdown = lv_event_get_target(e);
    
    char buf[16];
    lv_dropdown_get_selected_str(dropdown, buf, sizeof(buf));
    int baud_rate = atoi(buf);

    uart_config.baud_rate = baud_rate;
    ESP_LOGI(TAG, "Set baud rate: %d", uart_config.baud_rate);
    
    uart_param_config(UART_NUM_1, &uart_config);
}

void ui_portpage_screen_init(void)
{
    ESP_LOGI(TAG, "串口助手界面初始化");

    ui_portpage = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_portpage, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_portpage, lv_color_hex(0x5295B4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_portpage, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label14 = lv_label_create(ui_portpage);
    lv_obj_set_width(ui_Label14, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label14, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label14, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label14, "串口助手");
    lv_obj_set_style_text_font(ui_Label14, &ui_font_Terminal22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_inputtext = lv_textarea_create(ui_portpage);
    lv_obj_set_width(ui_inputtext, 237);
    lv_obj_set_height(ui_inputtext, 36);
    lv_obj_set_x(ui_inputtext, -36);
    lv_obj_set_y(ui_inputtext, -68);
    lv_obj_set_align(ui_inputtext, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_inputtext, "send data...");
    lv_obj_add_event_cb(ui_inputtext, ui_event_inputtext, LV_EVENT_ALL, ui_inputkeyboard);

    ui_inputkeyboard = lv_keyboard_create(lv_layer_top());
    lv_keyboard_set_textarea(ui_inputkeyboard, ui_inputtext);
    lv_obj_add_flag(ui_inputkeyboard, LV_OBJ_FLAG_HIDDEN);

    ui_sendbtn = lv_btn_create(ui_portpage);
    lv_obj_set_width(ui_sendbtn, 66);
    lv_obj_set_height(ui_sendbtn, 36);
    lv_obj_set_x(ui_sendbtn, 119);
    lv_obj_set_y(ui_sendbtn, -68);
    lv_obj_set_align(ui_sendbtn, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_sendbtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_sendbtn, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_bg_color(ui_sendbtn, lv_color_hex(0xA58EAB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_sendbtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_sendbtn, ui_event_sendbtn, LV_EVENT_CLICKED, NULL);

    ui_sendlabel = lv_label_create(ui_sendbtn);
    lv_obj_set_width(ui_sendlabel, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_sendlabel, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_sendlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_sendlabel, "Send");

    ui_outputtext = lv_textarea_create(ui_portpage);
    lv_obj_set_width(ui_outputtext, 305);
    lv_obj_set_height(ui_outputtext, 120);
    lv_obj_set_x(ui_outputtext, -2);
    lv_obj_set_y(ui_outputtext, 16);
    lv_obj_set_align(ui_outputtext, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_outputtext, "receive data...");

    ui_baudrateDropdown = lv_dropdown_create(ui_portpage);
    lv_dropdown_set_options(ui_baudrateDropdown, "115200\n4800\n9600\n14400\n19200\n38400\n56000\n57600");
    lv_obj_set_width(ui_baudrateDropdown, 233);
    lv_obj_set_height(ui_baudrateDropdown, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_baudrateDropdown, -37);
    lv_obj_set_y(ui_baudrateDropdown, 100);
    lv_obj_set_align(ui_baudrateDropdown, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_baudrateDropdown, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    
    lv_obj_add_event_cb(ui_baudrateDropdown, ui_event_baudrate_dropdown, LV_EVENT_VALUE_CHANGED, NULL);

    ui_clearbtn = lv_btn_create(ui_portpage);
    lv_obj_set_width(ui_clearbtn, 66);
    lv_obj_set_height(ui_clearbtn, 36);
    lv_obj_set_x(ui_clearbtn, 118);
    lv_obj_set_y(ui_clearbtn, 100);
    lv_obj_set_align(ui_clearbtn, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_clearbtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_clearbtn, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_bg_color(ui_clearbtn, lv_color_hex(0xA48DAC), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_clearbtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui_clearbtn, ui_event_clearbtn, LV_EVENT_CLICKED, NULL);

    ui_clearlabel = lv_label_create(ui_clearbtn);
    lv_obj_set_width(ui_clearlabel, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_clearlabel, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_clearlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_clearlabel, "Clear");

    lv_obj_add_event_cb(ui_portpage, ui_event_portpage, LV_EVENT_ALL, NULL);
}
