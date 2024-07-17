# ESP32C3 LVGL Multi-Function Example

This example demonstrates how to use LVGL on ESP32-C3.

参考链接：https://lceda001.feishu.cn/wiki/Xqx3wH8wMi3BrrkmeTXcgLL7nQk

## Hardware

立创esp32c3开发板

## Software

- esp-idf-v5.2.1
- lvgl-8.3.0

在官方掌机例程的基础上，修改部分功能：
  1. 取消了回声应用
  2. 修改了页面显示框架
  3. 加入了更多的LOG信息，方便调试
  4. 加入了串口助手的功能，可以实现和桌面端串口助手进行收发消息，不支持中文

后续计划：加入配网功能，实现一个联网应用

- email:longfei_jia@163.com
