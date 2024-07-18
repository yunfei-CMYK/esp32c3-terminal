# ESP32C3 LVGL Multi-Function Example

This example demonstrates how to use LVGL on ESP32-C3.

---
## Introduction

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

后续计划：加入配网功能，实现一个联网应用（已实现）

## Usage
- 上面进入开始界面，会自动配网
- 点击进入按钮——————进入功能界面（依次点击按钮实现不同功能）
- 开始界面下滑进入网络应用界面（首次会先开始联网获取信息，之后展示天气桌面），上划可返回开始界面
- 功能界面依次点击按钮进入不同功能界面（温湿度，游戏，串口，陀螺仪这些界面为右滑返回；指南针为长按返回）
- 功能界面长按返回开始界面

### email:longfei_jia@163.com
