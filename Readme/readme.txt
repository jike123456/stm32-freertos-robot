STM32 FreeRTOS 下位机工程阅读入口

项目简介、功能、构建方法及仿真边界见根目录 README.md。
完整来源与许可说明见根目录 THIRD_PARTY_NOTICES.md。

建议阅读顺序：
1. Core/Src/main.c 与 Core/Src/freertos.c：外设初始化与任务创建。
2. App/Src/app_init.c：应用初始化。
3. App/Src/comm_task.c 与 Bsp/Src/comm_parser.c：串口字节流及协议解析。
4. App/Src/control_task.c：命令处理、差速逆解、PID 与超时处理。
5. Bsp/Src/motor.c、encoder.c、capture.c：输出、仿真和反馈。
6. App/Src/log_task.c、monitor_task.c、iwdg_task.c：日志和运行监控。

Keil 工程入口：MDK-ARM/KaoYa_Project.uvprojx。
协议文件位于 Bsp/Inc 和 Bsp/Src，没有单独的 Protocol 目录。

本工程基于 Kaoya Robot 框架整理，原作者为 Kaoya（烤鸭）。
Copyright (c) 2025-2035 Kaoya. All rights reserved.
STM32 HAL、CMSIS、FreeRTOS 等组件保留各自的版权与许可文件。
本说明替换原有课程介绍和购买宣传，不改变源码归属与许可。



