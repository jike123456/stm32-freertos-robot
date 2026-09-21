# STM32 FreeRTOS 下位机控制与监控

运行于 STM32F407 的差速小车下位机工程，采用 STM32 HAL、FreeRTOS 和 CMSIS-RTOS2，支持串口通信、轮速控制、传感器采集与运行监控。通过板上定时器和电机模型，可以在没有实际电机的条件下开展控制链路联调。

本仓库定位于既有框架的学习、配置与板端联调。重点是理解数据从串口接收、协议解析、控制计算到执行反馈的路径，以及多任务环境下的通信和异常处理。

在此基础上扩展的控制状态机、CAN、诊断与 Bootloader 项目单独存放于 [Mini VCU ECU](https://github.com/jike123456/mini-vcu-ecu)。

## 功能与实现

| 模块 | 实现内容 | 主要代码入口 |
|---|---|---|
| 串口通信 | UART1 DMA+IDLE、StreamBuffer、逐字节协议状态机与 CRC16 | `App/Src/comm_task.c`、`Bsp/Src/comm_parser.c` |
| 运动控制 | 差速逆运动学、左右轮 PID、方向 GPIO 与 TIM3 PWM | `App/Src/control_task.c`、`Bsp/Src/car.c`、`Bsp/Src/pid.c` |
| 反馈仿真 | 一阶惯性模型、TIM10/TIM11 脉冲模拟、TIM2 输入捕获 | `Bsp/Src/motor.c`、`Bsp/Src/encoder.c`、`Bsp/Src/capture.c` |
| 数据采集 | ADC DMA 电压采样、I2C IMU 数据读取 | `App/Src/sensor_task.c`、`Bsp/Src/battery.c` |
| 状态显示 | UART2 异步日志、串口状态上报与 LCD 显示 | `App/Src/log_task.c`、`App/Src/uplink_task.c`、`App/Src/lcd_task.c` |
| 运行监控 | 任务心跳、栈水位、堆检查、异常 Hook 与看门狗 | `App/Src/monitor_task.c`、`App/Src/iwdg_task.c` |

## 阅读与联调重点

- 通信链路：从 DMA 缓冲区、IDLE 回调和 StreamBuffer 跟踪字节流，理解半包、连续帧与 CRC 错误的处理。
- 控制链路：从速度指令进入控制任务，检查差速逆解、PID 输出、PWM 与输入捕获反馈。
- 多任务协作：核对任务通知、共享数据、日志队列与控制节拍，区分配置周期和实测时序。
- 运行观测：使用日志、LCD 和任务心跳观察状态，检查指令超时、栈和堆异常处理。

## 目录

```text
App/                              应用初始化与业务任务
Bsp/                              驱动、协议解析、控制算法和仿真模块
Core/                             主程序、RTOS 集成与外设初始化
Drivers/                          STM32 HAL 与 CMSIS
Middlewares/Third_Party/FreeRTOS/   FreeRTOS 源码
MDK-ARM/                          Keil 工程、启动文件和 RTE 配置
Readme/                           工程阅读入口
KaoYa_Project.ioc                  STM32CubeMX 配置
```

协议文件位于 `Bsp/Inc/` 和 `Bsp/Src/`。建议从 `Core/Src/main.c`、`Core/Src/freertos.c` 和 `App/Src/app_init.c` 开始阅读。

## 编译与调试

1. 使用 Keil µVision 打开 `MDK-ARM/KaoYa_Project.uvprojx`，选择 `KaoYa_Project` 目标。
2. 现有工程使用 ARM Compiler 5.06 update 1、`Keil.STM32F4xx_DFP.2.17.1`，目标器件配置为 `STM32F407ZGTx`。更换工具链或器件包后需重新验证。
3. 根据实际板卡和 DAPLink/CMSIS-DAP 配置 SWD、复位与 Flash 算法。个人调试器设置不随仓库分发。
4. USART1、USART2 当前均配置为 115200 波特率；接线须以工程初始化配置及板卡原理图为准。

仓库不包含编译产物和教学 PDF；先本地构建，不要将其他项目的固件直接混用。

保留原工程名称和符号以兼容已有构建配置。Mini VCU 的重定位应用与本工程不能混用；其覆盖率与升级测试结果也不等同本仓库的测试结果。

## 仿真与硬件边界

原工程默认启用电机一阶惯性模型和编码器模拟。`EncoderSim` 使用 TIM10/TIM11 输出脉冲，控制任务通过 TIM2 输入捕获读取轮速，因此这一链路依赖 STM32 板上定时器及相应信号连接，并非可直接在电脑运行的纯软件仿真。

启用仿真时，电机接口仍会写入实际 GPIO 和 PWM。接入电机前，应核对驱动器、引脚、反馈接线、轮径和脉冲参数。本仓库不代表已完成真实电机、整车或全部外设验收。

## 来源与许可

保留原作者及第三方组件的版权和许可声明。详细来源与使用限制见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)，不对整库添加新的统一开源许可。
