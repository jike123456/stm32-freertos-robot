# STM32F407 FreeRTOS 小车下位机学习项目

基于 Kaoya Robot 整理的 STM32F407 下位机学习工程，采用 STM32 HAL、FreeRTOS 和 CMSIS-RTOS2。原工程及多处业务代码署名 Kaoya（烤鸭）；本仓库不主张全部代码为上传者原创。

工程包含串口通信、协议状态机、差速运动学、轮速 PID、PWM 输出、传感器采集、状态上报、LCD 显示、异步日志、任务监控与独立看门狗。

在此基础上扩展的控制状态机、CAN、诊断与 Bootloader 项目单独存放于 [Mini VCU ECU](https://github.com/jike123456/mini-vcu-ecu)。

## 目录

```text
App/                              应用初始化与业务任务
Bsp/                              驱动、协议解析、控制算法和仿真模块
Core/                             主程序、RTOS 集成与外设初始化
Drivers/                          STM32 HAL 与 CMSIS
Middlewares/Third_Party/FreeRTOS/   FreeRTOS 源码
MDK-ARM/                          Keil 工程、启动文件和 RTE 配置
Readme/                           原作者说明
KaoYa_Project.ioc                  STM32CubeMX 配置
```

协议文件位于 `Bsp/Inc/` 和 `Bsp/Src/`。建议从 `Core/Src/main.c`、`Core/Src/freertos.c` 和 `App/Src/app_init.c` 开始阅读。

## 编译与调试

1. 使用 Keil µVision 打开 `MDK-ARM/KaoYa_Project.uvprojx`，选择 `KaoYa_Project` 目标。
2. 现有工程使用 ARM Compiler 5.06 update 1、`Keil.STM32F4xx_DFP.2.17.1`，目标器件配置为 `STM32F407ZGTx`。更换工具链或器件包后需重新验证。
3. 根据实际板卡和 DAPLink/CMSIS-DAP 配置 SWD、复位与 Flash 算法。个人调试器设置不随仓库分发。
4. USART1、USART2 当前均配置为 115200 波特率；接线须以工程初始化配置及板卡原理图为准。

仓库不包含编译产物和教学 PDF；先本地构建，不要将其他项目的固件直接混用。

## 仿真与硬件边界

原工程默认启用电机一阶惯性模型和编码器模拟。`EncoderSim` 使用 TIM10/TIM11 输出脉冲，控制任务通过 TIM2 输入捕获读取轮速，因此这一链路依赖 STM32 板上定时器及相应信号连接，并非可直接在电脑运行的纯软件仿真。

启用仿真时，电机接口仍会写入实际 GPIO 和 PWM。接入电机前，应核对驱动器、引脚、反馈接线、轮径和脉冲参数。本仓库不代表已完成真实电机、整车或全部外设验收。

## 来源与许可

原始工程为 Kaoya Robot／烤鸭嵌入式校招项目，源码原作者署名和版权声明予以保留。STM32、CMSIS、FreeRTOS 等组件遵循其各自许可。

原工程未提供覆盖全部代码的统一开源许可证，本仓库不添加整库 MIT 或其他再许可。公开访问不意味着所有代码均可任意再分发或商用。详见 [第三方来源与许可说明](THIRD_PARTY_NOTICES.md)。
