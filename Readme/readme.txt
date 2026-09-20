# Kaoya Robot｜烤鸭嵌入式校招项目

> 一句话：这是一个**能跑、能改、能讲、能经得起面试追问**的嵌入式主项目。  
> 平台：**STM32F407ZGT6 + FreeRTOS + HAL**，围绕“小车”搭建可交付的工程化下位机系统。

---

## 1. 项目简介

**Kaoya Robot** 是一个面向真实工程交付思路设计的下位机项目，核心目标不是“堆功能”，而是把**分层、可维护、可调试、可扩展**做到位。

你可以把它理解为一个“小型自动驾驶底盘下位机”：
- 上位机下发速度指令（vx / wz）
- 下位机完成协议解析、运动学解算、PID 闭环、PWM 执行
- 同时采集电池电压、编码器速度、IMU 数据
- 通过 Uplink 上报状态，并在 LCD 上实时展示
- MonitorTask 监控系统健康（栈水位、堆、心跳、异常 Hook）
- IWDG 看门狗兜底保证稳定性

---

## 2. 你能从这个项目获得什么

### 2.1 面试能讲清楚的“主项目闭环”
很多同学简历写了“会 FreeRTOS / 会串口 / 会 PID”，但一追问就露馅：  
**你怎么分层？怎么保证稳定？怎么做调试与监控？异常怎么兜底？**

本项目从一开始就按“可交付工程”的方式组织：
- FreeRTOS 多任务体系 + 明确职责边界
- 通信协议与解析器（带 CRC / 帧尾 / 抗粘包噪声）
- 运动学（差速模型）+ PID 控制闭环
- DMA/IDLE 接收、异步日志、监控告警、看门狗
- LCD 可视化监控（真正做到“系统状态可观测”）

### 2.2 真正工程化的“稳定性与可观测性”
- `vApplicationStackOverflowHook / vApplicationMallocFailedHook`
- `Monitor_RegisterTaskInfo()` 注册任务信息（栈大小、别名）
- Tick Hook 统计、Tickless Idle 的 Pre/Post Sleep 处理
- IWDG 任务心跳监控（某些任务卡死直接复位）

这些内容放到简历上，面试官会明显感受到差异：  
你不是写 Demo，你是在做工程。

---

## 3. 系统架构概览

### 3.1 任务划分（FreeRTOS）
项目采用 CMSIS-OS2 创建线程，统一转发到 `KaoYaApp_*` 入口，避免 freertos.c 堆业务逻辑。

任务列表（示例）：
- **CommTask**：UART DMA/IDLE 接收 + 协议解析 + 下发指令
- **ControlTask**：运动学解算 + PID 闭环 + 电机执行
- **SensorTask**：电压 / 编码器 / IMU 采集
- **UplinkTask**：状态打包上报（电压/温度/姿态/速度…）
- **LogTask**：异步日志输出（避免阻塞业务任务）
- **LcdTask**：LCD UI 刷新显示（Kaoya Robot Monitor）
- **MonitorTask**：栈/堆/任务心跳监控，异常 Hook 处理
- **IwdgTask**：看门狗喂狗/兜底复位
- **LedTask**：指示灯状态机

> 你可以在 `freertos.c` 看到任务创建、监控注册、IWDG 初始化，以及各类 Hook 的集成。

### 3.2 通信协议与解析
- 帧结构：SOF/VER/MSG/FLAGS/SEQ/LEN/PAYLOAD/CRC/EOF
- `CommParser_FeedByte()` 逐字节推进状态机
- 具备抗粘包、抗噪声能力：错误会 reset 并继续同步下一帧
- CRC16-CCITT-FALSE 校验

### 3.3 控制闭环
- 上位机给出 `vx(mm/s)` + `wz(mrad/s)`
- `CarDiff_Inverse()` 做差速逆解得到左右轮速度
- `PID_Update()` 输出 pwm_cmd（带积分限幅、输出限幅）
- `Motor_ApplyPwm()` 输出方向 GPIO + PWM（TIM3 CH1/CH2）
- 支持仿真链路：MotorSim + EncoderSim（无电机也能联调闭环）

### 3.4 传感器
- 电池电压：ADC DMA
- 编码器：真实采集/或 EncoderSim（TIM10/TIM11 PWM 方波仿真）
- IMU：MS6DSV（LSM6DSV16X）软件 I2C + ST 官方寄存器库适配

---
## 4. 目录结构
├── App/ # 业务层：各任务 app_*.c（Control/Sensor/Uplink/Monitor...）
├── Bsp/ # BSP 驱动：LCD / I2C / Encoder / Motor / IMU 等
├── Protocol/ # 协议定义/编解码/解析器（protocol.h/.c, comm_parser.c）
├── Middlewares/FreeRTOS # FreeRTOS
├── Core/ # CubeMX 生成
└── README.md

---

## 5. 快速开始

### 5.1 环境
- STM32CubeMX + STM32CubeIDE / Keil (任选其一)
- 芯片：STM32F407ZGT6
- FreeRTOS：CMSIS-OS2 接口创建任务

### 5.2 编译与运行
1. 拉取工程并打开 IDE
2. 配置好串口（用于日志/协议）
3. 下载到板子
4. 观察：
   - 串口日志输出（LogTask）
   - LCD 界面（LcdTask）
   - 任务监控信息（MonitorTask）
   - 上位机可通过协议下发速度命令

---

## 6. 调试建议（项目自带“工程化调试点”）

- **刷屏也不怕**：日志已做任务化输出（避免业务任务阻塞）
- **定位卡死**：看门狗 + 任务心跳 + MonitorTask
- **定位栈问题**：栈水位监控 + StackOverflowHook
- **定位堆问题**：MallocFailedHook
- **验证低功耗/空闲**：Tickless Idle + Pre/Post Sleep

---

## 7. Roadmap（可扩展方向）

- IMU 姿态解算：互补滤波 / Madgwick / Mahony（输出欧拉角/四元数）
- Encoder 真 AB 相采集与方向判定
- Uplink 协议扩展：姿态、里程、错误码、性能指标
- 控制增强：前馈、死区补偿、输出斜坡限制
- 故障注入：模拟传感器异常、通信错误，验证系统鲁棒性

---

## 8. License
本仓库默认遵循项目约定的 License（如 MIT/Apache-2.0/私有等），以仓库 LICENSE 为准。

---

# 9. 关于作者与获取完整资料

我是 **Kaoya（烤鸭）**。  
这个项目同时也是《**烤鸭嵌入式校招项目（下位机）**》的一部分，目标是帮应届生解决最真实的痛点：

- 想做项目但找不到切入点  
- 从零搭容易变成功能堆砌，写进简历也撑不住追问  
- 面试官追问“为什么这样设计？稳定性怎么保证？怎么调试？”答不上来  

**烤鸭项目给你一套能对标大厂工程实践的主项目闭环：**
- 源码 + 完整文档（阅读顺序/学习路线/工程解释/面试追问点）
- FreeRTOS 多任务、协议、控制、传感器、监控、看门狗、LCD 可视化
- 能跑、能改、能讲，写进简历“经得起面试追问”

?? 获取方式（示例，你按真实情况替换）：
- 源码：百度网盘发货  
- 文档：飞书文档（电脑/平板/手机可读）  
- 咨询/购买：<你的联系方式/小红书/微信/公众号/群链接>

> 如果你是 26/27/28 届，想用一个项目把“简历-项目-面试”闭环打通，欢迎来找我。



