# 单车自平衡车

基于 **英飞凌 TC264D** + **IMU660RC (LSM6DSV16X)** 的单轮自平衡自行车项目，使用逐飞 TC264 开源库。

## 控制架构

三级串级 PID 控制，由中断驱动：

```
航向环 (20ms)  →  角度环 (5ms)  →  角速度环 (1ms)  →  舵机 PWM
  yaw误差          pitch误差          gyro_y误差
  → 目标倾角        → 目标角速度        → 舵机输出
```

| 中断 | 周期 | 功能 |
|------|------|------|
| CCU60_CH0 | 1ms | 系统节拍 `uwtick` |
| CCU60_CH1 | 1ms | IMU 数据拷贝 + 角速度环 |
| CCU61_CH0 | 5ms | 角度环 |
| CCU61_CH1 | 20ms | 航向环 + 编码器采集 + 电机速度环 |
| ERU_CH3 (INT2) | IMU 数据就绪 | SFLP 四元数 + 陀螺仪 + 加速度计读取 |

## 主要特性

- **SFLP 480Hz 四元数融合**：芯片内部加速度计+陀螺仪融合，输出欧拉角，无需手写互补/卡尔曼滤波
- **裁剪均值零偏校准**：开机调用 `imu660rc_bias_calibrate(2000)` 进行 2 秒静止校准，偏差注入 SFLP 内部（AN5763 Section 6.5.1）
- **Pitch 毛刺检测**：单帧跳变超过 30° 自动用前一帧替代
- **航向斜坡平滑**：目标航向每 20ms 最多变化 1.5°，防止猛打方向盘
- **330Hz 舵机 PWM**：相比 50Hz 更高的控制分辨率
- **FOC 无刷电机**：UART 协议驱动，启动 300ms kick + PID 速度闭环
- **SW2 延迟锁定航向**：电机启动 500ms 后再锁定航向，避免启动晃动引入偏转
- **IPS114 LCD 显示** + **无线串口遥测**（10ms 周期）
- **按键调速**：K1/K2 增减电机目标速度

## 硬件

| 模块 | 型号 / 接口 |
|------|------------|
| 主控 | 英飞凌 TC264D (TriCore 双核) |
| IMU | IMU660RC (ST LSM6DSV16X)，SPI + INT2 中断 |
| 舵机 | 330Hz PWM，ATOM1_CH1_P33_9 |
| 电机驱动 | FOC 无刷驱动板，UART3 @ 460800 baud |
| 编码器 | 20ms 差分采样 + 一阶低通滤波 |
| 显示 | IPS114 SPI LCD (160×128) |
| 无线 | 逐飞无线串口模块（遥测 + 调参） |

## 目录结构

```
code/
├── app/                  # 应用层
│   ├── balance_app.c     # 三级串级平衡控制
│   ├── imu_app.c         # IMU 数据处理 + SFLP 融合
│   ├── motor_app.c       # 电机速度控制
│   ├── servo_app.c       # 舵机输出
│   ├── hmi_app.c         # 屏幕显示 + 按键 + 遥测
│   └── schedule.c        # 协作式调度器
├── driver/
│   ├── pid/              # PID 控制器库
│   └── motor/            # FOC 驱动板 UART 协议
└── test/
    └── justfloat.c       # VOFA+ 实时波形调试

user/
├── cpu0_main.c           # 初始化入口
├── isr.c                 # 中断路由
└── isr_config.h          # 中断优先级配置

libraries/                # 逐飞 TC264 开源库（含 IMU660RC SFLP 驱动）
```

## PID 参数

| 环路 | Kp | Ki | Kd | 输出限幅 | 备注 |
|------|-----|-----|-----|---------|-----|
| 航向环 | 0.15 | 0 | 0.05 | ±1.5° | 单车 3.5° 对 angle 环冲击过大，已收紧 |
| 角度环 | 22.0 | 0.005 | -0.3 | ±250 deg/s | INT_LIMIT 30 |
| 角速度环 | 5.2 | 0 | 0 | ±1300 | 与舵机 SAFE_LIMIT 对齐；输入 ±1500 dps；INT_LIMIT 200（KI=0 时哑） |
| 电机速度环 | 0.15 | 0.07 | 0 | ±10000 | INT_LIMIT ±5000 |

**饱和保护**：
- 角速度环条件积分（撞限+同号误差→停止累加），当前 KI=0 是哑保护，调 KI≠0 时防止 windup
- 角速度环 OUT_LIMIT (1300) = 舵机 SAFE_LIMIT (1300) → PID 看得见自己被夹

**零位锁定**：
- 开机 `balance_capture_zero_static`：32×5ms=160ms 阻塞采样取 pitch 平均
- SW2 运行时锁零：`gyro_y_rate < 30 dps` 静止守卫，否则保留上次零位

## 开发环境

- **IDE**：AURIX Development Studio (Eclipse)
- **工具链**：TriCore GCC
- **库**：逐飞 TC264 开源库
- **调试**：VOFA+ (JustFloat 协议) + 无线串口
