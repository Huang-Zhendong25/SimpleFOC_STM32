# SimpleFOC_STM32 —— INMOP FOC 开发板 有感 FOC 电机驱动

基于 [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) 的 STM32 有感 FOC 电机驱动底层项目，目标是逐层实现完整的电机驱动闭环（电压开环 → 电流闭环 → 速度闭环 → 位置闭环），面向具身智能电机驱动岗位的底层能力建设。

- 开发环境：VSCode + PlatformIO（STM32duino）
- 当前阶段：**电压开环（velocity_openloop）**

---

## 一、硬件平台

| 项目 | 参数 |
|---|---|
| 主控 | STM32F407VET6（PlatformIO 板级 `genericSTM32F407VET6`） |
| 母线电压 | 12V（板子最大 30V / 30A） |
| 电机 | 3505 无刷电机，极对数 P=10，Rs=0.1Ω，Ls=42.3µH |
| 编码器 | MT6701 磁编，ABZ 增量模式，1024 PPR → 四倍频 4096 CPR，A=PA6 B=PA7 |
| PWM | TIM1 六路互补：A=PE9/PE8，B=PE11/PE10，C=PE13/PE12 |
| 电流采样 | 2×CC6903SO-10A（0.132 V/A），U=PC5 W=PB1，V 相由 iu+iv+iw=0 推算 |
| 电源保持 | PB10（软开关机 PWR 引脚，拉高保持供电） |
| 按键 | KEY1=PB12，KEY2=PB13，KEY3=PB14（内部上拉，按下为低电平） |

---

## 二、开发路线（整体流程）

FOC 按“由内到外、逐层闭环”推进，这也是本项目的主线：

```
电压开环  →  电流闭环  →  速度闭环  →  位置闭环
（当前）      （待做）      （待做）      （待做）
```

| 阶段 | 控制目标 | 反馈量 | 电角度来源 |
|---|---|---|---|
| 电压开环 | 无（只让电机转起来） | 无 | 积分自生成 |
| 电流闭环 | iq（转矩电流） | 三相电流 | 编码器 |
| 速度闭环 | 转速 | 编码器 | 编码器 |
| 位置闭环 | 转角 | 编码器 | 编码器 |

### 各阶段原理简述

1. **电压开环**：无反馈，电角度 `θe = ∫ωdt × P` 由积分生成，施加固定电压。用于验证 PWM、编码器、电流采样、串口等硬件链路。
2. **电流闭环**：采样三相电流 → Clarke → Park(真实 θe) → 电流 PI → 反 Park → SVPWM。控制 iq 即控制转矩。
3. **速度闭环**：在电流环外再套一个速度 PI，速度误差 → iq 给定，控制转速。
4. **位置闭环**：最外层套位置 P（或 P+D），位置误差 → 速度给定，控制转角。

---

## 三、FOC 原理总览

完整 FOC 坐标变换链路：

```
三相电流 ia, ib, ic
   │ Clarke（静止变换，定子固定）
   ▼
iα, iβ
   │ Park（旋转 θe，跟随转子）
   ▼
id, iq
   │ 电流 PI
   ▼
ud, uq
   │ 反 Park
   ▼
uα, uβ
   │ SVPWM
   ▼
6 路 PWM → 逆变桥 → 电机三相
```

- **Clarke**：三相 → 两相静止（α 沿 A 相，β 超前 90°）。
- **Park**：两相静止 → 两相旋转（d 沿转子磁场，q 超前 90°），需要电角度 θe。
- **d 轴** = 转子永磁体 N 极方向（励磁/弱磁），**q 轴** = 转矩方向。
- 约定：**iq 控转矩，id 控励磁**（表贴式电机一般保持 id = 0）。

---

## 四、当前实现：电压开环

### 4.1 原理

电压开环下**不采样电流、不使用编码器反馈**：

- `motor.target` 是目标速度（rad/s）；
- 电角度由 `shaft_angle += target × Δt` 积分生成，`θe = shaft_angle × 极对数`；
- 施加固定幅值电压 `voltage_limit`（当前 2V）；
- 编码器只用来测速/显示，不参与控制。

数据流只走最后两段：`反 Park（自生成 θe）→ SVPWM → PWM`。

### 4.2 代码结构（`src/main.cpp`）

- `Encoder encoder(PA6, PA7, 1024)`：编码器，`quadrature = Quadrature::ON` 四倍频解码，`enableInterrupts(doA, doB)` 挂中断。
- `BLDCDriver6PWM driver(PE9, PE8, ...)`：TIM1 六路互补 PWM。
- `GenericCurrentSense current_sense`：CC6903 两相电流采样（开环下只初始化、不参与控制）。
- `BLDCMotor motor(10)`：极对数 10，`controller = MotionControlType::velocity_openloop`。
- 按键单击/双击状态机：见 4.3。

### 4.3 按键控制

| 按键 | 单击 | 双击 |
|---|---|---|
| KEY1 (PB12) | 使能电机 | 失能电机 |
| KEY2 (PB13) | 速度 +5 | 速度 −5 |
| KEY3 (PB14) | 反转 | — |

速度步长、上限、双击判定窗口等参数可在 `main.cpp` 顶部的宏里修改。

### 4.4 编译与烧录

```bash
pio run              # 编译
pio run -t upload    # 烧录（ST-Link SWD）
```

**烧录注意（重要，踩坑记录）**：

1. `platformio.ini` 里 `upload_speed = 1000`：SWD 时钟降到 1MHz，否则这块软开关机板在烧录/复位瞬间通信不稳，报 `HardFault / Polling failed`。
2. 该板级烧录完成后**不会自动复位运行**，烧完需要**断电重启板子**。
3. `PB10` 拉高保持供电（软开关机电源保持引脚）。
4. 板子 USB 串口（CH340）接在 USART1 的 PB6(TX)/PB7(RX)，代码里已用 `Serial.setRx(PB7); Serial.setTx(PB6);` 重映射。
5. 电流零点校准（`current_sense.init()`）必须在电机上电/转动之前完成。

---

## 五、后续阶段（待补充）

> 以下为占位说明，后续每实现一个阶段，就在对应小节补充“原理 + 实现方法”。

### 5.1 电流闭环（待实现）

- 原理：控制 d/q 轴电流（iq 即转矩），实现真正的力矩控制。
- 实现：设置 `motor.torque_controller = TorqueControlType::foc_current`、`motor.controller = MotionControlType::torque`，用 `tuneCurrentController()` 或手动整定电流 PI。

### 5.2 速度闭环（待实现）

- 原理：在电流环外加速度 PI 外环，速度误差输出为 iq 给定。
- 实现：`motor.controller = MotionControlType::velocity`，整定速度 PI。

### 5.3 位置闭环（待实现）

- 原理：最外层加位置环，位置误差输出为速度给定。
- 实现：`motor.controller = MotionControlType::angle`，整定位置 P。

---

## 六、目录结构

```
SimpleFOC_STM32/
├── platformio.ini      # PlatformIO 工程配置（板级、库、烧录参数）
├── src/main.cpp        # 主程序（当前：电压开环按键控制版）
├── include/            # 头文件（预留）
├── lib/                # 本地库（预留）
└── test/               # 测试（预留）
```