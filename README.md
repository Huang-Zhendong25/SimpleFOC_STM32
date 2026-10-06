# SimpleFOC_STM32 —— INMOP FOC 开发板 有感 FOC 电机驱动

基于 [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) 的 STM32 有感 FOC 电机驱动底层项目，目标逐层实现完整闭环（电压开环 → 电流闭环 → 速度闭环 → 位置闭环），面向具身智能电机驱动岗位的底层能力建设。

- 开发环境：VSCode + PlatformIO（STM32duino）
- 当前阶段：**速度闭环（velocity + foc_current 级联）**（已依次完成：电压开环、电流闭环、速度闭环）

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

```
电压开环 ✅  →  电流闭环 ✅  →  速度闭环 ✅（当前）  →  位置闭环（待做）
```

| 阶段 | 状态 | 控制目标 | 反馈量 | 电角度来源 |
|---|---|---|---|---|
| 电压开环 | ✅ 已实现 | 无（验证硬件链路） | 无 | 积分自生成 |
| 电流闭环 | ✅ 已实现 | iq（转矩电流） | 三相电流 | 编码器 |
| 速度闭环 | ✅ 已实现（当前） | 转速 | 编码器 | 编码器 |
| 位置闭环 | 待做 | 转角 | 编码器 | 编码器 |

---

## 三、FOC 原理总览

完整 FOC 坐标变换链：

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
- 约定：**iq 控转矩，id 控励磁**（表贴式电机一般 id = 0）。

---

## 四、实现一：电压开环（velocity_openloop）

### 4.1 原理

电压开环下**不采样电流、不使用编码器反馈**：

- `motor.target` 是目标速度（rad/s）；
- 电角度由 `shaft_angle += target × Δt` 积分生成，`θe = shaft_angle × 极对数`；
- 施加固定幅值电压 `voltage_limit`（2V）；
- 编码器只用来测速/显示，不参与控制。

数据流只走最后两段：

```
反 Park（自生成 θe）→ SVPWM → PWM
```

### 4.2 代码结构（`src/main.cpp`）

- `Encoder encoder(PA6, PA7, 1024)`：编码器，`quadrature=ON` 四倍频，`enableInterrupts(doA, doB)` 挂中断。
- `BLDCDriver6PWM driver(PE9, PE8, ...)`：TIM1 六路互补 PWM。
- `GenericCurrentSense current_sense`：CC6903 两相电流采样（开环下只初始化、不参与控制）。
- `BLDCMotor motor(10)`：极对数 10，`controller = MotionControlType::velocity_openloop`。
- 按键单击/双击状态机（见 4.3）。

### 4.3 按键控制

| 按键 | 单击 | 双击 |
|---|---|---|
| KEY1 (PB12) | 使能电机 | 失能电机 |
| KEY2 (PB13) | 速度 +5 | 速度 −5 |
| KEY3 (PB14) | 反转 | — |

速度步长、上限、双击判定窗口等参数在 `main.cpp` 顶部宏里可改。

### 4.4 编译与烧录

```bash
pio run              # 编译
pio run -t upload    # 烧录（ST-Link SWD）
```

**烧录注意（踩坑记录）**：

1. `upload_speed = 1000`：SWD 降到 1MHz，否则软开关机板烧录/复位瞬间通信不稳，报 `HardFault / Polling failed`。
2. 该板级烧录后不会自动复位运行，烧完需**断电重启板子**。
3. `PB10` 拉高保持供电（软开关机电源保持）。
4. 板子 USB 串口（CH340）接在 USART1 的 PB6(TX)/PB7(RX)，代码里用 `Serial.setRx(PB7)/setTx(PB6)` 重映射。
5. 电流零点校准（`current_sense.init()`）必须在电机上电/转动之前完成。

---

## 五、实现二：电流闭环（foc_current）

### 5.1 原理

电流闭环的控制目标是 **iq（q 轴电流，即转矩电流）**：

```
target(iq 给定) → 电流 PI → uq/ud → 反Park → SVPWM → PWM → 电机
                      ↑
              实测 iq/id（Clarke + Park 反馈）
```

- 电角度 θe 来自**编码器实测**（不再积分）；
- 三相电流经 Clarke、Park 得到实测 id/iq；
- dq 两个电流 PI 分别控制 id→0、iq→target；
- 电压从“目标”降级为“手段”（PI 的输出，受 `voltage_limit` 限幅）。

### 5.2 代码结构（相对电压开环的关键改动）

```cpp
motor.phase_resistance = 0.1f;      // Rs，用于 PI 整定
motor.phase_inductance = 42.3e-6f;  // Ls

motor.torque_controller = TorqueControlType::foc_current;  // 电流闭环
motor.controller        = MotionControlType::torque;       // target = iq 电流

motor.voltage_limit  = 2.0f;   // 电流 PI 输出电压上限
motor.current_limit  = 1.0f;   // 电流目标上限（安全）
motor.tuneCurrentController(500.0f);  // 按 Rs/Ls/带宽自动算 PI
```

`tuneCurrentController(500)` 内部公式：

```
P = Ls · 2π · BW = 42.3e-6 · 2π · 500 ≈ 0.133
I = Rs · 2π · BW = 0.1 · 2π · 500 ≈ 314
```

### 5.3 按键控制

| 按键 | 单击 | 双击 |
|---|---|---|
| KEY1 (PB12) | 使能电机 | 失能电机 |
| KEY2 (PB13) | 电流 +0.1A | 电流 −0.1A |
| KEY3 (PB14) | 反转（iq 取反） | — |

电流步长 `CURRENT_STEP`、上限 `CURRENT_MAX` 等可在 `main.cpp` 顶部宏里改。

### 5.4 ⚠️ 注意事项（重要）

1. **必须手动确认三相电流方向**：
   - 本项目用的 `GenericCurrentSense`，其 `driverAlign()` 是空操作，**不会自动校正电流方向/相序**；
   - 若方向反了，电流环会变成正反馈（iq 为负 / 发散 / 电机狂抖）；
   - 当前代码已按实测方向对 `readPhaseCurrents()` 的三相电流整体取负（`c.a = -iu` 等），**换板子或改接线后需重新确认**。
   - 验证方法：握轴/加负载，给正 iq（如 0.2A），看 `iq` 是否为正、电机是否产生正向转矩；否则取负。

2. **测试电流环必须加负载或握轴**：torque 模式 + 空载会飞转到最高速（反电动势 = voltage_limit），导致 iq 振荡、顿挫，这是正常现象，不是代码 bug。

3. **电流从 0.1~0.2A 起步**，`current_limit=1A` 是保护上限，不要轻易调大（Rs=0.1Ω 很小，电流容易起大）。

### 5.5 编译与烧录

```bash
pio run              # 编译
pio run -t upload    # 烧录（ST-Link SWD）
```

**烧录/开发注意事项（踩坑记录）**：

1. `upload_speed = 1000`：SWD 降到 1MHz，否则软开关机板烧录/复位瞬间通信不稳，报 `HardFault / Polling failed`。
2. 该板级烧录后**不会自动复位运行**，烧完需**断电重启板子**。
3. `PB10` 拉高保持供电（软开关机电源保持）。
4. **`build_flags = -DSERIAL_UART_INSTANCE=1`**：`genericSTM32F407VET6` 默认把 `Serial` 映射到 UART4，而板子 CH340 接在 USART1(PB6/PB7)，会导致串口收发中断失效（只能打印、无法接收、打印会停）。强制 `Serial = USART1` 修复（代码里保留 `Serial.setRx(PB7)/setTx(PB6)`）。
5. 串口打印用**非阻塞**方式（`printStatus` + `servicePrint`），避免阻塞 FOC 循环造成顿挫。
6. 电流零点校准（`current_sense.init()`）必须在电机上电/转动之前完成。

---

## 六、实现三：速度闭环（velocity + foc_current 级联）

### 6.1 原理

速度闭环是**级联控制**：在电流环外再套一个速度 PI 外环。

```
target(速度) → 速度PI(外环) → iq* → 电流PI(内环) → uq/ud → SVPWM → PWM → 电机
                   ↑                        ↑
              实测速度(编码器)        实测电流(CC6903)
```

- 外环**速度 PI**：速度误差 → iq 给定（转矩），控制转速；
- 内环**电流 PI**：iq 跟踪（复用实现二）；
- 与电压开环的“速度”不同：这里是**转子实际转速的反馈控制**（有编码器反馈、被保证），空载也不会飞转。

### 6.2 代码结构（相对电流闭环的关键改动）

```cpp
motor.torque_controller = TorqueControlType::foc_current;  // 内环电流闭环（保留）
motor.controller        = MotionControlType::velocity;      // 外环速度闭环（由 torque 改为 velocity）

motor.velocity_limit = 50;   // 最大转速 rad/s

// 速度 PI（外环）
motor.PID_velocity.P = 0.5f;
motor.PID_velocity.I = 10.0f;
motor.PID_velocity.D = 0.0f;
motor.LPF_velocity.Tf = 0.01f;   // 速度低通滤波

motor.tuneCurrentController(500.0f);  // 内环电流 PI（保留）
```

### 6.3 按键控制

| 按键 | 单击 | 双击 |
|---|---|---|
| KEY1 (PB12) | 使能电机 | 失能电机 |
| KEY2 (PB13) | 速度 +5 rad/s | 速度 −5 rad/s |
| KEY3 (PB14) | 反转 | — |

速度步长 `SPEED_STEP=5`、上限 `SPEED_MAX=50` 在 `main.cpp` 顶部宏里可改。

### 6.4 ⚠️ 注意事项（重要）

1. **速度 PI 参数尚未完成整定**：当前 `PID_velocity.P=0.5、I=10、D=0` 是**起步值**，需根据实测整定：
   - 速度超调 / 震荡 → 减小 `P`；
   - 稳态有偏差 / 跟不上 → 增大 `I`；
   - 抖得厉害 → 增大 `LPF_velocity.Tf`（0.01 → 0.02）。
2. 速度环空载也能稳速，这是相对电流环（空载飞转）的主要改进。
3. 内环电流方向仍需与实现二一致地确认（`readPhaseCurrents()` 已取负）。

### 6.5 编译与烧录

```bash
pio run              # 编译
pio run -t upload    # 烧录（ST-Link SWD）
```

烧录/开发注意事项与 5.5 相同（`upload_speed=1000`、断电重启、PB10、`-DSERIAL_UART_INSTANCE=1`、非阻塞打印、零点校准时序）。

---

## 七、后续阶段（待补充）

> 以下为占位说明，后续每实现一个阶段就补充“原理 + 实现”。

### 7.1 位置闭环（待实现）

- 原理：最外层加位置环，位置误差 → 速度给定，控制转角。
- 实现：`motor.controller = MotionControlType::angle`，整定位置 P。

---

## 八、目录结构

```
SimpleFOC_STM32/
├── platformio.ini      # PlatformIO 工程配置（板级、库、烧录参数、串口修复宏）
├── src/main.cpp        # 主程序（当前：速度闭环按键控制版）
├── include/            # 头文件（预留）
├── lib/                # 本地库（预留）
└── test/               # 测试（预留）
```