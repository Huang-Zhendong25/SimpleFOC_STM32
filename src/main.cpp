/**
 * INMOP FOC 开发板 —— SimpleFOC 有感 FOC 底层测试程序
 *
 * 硬件配置（依据《INMOP_FOC开发板硬件详细设计说明》及板载官方例程）：
 *   MCU      : STM32F407VET6（PlatformIO 板级 black_f407ve）
 *   母线电压 : 12V（板子最大 30V / 30A）
 *   电机     : 3505 无刷电机，极对数 P = 10，Rs = 0.1Ω，Ls = 42.3µH
 *   编码器   : MT6701 磁编码器，ABZ 增量模式，1024 PPR -> 4 倍频 = 4096 CPR
 *              A -> PA6，B -> PA7，Z 未接
 *   PWM      : TIM1 六路互补
 *              A 相：PE9(高)/PE8(低)，B 相：PE11(高)/PE10(低)，C 相：PE13(高)/PE12(低)
 *   电流采样 : 2 路 CC6903SO-10A 霍尔电流传感器（10A，灵敏度 0.132 V/A），两相采样 + 第三相推算
 *              U 相 -> PC5 (ADC1_IN15)，W 相 -> PB1 (ADC1_IN9)
 *   电源保持 : PB10（软开关机 PWR 引脚，拉高保持供电）
 *
 * 本程序采用方式A：SimpleFOC 内置 Encoder（软件正交解码，零额外依赖），
 * 先以 velocity_openloop 模式验证 PWM、编码器与串口链路。
 * 注意：velocity_openloop 的 target 是“轴速度(rad/s)”，施加的电压幅值固定为 motor.voltage_limit。
 */

#include <Arduino.h>
#include <SimpleFOC.h>

/* =========================================================================
 * 1. 编码器：MT6701（ABZ 增量模式，方式A：软件正交解码）
 *    构造参数：A 引脚, B 引脚, PPR（4 倍频自动得 CPR = 4096）
 * ========================================================================= */
Encoder encoder = Encoder(PA6, PA7, 1024);

// 编码器 A/B 通道中断回调（软件正交解码必须提供，否则不会计数）
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

/* =========================================================================
 * 2. 驱动：BLDCDriver6PWM，TIM1 六路互补 PWM
 *    构造参数顺序：phA_h, phA_l, phB_h, phB_l, phC_h, phC_l
 * ========================================================================= */
BLDCDriver6PWM driver = BLDCDriver6PWM(PE9, PE8, PE11, PE10, PE13, PE12);

/* =========================================================================
 * 3. 电流采样：CC6903SO-10A 霍尔电流传感器（两相采样）
 *    输出电压 V = VCC/2 + 0.132*I，灵敏度 0.132 V/A（10A 量程）
 * ========================================================================= */
#define CUR_SENS_SENSITIVITY 0.132f   // [V/A]
#define CUR_SENS_ZERO_VOLT   1.65f    // VCC/2，VCC = 3.3V
#define ADC_REF_VOLT         3.3f     // ADC 参考电压
#define ADC_MAX              4095.0f  // 12 位 ADC

#define PIN_CUR_U  PC5               // U 相电流采样
#define PIN_CUR_W  PB1               // W 相电流采样（V 相由 iu+iv+iw=0 推算）

// 用户自定义回调：读取三相电流，返回单位安培(A)
PhaseCurrent_s readPhaseCurrents() {
  PhaseCurrent_s c;
  // ADC 原始值 -> 电压 -> 电流：(零漂 - raw) * Vref/4095 / 灵敏度
  float iu = (CUR_SENS_ZERO_VOLT - analogRead(PIN_CUR_U) * ADC_REF_VOLT / ADC_MAX)
             / CUR_SENS_SENSITIVITY;
  float iw = (CUR_SENS_ZERO_VOLT - analogRead(PIN_CUR_W) * ADC_REF_VOLT / ADC_MAX)
             / CUR_SENS_SENSITIVITY;

  c.a = iu;          // U 相
  c.b = -iu - iw;    // V 相 = -U - W
  c.c = iw;          // W 相
  return c;
}

// 用户自定义回调：电流采样初始化（配置 ADC 分辨率与引脚）
void initPhaseCurrentSensing() {
  analogReadResolution(12);
  pinMode(PIN_CUR_U, INPUT);
  pinMode(PIN_CUR_W, INPUT);
}

GenericCurrentSense current_sense = GenericCurrentSense(readPhaseCurrents,
                                                        initPhaseCurrentSensing);

/* =========================================================================
 * 4. 电机：极对数 P = 10
 * ========================================================================= */
BLDCMotor motor = BLDCMotor(10);

// 串口命令行（用于实时调参）
Commander commander = Commander(Serial);

// 命令行回调：设置 target（velocity_openloop 下 target 单位是 轴速度 rad/s）
void doTarget(char* cmd) {
  commander.scalar(&motor.target, cmd);
}

// 命令行回调：使能/失能电机（失能后不施加电压，用于安全停机）
void doEnable(char* cmd) {
  if (motor.enabled) {
    motor.disable();
    Serial.println("Motor disabled.");
  } else {
    motor.enable();
    Serial.println("Motor enabled.");
  }
}

void setup() {
  // ★ 关键：PB10 是软开关机的“电源保持”引脚（官方例程 PWR_Pin = PB10）。
  //   拉高才能让板子在松开电源键 / 复位后继续保持供电，否则板子会掉电。
  pinMode(PB10, OUTPUT);
  digitalWrite(PB10, HIGH);

  // ★ 板子 USB 串口(CH340) 接在 USART1 的 PB6(TX)/PB7(RX)，需重映射（默认是 PA9/PA10）
  Serial.setRx(PB7);
  Serial.setTx(PB6);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // ---- 驱动初始化 ----
  driver.voltage_power_supply = 12;   // 母线电压 12V
  driver.pwm_frequency = 20000;       // PWM 频率 20kHz（与官方例程一致）
  driver.dead_zone = 0.02f;           // 死区 2%
  driver.init();

  // ---- 电流采样初始化（必须在电机上电/转动前完成零点校准）----
  current_sense.linkDriver(&driver);
  current_sense.init();

  // ---- 编码器初始化（方式A：软件正交解码）----
  encoder.quadrature = Quadrature::ON; // 4 倍频：1024 PPR -> 4096 CPR
  encoder.init();
  encoder.enableInterrupts(doA, doB);  // 必须传入 A/B 回调，否则不计数
  motor.linkSensor(&encoder);

  // ---- 电机初始化 ----
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // SVPWM

  motor.voltage_limit = 2;                    // 开环施加电压幅值（V），从小开始，避免堵转大电流
  motor.voltage_sensor_align = 3;             // 传感器对齐电压（只在 initFOC 对齐瞬间使用）
  motor.controller = MotionControlType::velocity_openloop; // 第一步：开环测速
  motor.init();

  // ---- 传感器/电流对齐 + 启动 FOC ----
  if (motor.initFOC()) {
    Serial.println("FOC ready.");
  } else {
    Serial.println("FOC init failed!");
  }

  // 对齐完成后先失能，避免 target=0 时 velocity_openloop 仍以 voltage_limit 锁住电机产生堵转电流
  motor.disable();

  Serial.println("Commands (115200):");
  Serial.println("  E      -> enable/disable motor");
  Serial.println("  T<val> -> set target velocity [rad/s], e.g. T20");
  commander.add('T', doTarget, "target velocity [rad/s]");
  commander.add('E', doEnable, "enable/disable motor");
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();

  // 每秒打印一次编码器角度与速度，验证 MT6701 + 软件正交解码链路
  static unsigned long last = 0;
  if (millis() - last > 1000) {
    last = millis();
    Serial.print("enabled:");
    Serial.print(motor.enabled ? "1" : "0");
    Serial.print("  angle:");
    Serial.print(encoder.getAngle());
    Serial.print(" rad  velocity:");
    Serial.print(encoder.getVelocity());
    Serial.print(" rad/s  target:");
    Serial.println(motor.target);
  }

  // 串口命令行处理
  commander.run();
}
