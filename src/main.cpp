/**
 * INMOP FOC 开发板 —— SimpleFOC 有感 FOC 速度闭环（VOFA+ 可视化版）
 *
 * 硬件配置：
 *   MCU      : STM32F407VET6（PlatformIO 板级 genericSTM32F407VET6）
 *   母线电压 : 12V
 *   电机     : 3505 无刷，极对数 P = 10，Rs = 0.1Ω，Ls = 42.3µH
 *   编码器   : MT6701 磁编 ABZ，1024 PPR -> 4096 CPR，A=PA6 B=PA7
 *   PWM      : TIM1 六路互补 PE9/PE8, PE11/PE10, PE13/PE12
 *   电流采样 : CC6903SO-10A 两相（0.132 V/A），U=PC5 W=PB1
 *   电源保持 : PB10
 *
 * 控制模式：速度闭环（外环速度 PI + 内环电流 PI，foc_current）
 *
 * 串口协议：VOFA+ JustFloat（浮点数据流）
 *   - 波特率 115200
 *   - 6 个通道，每个通道 float32 小端，帧尾 4 字节 00 00 80 7F
 *   - 通道顺序：
 *       0 = 目标速度 target (rad/s)
 *       1 = 实测速度 velocity (rad/s)
 *       2 = iq 给定 current_sp (A)，即速度环输出的“目标转矩”
 *       3 = iq 实测 (A)
 *       4 = id 实测 (A)
 *       5 = 使能状态 (0/1)
 *
 * 按键（内部上拉，按下为低电平）：
 *   KEY1(PB12) 单击=使能，双击=失能
 *   KEY2(PB13) 单击=速度+5 rad/s，双击=速度-5 rad/s
 *   KEY3(PB14) 单击=反转
 */

#include <Arduino.h>
#include <SimpleFOC.h>

/* ========== 编码器 ========== */
Encoder encoder = Encoder(PA6, PA7, 1024);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

/* ========== 驱动 ========== */
BLDCDriver6PWM driver = BLDCDriver6PWM(PE9, PE8, PE11, PE10, PE13, PE12);

/* ========== 电流采样 ========== */
#define CUR_SENS_SENSITIVITY 0.132f
#define CUR_SENS_ZERO_VOLT   1.65f
#define ADC_REF_VOLT         3.3f
#define ADC_MAX              4095.0f
#define PIN_CUR_U  PC5
#define PIN_CUR_W  PB1

PhaseCurrent_s readPhaseCurrents() {
  PhaseCurrent_s c;
  float iu = (CUR_SENS_ZERO_VOLT - analogRead(PIN_CUR_U) * ADC_REF_VOLT / ADC_MAX)
             / CUR_SENS_SENSITIVITY;
  float iw = (CUR_SENS_ZERO_VOLT - analogRead(PIN_CUR_W) * ADC_REF_VOLT / ADC_MAX)
             / CUR_SENS_SENSITIVITY;
  c.a = -iu;
  c.b = iu + iw;
  c.c = -iw;
  return c;
}
void initPhaseCurrentSensing() {
  analogReadResolution(12);
  pinMode(PIN_CUR_U, INPUT);
  pinMode(PIN_CUR_W, INPUT);
}
GenericCurrentSense current_sense = GenericCurrentSense(readPhaseCurrents,
                                                        initPhaseCurrentSensing);

/* ========== 电机 ========== */
BLDCMotor motor = BLDCMotor(10);

/* ========== 按键 ========== */
#define PIN_KEY1 PB12
#define PIN_KEY2 PB13
#define PIN_KEY3 PB14

#define DEBOUNCE_MS     20
#define DOUBLE_CLICK_MS 300
#define SPEED_STEP      5.0f
#define SPEED_MAX       50.0f

float targetSpeed = 0.0f;

struct Button {
  uint8_t pin;
  bool state = false;
  bool lastReading = false;
  unsigned long lastDebounce = 0;
  uint8_t pending = 0;
  unsigned long releaseTime = 0;
};
enum ClickEvent { CLICK_NONE, CLICK_SINGLE, CLICK_DOUBLE };
Button btn1 = { PIN_KEY1 };
Button btn2 = { PIN_KEY2 };
Button btn3 = { PIN_KEY3 };

ClickEvent updateButton(Button& b) {
  ClickEvent ev = CLICK_NONE;
  unsigned long now = millis();
  bool reading = (digitalRead(b.pin) == LOW);
  if (reading != b.lastReading) b.lastDebounce = now;
  if ((now - b.lastDebounce) >= DEBOUNCE_MS) {
    if (reading != b.state) {
      if (reading) b.pending++;
      else b.releaseTime = now;
      b.state = reading;
    }
  }
  b.lastReading = reading;
  if (b.pending > 0 && !b.state && (now - b.releaseTime) >= DOUBLE_CLICK_MS) {
    ev = (b.pending == 1) ? CLICK_SINGLE : CLICK_DOUBLE;
    b.pending = 0;
  }
  return ev;
}

void handleButtons() {
  ClickEvent e1 = updateButton(btn1);
  ClickEvent e2 = updateButton(btn2);
  ClickEvent e3 = updateButton(btn3);

  if (e1 == CLICK_SINGLE) {
    motor.target = targetSpeed;
    motor.enable();
  } else if (e1 == CLICK_DOUBLE) {
    motor.disable();
  }

  if (e2 == CLICK_SINGLE) {
    targetSpeed = constrain(targetSpeed + SPEED_STEP, -SPEED_MAX, SPEED_MAX);
    if (motor.enabled) motor.target = targetSpeed;
  } else if (e2 == CLICK_DOUBLE) {
    targetSpeed = constrain(targetSpeed - SPEED_STEP, -SPEED_MAX, SPEED_MAX);
    if (motor.enabled) motor.target = targetSpeed;
  }

  if (e3 == CLICK_SINGLE) {
    targetSpeed = -targetSpeed;
    if (motor.enabled) motor.target = targetSpeed;
  }
}

/* ========== VOFA+ JustFloat 非阻塞发送 ========== */
#define FRAME_CHANNELS 6
#define FRAME_BYTES    (FRAME_CHANNELS * 4 + 4)   // 6 通道 + 4 字节帧尾

uint8_t frame[FRAME_BYTES];
volatile uint16_t frameLen = FRAME_BYTES;
volatile uint16_t framePos = FRAME_BYTES;   // 初始视为已发完，触发第一帧构建

inline void packFloat(uint8_t* dst, float v) {
  memcpy(dst, &v, 4);   // float32 小端（STM32 为小端）
}

// 构建一帧（只填充缓冲区，不发送）
void buildFrame() {
  uint8_t* p = frame;
  packFloat(p, motor.target);              p += 4;   // 0 目标速度
  packFloat(p, motor.shaft_velocity);      p += 4;   // 1 实测速度
  packFloat(p, motor.current_sp);          p += 4;   // 2 iq 给定（目标转矩）
  packFloat(p, motor.current.q);           p += 4;   // 3 iq 实测
  packFloat(p, motor.current.d);           p += 4;   // 4 id 实测
  packFloat(p, motor.enabled ? 1.0f : 0.0f); p += 4; // 5 使能
  // 帧尾：00 00 80 7F（float +inf，JustFloat 帧分隔）
  frame[FRAME_BYTES - 4] = 0x00;
  frame[FRAME_BYTES - 3] = 0x00;
  frame[FRAME_BYTES - 2] = 0x80;
  frame[FRAME_BYTES - 1] = 0x7F;
  frameLen = FRAME_BYTES;
  framePos = 0;
}

// 非阻塞发送：每圈只发缓冲区能容纳的字节
void servicePrint() {
  while (framePos < frameLen && Serial.availableForWrite() > 0) {
    Serial.write(frame[framePos++]);
  }
}

void setup() {
  // 电源保持
  pinMode(PB10, OUTPUT);
  digitalWrite(PB10, HIGH);

  // 串口（只发 VOFA+ 二进制帧，不打文本，避免污染数据流）
  Serial.setRx(PB7);
  Serial.setTx(PB6);
  Serial.begin(115200);

  // 按键
  pinMode(PIN_KEY1, INPUT_PULLUP);
  pinMode(PIN_KEY2, INPUT_PULLUP);
  pinMode(PIN_KEY3, INPUT_PULLUP);

  // 驱动
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000;
  driver.dead_zone = 0.02f;
  driver.init();

  // 电流采样（零点校准，必须在电机上电前）
  current_sense.linkDriver(&driver);
  current_sense.init();

  // 编码器
  encoder.quadrature = Quadrature::ON;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  // 电机（速度闭环）
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.phase_resistance = 0.1f;
  motor.phase_inductance = 42.3e-6f;

  motor.torque_controller = TorqueControlType::foc_current;  // 内环电流
  motor.controller = MotionControlType::velocity;            // 外环速度

  motor.voltage_limit = 2.0f;
  motor.current_limit = 1.0f;
  motor.velocity_limit = SPEED_MAX;
  motor.voltage_sensor_align = 3;

  motor.init();

  // 速度 PI（外环，保守起步，避免振荡）
  motor.PID_velocity.P = 0.09f;
  motor.PID_velocity.I = 1.0f;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.05f;  // 加强速度滤波，压噪声

  // 内环电流 PI
  motor.tuneCurrentController(500.0f);

  motor.initFOC();
  motor.disable();
}

void loop() {
  motor.loopFOC();
  motor.move();
  handleButtons();

  // 上一帧发完后构建新帧，持续以串口带宽上限速率发送
  if (framePos >= frameLen) {
    buildFrame();
  }
  servicePrint();
}