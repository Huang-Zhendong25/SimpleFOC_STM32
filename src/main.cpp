/**
 * INMOP FOC 开发板 —— SimpleFOC 有感 FOC 电流闭环测试（按键控制版）
 *
 * 硬件配置（依据《INMOP_FOC开发板硬件详细设计说明》及板载官方例程）：
 *   MCU      : STM32F407VET6（PlatformIO 板级 genericSTM32F407VET6）
 *   母线电压 : 12V
 *   电机     : 3505 无刷，极对数 P = 10，Rs = 0.1Ω，Ls = 42.3µH
 *   编码器   : MT6701 磁编 ABZ，1024 PPR -> 4096 CPR，A=PA6 B=PA7
 *   PWM      : TIM1 六路互补 PE9/PE8, PE11/PE10, PE13/PE12
 *   电流采样 : CC6903SO-10A 两相（0.132 V/A），U=PC5 W=PB1
 *   电源保持 : PB10
 *
 * 控制模式：电流闭环（foc_current，直接控制 iq = 转矩电流）
 * 按键（内部上拉，按下为低电平）：
 *   KEY1(PB12) 单击=使能，双击=失能
 *   KEY2(PB13) 单击=电流+0.1A，双击=电流-0.1A
 *   KEY3(PB14) 单击=反转（iq 取反）
 */

#include <Arduino.h>
#include <SimpleFOC.h>

/* ========== 编码器：MT6701 ABZ，软件正交解码 ========== */
Encoder encoder = Encoder(PA6, PA7, 1024);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

/* ========== 驱动：BLDCDriver6PWM（TIM1 六路互补） ========== */
BLDCDriver6PWM driver = BLDCDriver6PWM(PE9, PE8, PE11, PE10, PE13, PE12);

/* ========== 电流采样：CC6903SO-10A 两相（0.132 V/A） ========== */
#define CUR_SENS_SENSITIVITY 0.132f   // [V/A]
#define CUR_SENS_ZERO_VOLT   1.65f    // VCC/2
#define ADC_REF_VOLT         3.3f
#define ADC_MAX              4095.0f  // 12 位 ADC
#define PIN_CUR_U  PC5
#define PIN_CUR_W  PB1

PhaseCurrent_s readPhaseCurrents() {
  PhaseCurrent_s c;
  // 注意：如果电流环发散 / iq 为负且电机狂抖，说明方向反了，把下面三行整体取负。
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

/* ========== 电机：极对数 10 ========== */
BLDCMotor motor = BLDCMotor(10);

/* ========== 按键（内部上拉，按下为 LOW） ========== */
#define PIN_KEY1 PB12
#define PIN_KEY2 PB13
#define PIN_KEY3 PB14

#define DEBOUNCE_MS     20
#define DOUBLE_CLICK_MS 300
#define CURRENT_STEP    0.1f
#define CURRENT_MAX     1.0f

float targetCurrent = 0.0f;

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

  if (reading != b.lastReading) {
    b.lastDebounce = now;
  }
  if ((now - b.lastDebounce) >= DEBOUNCE_MS) {
    if (reading != b.state) {
      if (reading) {
        b.pending++;
      } else {
        b.releaseTime = now;
      }
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
    motor.target = targetCurrent;
    motor.enable();
    Serial.println("KEY1 single: motor enabled");
  } else if (e1 == CLICK_DOUBLE) {
    motor.disable();
    Serial.println("KEY1 double: motor disabled");
  }

  if (e2 == CLICK_SINGLE) {
    targetCurrent = constrain(targetCurrent + CURRENT_STEP, -CURRENT_MAX, CURRENT_MAX);
    if (motor.enabled) motor.target = targetCurrent;
    Serial.print("KEY2 single: iq target = ");
    Serial.println(targetCurrent);
  } else if (e2 == CLICK_DOUBLE) {
    targetCurrent = constrain(targetCurrent - CURRENT_STEP, -CURRENT_MAX, CURRENT_MAX);
    if (motor.enabled) motor.target = targetCurrent;
    Serial.print("KEY2 double: iq target = ");
    Serial.println(targetCurrent);
  }

  if (e3 == CLICK_SINGLE) {
    targetCurrent = -targetCurrent;
    if (motor.enabled) motor.target = targetCurrent;
    Serial.print("KEY3 single: reversed, iq target = ");
    Serial.println(targetCurrent);
  }
}

/* ========== 非阻塞串口打印（避免阻塞 FOC 循环导致顿挫） ========== */
#define NBUF 96
char nbuf[NBUF];
volatile uint8_t nlen = 0;
volatile uint8_t npos = 0;

// 把一行状态格式化到 nbuf（只格式化，不发送）
void printStatus() {
  char iq[12], id[12], tg[12], vl[12];
  dtostrf(motor.current.q, 0, 3, iq);
  dtostrf(motor.current.d, 0, 3, id);
  dtostrf(targetCurrent, 0, 2, tg);
  dtostrf(encoder.getVelocity(), 0, 1, vl);
  nlen = snprintf(nbuf, NBUF, "en:%d iq:%s id:%s target:%s vel:%s\r\n",
                  motor.enabled ? 1 : 0, iq, id, tg, vl);
  npos = 0;
}

// 每次 loop 调用一次：只发送缓冲区能容纳的字节，绝不阻塞
void servicePrint() {
  while (npos < nlen && Serial.availableForWrite() > 0) {
    Serial.write(nbuf[npos++]);
  }
}

void setup() {
  // 电源保持：PB10 拉高
  pinMode(PB10, OUTPUT);
  digitalWrite(PB10, HIGH);

  // 串口：只用于打印
  Serial.setRx(PB7);
  Serial.setTx(PB6);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

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

  // ---- 电机（电流闭环配置）----
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.phase_resistance = 0.1f;
  motor.phase_inductance = 42.3e-6f;

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  motor.voltage_limit = 2.0f;
  motor.current_limit = 1.0f;
  motor.voltage_sensor_align = 3;

  motor.init();
  motor.tuneCurrentController(500.0f);

  if (motor.initFOC()) {
    Serial.println("FOC ready.");
  } else {
    Serial.println("FOC init failed!");
  }
  motor.disable();

  Serial.println("KEY1: single=enable, double=disable");
  Serial.println("KEY2: single=+0.1A, double=-0.1A");
  Serial.println("KEY3: single=reverse");
}

void loop() {
  motor.loopFOC();
  motor.move();
  handleButtons();

  // 每 2 秒准备一次状态行（只格式化，不阻塞）
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    printStatus();
  }
  servicePrint();  // 非阻塞逐字节发送
}