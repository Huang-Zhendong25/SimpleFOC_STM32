/**
 * INMOP FOC 开发板 —— SimpleFOC 有感 FOC 位置闭环测试（按键控制版）
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
 * 控制模式：位置闭环（级联：位置 P + 速度 PI + 电流 PI，foc_current）
 * 按键（内部上拉，按下为低电平）：
 *   KEY1(PB12) 单击=使能，双击=失能
 *   KEY2(PB13) 单击=目标角度+0.5rad，双击=目标角度-0.5rad
 *   KEY3(PB14) 单击=回到零点
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
#define CUR_SENS_SENSITIVITY 0.132f
#define CUR_SENS_ZERO_VOLT   1.65f
#define ADC_REF_VOLT         3.3f
#define ADC_MAX              4095.0f
#define PIN_CUR_U  PC5
#define PIN_CUR_W  PB1

PhaseCurrent_s readPhaseCurrents() {
  PhaseCurrent_s c;
  // 注意：如果电流方向反了（iq 为负/发散），把下面三行整体取负。
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
#define ANGLE_STEP      0.5f     // 每次角度增量(rad)
#define ANGLE_MAX       6.28318f // 2π，约一圈

float targetAngle = 0.0f;        // 目标角度(rad)

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

  // KEY1：单击使能，双击失能
  if (e1 == CLICK_SINGLE) {
    motor.target = targetAngle;
    motor.enable();
    Serial.println("KEY1 single: motor enabled");
  } else if (e1 == CLICK_DOUBLE) {
    motor.disable();
    Serial.println("KEY1 double: motor disabled");
  }

  // KEY2：单击目标角度+0.5rad，双击-0.5rad
  if (e2 == CLICK_SINGLE) {
    targetAngle = constrain(targetAngle + ANGLE_STEP, -ANGLE_MAX, ANGLE_MAX);
    if (motor.enabled) motor.target = targetAngle;
    Serial.print("KEY2 single: target angle = ");
    Serial.println(targetAngle);
  } else if (e2 == CLICK_DOUBLE) {
    targetAngle = constrain(targetAngle - ANGLE_STEP, -ANGLE_MAX, ANGLE_MAX);
    if (motor.enabled) motor.target = targetAngle;
    Serial.print("KEY2 double: target angle = ");
    Serial.println(targetAngle);
  }

  // KEY3：单击回到零点
  if (e3 == CLICK_SINGLE) {
    targetAngle = 0.0f;
    if (motor.enabled) motor.target = targetAngle;
    Serial.println("KEY3 single: back to zero");
  }
}

/* ========== 非阻塞串口打印 ========== */
#define NBUF 96
char nbuf[NBUF];
volatile uint8_t nlen = 0;
volatile uint8_t npos = 0;

void printStatus() {
  char tg[12], ag[12], vl[12], iq[12];
  dtostrf(targetAngle, 0, 2, tg);
  dtostrf(motor.shaft_angle, 0, 2, ag);
  dtostrf(encoder.getVelocity(), 0, 1, vl);
  dtostrf(motor.current.q, 0, 3, iq);
  nlen = snprintf(nbuf, NBUF, "en:%d target:%s angle:%s vel:%s iq:%s\r\n",
                  motor.enabled ? 1 : 0, tg, ag, vl, iq);
  npos = 0;
}

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

  // ---- 电机（位置闭环配置）----
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.phase_resistance = 0.1f;
  motor.phase_inductance = 42.3e-6f;

  // 内环：电流闭环；中环：速度闭环；外环：位置闭环
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;

  motor.voltage_limit = 2.0f;    // 电流 PI 输出电压上限
  motor.current_limit = 1.0f;    // 电流上限（同时是速度 PI 输出上限）
  motor.velocity_limit = 50.0f;  // 向目标移动时的最大转速
  motor.voltage_sensor_align = 3;

  motor.init();

  // 位置 P（外环，可调；角度误差 -> 速度给定）
  motor.P_angle.P = 20.0f;

  // 速度 PI（中环，起步值，可调）
  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 10.0f;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.01f;

  // 内环电流 PI
  motor.tuneCurrentController(500.0f);

  if (motor.initFOC()) {
    Serial.println("FOC ready.");
  } else {
    Serial.println("FOC init failed!");
  }
  motor.disable();

  Serial.println("KEY1: single=enable, double=disable");
  Serial.println("KEY2: single=+0.5rad, double=-0.5rad");
  Serial.println("KEY3: single=back to zero");
}

void loop() {
  motor.loopFOC();
  motor.move();
  handleButtons();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    printStatus();
  }
  servicePrint();
}