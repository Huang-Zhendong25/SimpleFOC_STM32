/**
 * INMOP FOC 开发板 —— SimpleFOC 有感 FOC 底层测试程序（按键控制版 / 电压开环）
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
 * 控制模式：电压开环（velocity_openloop）
 * 按键（内部上拉，按下为低电平）：
 *   KEY1(PB12) 单击=使能，双击=失能
 *   KEY2(PB13) 单击=速度+5，双击=速度-5
 *   KEY3(PB14) 单击=反转
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
  float iu = (CUR_SENS_ZERO_VOLT - analogRead(PIN_CUR_U) * ADC_REF_VOLT / ADC_MAX)
             / CUR_SENS_SENSITIVITY;
  float iw = (CUR_SENS_ZERO_VOLT - analogRead(PIN_CUR_W) * ADC_REF_VOLT / ADC_MAX)
             / CUR_SENS_SENSITIVITY;
  c.a = iu;
  c.b = -iu - iw;
  c.c = iw;
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

#define DEBOUNCE_MS     20      // 消抖时间
#define DOUBLE_CLICK_MS 300     // 双击判定窗口
#define SPEED_STEP      5.0f    // 每次速度增量
#define SPEED_MAX       50.0f   // 速度上限

float targetSpeed = 0.0f;       // 当前速度设定值(rad/s)

// 单击/双击检测状态机
struct Button {
  uint8_t pin;
  bool state = false;            // 消抖后的稳定状态（true=按下）
  bool lastReading = false;      // 上次原始读数
  unsigned long lastDebounce = 0;
  uint8_t pending = 0;           // 已检测到的点击次数（等待判定）
  unsigned long releaseTime = 0; // 最近一次释放时间
};

enum ClickEvent { CLICK_NONE, CLICK_SINGLE, CLICK_DOUBLE };

Button btn1 = { PIN_KEY1 };
Button btn2 = { PIN_KEY2 };
Button btn3 = { PIN_KEY3 };

// 处理一个按键，返回本次事件：无 / 单击 / 双击
ClickEvent updateButton(Button& b) {
  ClickEvent ev = CLICK_NONE;
  unsigned long now = millis();
  bool reading = (digitalRead(b.pin) == LOW);  // 按下=true

  // 消抖：读数变化时重新计时
  if (reading != b.lastReading) {
    b.lastDebounce = now;
  }

  // 读数稳定超过消抖时间后，才确认状态变化
  if ((now - b.lastDebounce) >= DEBOUNCE_MS) {
    if (reading != b.state) {
      if (reading) {
        b.pending++;          // 按下沿：点击次数+1
      } else {
        b.releaseTime = now;  // 释放沿：记录释放时间
      }
      b.state = reading;
    }
  }
  b.lastReading = reading;

  // 释放后超过双击窗口仍没再按：判定是单击还是双击
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
    motor.target = targetSpeed;   // 使能时把当前速度设定应用上去
    motor.enable();
    Serial.println("KEY1 single: motor enabled");
  } else if (e1 == CLICK_DOUBLE) {
    motor.disable();
    Serial.println("KEY1 double: motor disabled");
  }

  // KEY2：单击速度+5，双击速度-5
  if (e2 == CLICK_SINGLE) {
    targetSpeed = constrain(targetSpeed + SPEED_STEP, -SPEED_MAX, SPEED_MAX);
    if (motor.enabled) motor.target = targetSpeed;  // 运行中实时生效
    Serial.print("KEY2 single: speed = ");
    Serial.println(targetSpeed);
  } else if (e2 == CLICK_DOUBLE) {
    targetSpeed = constrain(targetSpeed - SPEED_STEP, -SPEED_MAX, SPEED_MAX);
    if (motor.enabled) motor.target = targetSpeed;
    Serial.print("KEY2 double: speed = ");
    Serial.println(targetSpeed);
  }

  // KEY3：单击反转（正在转动时也可直接反转）
  if (e3 == CLICK_SINGLE) {
    targetSpeed = -targetSpeed;
    if (motor.enabled) motor.target = targetSpeed;  // 直接反向
    Serial.print("KEY3 single: reversed, speed = ");
    Serial.println(targetSpeed);
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

  // 按键（内部上拉）
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

  // 电机（电压开环）
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = 2;
  motor.voltage_sensor_align = 3;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();

  if (motor.initFOC()) {
    Serial.println("FOC ready.");
  } else {
    Serial.println("FOC init failed!");
  }
  motor.disable();

  Serial.println("KEY1: single=enable, double=disable");
  Serial.println("KEY2: single=+5, double=-5");
  Serial.println("KEY3: single=reverse");
}

void loop() {
  motor.loopFOC();
  motor.move();
  handleButtons();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("enabled:");
    Serial.print(motor.enabled ? "1" : "0");
    Serial.print("  speed:");
    Serial.print(targetSpeed);
    Serial.print("  vel:");
    Serial.print(encoder.getVelocity());
    Serial.print(" rad/s  angle:");
    Serial.println(encoder.getAngle());
  }
}