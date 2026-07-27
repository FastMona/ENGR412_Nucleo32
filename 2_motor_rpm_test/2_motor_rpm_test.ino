/*
 * dual_motor_rpm_test.ino
 *
 * First live-spin test for TOP + BOT, both now wired to their SimpleFOCMini
 * drivers and the 18V FOC supply. Same incremental style as the LED test
 * (led_blink_100rpm.ino): starts at a low fixed target and takes typed Serial
 * input to change it live, clamped to a safe range. No rule policy, no
 * azimuth/phase-lock logic yet -- both motors just track the same commanded
 * RPM in velocity mode. That comes back once this is confirmed spinning
 * cleanly.
 *
 * Starts at 10 RPM. Type a new value + Enter to change it -- clamped to
 * [10, 500] RPM so a typo can't suddenly command something unsafe.
 *
 * Pin map / hardware exactly as verified in dual_motor_bringup_nucleo.ino /
 * README_nucleo.md -- POLE_PAIRS=7 (2804 gimbal outrunner), TOP on TIM1
 * (PA8/PA9/PA10, EN=PC10) + I2C1 (PB7 SDA/PB8 SCL), BOT on TIM3
 * (PA6/PA7/PB0, EN=PC11) + I2C2 (PB11 SDA/PB10 SCL).
 *
 * Before flashing:
 *   - voltage_power_supply below is set to 18 (your actual FOC supply) --
 *     change if that's not accurate.
 *   - voltage_limit is capped at MAX_MOTOR_VOLTAGE (12V). Both motors' FOC
 *     voltage command is clamped to this regardless of RPM target -- adjust
 *     the constant if 12V proves too low/high once spinning is confirmed safe.
 *   - Keep props clear at power-up -- each motor's initFOC() does a brief
 *     automatic alignment movement, same as the single/no-input sketches.
 *   - Flash via Tools > Upload method > STM32CubeProgrammer (SWD), not Mass
 *     Storage -- confirmed today that Mass Storage was causing intermittent
 *     silent-board/no-Serial failures unrelated to sketch content. SWD resets
 *     the target deterministically on every upload.
 */

#include <SimpleFOC.h>

const int POLE_PAIRS = 7;  // 2804 gimbal outrunner, 12N14P -> 7 pole pairs

const float RPM_MIN = 10.0f;
const float RPM_MAX = 1000.0f;
float target_rpm = 10.0f;  // user-commanded value -- can jump instantly on input
float current_rpm = 10.0f;  // smoothed setpoint actually sent to the motors
bool running = true;

const float RPM_RAMP_RATE = 200.0f;  // max RPM/s change -- lower = gentler transitions
unsigned long last_ramp_update = 0;

const float MAX_MOTOR_VOLTAGE = 17.0f;  // hard cap on voltage_limit for both motors -- close to the 18V supply, testing whether 900+ RPM pulsing was voltage saturation

// ================= PIN MAP (verified, STM32 Nucleo-64 L452RE-P) =================
#define M_TOP_PWM_A PA8
#define M_TOP_PWM_B PA9
#define M_TOP_PWM_C PA10
#define M_TOP_EN    PC10

#define M_BOT_PWM_A PA6
#define M_BOT_PWM_B PA7
#define M_BOT_PWM_C PB0
#define M_BOT_EN    PC11

#define I2C_TOP_SDA_PIN PB7
#define I2C_TOP_SCL_PIN PB8
#define I2C_BOT_SDA_PIN PB11
#define I2C_BOT_SCL_PIN PB10

// ================= objects =================

BLDCMotor motorTop = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driverTop = BLDCDriver3PWM(M_TOP_PWM_A, M_TOP_PWM_B, M_TOP_PWM_C, M_TOP_EN);
MagneticSensorI2C encTop = MagneticSensorI2C(AS5600_I2C);
TwoWire I2C_TOP(I2C_TOP_SDA_PIN, I2C_TOP_SCL_PIN);

BLDCMotor motorBot = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driverBot = BLDCDriver3PWM(M_BOT_PWM_A, M_BOT_PWM_B, M_BOT_PWM_C, M_BOT_EN);
MagneticSensorI2C encBot = MagneticSensorI2C(AS5600_I2C);
TwoWire I2C_BOT(I2C_BOT_SDA_PIN, I2C_BOT_SCL_PIN);

String rx;

// ================= setup =================

void setup() {
  Serial.begin(115200);

  I2C_TOP.begin();
  encTop.init(&I2C_TOP);
  motorTop.linkSensor(&encTop);
  driverTop.voltage_power_supply = 18;  // <-- SET to your actual supply voltage
  driverTop.init();
  motorTop.linkDriver(&driverTop);
  motorTop.controller = MotionControlType::velocity;
  motorTop.PID_velocity.P = 0.5;
  motorTop.PID_velocity.I = 2.0;
  motorTop.PID_velocity.D = 0.0;
  motorTop.PID_velocity.output_ramp = 500;
  motorTop.LPF_velocity.Tf = 0.05;
  motorTop.voltage_limit = MAX_MOTOR_VOLTAGE;
  motorTop.init();
  motorTop.initFOC();

  I2C_BOT.begin();
  encBot.init(&I2C_BOT);
  motorBot.linkSensor(&encBot);
  driverBot.voltage_power_supply = 18;  // <-- SET to your actual supply voltage
  driverBot.init();
  motorBot.linkDriver(&driverBot);
  motorBot.controller = MotionControlType::velocity;
  motorBot.PID_velocity.P = 0.5;
  motorBot.PID_velocity.I = 2.0;
  motorBot.PID_velocity.D = 0.0;
  motorBot.PID_velocity.output_ramp = 500;
  motorBot.LPF_velocity.Tf = 0.05;
  motorBot.voltage_limit = MAX_MOTOR_VOLTAGE;
  motorBot.init();
  motorBot.initFOC();

  Serial.print("# TOP+BOT spin test ready. Rate: ");
  Serial.print(target_rpm);
  Serial.print(" RPM. Type a new value (");
  Serial.print(RPM_MIN);
  Serial.print("-");
  Serial.print(RPM_MAX);
  Serial.println(") + Enter to change it.");
}

// ================= loop =================

void loop() {
  motorTop.loopFOC();
  motorBot.loopFOC();

  // -- non-blocking serial command read: a typed RPM value terminated by newline --
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (rx.length() > 0) {
        float val = rx.toFloat();
        if (val == 0) {
          running = false;
          Serial.println("# shut down -- type a positive value to resume");
        } else if (val >= RPM_MIN && val <= RPM_MAX) {
          target_rpm = val;
          running = true;
          Serial.print("# new target: ");
          Serial.print(target_rpm);
          Serial.println(" RPM");
        } else {
          Serial.print("# ignored -- enter 0 (shut down) or a value between ");
          Serial.print(RPM_MIN);
          Serial.print(" and ");
          Serial.println(RPM_MAX);
        }
      }
      rx = "";
    } else if (c != '\r') {
      rx += c;
    }
  }

  // -- ramp current_rpm toward the commanded value at RPM_RAMP_RATE per second,
  //    instead of snapping the target instantly -- this is what actually
  //    smooths out the transition (output_ramp alone limits voltage slew, not
  //    how big a step the target itself jumps by) --
  unsigned long now = millis();
  float dt = (now - last_ramp_update) / 1000.0f;
  last_ramp_update = now;
  float rpm_setpoint = running ? target_rpm : 0.0f;
  float max_step = RPM_RAMP_RATE * dt;
  if (current_rpm < rpm_setpoint) {
    current_rpm = fminf(current_rpm + max_step, rpm_setpoint);
  } else if (current_rpm > rpm_setpoint) {
    current_rpm = fmaxf(current_rpm - max_step, rpm_setpoint);
  }

  float target_rad_s = current_rpm * 2.0f * PI / 60.0f;
  motorTop.move(target_rad_s);
  motorBot.move(-target_rad_s);  // BOT runs reversed relative to TOP

  // -- Serial Plotter output (labeled fields -> auto legend in Arduino IDE 2.x) --
  static unsigned long last_print = 0;
  if (millis() - last_print > 50) {  // ~20 Hz plot rate
    last_print = millis();
    Serial.print("rpm_cmd:");     Serial.print(current_rpm);
    Serial.print(",rpm_top_act:"); Serial.print(motorTop.shaft_velocity * 60.0f / (2.0f * PI));
    Serial.print(",rpm_bot_act:"); Serial.println(motorBot.shaft_velocity * 60.0f / (2.0f * PI));
  }
}