/*
 * top_only_commander.ino
 *
 * Ports your previously-working Uno + SimpleFOCMini + AS5600 single-motor
 * sketch to TOP on the Nucleo L452RE-P -- same settings that worked before
 * (PID_velocity gains, output_ramp, LPF_velocity.Tf, voltage_sensor_align,
 * no useMonitoring(), Commander for live control), only the pins/I2C/supply
 * voltage changed to match this board and TOP's wiring.
 *
 * Differences from your Uno version, and why:
 *   - Pins: TOP's verified Nucleo pin map (PA8/PA9/PA10, EN=PC10) instead of
 *     the Uno's (11,10,9,8).
 *   - Sensor: explicit TwoWire I2C_TOP(PB7,PB8) passed into sensor.init(&I2C_TOP)
 *     instead of the Uno's default Wire -- this board's I2C1 isn't on the same
 *     pins a default Wire object would normally assume, so it's passed
 *     explicitly (same reasoning as the rest of this project's Nucleo sketches).
 *   - driver.voltage_power_supply = 18 (your actual FOC supply, vs. the Uno
 *     sketch's 19) and voltage_limit / motor.voltage_limit capped at 8
 *     (MAX_MOTOR_VOLTAGE, as you set earlier) instead of the Uno's 12 --
 *     starting lower on new hardware, raise later if needed.
 *   - Everything else (PID gains, output_ramp, LPF, voltage_sensor_align=8,
 *     velocity_limit=270, Commander 'T<val>' interface, no monitoring) is
 *     unchanged from your working version.
 *
 * Usage: open Serial Monitor at 115200 baud, type e.g. "T5" + Enter for a
 * ~5 rad/s target (~48 RPM), "T-5" for reverse, "T0" to stop.
 */

#include <SimpleFOC.h>

const float MAX_MOTOR_VOLTAGE = 12.0f;

// ================= PIN MAP (verified, STM32 Nucleo-64 L452RE-P, TOP) =================
#define M_TOP_PWM_A PA8
#define M_TOP_PWM_B PA9
#define M_TOP_PWM_C PA10
#define M_TOP_EN    PC10

#define I2C_TOP_SDA_PIN PB7
#define I2C_TOP_SCL_PIN PB8

// --- AS5600 via I2C ---
TwoWire I2C_TOP(I2C_TOP_SDA_PIN, I2C_TOP_SCL_PIN);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// --- SimpleFOC Mini on Nucleo, TOP motor ---
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(M_TOP_PWM_A, M_TOP_PWM_B, M_TOP_PWM_C, M_TOP_EN);

// --- Commander ---
Commander command = Commander(Serial);
void onTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void setup() {
  Serial.begin(115200);
  delay(500);  // give the USB/VCP serial link time to come up before printing
  Serial.println("# setup() started -- before any I2C/driver/motor init");
  Serial.flush();

  I2C_TOP.begin();
  Serial.println("# I2C_TOP.begin() done");
  Serial.flush();

  sensor.init(&I2C_TOP);
  Serial.println("# sensor.init() done");
  Serial.flush();

  motor.linkSensor(&sensor);
  Serial.println("# motor.linkSensor() done");
  Serial.flush();

  driver.voltage_power_supply = 18;  // <-- your actual FOC supply
  driver.voltage_limit = MAX_MOTOR_VOLTAGE;
  driver.init();
  Serial.println("# driver.init() done -- driver/PWM configured, motor not yet powered");
  Serial.flush();

  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 2.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 500;
  motor.LPF_velocity.Tf = 0.05;
  motor.voltage_limit = MAX_MOTOR_VOLTAGE;
  motor.velocity_limit = 270;

  // no useMonitoring() -- saves significant flash
  motor.voltage_sensor_align = 8;

  Serial.println("# about to call motor.init()...");
  Serial.flush();
  motor.init();
  Serial.println("# motor.init() done");
  Serial.flush();

  Serial.println("# about to call motor.initFOC() -- motor will now be powered/twitch...");
  Serial.flush();
  motor.initFOC();
  Serial.println("# motor.initFOC() done");
  Serial.flush();

  command.add('T', onTarget, "target velocity");

  motor.enable();
  motor.target = 0;

  Serial.println("Ready. T<val>=velocity (e.g. T5, T-5, T0)");
}

void loop() {
  motor.loopFOC();
  motor.move();
  command.run();
}