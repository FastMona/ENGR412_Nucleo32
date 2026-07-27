/*
*  dual_motor_rpm_test.ino (MLX90374 PWM Edition)
*  First live-spin test for TOP + BOT, both wired to their SimpleFOCMini
*  drivers and the 18V FOC supply. 
*  Pin map: POLE_PAIRS=7 (2804 gimbal outrunner)
*  TOP: TIM1 (PA8/PA9/PA10, EN=PC10) + PWM Sensor on PB9 (D14)
*  BOT: TIM3 (PA6/PA7/PB0, EN=PC11) + PWM Sensor on PB8 (D15)
*/
#include <SimpleFOC.h>

const int POLE_PAIRS = 7;  
const float RPM_MIN = 10.0f; 
const float RPM_MAX = 1000.0f; 
float target_rpm = 10.0f;  
float current_rpm = 10.0f;  
bool running = true;
const float RPM_RAMP_RATE = 200.0f;  
unsigned long last_ramp_update = 0;
const float MAX_MOTOR_VOLTAGE = 17.0f;  

// ================= PIN MAP (verified, STM32 Nucleo-64 L452RE-P) ================= 
#define M_TOP_PWM_A PA8 
#define M_TOP_PWM_B PA9 
#define M_TOP_PWM_C PA10 
#define M_TOP_EN    PC10

#define M_BOT_PWM_A PA6 
#define M_BOT_PWM_B PA7 
#define M_BOT_PWM_C PB0 
#define M_BOT_EN    PC11

// New MLX90374 PWM Input Pins (Using the old I2C headers D14/D15)
#define M_TOP_ENC_PWM PB9 
#define M_BOT_ENC_PWM PB8 

// ================= objects =================

// TOP MOTOR
BLDCMotor motorTop = BLDCMotor(POLE_PAIRS); 
BLDCDriver3PWM driverTop = BLDCDriver3PWM(M_TOP_PWM_A, M_TOP_PWM_B, M_TOP_PWM_C, M_TOP_EN); 
// Initialize PWM sensor (Pin, min_pulse_us, max_pulse_us)
MagneticSensorPWM encTop = MagneticSensorPWM(M_TOP_ENC_PWM, 50, 950); 
void doPWMTop() { encTop.handlePWM(); } // Interrupt wrapper

// BOT MOTOR
BLDCMotor motorBot = BLDCMotor(POLE_PAIRS); 
BLDCDriver3PWM driverBot = BLDCDriver3PWM(M_BOT_PWM_A, M_BOT_PWM_B, M_BOT_PWM_C, M_BOT_EN); 
// Initialize PWM sensor (Pin, min_pulse_us, max_pulse_us)
MagneticSensorPWM encBot = MagneticSensorPWM(M_BOT_ENC_PWM, 50, 950); 
void doPWMBot() { encBot.handlePWM(); } // Interrupt wrapper

String rx;

// ================= setup =================
void setup() { 
  Serial.begin(115200);

  // --- TOP MOTOR INIT ---
  encTop.init(); 
  encTop.enableInterrupt(doPWMTop); // Attach background interrupt
  motorTop.linkSensor(&encTop); 
  
  driverTop.voltage_power_supply = 18;  
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

  // --- BOT MOTOR INIT ---
  encBot.init(); 
  encBot.enableInterrupt(doPWMBot); // Attach background interrupt
  motorBot.linkSensor(&encBot); 
  
  driverBot.voltage_power_supply = 18;  
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

  Serial.print("# TOP+BOT MLX PWM spin test ready. Rate: "); 
  Serial.print(target_rpm); 
  Serial.println(" RPM."); 
}

// ================= loop =================
void loop() { 
  motorTop.loopFOC(); 
  motorBot.loopFOC();

  // -- non-blocking serial command read -- 
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
          Serial.print(RPM_MIN); Serial.print(" and "); Serial.println(RPM_MAX); 
        } 
      } 
      rx = ""; 
    } else if (c != '\r') { 
      rx += c; 
    } 
  }

  // -- ramp current_rpm toward the commanded value -- 
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

  // -- Serial Plotter output -- 
  static unsigned long last_print = 0; 
  if (millis() - last_print > 50) {  
    last_print = millis(); 
    Serial.print("rpm_cmd:");     Serial.print(current_rpm); 
    Serial.print(",rpm_top_act:"); Serial.print(motorTop.shaft_velocity * 60.0f / (2.0f * PI)); 
    Serial.print(",rpm_bot_act:"); Serial.println(motorBot.shaft_velocity * 60.0f / (2.0f * PI)); 
  } 
}