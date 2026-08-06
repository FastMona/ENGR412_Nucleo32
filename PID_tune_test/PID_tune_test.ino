// /*
 * PID_tune_test.ino
 *
 * Small standalone bench tool for finding smooth PID_velocity gains for the
 * TOP/BOT gimbal motors, across different RPM targets, without editing and
 * re-flashing 2_motor_rpm_test.ino for every trial. Same hardware wiring as
 * that sketch (both motors), but no boot hand-align, no azimuth/PLL logic --
 * purely gain + speed tuning. Once good gains are found here, copy the P/I/D
 * and Tf values back into 2_motor_rpm_test.ino's setup().
 *
 * Pin map identical to 2_motor_rpm_test.ino -- POLE_PAIRS=7 (2804 gimbal
 * outrunner), TOP on TIM1 (PA8/PA9/PA10, EN=PC10) + I2C1 (PB7 SDA/PB8 SCL),
 * BOT on TIM3 (PA6/PA7/PB0, EN=PC11) + I2C2 (PB11 SDA/PB10 SCL). Flash via
 * STM32CubeProgrammer (SWD), not Mass Storage (see PS11 §2.18 / pending
 * revisions -- Mass Storage causes silent upload failures on this board).
 *
 * Starts both motors at rest, gains set to the known-good §2.18 baseline
 * (P=0.5, I=2.0, D=0.0, Tf=0.05) -- type a command to change speed or a
 * gain live, then watch the Serial Plotter (rpm_cmd / rpm_top_act /
 * rpm_bot_act / ripple_top_pp / ripple_bot_pp) and/or feel the motors for
 * vibration. The two *_ripple_pp fields are a numeric smoothness proxy:
 * peak-to-peak swing of each motor's instantaneous measured RPM over a
 * rolling RIPPLE_WINDOW_MS window -- lower is smoother, a spike in this
 * number is the same thing as visible/felt vibration, but quantified so
 * different gain sets can be compared by number instead of by feel alone.
 *
 * Serial commands (each terminated by Enter):
 *   <number>   -- new target RPM for both motors (RPM_MIN-RPM_MAX), ramped
 *   P<number>  -- set PID_velocity.P on both motors, e.g. "P0.7"
 *   I<number>  -- set PID_velocity.I on both motors, e.g. "I2.5"
 *   D<number>  -- set PID_velocity.D on both motors, e.g. "D0.01"
 *   T<number>  -- set LPF_velocity.Tf on both motors, e.g. "T0.03"
 *   ?          -- print current target RPM and all four gains
 *   s          -- SWEEP: holds each RPM in RPM_TEST_LIST (below) for
 *                 SWEEP_HOLD_MS using whatever gains are currently set,
 *                 printing a one-line ripple summary per step -- a quick
 *                 way to see which RPMs are rough vs smooth for a given
 *                 gain set without typing each speed by hand. Blocking
 *                 (like the align/freeze windows in the main sketch), but
 *                 checks Serial for "0" once per step so a long sweep can
 *                 be aborted between steps.
 *   a          -- AUTOTUNE `[2026-07-29, new]`: coarse grid search over
 *                 AUTOTUNE_P_LIST x AUTOTUNE_I_LIST x AUTOTUNE_TF_LIST
 *                 (D held at 0 -- never needed so far on this hardware),
 *                 tested at every RPM in AUTOTUNE_RPM_LIST (default
 *                 300/400/500, i.e. the "find the best PID for 300-500 RPM"
 *                 request this was built for -- edit the list to retarget
 *                 a different range). For each gain combo: holds every test
 *                 RPM in turn, scores the combo by its WORST ripple
 *                 peak-to-peak across all of them (a combo only counts as
 *                 good if it's smooth at every test speed, not just on
 *                 average), plus a penalty if the motors aren't actually
 *                 tracking the commanded RPM closely (a sluggish/undertuned
 *                 response can look falsely "smooth" by barely moving --
 *                 this guards against picking that). Prints every combo's
 *                 score as it goes (this takes a few minutes -- ~2-4 with
 *                 the default 3x3x3x3 grid), then applies and prints the
 *                 best-scoring combo automatically at the end. This is a
 *                 coarse grid search, not a real optimizer (no gradient
 *                 descent, no relay/Ziegler-Nichols auto-tune) -- deliberately
 *                 simple, matching the rest of this tool. Aborts the same
 *                 way as "s" (type "0").
 *   0          -- ramp to 0 RPM, then disable both drivers
 *
 * Gains apply to both motors identically and take effect immediately
 * (SimpleFOC PID struct fields are plain public members, no re-init
 * needed). Keep P/I matched between TOP and BOT when copying values back
 * into the main sketch -- mismatched gains make the two motors track
 * differently and drift out of index alignment (see PS11_pending_revisions).
 */

#include <SimpleFOC.h>

const int POLE_PAIRS = 7;  // 2804 gimbal outrunner, 12N14P -> 7 pole pairs

const float RPM_MIN = 5.0f;
const float RPM_MAX = 999.0f;
const float RPM_RAMP_RATE = 200.0f;  // max RPM/s change

const float MAX_MOTOR_VOLTAGE = 12.0f;  // same safety cap as 2_motor_rpm_test.ino

const unsigned long RIPPLE_WINDOW_MS = 250;  // ripple (peak-to-peak) measurement window

// -- Sweep settings --
const float RPM_TEST_LIST[] = {10, 50, 100, 150, 200, 300, 400, 500, 700, 900};
const int RPM_TEST_LIST_LEN = sizeof(RPM_TEST_LIST) / sizeof(RPM_TEST_LIST[0]);
const unsigned long SWEEP_HOLD_MS = 4000;  // how long to hold each sweep RPM before measuring/moving on
const unsigned long SWEEP_SETTLE_MS = 1500;  // ignore ripple during this initial part of each hold (let it settle first)

// -- Autotune settings `[2026-07-29]` -- coarse grid, D fixed at 0 (never
//    needed on this hardware so far). RPM list defaults to the 300-500
//    range this was built for; edit to retarget. Hold/settle are shorter
//    than the manual sweep's, since autotune runs many more trials
//    (P x I x Tf x RPM combos) and needs to stay a few-minutes bench task,
//    not tens of minutes. --
const float AUTOTUNE_P_LIST[] = {0.3f, 0.5f, 0.7f};
const float AUTOTUNE_I_LIST[] = {1.0f, 2.0f, 3.0f};
const float AUTOTUNE_TF_LIST[] = {0.03f, 0.05f, 0.08f};
const float AUTOTUNE_RPM_LIST[] = {300, 400, 500};
const unsigned long AUTOTUNE_HOLD_MS = 1500;
const unsigned long AUTOTUNE_SETTLE_MS = 800;
const float AUTOTUNE_TRACK_ERR_TOLERANCE_PCT = 3.0f;  // tracking error below this is free; above it
                                                        // adds to the score (see runAutoTune())
const float AUTOTUNE_TRACK_ERR_PENALTY_WEIGHT = 5.0f;  // score penalty per % tracking error beyond tolerance

float target_rpm = 0.0f;
float current_rpm = 0.0f;
bool running = false;
bool motors_enabled = true;
bool stop_requested = false;

unsigned long last_ramp_update = 0;

String rx;

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

BLDCMotor motorTop = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driverTop = BLDCDriver3PWM(M_TOP_PWM_A, M_TOP_PWM_B, M_TOP_PWM_C, M_TOP_EN);
MagneticSensorI2C encTop = MagneticSensorI2C(AS5600_I2C);
TwoWire I2C_TOP(I2C_TOP_SDA_PIN, I2C_TOP_SCL_PIN);

BLDCMotor motorBot = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driverBot = BLDCDriver3PWM(M_BOT_PWM_A, M_BOT_PWM_B, M_BOT_PWM_C, M_BOT_EN);
MagneticSensorI2C encBot = MagneticSensorI2C(AS5600_I2C);
TwoWire I2C_BOT(I2C_BOT_SDA_PIN, I2C_BOT_SCL_PIN);

// -- ripple tracking --
float top_vel_min = 0, top_vel_max = 0;
float bot_vel_min = 0, bot_vel_max = 0;
float top_ripple_pp = 0, bot_ripple_pp = 0;
unsigned long last_ripple_update = 0;

void printGains() {
  Serial.print("# target_rpm="); Serial.print(target_rpm);
  Serial.print(" P="); Serial.print(motorTop.PID_velocity.P, 4);
  Serial.print(" I="); Serial.print(motorTop.PID_velocity.I, 4);
  Serial.print(" D="); Serial.print(motorTop.PID_velocity.D, 4);
  Serial.print(" Tf="); Serial.println(motorTop.LPF_velocity.Tf, 4);
}

void setGains(float P, float I, float D, float Tf) {
  motorTop.PID_velocity.P = P;   motorBot.PID_velocity.P = P;
  motorTop.PID_velocity.I = I;   motorBot.PID_velocity.I = I;
  motorTop.PID_velocity.D = D;   motorBot.PID_velocity.D = D;
  motorTop.LPF_velocity.Tf = Tf; motorBot.LPF_velocity.Tf = Tf;
}

// Runs both motors' FOC loops for hold_ms milliseconds at the given target
// RPM, tracking ripple (peak-to-peak) and average measured speed throughout
// the post-settle portion of the hold. Used by both normal operation (via
// loop(), non-blocking) is NOT this function -- this one is only used by
// the blocking sweep/autotune below, which need a tight local hold with
// their own accounting per step rather than the global one used in loop().
// Returns false if aborted (typed "0" mid-hold), true on normal completion;
// out_* pointers are only valid on a true return.
bool runSweepStep(float rpm, unsigned long hold_ms, unsigned long settle_ms,
                   float* out_top_pp, float* out_bot_pp,
                   float* out_top_avg, float* out_bot_avg) {
  float rad_s = rpm * 2.0f * PI / 60.0f;
  unsigned long start = millis();
  float vmin_t = 1e9, vmax_t = -1e9, vmin_b = 1e9, vmax_b = -1e9;
  float sum_t = 0, sum_b = 0;
  unsigned long n_samples = 0;
  bool aborted = false;
  while (millis() - start < hold_ms) {
    motorTop.loopFOC();
    motorBot.loopFOC();
    motorTop.move(rad_s);
    motorBot.move(-rad_s);
    if (millis() - start >= settle_ms) {
      float vt = motorTop.shaft_velocity * 60.0f / (2.0f * PI);
      float vb = motorBot.shaft_velocity * 60.0f / (2.0f * PI);
      if (vt < vmin_t) vmin_t = vt;
      if (vt > vmax_t) vmax_t = vt;
      if (vb < vmin_b) vmin_b = vb;
      if (vb > vmax_b) vmax_b = vb;
      sum_t += vt;
      sum_b += vb;
      n_samples++;
    }
    if (Serial.available() && Serial.peek() == '0') { aborted = true; break; }
  }
  if (aborted) {
    while (Serial.available()) Serial.read();  // clear the "0" so it doesn't re-trigger after the caller returns
    Serial.println("# aborted");
    return false;
  }
  *out_top_pp = vmax_t - vmin_t;
  *out_bot_pp = vmax_b - vmin_b;
  *out_top_avg = n_samples > 0 ? sum_t / n_samples : 0.0f;
  *out_bot_avg = n_samples > 0 ? sum_b / n_samples : 0.0f;
  Serial.print("# rpm="); Serial.print(rpm);
  Serial.print(" ripple_top_pp="); Serial.print(*out_top_pp);
  Serial.print(" ripple_bot_pp="); Serial.println(*out_bot_pp);
  return true;
}

void runSweep() {
  Serial.println("# SWEEP starting -- current gains, holding each RPM below for a few seconds");
  printGains();
  if (!motors_enabled) {
    motorTop.enable();
    motorBot.enable();
    motors_enabled = true;
  }
  for (int i = 0; i < RPM_TEST_LIST_LEN; i++) {
    float tpp, bpp, tavg, bavg;
    bool ok = runSweepStep(RPM_TEST_LIST[i], SWEEP_HOLD_MS, SWEEP_SETTLE_MS, &tpp, &bpp, &tavg, &bavg);
    if (!ok) break;  // aborted
  }
  Serial.println("# SWEEP complete -- ramping to 0");
  target_rpm = 0.0f;
  running = false;
}

// AUTOTUNE `[2026-07-29]` -- coarse grid search over P x I x Tf (D fixed at
// 0), scoring each combo by its worst ripple peak-to-peak across every RPM
// in AUTOTUNE_RPM_LIST, penalized if the motors aren't actually tracking
// the commanded speed closely (guards against a sluggish/undertuned combo
// looking falsely "smooth" just because it barely responds). See header
// comment for the full description.
void runAutoTune() {
  const int n_p = sizeof(AUTOTUNE_P_LIST) / sizeof(AUTOTUNE_P_LIST[0]);
  const int n_i = sizeof(AUTOTUNE_I_LIST) / sizeof(AUTOTUNE_I_LIST[0]);
  const int n_tf = sizeof(AUTOTUNE_TF_LIST) / sizeof(AUTOTUNE_TF_LIST[0]);
  const int n_r = sizeof(AUTOTUNE_RPM_LIST) / sizeof(AUTOTUNE_RPM_LIST[0]);

  Serial.print("# AUTOTUNE starting -- ");
  Serial.print(n_p * n_i * n_tf);
  Serial.print(" gain combos x ");
  Serial.print(n_r);
  Serial.println(" RPM points, D=0 fixed. This will take a few minutes.");

  if (!motors_enabled) {
    motorTop.enable();
    motorBot.enable();
    motors_enabled = true;
  }

  float best_score = 1e9;
  float best_P = motorTop.PID_velocity.P;
  float best_I = motorTop.PID_velocity.I;
  float best_Tf = motorTop.LPF_velocity.Tf;
  bool aborted = false;

  for (int pi = 0; pi < n_p && !aborted; pi++) {
    for (int ii = 0; ii < n_i && !aborted; ii++) {
      for (int ti = 0; ti < n_tf && !aborted; ti++) {
        float P = AUTOTUNE_P_LIST[pi];
        float I = AUTOTUNE_I_LIST[ii];
        float Tf = AUTOTUNE_TF_LIST[ti];
        setGains(P, I, 0.0f, Tf);

        float worst_ripple = 0.0f;
        float worst_track_err_pct = 0.0f;
        for (int ri = 0; ri < n_r; ri++) {
          float rpm = AUTOTUNE_RPM_LIST[ri];
          float tpp, bpp, tavg, bavg;
          bool ok = runSweepStep(rpm, AUTOTUNE_HOLD_MS, AUTOTUNE_SETTLE_MS, &tpp, &bpp, &tavg, &bavg);
          if (!ok) { aborted = true; break; }
          if (tpp > worst_ripple) worst_ripple = tpp;
          if (bpp > worst_ripple) worst_ripple = bpp;
          float top_err_pct = fabsf(tavg - rpm) / rpm * 100.0f;
          float bot_err_pct = fabsf(fabsf(bavg) - rpm) / rpm * 100.0f;  // bot avg is negative (reversed)
          if (top_err_pct > worst_track_err_pct) worst_track_err_pct = top_err_pct;
          if (bot_err_pct > worst_track_err_pct) worst_track_err_pct = bot_err_pct;
        }
        if (aborted) break;

        float penalty = 0.0f;
        if (worst_track_err_pct > AUTOTUNE_TRACK_ERR_TOLERANCE_PCT) {
          penalty = (worst_track_err_pct - AUTOTUNE_TRACK_ERR_TOLERANCE_PCT) * AUTOTUNE_TRACK_ERR_PENALTY_WEIGHT;
        }
        float score = worst_ripple + penalty;

        Serial.print("# combo P="); Serial.print(P, 2);
        Serial.print(" I="); Serial.print(I, 2);
        Serial.print(" Tf="); Serial.print(Tf, 2);
        Serial.print(" -> worst_ripple_pp="); Serial.print(worst_ripple, 2);
        Serial.print(" worst_track_err_pct="); Serial.print(worst_track_err_pct, 2);
        Serial.print(" score="); Serial.println(score, 2);

        if (score < best_score) {
          best_score = score;
          best_P = P;
          best_I = I;
          best_Tf = Tf;
        }
      }
    }
  }

  if (aborted) {
    Serial.println("# AUTOTUNE aborted -- gains left at last-tried combo, not necessarily the best one");
  } else {
    setGains(best_P, best_I, 0.0f, best_Tf);
    Serial.print("# AUTOTUNE complete -- best: P="); Serial.print(best_P, 2);
    Serial.print(" I="); Serial.print(best_I, 2);
    Serial.print(" D=0 Tf="); Serial.print(best_Tf, 2);
    Serial.print(" (score="); Serial.print(best_score, 2);
    Serial.println(") -- applied. Copy these into 2_motor_rpm_test.ino's setup() to keep them.");
  }
  target_rpm = 0.0f;
  running = false;
}

void setup() {
  Serial.begin(115200);

  I2C_TOP.begin();
  encTop.init(&I2C_TOP);
  motorTop.linkSensor(&encTop);
  driverTop.voltage_power_supply = 18;
  driverTop.init();
  motorTop.linkDriver(&driverTop);
  motorTop.controller = MotionControlType::velocity;
  motorTop.PID_velocity.output_ramp = 500;
  motorTop.voltage_limit = MAX_MOTOR_VOLTAGE;
  motorTop.init();
  motorTop.initFOC();

  I2C_BOT.begin();
  encBot.init(&I2C_BOT);
  motorBot.linkSensor(&encBot);
  driverBot.voltage_power_supply = 18;
  driverBot.init();
  motorBot.linkDriver(&driverBot);
  motorBot.controller = MotionControlType::velocity;
  motorBot.PID_velocity.output_ramp = 500;
  motorBot.voltage_limit = MAX_MOTOR_VOLTAGE;
  motorBot.init();
  motorBot.initFOC();

  setGains(0.5f, 2.0f, 0.0f, 0.05f);  // known-good §2.18 baseline -- starting point, not a floor

  Serial.println("# PID_tune_test ready.");
  Serial.println("# Commands: <rpm> (5-999), P<v>, I<v>, D<v>, T<v>, ? (show gains), s (sweep), a (autotune 300-500), 0 (stop)");
  printGains();
}

void loop() {
  motorTop.loopFOC();
  motorBot.loopFOC();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      rx.trim();
      if (rx.length() > 0) {
        char c0 = rx.charAt(0);
        if (rx == "0") {
          stop_requested = true;
          running = false;
          Serial.println("# STOP -- ramping to 0, then disabling drivers");
        } else if (rx == "?") {
          printGains();
        } else if (rx == "s") {
          runSweep();
        } else if (rx == "a") {
          runAutoTune();
        } else if (c0 == 'P' || c0 == 'p') {
          motorTop.PID_velocity.P = motorBot.PID_velocity.P = rx.substring(1).toFloat();
          printGains();
        } else if (c0 == 'I' || c0 == 'i') {
          motorTop.PID_velocity.I = motorBot.PID_velocity.I = rx.substring(1).toFloat();
          printGains();
        } else if (c0 == 'D' || c0 == 'd') {
          motorTop.PID_velocity.D = motorBot.PID_velocity.D = rx.substring(1).toFloat();
          printGains();
        } else if (c0 == 'T' || c0 == 't') {
          motorTop.LPF_velocity.Tf = motorBot.LPF_velocity.Tf = rx.substring(1).toFloat();
          printGains();
        } else {
          float val = rx.toFloat();
          if (val >= RPM_MIN && val <= RPM_MAX) {
            if (!motors_enabled) {
              motorTop.enable();
              motorBot.enable();
              motors_enabled = true;
            }
            stop_requested = false;
            target_rpm = val;
            running = true;
            Serial.print("# new target: "); Serial.print(target_rpm); Serial.println(" RPM");
          } else {
            Serial.println("# ignored -- <rpm> (5-999), P<v>, I<v>, D<v>, T<v>, ?, s, a, 0");
          }
        }
      }
      rx = "";
    } else if (c != '\r') {
      rx += c;
    }
  }

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

  if (!running && current_rpm == 0.0f && stop_requested && motors_enabled) {
    motorTop.disable();
    motorBot.disable();
    motors_enabled = false;
    stop_requested = false;
    Serial.println("# drivers disabled -- type an RPM to resume");
  }

  float rad_s = current_rpm * 2.0f * PI / 60.0f;
  motorTop.move(rad_s);
  motorBot.move(-rad_s);

  // -- ripple tracking: running min/max of instantaneous measured RPM,
  //    reset every RIPPLE_WINDOW_MS -- peak-to-peak swing is the smoothness
  //    proxy printed alongside the plotter fields below. --
  float vt = motorTop.shaft_velocity * 60.0f / (2.0f * PI);
  float vb = motorBot.shaft_velocity * 60.0f / (2.0f * PI);
  if (vt < top_vel_min) top_vel_min = vt;
  if (vt > top_vel_max) top_vel_max = vt;
  if (vb < bot_vel_min) bot_vel_min = vb;
  if (vb > bot_vel_max) bot_vel_max = vb;
  if (now - last_ripple_update >= RIPPLE_WINDOW_MS) {
    top_ripple_pp = top_vel_max - top_vel_min;
    bot_ripple_pp = bot_vel_max - bot_vel_min;
    top_vel_min = bot_vel_min = 1e9;
    top_vel_max = bot_vel_max = -1e9;
    last_ripple_update = now;
  }

  static unsigned long last_print = 0;
  if (millis() - last_print > 50) {  // ~20 Hz plot rate
    last_print = millis();
    Serial.print("rpm_cmd:");        Serial.print(current_rpm);
    Serial.print(",rpm_top_act:");   Serial.print(vt);
    Serial.print(",rpm_bot_act:");   Serial.print(vb);
    Serial.print(",ripple_top_pp:"); Serial.print(top_ripple_pp);
    Serial.print(",ripple_bot_pp:"); Serial.println(bot_ripple_pp);
  }
}
