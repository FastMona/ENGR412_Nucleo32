/*
 * dual_motor_rpm_test.ino
 *
 * First live-spin test for TOP + BOT, both now wired to their SimpleFOCMini
 * drivers and the 18V FOC supply. Same incremental style as the LED test
 * (led_blink_100rpm.ino): starts at a fixed target and takes typed Serial
 * input to change it live, clamped to a safe range.
 *
 * Serial command format `[2026-07-29, revised]`:
 *   XXX   -- exactly 3 digit characters, RPM target, 100-999 (e.g. "250").
 *            Anything not exactly 3 digits, or out of 100-999, is rejected
 *            with an error message rather than silently clamped -- a typo
 *            like "25" or "2500" won't be interpreted as a nearby valid
 *            value.
 *   0     -- FREEZE (5s), see below.
 *   00    -- STOP, see below.
 *   +ANGLE -- azimuth target, 1-360 degrees (e.g. "+180"), see below.
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
 *
 * Boot manual alignment `[2026-07-29]`: after both motors' initFOC()
 * completes, both drivers are disabled for SYNC_WINDOW_MS (10s) so the
 * rotors can be hand-aligned by eye -- no props on this rig, a piece of tape
 * aligned to each rotor's off-center screw hole is the visual index mark.
 * The onboard LED (LED_BUILTIN) flashes for the duration as the cue to align
 * now. Drivers re-enable automatically when the window ends and closed-loop
 * velocity control starts at RPM_START (300). This is BOOT-ONLY
 * (runAlignWindow()) -- runtime "0" does something different (freeze, not
 * hand-align -- see below). This hand-align moment is also where the PLL's
 * zero reference is captured (top_angle_ref / bot_angle_ref): whatever the
 * physical offset is when the drivers re-enable becomes phase_target 0 deg
 * ("in sync") for the rest of the run. (An angle-mode self-align was tried
 * earlier this session instead of hand-turning, automatic with a live +/-
 * Serial trim, but had a persistent ~45deg oscillation that a conservative
 * P_angle drop (20 -> 5) didn't fix, so boot reverted to hand-turning. Full
 * history, including a real multi-turn-angle-wrap bug found along the way,
 * is in PS11_pending_revisions. bot_align_offset_rad,
 * BOT_ALIGN_OFFSET_DEFAULT_RAD, and TRIM_STEP_RAD are unused leftovers from
 * that attempt, not deleted.)
 *
 * PID_velocity gains `[2026-07-29, updated]`: P=0.3, I=2.0, D=0, Tf=0.08 --
 * result of PID_tune_test.ino's automated grid-search autotune ("a" command),
 * scored on worst-case ripple across 300/400/500 RPM specifically (this
 * sketch's actual operating range now that RPM_START/ALIGN_RESUME_RPM are
 * both 300). Supersedes the earlier §2.18 baseline (P=0.5, I=2.0, Tf=0.05),
 * which was tuned around 10-50 RPM behavior, not this range. Not yet
 * bench-confirmed in this sketch itself (only in the separate tuning tool) --
 * see PS11_pending_revisions.
 *
 * Runtime commands `[2026-07-29, revised -- azimuth hold is now a real PLL]`:
 *   "0"  -- FREEZE. Ramps to 0 RPM, then holds both motors at their current
 *           position (angle mode, runFreezeWindow()) for FREEZE_WINDOW_MS
 *           (5s) with the LED flashing, so index alignment can be visually
 *           checked -- actively holds wherever the shafts already are, so
 *           relative sync is preserved through the pause rather than left
 *           to drift. Auto-resumes at ALIGN_RESUME_RPM (300) when the
 *           window ends. phase_target_rad (the PLL's setpoint) is untouched
 *           by a freeze -- it just holds still along with the motors, and
 *           the PLL picks back up correcting toward it the moment velocity
 *           mode resumes.
 *   "+ANGLE" -- AZIMUTH TARGET `[2026-07-29, redesigned -- now a persistent
 *           PLL setpoint, not a one-shot maneuver]`. Type '+' followed by
 *           1-3 digits, 1-360 (e.g. "+180", "+45", "+270"). This just moves
 *           the PLL's target (phase_target_rad) -- it does not itself move
 *           anything. The PLL (see below) continuously measures the actual
 *           relative offset and drives BOT toward the new target, then
 *           keeps holding it there indefinitely afterward -- no separate
 *           "shift complete" event, no boost-then-stop state machine. TOP
 *           is NEVER touched by this, at any point. Takes the shortest path
 *           (may speed BOT up OR slow it down, whichever is closer) --
 *           this replaces the previous "+ANGLE" implementation's
 *           always-advance-forward rule, which doesn't apply to a real PLL.
 *   "00" -- STOP. Ramps to 0 RPM, then fully disables both drivers (motors
 *           free, no torque) and stays disabled. Not required to maintain
 *           sync -- this is a plain pause for pending re-upload / bench
 *           safety, not a sync-preserving action (unlike "0"). The PLL
 *           correction is suspended while stopped (nothing to correct with
 *           drivers off) and resumes cleanly on restart. Type any valid
 *           RPM or "0" to re-enable and resume.
 *
 * Azimuth PLL `[2026-07-29, new]`: replaces the old "boost BOT until the
 * measured gap crosses a threshold, then stop correcting" mechanism with a
 * continuous phase-locked loop that runs every control cycle, permanently.
 * TOP is the master/reference, driven purely by current_rpm, never touched
 * by the loop. Every cycle: measured relative offset is computed from how
 * far each motor has actually traveled (in its own commanded direction)
 * since the boot hand-align (top_angle_ref / bot_angle_ref, captured once
 * right after runAlignWindow() completes -- this is the PLL's "phase
 * detector" reference, i.e. the physically hand-aligned position = 0 deg
 * offset). The wrapped error between that measured offset and
 * phase_target_rad feeds a PI controller (PLL_KP, PLL_KI) whose output is a
 * small trim speed added to BOT's velocity command, clamped to
 * +/-PLL_MAX_TRIM_RPM. Because this runs every cycle rather than stopping
 * once "close enough," it does two jobs at once: it drives a step change in
 * phase_target_rad ("+ANGLE") toward the new target the same way the old
 * boost did (large error -> correction saturates at the clamp), AND it
 * keeps correcting indefinitely afterward, which is the actual point --
 * this is what stops the slow azimuth drift that plain matched-velocity
 * commands can't prevent (see the earlier PLL feasibility discussion in
 * PS11_pending_revisions). PLL_KP/PLL_KI are conservative untested starting
 * guesses -- same caution as every other gain tuned this session (P_angle,
 * PID_velocity): if it oscillates or hunts, these are the first things to
 * turn down, not up.
 *
 * RPM-dependent vibration `[2026-07-29]`: bench data shows smooth running at
 * 10 and 50 RPM, definite vibration (visible and felt) at 100 RPM, and less
 * again by 200+ -- looks like a mechanical/electrical resonance peaking
 * near 100 RPM specifically, separate from the ~900 RPM pulsing already
 * noted in PS11 §2.18. RPM_START / ALIGN_RESUME_RPM were briefly set to 100
 * (right on the resonance) before being raised to 300 by request -- 300 is
 * comfortably in the "less again by 200+" smoother range noted above, so
 * this incidentally moves boot/resume off the resonance again, though that
 * wasn't the stated reason for the change. See PS11_pending_revisions --
 * the resonance itself remains unexplained and unfixed either way.
 */

#include <SimpleFOC.h>

const int POLE_PAIRS = 7;  // 2804 gimbal outrunner, 12N14P -> 7 pole pairs

const float RPM_MIN = 100.0f;   // `[2026-07-29]` floor for typed Serial input -- must be
const float RPM_MAX = 999.0f;   // typed as exactly 3 digit characters (XXX), see parsing below
const float RPM_START = 300.0f;         // boot starting RPM `[2026-07-29, raised from 100 by request]`
const float ALIGN_RESUME_RPM = 300.0f;  // speed to resume at automatically after a "0" freeze cycle -- raised with RPM_START, same request

const float PLL_MAX_TRIM_RPM = 30.0f;   // hard clamp on the PLL's total (P+I) correction to BOT,
                                         // same magnitude as the old one-shot boost -- caps how
                                         // aggressively the loop can chase a large error. Untested.
const float PLL_MAX_TRIM_RAD_S = PLL_MAX_TRIM_RPM * 2.0f * PI / 60.0f;
const float PLL_KP = (PLL_MAX_TRIM_RAD_S / PI) * 2.0f;  // `[2026-07-29, doubled]` base value was
                                                // rad/s of trim needed to saturate at 180deg
                                                // error; doubled by request as a first-pass fix
                                                // for observed +/-10-12deg residual drift (goal
                                                // +/-2deg). Quick test, not a final value -- if
                                                // this makes it worse/oscillate instead of
                                                // tighter, the more likely real limiter is the
                                                // softened PID_velocity.P (0.3, chosen purely for
                                                // RPM ripple, not phase-loop responsiveness) --
                                                // back this off and raise PID_velocity.P instead
                                                // of pushing this further. See PS11_pending_revisions.
const float PLL_KI = PLL_KP * 0.15f;     // integral ratio also nudged up slightly (was 0.1) --
                                          // removes small steady-state offset (e.g. from the two
                                          // motors not tracking identically) that the P term alone
                                          // can't fully cancel. Untested, same caveat as PLL_KP.

float target_rpm = RPM_START;  // user-commanded value -- can jump instantly on input
float current_rpm = RPM_START;  // smoothed setpoint actually sent to the motors
bool running = true;
bool motors_enabled = true;      // tracks driver PWM output -- false after a "00" hard stop
bool stop_requested = false;     // "00" was typed -- disable drivers once ramped to 0
bool freeze_requested = false;   // "0" was typed -- freeze once ramped to 0, then auto-resume

// -- azimuth PLL state. top_angle_ref / bot_angle_ref are captured once,
//    right after the boot hand-align window completes -- this is the PLL's
//    zero reference (the physically hand-aligned position = phase_target
//    0 deg). phase_target_rad is the live setpoint, moved instantly by
//    "+ANGLE" (the PLL itself does the actual moving, gradually, every
//    loop). phase_error_integral is the PI controller's accumulator,
//    anti-windup clamped in loop(). --
float top_angle_ref = 0;
float bot_angle_ref = 0;
float phase_target_rad = 0;
float phase_error_integral = 0;

// Wrap an angle (radians) into (-PI, PI] -- used to find the shortest
// signed path from measured phase to phase_target, so the PLL can correct
// in either direction (speed BOT up or slow it down) rather than always
// advancing forward.
float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a <= -PI) a += 2.0f * PI;
  return a;
}

const float RPM_RAMP_RATE = 200.0f;  // max RPM/s change -- lower = gentler transitions
unsigned long last_ramp_update = 0;

const float MAX_MOTOR_VOLTAGE = 12.0f;  // hard cap on voltage_limit for both motors -- close to the 18V supply, testing whether 900+ RPM pulsing was voltage saturation

const unsigned long SYNC_WINDOW_MS = 10000;       // boot hand-align window duration
const unsigned long FREEZE_WINDOW_MS = 5000;      // `[2026-07-29, new]` runtime "0" freeze duration -- shorter than boot align, just long enough to visually check alignment
const unsigned long SYNC_FLASH_PERIOD_MS = 250;   // LED toggle period during align/freeze windows
const float TRIM_STEP_RAD = 0.0349f;              // unused leftover from the abandoned self-align attempt -- see header comment

// [CALIBRATE] Unused leftover from the abandoned angle-mode self-align
// attempt -- left defined, not deleted, in case that approach is revisited.
// See header comment / PS11_pending_revisions.
const float BOT_ALIGN_OFFSET_DEFAULT_RAD = 0.0f;
float bot_align_offset_rad = BOT_ALIGN_OFFSET_DEFAULT_RAD;

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

// ================= align routine (manual, hand-turn -- boot only) =================
// Disables both drivers (shafts free to hand-turn) and flashes the onboard
// LED for SYNC_WINDOW_MS while the user physically aligns the TOP/BOT tape
// index marks by eye, then re-enables both drivers. Whatever the physical
// offset is at that moment becomes the PLL's "in sync" reference -- setup()
// captures top_angle_ref/bot_angle_ref immediately after this returns.
void runAlignWindow() {
  motorTop.disable();
  motorBot.disable();
  Serial.print("# ALIGN -- hand-align TOP/BOT tape index marks now. Motors free for ");
  Serial.print(SYNC_WINDOW_MS / 1000);
  Serial.println("s...");
  unsigned long sync_start = millis();
  unsigned long last_flash = sync_start;
  bool led_on = false;
  while (millis() - sync_start < SYNC_WINDOW_MS) {
    if (millis() - last_flash >= SYNC_FLASH_PERIOD_MS) {
      last_flash = millis();
      led_on = !led_on;
      digitalWrite(LED_BUILTIN, led_on);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  motorTop.enable();
  motorBot.enable();
  Serial.println("# Align window complete -- resuming closed-loop control.");
}

// ================= freeze window (runtime "0") =================
// Holds both motors at their CURRENT position (angle mode, drivers stay
// enabled) for FREEZE_WINDOW_MS with the onboard LED flashing, then returns
// to velocity mode -- caller resumes at ALIGN_RESUME_RPM. Targets wherever
// the shafts already are, not a fixed reference, so whatever TOP/BOT
// relative offset exists going in is actively held, not pulled toward
// anything. The PLL's phase_target_rad is untouched by this function --
// top_angle_ref/bot_angle_ref (its zero reference) don't move either, so
// the loop picks up exactly where it left off once velocity mode resumes.
void runFreezeWindow() {
  motorTop.controller = MotionControlType::angle;
  motorBot.controller = MotionControlType::angle;
  float top_target = motorTop.shaft_angle;
  float bot_target = motorBot.shaft_angle;
  Serial.println("# FROZEN -- holding position, check index alignment now");
  unsigned long sync_start = millis();
  unsigned long last_flash = sync_start;
  bool led_on = false;
  while (millis() - sync_start < FREEZE_WINDOW_MS) {
    motorTop.loopFOC();
    motorBot.loopFOC();
    motorTop.move(top_target);
    motorBot.move(bot_target);
    if (millis() - last_flash >= SYNC_FLASH_PERIOD_MS) {
      last_flash = millis();
      led_on = !led_on;
      digitalWrite(LED_BUILTIN, led_on);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  motorTop.controller = MotionControlType::velocity;
  motorBot.controller = MotionControlType::velocity;
}

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
  motorTop.PID_velocity.P = 0.3;   // `[2026-07-29]` PID_tune_test.ino autotune result for 300-500 RPM
  motorTop.PID_velocity.I = 2.0;   // (worst-case ripple score, D fixed 0) -- supersedes the old
  motorTop.PID_velocity.D = 0.0;   // §2.18 baseline (P=0.5/I=2.0/Tf=0.05) for this RPM range
  motorTop.PID_velocity.output_ramp = 500;
  motorTop.LPF_velocity.Tf = 0.08;
  motorTop.voltage_limit = MAX_MOTOR_VOLTAGE;
  motorTop.P_angle.P = 5;  // used only by runFreezeWindow() (angle mode). Conservative untested
  motorTop.P_angle.I = 0;  // guess -- library default (~20) suspected too aggressive for these
  motorTop.P_angle.D = 0;  // low-inertia gimbal motors. See PS11_pending_revisions.
  motorTop.init();
  motorTop.initFOC();

  I2C_BOT.begin();
  encBot.init(&I2C_BOT);
  motorBot.linkSensor(&encBot);
  driverBot.voltage_power_supply = 18;  // <-- SET to your actual supply voltage
  driverBot.init();
  motorBot.linkDriver(&driverBot);
  motorBot.controller = MotionControlType::velocity;
  motorBot.PID_velocity.P = 0.3;   // kept identical to motorTop -- must match or TOP/BOT will
  motorBot.PID_velocity.I = 2.0;   // track differently and drift out of index alignment
  motorBot.PID_velocity.D = 0.0;
  motorBot.PID_velocity.output_ramp = 500;
  motorBot.LPF_velocity.Tf = 0.08;
  motorBot.voltage_limit = MAX_MOTOR_VOLTAGE;
  motorBot.P_angle.P = 5;   // kept identical to motorTop -- see above
  motorBot.P_angle.I = 0;
  motorBot.P_angle.D = 0;
  motorBot.init();
  motorBot.initFOC();

  // ================= hand-align window (boot) =================
  pinMode(LED_BUILTIN, OUTPUT);
  runAlignWindow();
  // -- PLL zero reference: whatever the hand-aligned position is right now
  //    becomes phase_target 0 deg for the rest of the run. --
  top_angle_ref = motorTop.shaft_angle;
  bot_angle_ref = motorBot.shaft_angle;
  last_ramp_update = millis();  // `[2026-07-29, bugfix]` runAlignWindow() blocks for
                                 // SYNC_WINDOW_MS -- reset here too, same stale-dt issue as the
                                 // freeze/resume fix below (benign here since phase_error is ~0
                                 // right at this reference point, but fixed defensively).
  Serial.println("# Resuming closed-loop velocity control. Azimuth PLL zeroed to this position.");

  Serial.print("# TOP+BOT spin test ready. Rate: ");
  Serial.print(target_rpm);
  Serial.println(" RPM.");
  Serial.println("# Commands: XXX = 3-digit RPM (100-999), 0 = freeze 5s, 00 = stop, +ANGLE = azimuth PLL target (1-360 deg).");
}

// ================= loop =================

void loop() {
  motorTop.loopFOC();
  motorBot.loopFOC();

  // -- non-blocking serial command read: a typed command terminated by newline --
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (rx.length() > 0) {
        if (rx == "00") {
          stop_requested = true;
          freeze_requested = false;
          running = false;
          Serial.println("# STOP requested -- ramping to 0 RPM, then disabling drivers (pause for re-upload, sync not preserved)");
        } else if (rx == "0") {
          if (!motors_enabled) {
            motorTop.enable();
            motorBot.enable();
            motors_enabled = true;
          }
          freeze_requested = true;
          stop_requested = false;
          running = false;
          Serial.println("# FREEZE requested -- ramping to 0 RPM, then holding 5s for a sync check (PLL target preserved)");
        } else if (rx.charAt(0) == '+') {
          String angle_str = rx.substring(1);
          bool valid = angle_str.length() >= 1 && angle_str.length() <= 3;
          for (unsigned int i = 0; valid && i < angle_str.length(); i++) {
            if (!isDigit(angle_str.charAt(i))) valid = false;
          }
          int angle = valid ? angle_str.toInt() : 0;
          if (valid && angle >= 1 && angle <= 360) {
            if (!motors_enabled) {
              motorTop.enable();
              motorBot.enable();
              motors_enabled = true;
              Serial.println("# drivers re-enabled");
            }
            phase_target_rad = (float)angle * PI / 180.0f;
            Serial.print("# AZIMUTH -- PLL target set to ");
            Serial.print(angle);
            Serial.println("deg, correcting toward it now, no stop needed");
          } else {
            Serial.println("# ignored -- use +ANGLE where ANGLE is 1-360");
          }
        } else {
          bool valid_rpm = rx.length() == 3;
          for (unsigned int i = 0; valid_rpm && i < rx.length(); i++) {
            if (!isDigit(rx.charAt(i))) valid_rpm = false;
          }
          float val = valid_rpm ? rx.toFloat() : -1.0f;
          if (valid_rpm && val >= RPM_MIN && val <= RPM_MAX) {
            if (!motors_enabled) {
              motorTop.enable();
              motorBot.enable();
              motors_enabled = true;
              Serial.println("# drivers re-enabled");
            }
            stop_requested = false;
            freeze_requested = false;
            target_rpm = val;
            running = true;
            Serial.print("# new target: ");
            Serial.print(target_rpm);
            Serial.println(" RPM");
          } else {
            Serial.println("# ignored -- enter 0 (freeze), 00 (stop), +ANGLE (1-360), or a 3-digit RPM (100-999)");
          }
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

  // -- once ramped fully down to 0, act on whichever command triggered the
  //    ramp-down: "00" disables the drivers outright and stays that way;
  //    "0" runs the blocking freeze window (see runFreezeWindow()) and then
  //    auto-resumes at ALIGN_RESUME_RPM. "+" is NOT handled here -- it never
  //    sets running=false, so it doesn't wait for a ramp-down at all; see
  //    the fluid azimuth-shift block below instead. --
  if (!running && current_rpm == 0.0f) {
    if (stop_requested && motors_enabled) {
      motorTop.disable();
      motorBot.disable();
      motors_enabled = false;
      stop_requested = false;
      phase_error_integral = 0;  // pause PLL windup while disabled -- nothing to correct with no torque
      Serial.println("# drivers disabled -- motors free to spin. Type 0 (freeze), 00 (stop), +ANGLE, or a 3-digit RPM to resume");
    } else if (freeze_requested) {
      freeze_requested = false;
      runFreezeWindow();
      last_ramp_update = millis();  // `[2026-07-29, bugfix]` runFreezeWindow() blocks for
                                     // FREEZE_WINDOW_MS -- without this, the next loop()'s dt
                                     // calc sees ~5s of elapsed time and injects a huge one-shot
                                     // kick into phase_error_integral below. This was corrupting
                                     // the PLL lock on every single freeze/resume cycle -- see
                                     // PS11_pending_revisions.
      phase_error_integral = 0.0f;  // start the PLL's integral term clean on resume -- freeze
                                     // already holds the correct relative phase exactly (angle
                                     // mode), so there's no stale correction worth carrying
                                     // forward from before the freeze.
      target_rpm = ALIGN_RESUME_RPM;
      current_rpm = 0.0f;  // ramp back up from a standstill after the blocking freeze window
      running = true;
      Serial.print("# freeze done -- resuming at ");
      Serial.print(ALIGN_RESUME_RPM);
      Serial.println(" RPM");
    }
  }

  float target_rad_s = current_rpm * 2.0f * PI / 60.0f;
  motorTop.move(target_rad_s);  // TOP is never touched by the PLL -- always just this

  // -- azimuth PLL, runs every cycle. top_progress/bot_progress are how far
  //    each motor has actually traveled, in its own commanded direction,
  //    since the boot hand-align reference -- both expected >= 0 as long as
  //    TOP is always commanded positive and BOT always negative (true for
  //    this sketch, RPM input is 100-999 only, no reverse). measured_rad is
  //    therefore a continuously valid signed relative-phase quantity, no
  //    per-shift reset needed. phase_error is that measured value compared
  //    to phase_target_rad, wrapped to the shortest signed path. --
  float bot_rad_s = -target_rad_s;
  float top_progress = motorTop.shaft_angle - top_angle_ref;
  float bot_progress = -(motorBot.shaft_angle - bot_angle_ref);
  float measured_rad = bot_progress - top_progress;
  float phase_error = wrapPi(measured_rad - phase_target_rad);
  if (motors_enabled) {  // `[2026-07-29, bugfix]` was gated on `running`, which goes false the
                          // instant "0"/"00" is typed -- silencing correction during the ~1.5s
                          // ramp-down to every freeze, letting fresh uncorrected drift creep in
                          // right before the snapshot. Gating on motors_enabled instead keeps the
                          // PLL correcting through ramp-down for both "0" and "00", only pausing
                          // once drivers are actually off. See PS11_pending_revisions.
    phase_error_integral += phase_error * dt;
    float max_integral = PLL_MAX_TRIM_RAD_S / PLL_KI;
    phase_error_integral = constrain(phase_error_integral, -max_integral, max_integral);
    float phase_correction_rad_s = PLL_KP * phase_error + PLL_KI * phase_error_integral;
    phase_correction_rad_s = constrain(phase_correction_rad_s, -PLL_MAX_TRIM_RAD_S, PLL_MAX_TRIM_RAD_S);
    bot_rad_s += phase_correction_rad_s;
  }
  motorBot.move(bot_rad_s);  // BOT runs reversed relative to TOP, plus the PLL's continuous trim

  // -- Serial Plotter output (labeled fields -> auto legend in Arduino IDE 2.x) --
  static unsigned long last_print = 0;
  if (millis() - last_print > 50) {  // ~20 Hz plot rate
    last_print = millis();
    Serial.print("rpm_cmd:");     Serial.print(current_rpm);
    Serial.print(",rpm_top_act:"); Serial.print(motorTop.shaft_velocity * 60.0f / (2.0f * PI));
    Serial.print(",rpm_bot_act:"); Serial.print(motorBot.shaft_velocity * 60.0f / (2.0f * PI));
    Serial.print(",az_meas_deg:"); Serial.print(measured_rad * 180.0f / PI);
    Serial.print(",az_target_deg:"); Serial.println(phase_target_rad * 180.0f / PI);
  }
}
