/**
 * @file SecureIndustrialGate.ino
 * @brief Scenario 1 gate control with button-selected hold-delay tuning.
 *
 * PIR motion opens the gate. With a clear path, the selected delay controls
 * automatic closure. BOOT enters/leaves MANUAL delay adjustment (3-15 s).
 * An obstruction below 20 cm during closure freezes the gate and chirps once.
 * After the path clears, the OLED counts down five seconds before closing.
 * A returning obstruction or lost echo cancels the countdown.
 * UART reports live status and accepts HOLD 3-15, OPEN and CLOSE.
 * OLED SPI/I2C options and all supplied GPIO assignments are preserved.
 */

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <esp_arduino_version.h>
#include <soc/gpio_struct.h>

#ifndef OLED_USE_SPI
#define OLED_USE_SPI 1
#endif
#ifndef ACTIVE_BUZZER
#define ACTIVE_BUZZER 1
#endif

// Pin assignments and timing.
namespace Pins {
constexpr uint8_t OLED_SCL = 22, OLED_SDA = 21, OLED_RESET = 32, OLED_DC = 33, OLED_CS = 23;
constexpr uint8_t POTENTIOMETER = 34, PIR = 27, ULTRASONIC_TRIGGER = 5, ULTRASONIC_ECHO = 18;
constexpr uint8_t BUZZER = 14, SERVO = 13, BOOT_BUTTON = 0;
}
namespace Timing {
constexpr uint32_t PIR_WARM_UP_MS = 30000, POT_SAMPLE_MS = 50, ULTRASONIC_INTERVAL_MS = 100;
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 30000, OLED_REFRESH_MS = 200;
constexpr uint32_t BUTTON_POLL_MS = 10, BUTTON_DEBOUNCE_MS = 35;
constexpr uint32_t SERVO_OPEN_STEP_MS = 15, SERVO_CLOSE_STEP_MS = 22, BUZZER_ON_MS = 130;
constexpr uint32_t SAFETY_COUNTDOWN_MS = 5000, SERIAL_STATUS_MS = 1000;
}
namespace Limits {
constexpr float OBSTRUCTION_CM = 20.0F, MIN_VALID_DISTANCE_CM = 2.0F,
                MAX_VALID_DISTANCE_CM = 450.0F;
constexpr uint32_t HOLD_MIN_MS = 3000, HOLD_MAX_MS = 15000;
constexpr int CLOSED_ANGLE = 10, OPEN_ANGLE = 100;
constexpr uint16_t SERVO_MIN_PULSE_US = 500, SERVO_MAX_PULSE_US = 2400, SERVO_PERIOD_US = 20000;
constexpr uint8_t SERVO_PWM_CHANNEL = 0, SERVO_PWM_BITS = 16;
constexpr uint16_t PASSIVE_BUZZER_HZ = 2400;
}

constexpr uint8_t OLED_WIDTH = 128, OLED_HEIGHT = 64;
#if OLED_USE_SPI
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, Pins::OLED_SDA, Pins::OLED_SCL, Pins::OLED_DC,
                         Pins::OLED_RESET, Pins::OLED_CS);
#else
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, Pins::OLED_RESET);
#endif

enum class Mode : uint8_t { AUTO, MANUAL, SAFETY };
enum class Motion : uint8_t { CLOSED, OPENING, OPEN, CLOSING, HALTED };
enum class RangePhase : uint8_t { IDLE, TRIGGER_HIGH, WAITING_FOR_ECHO };

// Controller state.
Mode mode = Mode::AUTO;
Motion motion = Motion::OPEN;
RangePhase ultrasonicPhase = RangePhase::IDLE;
int gateAngle = Limits::OPEN_ANGLE, targetAngle = Limits::OPEN_ANGLE;
uint32_t holdOpenMs = Limits::HOLD_MIN_MS, holdStartedMs = 0, bootMs = 0;
uint32_t lastServoStepMs = 0, lastPotSampleMs = 0, lastOledRefreshMs = 0;
uint32_t ultrasonicCycleStartedUs = 0, lastUltrasonicStartMs = 0;
uint32_t potFiltered = 0;
bool potInitialised = false, pirMotion = false, oledReady = false;
float distanceCm = NAN;
bool distanceValid = false;
uint32_t distanceUpdatedMs = 0;
bool buzzerOn = false;
uint32_t buzzerStartedMs = 0;
bool safetyCountingDown = false;
uint32_t safetyClearStartedMs = 0;
volatile uint32_t echoRiseUs = 0, capturedEchoDurationUs = 0;
volatile bool echoSampleReady = false, echoCaptureArmed = false, bootEdgePending = false;
portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;
bool buttonRawPressed = false, buttonStablePressed = false, buttonPressArmed = false;
uint32_t buttonRawChangedMs = 0, lastButtonPollMs = 0;
char serialCommand[24] = {};
uint8_t serialCommandLength = 0;
bool serialCommandInvalid = false;
char serialOutput[128] = "Ready. Commands: HOLD 3-15, OPEN, CLOSE. Press Enter to send.\n";
size_t serialOutputOffset = 0;
uint32_t lastSerialStatusMs = 0;

/**
 * @brief Check an elapsed interval safely across timer rollover.
 * @param now Current timer value.
 * @param since Starting timer value.
 * @param interval Required interval in the same units.
 * @return True when the interval has elapsed.
 */
bool hasElapsed(const uint32_t now, const uint32_t since, const uint32_t interval) {
  return static_cast<uint32_t>(now - since) >= interval;
}

/**
 * @brief Configure the existing 50 Hz servo PWM output.
 * @return True when PWM initialization succeeds.
 */
bool initialiseServoPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttachChannel(Pins::SERVO, 50, Limits::SERVO_PWM_BITS, Limits::SERVO_PWM_CHANNEL);
#else
  ledcSetup(Limits::SERVO_PWM_CHANNEL, 50, Limits::SERVO_PWM_BITS);
  ledcAttachPin(Pins::SERVO, Limits::SERVO_PWM_CHANNEL);
  return true;
#endif
}

/**
 * @brief Convert an angle to the calibrated PWM pulse and duty cycle.
 * @param angle Requested servo angle in degrees.
 * @return Nothing.
 */
void writeServoAngle(const int angle) {
  const int constrainedAngle = constrain(angle, 0, 180);
  const uint32_t pulseUs =
    map(constrainedAngle, 0, 180, Limits::SERVO_MIN_PULSE_US, Limits::SERVO_MAX_PULSE_US);
  const uint32_t maximumDuty = (1UL << Limits::SERVO_PWM_BITS) - 1UL;
  const uint32_t duty = (pulseUs * maximumDuty) / Limits::SERVO_PERIOD_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(Limits::SERVO_PWM_CHANNEL, duty);
#else
  ledcWrite(Limits::SERVO_PWM_CHANNEL, duty);
#endif
}

/**
 * @brief Capture ultrasonic echo edges through the GPIO input register.
 * @return Nothing.
 */
void IRAM_ATTR onUltrasonicEchoChange() {
  if (!echoCaptureArmed) return;
  uint32_t nowUs = micros();
  if (GPIO.in & (1UL << Pins::ULTRASONIC_ECHO)) {
    echoRiseUs = nowUs;
    return;
  }
  if (echoRiseUs == 0) return;
  portENTER_CRITICAL_ISR(&echoMux);
  capturedEchoDurationUs = static_cast<uint32_t>(nowUs - echoRiseUs);
  echoSampleReady = true;
  echoCaptureArmed = false;
  portEXIT_CRITICAL_ISR(&echoMux);
  echoRiseUs = 0;
}

/**
 * @brief Flag a BOOT edge for debouncing in the main loop.
 * @return Nothing.
 */
void IRAM_ATTR onBootButtonChange() {
  bootEdgePending = true;
}

/**
 * @brief Get the gate motion label for the OLED and UART.
 * @param value Current motion state.
 * @return The corresponding display label.
 */
const char *motionName(Motion value) {
  static const char *names[] = { "CLOSED", "OPENING", "OPEN", "CLOSING", "HALTED" };
  return names[static_cast<uint8_t>(value)];
}

/**
 * @brief Switch the active buzzer or passive piezo output.
 * @param enabled Whether the warning should sound.
 * @return Nothing.
 */
void setBuzzerOutput(bool enabled) {
#if ACTIVE_BUZZER
  digitalWrite(Pins::BUZZER, enabled ? HIGH : LOW);
#else
  if (enabled) tone(Pins::BUZZER, Limits::PASSIVE_BUZZER_HZ);
  else noTone(Pins::BUZZER);
#endif
}

/**
 * @brief End the single safety warning chirp without blocking.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updateBuzzer(uint32_t now) {
  if (buzzerOn && hasElapsed(now, buzzerStartedMs, Timing::BUZZER_ON_MS)) {
    setBuzzerOutput(false);
    buzzerOn = false;
  }
}

/**
 * @brief Read the dial only in MANUAL and set the hold-open delay.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updatePotentiometer(uint32_t now) {
  if (mode != Mode::MANUAL || !hasElapsed(now, lastPotSampleMs, Timing::POT_SAMPLE_MS)) return;
  lastPotSampleMs = now;
  const uint16_t sample = analogRead(Pins::POTENTIOMETER);
  if (!potInitialised) {
    potFiltered = sample;
    potInitialised = true;
  } else {
    potFiltered = (potFiltered * 7U + sample) / 8U;
  }
  holdOpenMs = map(potFiltered, 0, 4095, Limits::HOLD_MIN_MS, Limits::HOLD_MAX_MS);
}

/**
 * @brief Schedule trigger pulses and consume interrupt-captured echoes without blocking.
 * @param now Current millisecond timestamp.
 * @param nowUs Current microsecond timestamp.
 * @return Nothing.
 */
void updateUltrasonic(uint32_t now, uint32_t nowUs) {
  if (ultrasonicPhase == RangePhase::IDLE) {
    if (hasElapsed(now, lastUltrasonicStartMs, Timing::ULTRASONIC_INTERVAL_MS)) {
      echoRiseUs = 0;
      portENTER_CRITICAL(&echoMux);
      echoSampleReady = false;
      echoCaptureArmed = true;
      portEXIT_CRITICAL(&echoMux);
      digitalWrite(Pins::ULTRASONIC_TRIGGER, HIGH);
      ultrasonicCycleStartedUs = nowUs;
      lastUltrasonicStartMs = now;
      ultrasonicPhase = RangePhase::TRIGGER_HIGH;
    }
    return;
  }
  if (ultrasonicPhase == RangePhase::TRIGGER_HIGH) {
    if (hasElapsed(nowUs, ultrasonicCycleStartedUs, 10U)) {
      digitalWrite(Pins::ULTRASONIC_TRIGGER, LOW);
      ultrasonicCycleStartedUs = nowUs;
      ultrasonicPhase = RangePhase::WAITING_FOR_ECHO;
    }
    return;
  }
  bool sampleReady = false;
  uint32_t echoDurationUs = 0;
  portENTER_CRITICAL(&echoMux);
  if (echoSampleReady) {
    sampleReady = true;
    echoDurationUs = capturedEchoDurationUs;
    echoSampleReady = false;
  }
  portEXIT_CRITICAL(&echoMux);
  if (sampleReady) {
    float cm = (echoDurationUs * 0.0343F) / 2.0F;
    distanceUpdatedMs = now;
    distanceValid = cm >= Limits::MIN_VALID_DISTANCE_CM && cm <= Limits::MAX_VALID_DISTANCE_CM;
    distanceCm = distanceValid ? cm : NAN;
    ultrasonicPhase = RangePhase::IDLE;
    return;
  }
  if (hasElapsed(nowUs, ultrasonicCycleStartedUs, Timing::ULTRASONIC_TIMEOUT_US)) {
    portENTER_CRITICAL(&echoMux);
    echoCaptureArmed = false;
    echoSampleReady = false;
    portEXIT_CRITICAL(&echoMux);
    echoRiseUs = 0;
    distanceCm = NAN;
    distanceValid = false;
    distanceUpdatedMs = now;
    ultrasonicPhase = RangePhase::IDLE;
  }
}

/**
 * @brief Check whether the last ultrasonic measurement is valid and recent.
 * @param now Current millisecond timestamp.
 * @return True for a valid reading less than three sample intervals old.
 */
bool distanceIsFresh(uint32_t now) {
  return distanceValid && !hasElapsed(now, distanceUpdatedMs, Timing::ULTRASONIC_INTERVAL_MS * 3U);
}

/**
 * @brief Require a fresh distance above 20 cm for closing or recovery.
 * @param now Current millisecond timestamp.
 * @return True when the path is clear.
 */
bool isPathClear(uint32_t now) {
  return distanceIsFresh(now) && distanceCm > Limits::OBSTRUCTION_CM;
}

/**
 * @brief Command opening; start the hold interval once fully open.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void openGate(uint32_t now) {
  targetAngle = Limits::OPEN_ANGLE;
  motion = gateAngle == targetAngle ? Motion::OPEN : Motion::OPENING;
  if (motion == Motion::OPEN) holdStartedMs = now;
}

/**
 * @brief Advance the servo one degree when its next step is due.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updateServo(uint32_t now) {
  if (gateAngle == targetAngle) return;
  const bool opening = targetAngle > gateAngle;
  const uint32_t stepMs = opening ? Timing::SERVO_OPEN_STEP_MS : Timing::SERVO_CLOSE_STEP_MS;
  if (!hasElapsed(now, lastServoStepMs, stepMs)) return;
  lastServoStepMs = now;
  gateAngle += opening ? 1 : -1;
  writeServoAngle(gateAngle);
  if (gateAngle == targetAngle) {
    motion = opening ? Motion::OPEN : Motion::CLOSED;
    if (opening) holdStartedMs = now;
  }
}

/**
 * @brief Halt below 20 cm, then close after five continuously clear seconds.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updateSafetyLogic(uint32_t now) {
  if (mode == Mode::SAFETY) {
    if (!isPathClear(now)) {
      safetyCountingDown = false;
    } else if (!safetyCountingDown) {
      safetyCountingDown = true;
      safetyClearStartedMs = now;
    } else if (hasElapsed(now, safetyClearStartedMs, Timing::SAFETY_COUNTDOWN_MS)) {
      safetyCountingDown = false;
      mode = Mode::AUTO;
      targetAngle = Limits::CLOSED_ANGLE;
      motion = Motion::CLOSING;
    }
    return;
  }
  if (motion != Motion::CLOSING || !distanceIsFresh(now) || distanceCm >= Limits::OBSTRUCTION_CM)
    return;
  targetAngle = gateAngle;
  motion = Motion::HALTED;
  mode = Mode::SAFETY;
  safetyCountingDown = false;
  buttonPressArmed = false;
  buzzerOn = true;
  buzzerStartedMs = now;
  setBuzzerOutput(true);
}

/**
 * @brief Toggle delay tuning; BOOT has no action during safety recovery.
 * @return Nothing.
 */
void handleButtonPress() {
  if (mode == Mode::SAFETY) return;
  mode = mode == Mode::AUTO ? Mode::MANUAL : Mode::AUTO;
  potInitialised = false;
}

/**
 * @brief Debounce BOOT and handle one action on release of each new press.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updateBootButton(uint32_t now) {
  if (!bootEdgePending && !hasElapsed(now, lastButtonPollMs, Timing::BUTTON_POLL_MS)) return;
  bootEdgePending = false;
  lastButtonPollMs = now;
  const bool pressed = digitalRead(Pins::BOOT_BUTTON) == LOW;
  if (pressed != buttonRawPressed) {
    buttonRawPressed = pressed;
    buttonRawChangedMs = now;
  }
  if (buttonRawPressed == buttonStablePressed
      || !hasElapsed(now, buttonRawChangedMs, Timing::BUTTON_DEBOUNCE_MS))
    return;
  buttonStablePressed = buttonRawPressed;
  if (buttonStablePressed) {
    buttonPressArmed = true;
  } else if (buttonPressArmed) {
    buttonPressArmed = false;
    handleButtonPress();
  }
}

/**
 * @brief Run PIR opening and safe timed closing in AUTO and MANUAL tuning.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updateGate(uint32_t now) {
  if (mode == Mode::SAFETY || !hasElapsed(now, bootMs, Timing::PIR_WARM_UP_MS)) return;
  if (pirMotion) {
    holdStartedMs = now;
    if (motion == Motion::CLOSED || motion == Motion::CLOSING) openGate(now);
  } else if (motion == Motion::OPEN && hasElapsed(now, holdStartedMs, holdOpenMs)
             && isPathClear(now)) {
    targetAngle = Limits::CLOSED_ANGLE;
    motion = Motion::CLOSING;
  }
}

/**
 * @brief Apply HOLD, OPEN or CLOSE while preserving safety checks and servo endpoints.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void handleSerialCommand(uint32_t now) {
  if (!serialCommandInvalid
      && (strcmp(serialCommand, "OPEN") == 0 || strcmp(serialCommand, "CLOSE") == 0)) {
    const bool opening = strcmp(serialCommand, "OPEN") == 0;
    if (mode == Mode::SAFETY) {
      snprintf(serialOutput, sizeof(serialOutput), "Blocked: wait for safety recovery.\n");
    } else if (opening) {
      openGate(now);
      snprintf(serialOutput, sizeof(serialOutput), "OK: opening. Normal hold delay applies.\n");
    } else if (!hasElapsed(now, bootMs, Timing::PIR_WARM_UP_MS)) {
      snprintf(serialOutput, sizeof(serialOutput), "Blocked: PIR is warming up.\n");
    } else if (pirMotion || !isPathClear(now)) {
      snprintf(serialOutput, sizeof(serialOutput),
               "Blocked: closing needs no motion and a clear range reading.\n");
    } else {
      targetAngle = Limits::CLOSED_ANGLE;
      motion = gateAngle == targetAngle ? Motion::CLOSED : Motion::CLOSING;
      snprintf(serialOutput, sizeof(serialOutput), "OK: closing.\n");
    }
    return;
  }
  char *end = nullptr;
  const bool hasPrefix = strncmp(serialCommand, "HOLD ", 5) == 0;
  const long seconds = hasPrefix ? strtol(serialCommand + 5, &end, 10) : 0;
  if (serialCommandInvalid || !hasPrefix || end == serialCommand + 5 || *end != '\0'
      || seconds < static_cast<long>(Limits::HOLD_MIN_MS / 1000U)
      || seconds > static_cast<long>(Limits::HOLD_MAX_MS / 1000U)) {
    snprintf(serialOutput, sizeof(serialOutput),
             "Use HOLD 3-15 (whole seconds), OPEN or CLOSE, then Enter.\n");
    return;
  }
  holdOpenMs = static_cast<uint32_t>(seconds) * 1000U;
  if (mode == Mode::MANUAL) {
    mode = Mode::AUTO;
    potInitialised = false;
  }
  snprintf(serialOutput, sizeof(serialOutput), "OK: hold open = %ld s.\n", seconds);
}

/**
 * @brief Read bounded UART input and send buffered replies or one-second live status.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updateSerial(uint32_t now) {
  // Only write bytes that fit now; a busy terminal must not stall gate control.
  if (serialOutput[serialOutputOffset] != '\0') {
    const int room = Serial.availableForWrite();
    if (room <= 0) return;
    const size_t remaining = strlen(serialOutput + serialOutputOffset);
    const size_t count = remaining < static_cast<size_t>(room) ? remaining : room;
    serialOutputOffset +=
      Serial.write(reinterpret_cast<const uint8_t *>(serialOutput + serialOutputOffset), count);
    if (serialOutput[serialOutputOffset] != '\0') return;
    serialOutputOffset = 0;
    serialOutput[0] = '\0';
  }

  // Limit work per loop, including when pasted input is malformed or very long.
  for (uint8_t consumed = 0; consumed < 16 && Serial.available() > 0; ++consumed) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\r' || value == '\n') {
      if (serialCommandLength == 0 && !serialCommandInvalid) continue;
      serialCommand[serialCommandLength] = '\0';
      handleSerialCommand(now);
      serialCommandLength = 0;
      serialCommandInvalid = false;
      return;
    }
    if (serialCommandInvalid) continue;
    if (value < ' ' || value > '~' || serialCommandLength >= sizeof(serialCommand) - 1) {
      serialCommandInvalid = true;
    } else {
      serialCommand[serialCommandLength++] = static_cast<char>(toupper(value));
    }
  }

  if (!hasElapsed(now, lastSerialStatusMs, Timing::SERIAL_STATUS_MS)) return;
  lastSerialStatusMs = now;
  char range[16];
  if (distanceIsFresh(now)) snprintf(range, sizeof(range), "%.1f cm", distanceCm);
  else snprintf(range, sizeof(range), "no echo");
  const char *modeLabel = mode == Mode::SAFETY   ? "SAFETY"
                          : mode == Mode::MANUAL ? "MANUAL"
                                                 : "AUTO";
  const char *pirLabel = !hasElapsed(now, bootMs, Timing::PIR_WARM_UP_MS) ? "warm-up"
                         : pirMotion                                      ? "motion"
                                                                          : "clear";
  snprintf(serialOutput, sizeof(serialOutput),
           "%s | Gate: %s | Hold: %.1f s | Range: %s | PIR: %s%s\n", modeLabel, motionName(motion),
           holdOpenMs / 1000.0, range, pirLabel, safetyCountingDown ? " | Recovery countdown" : "");
}

/**
 * @brief Show the measured distance or a missing-echo indication.
 * @return Nothing.
 */
void drawDistance() {
  display.print(F("Range: "));
  if (distanceValid) {
    display.print(distanceCm, 1);
    display.println(F(" cm"));
  } else {
    display.println(F("no echo"));
  }
}

/**
 * @brief Refresh the OLED with delay tuning, gate status, or safety instructions.
 * @param now Current millisecond timestamp.
 * @return Nothing.
 */
void updateDisplay(uint32_t now) {
  if (!oledReady || !hasElapsed(now, lastOledRefreshMs, Timing::OLED_REFRESH_MS)) return;
  lastOledRefreshMs = now;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  if (mode == Mode::SAFETY) {
    if (safetyCountingDown) {
      const uint32_t seconds =
        (Timing::SAFETY_COUNTDOWN_MS - (now - safetyClearStartedMs) + 999U) / 1000U;
      display.println(F("Obstruction gone"));
      display.println(F("Gate is closing"));
      display.print(F("in "));
      display.print(seconds);
      display.println(F(" seconds"));
    } else {
      display.println(F("!! SAFETY BLOCK !!"));
      display.println(F("Gate: HALTED"));
      display.println(F("Waiting for clear"));
    }
    drawDistance();
  } else {
    display.println(mode == Mode::MANUAL ? F("MANUAL - SET DELAY") : F("AUTO"));
    display.print(F("Hold open: "));
    display.print(static_cast<float>(holdOpenMs) / 1000.0F, 1);
    display.println(F(" s"));
    if (mode == Mode::MANUAL) display.println(F("Turn dial: 3-15 s"));
    display.print(F("Gate: "));
    display.println(motionName(motion));
    if (!hasElapsed(now, bootMs, Timing::PIR_WARM_UP_MS)) {
      display.print(F("PIR warm-up: "));
      display.print((Timing::PIR_WARM_UP_MS - (now - bootMs) + 999U) / 1000U);
      display.println(F("s"));
    } else {
      display.println(pirMotion ? F("PIR: MOTION") : F("PIR: clear"));
    }
    drawDistance();
    display.println(mode == Mode::MANUAL ? F("BOOT: done") : F("BOOT: set delay"));
  }
  display.display();
}

/**
 * @brief Initialize the existing SPI or I2C OLED build option.
 * @return True when the display initializes successfully.
 */
bool initialiseDisplay() {
#if OLED_USE_SPI
  return display.begin(SSD1306_SWITCHCAPVCC);
#else
  Wire.begin(Pins::OLED_SDA, Pins::OLED_SCL);
  return display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
#endif
}

/**
 * @brief Initialize the pins, interrupts and display, starting with the gate open.
 * @return Nothing.
 */
void setup() {
  Serial.begin(115200);
  pinMode(Pins::PIR, INPUT);
  pinMode(Pins::ULTRASONIC_TRIGGER, OUTPUT);
  pinMode(Pins::ULTRASONIC_ECHO, INPUT);
  pinMode(Pins::BUZZER, OUTPUT);
  pinMode(Pins::BOOT_BUTTON, INPUT_PULLUP);
  digitalWrite(Pins::ULTRASONIC_TRIGGER, LOW);
  setBuzzerOutput(false);
  analogReadResolution(12);
  analogSetPinAttenuation(Pins::POTENTIOMETER, ADC_11db);
  initialiseServoPwm();
  writeServoAngle(Limits::OPEN_ANGLE);
  attachInterrupt(digitalPinToInterrupt(Pins::ULTRASONIC_ECHO), onUltrasonicEchoChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pins::BOOT_BUTTON), onBootButtonChange, CHANGE);
  oledReady = initialiseDisplay();
  bootMs = millis();
  lastSerialStatusMs = bootMs;
  holdStartedMs = bootMs;
  lastUltrasonicStartMs = bootMs - Timing::ULTRASONIC_INTERVAL_MS;
  buttonRawPressed = digitalRead(Pins::BOOT_BUTTON) == LOW;
  buttonStablePressed = buttonRawPressed;
}

/**
 * @brief Update sensors, safety, controls and outputs without blocking.
 * @return Nothing.
 */
void loop() {
  const uint32_t now = millis();
  updateUltrasonic(now, micros());
  updateSafetyLogic(now);
  updateBootButton(now);
  updatePotentiometer(now);
  pirMotion = hasElapsed(now, bootMs, Timing::PIR_WARM_UP_MS) && digitalRead(Pins::PIR) == HIGH;
  updateGate(now);
  updateServo(now);
  updateBuzzer(now);
  updateDisplay(now);
  updateSerial(now);
}
