#pragma once
// Motion core ported from Suszarnia_test_WROOM v2.1.
// WT32-ETH01: AS5600 SDA17 SCL5; L298N IN1=32 IN2=33 IN3=14 IN4=4.
// L298N ENA/ENB set HIGH in hardware. Second channel drives lift in 300 ms test pulses.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

constexpr int FLAP_A = 32, FLAP_B = 33;
constexpr int LIFT_A = 14, LIFT_B = 4;
constexpr uint32_t LIFT_PULSE_MS = 300;
constexpr uint32_t LIFT_PAUSE_MS = 200;
constexpr int ENCODER_SIGN = 1; // Set -1 if opening decreases encoder angle.
constexpr bool MOTOR_REVERSE = false; // Reverse flap motor direction here if needed.
constexpr float MAX_ANGLE = 360.0f; // Set to actual mechanical travel before testing.
constexpr float TOLERANCE = 0.1f;
constexpr uint32_t MOVE_TIMEOUT_MS = 60000;
constexpr uint32_t STALL_TIMEOUT_MS = 1500;

Preferences settings;
uint16_t motionFault = 0, motionFaultReason = 0, encoderFault = 0;
const char* motionFaultText = "";
const char* motionFaultHint = "";
float faultAngle = 0, faultTarget = 0;
int flapOutputPwm = 0, faultOutputPwm = 0;

const char* encoderFaultText() {
  switch (encoderFault) {
    case 1: return "AS5600 nie odpowiada prawidlowo po I2C.";
    case 2: return "AS5600 nie wykrywa magnesu.";
    case 3: return "Pole magnesu jest za slabe.";
    case 4: return "Pole magnesu jest za silne.";
    default: return "Odczyt AS5600 prawidlowy.";
  }
}

void clearMotionFault() {
  motionFault = motionFaultReason = 0;
  motionFaultText = motionFaultHint = "";
  faultOutputPwm = 0;
}


bool calibrated = false, sensorOK = false, moving = false;
bool manualJog = false;
int jogDirection = 0;
uint32_t jogAt = 0, jogStarted = 0, jogStopped = 0;
uint16_t rawAngle = 0, zeroAngle = 0;
float angle = 0, target = 0, initialError = 0, lastProgressAngle = 0;
// SPEED in deg/s; ACCEL in deg/s^2. 12 deg/s equals 2 rpm.
float maxSpeed = 12.0f, acceleration = 12.0f;
float startPosition = 0, goalPosition = 0, position = 0;
float measuredSpeed = 0, speedIntegral = 0;
float profileDistance = 0, peakSpeed = 0, rampTime = 0, cruiseTime = 0;
int motionSign = 1;
uint32_t previousEncoderTime = 0, lastControlTime = 0;
float previousAngle = 0;
bool trackingReady = false, settling = false, correcting = false, correctionPulse = false;
uint32_t phaseStarted = 0;
unsigned correctionCount = 0;
int correctionPwm = 95;
uint32_t correctionDuration = 20;
float correctionStartPosition = 0;
bool correctionAwaitingCheck = false;
uint32_t moveStarted = 0, progressTime = 0, nextSample = 0;
int liftDirection = 0;
uint32_t liftStarted = 0, liftStopped = 0;
char input[64];
size_t inputLength = 0;
bool inputOverflow = false;
uint32_t lastInputAt = 0, lastStatusAt = 0;

float wrap360(float value) {
  value = fmodf(value, 360.0f);
  return value < 0 ? value + 360.0f : value;
}

float angleDifference(float to, float from) {
  float difference = wrap360(to - from);
  return difference > 180.0f ? difference - 360.0f : difference;
}

void recordMotionFault(uint16_t category, uint16_t reason, const char* message, const char* hint) {
  motionFault = category;
  motionFaultReason = reason;
  motionFaultText = message;
  motionFaultHint = hint;
  faultAngle = angle;
  faultTarget = target;
  faultOutputPwm = flapOutputPwm;
}

void flapOff() {
  flapOutputPwm = 0;
  analogWrite(FLAP_A, 0);
  analogWrite(FLAP_B, 0);
}

void stopFlap() {
  if (manualJog) jogStopped = millis();
  manualJog = false;
  flapOff();
  moving = false;
  settling = correcting = correctionPulse = false;
}

void stopLift() {
  analogWrite(LIFT_A, 0);
  analogWrite(LIFT_B, 0);
  if (liftDirection != 0) liftStopped = millis();
  liftDirection = 0;
}

bool readBytes(uint8_t reg, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(0x36);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(uint8_t(0x36), length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool readEncoder() {
  uint8_t status, bytes[2];
  if (!readBytes(0x0B, &status, 1)) { encoderFault = 1; return false; }
  // MD must be set; ML and MH must both be clear.
  if (!(status & 0x20)) { encoderFault = 2; return false; }
  if (status & 0x10) { encoderFault = 3; return false; }
  if (status & 0x08) { encoderFault = 4; return false; }
  if (!readBytes(0x0C, bytes, 2)) { encoderFault = 1; return false; }
  encoderFault = 0;
  rawAngle = ((uint16_t(bytes[0]) << 8) | bytes[1]) & 0x0FFF;
  // Position is derived directly from the absolute reading on EVERY sample.
  angle = wrap360(ENCODER_SIGN * (int(rawAngle) - int(zeroAngle)) * (360.0f / 4096.0f));
  const uint32_t sampleTime = millis();
  if (!trackingReady) {
    position = angle;
    measuredSpeed = 0;
    trackingReady = true;
  } else {
    const float delta = angleDifference(angle, previousAngle);
    position += delta; // Local continuous coordinate for the current motion profile.
    const float dt = (sampleTime - previousEncoderTime) / 1000.0f;
    if (dt > 0) {
      const float blend = dt / (0.10f + dt);
      measuredSpeed += blend * (delta / dt - measuredSpeed);
    }
  }
  previousAngle = angle;
  previousEncoderTime = sampleTime;
  return true;
}

void failFlap(uint16_t reason, const char* message, const char* hint) {
  recordMotionFault(1, reason, message, hint);
  stopFlap();
  Serial.println(message);
}

void updateFlap(uint32_t now) {
  if (!moving) return;
  if (manualJog) {
    // Open-loop bench test: deliberately independent of encoder and saved zero.
    if (now - jogAt >= 500 || now - jogStarted >= 10000) stopFlap();
    return;
  }
  if (!sensorOK) {
    trackingReady = false;
    failFlap(encoderFault, encoderFaultText(), "Sprawdz SDA GPIO17, SCL GPIO5, zasilanie 3,3 V, mase i ustawienie magnesu.");
    return;
  }
  const uint32_t profileTimeout = uint32_t((2 * rampTime + cruiseTime) * 1000) + 15000;
  const uint32_t allowedTime = profileTimeout > MOVE_TIMEOUT_MS ? profileTimeout : MOVE_TIMEOUT_MS;
  if (now - moveStarted >= allowedTime) {
    failFlap(7, "Przekroczony czas dojazdu i korekt.", "Silnik nie osiagnal celu w wyznaczonym czasie. Sprawdz obciazenie, zasilanie oraz nastawy predkosci i przyspieszenia.");
    return;
  }
  const float error = goalPosition - position;
  if (settling) {
    if (now - phaseStarted < 200) return;
    settling = false;
    if (fabsf(error) <= TOLERANCE) {
      stopFlap();
      Serial.println("Dojazd zakonczony: blad <= 0.1 stopnia.");
      return;
    }
    if (correctionAwaitingCheck) {
      if (fabsf(position - correctionStartPosition) < 0.04f) {
        correctionPwm = min(correctionPwm + 20, 255);
      } else {
        correctionPwm = 95;
      }
      correctionAwaitingCheck = false;
    }
    correcting = true;
  }
  if (correcting) {
    if (correctionPulse) {
      if (now - phaseStarted >= correctionDuration) {
        flapOff();
        correctionPulse = false;
        settling = true;
        phaseStarted = now;
      }
      return;
    }
    if (++correctionCount > 80) {
      failFlap(8, "Nie uzyskano tolerancji 0,1 stopnia po 80 korektach.", "Sprawdz luzy przekladni i magnesu, obciazenie oraz minimalna moc potrzebna do ruszenia silnika.");
      return;
    }
    // Longer pulses for larger errors; increase duty only when no motion was detected.
    correctionDuration = fabsf(error) > 1.0f ? 100 : (fabsf(error) > 0.3f ? 30 : 10);
    correctionStartPosition = position;
    correctionAwaitingCheck = true;
    const bool forward = (error > 0) != MOTOR_REVERSE;
    analogWrite(forward ? FLAP_B : FLAP_A, 0);
    analogWrite(forward ? FLAP_A : FLAP_B, correctionPwm);
    flapOutputPwm = correctionPwm;
    correctionPulse = true;
    phaseStarted = now;
    return;
  }
  if (fabsf(error) <= TOLERANCE || error * initialError <= 0) {
    flapOff();
    settling = true;
    phaseStarted = now;
    return;
  }
  if (fabsf(error) > fabsf(initialError) + 2.0f) {
    failFlap(5, "Ruch w przeciwnym kierunku: klapka oddala sie od celu.", "Sprawdz kierunek obrotu. Po wylaczeniu zasilania silnika mozna zamienic OUT1 z OUT2. Sprawdz tez ENCODER_SIGN w motion.h.");
    return;
  }
  if (fabsf(position - lastProgressAngle) >= 0.3f) {
    lastProgressAngle = position;
    progressTime = now;
  }
  if (now - progressTime >= STALL_TIMEOUT_MS) {
    failFlap(6, "Brak ruchu odczytanego z enkodera przez 1,5 sekundy.", "Sprawdz zasilanie silnika i logiki 5 V, zworke ENA, wspolna mase, IN1 GPIO32, IN2 GPIO33 oraz polaczenie magnesu z osia. To mozliwe przyczyny.");
    return;
  }
  const float elapsed = (now - moveStarted) / 1000.0f;
  float referenceDistance, referenceSpeed;
  const float rampDistance = 0.5f * acceleration * rampTime * rampTime;
  if (elapsed < rampTime) {
    referenceSpeed = acceleration * elapsed;
    referenceDistance = 0.5f * acceleration * elapsed * elapsed;
  } else if (elapsed < rampTime + cruiseTime) {
    referenceSpeed = peakSpeed;
    referenceDistance = rampDistance + peakSpeed * (elapsed - rampTime);
  } else if (elapsed < 2 * rampTime + cruiseTime) {
    const float remainingTime = 2 * rampTime + cruiseTime - elapsed;
    referenceSpeed = acceleration * remainingTime;
    referenceDistance = profileDistance - 0.5f * acceleration * remainingTime * remainingTime;
  } else {
    // Check settled position and use bounded corrections if the motor lagged.
    flapOff();
    settling = true;
    phaseStarted = now;
    return;
  }
  const float referencePosition = startPosition + motionSign * referenceDistance;
  const float alongTrackError = motionSign * (referencePosition - position);
  const float requestedSpeed = constrain(referenceSpeed + 2.0f * alongTrackError, 0.0f, maxSpeed);
  const float speedError = requestedSpeed - motionSign * measuredSpeed;
  const float dt = (now - lastControlTime) / 1000.0f;
  lastControlTime = now;
  // Feed-forward plus velocity feedback. Gains require verification on the motor.
  const float feedForward = 255.0f * requestedSpeed / 12.0f;
  const float candidateIntegral = constrain(speedIntegral + 8.0f * speedError * dt, -60.0f, 60.0f);
  const float candidateOutput = feedForward + 6.0f * speedError + candidateIntegral;
  if ((candidateOutput >= 0 && candidateOutput <= 255) ||
      (candidateOutput > 255 && speedError < 0) ||
      (candidateOutput < 0 && speedError > 0)) speedIntegral = candidateIntegral;
  const int pwm = requestedSpeed < 0.05f ? 0 :
                  int(constrain(feedForward + 6.0f * speedError + speedIntegral, 0.0f, 255.0f));
  const bool forward = (motionSign > 0) != MOTOR_REVERSE;
  analogWrite(forward ? FLAP_B : FLAP_A, 0);
  analogWrite(forward ? FLAP_A : FLAP_B, pwm);
  flapOutputPwm = pwm;
}

void printStatus() {
  Serial.printf("AS5600=%s raw=%u zero=%s kat=%.2f cel=%.2f ruch=%d rynna=%d v=%.2f vmax=%.2f accel=%.2f\n",
                sensorOK ? "OK" : "BLAD", unsigned(rawAngle),
                calibrated ? "TAK" : "NIE", angle, target, moving, liftDirection, measuredSpeed, maxSpeed, acceleration);
}

void motionCommand(char* line) {
  // Accept either decimal separator in commands typed in the serial monitor.
  for (char* c = line; *c; ++c) if (*c == ',') *c = '.';
  const bool gotoCommand = strncmp(line, "GOTO ", 5) == 0;
  const bool numericCommand = isdigit(static_cast<unsigned char>(line[0])) ||
                              line[0] == '+' || line[0] == '-' || line[0] == '.';
  if (strcmp(line, "STOP") == 0) {
    stopFlap();
    stopLift();
    Serial.println("STOP wszystkich napedow.");
  } else if (strcmp(line, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(line, "ZERO") == 0) {
    stopFlap();
    stopLift();
    sensorOK = readEncoder();
    if (!sensorOK) {
      Serial.println("Brak poprawnego odczytu AS5600.");
      return;
    }
    if (settings.putUInt("zero", rawAngle) != sizeof(uint32_t)) {
      recordMotionFault(3, 12, "Nie udalo sie zapisac pozycji zerowej w pamieci.", "Poprzednie odniesienie zostalo zachowane. Sprawdz zasilanie i pamiec NVS.");
      Serial.println("BLAD zapisu zera. Zachowano poprzednie odniesienie.");
      return;
    }
    zeroAngle = rawAngle;
    calibrated = true;
    clearMotionFault();
    angle = target = 0;
    position = 0;
    trackingReady = false;
    Serial.println(calibrated ? "Zero zapisane." : "BLAD zapisu zera.");
  } else if (gotoCommand || numericCommand) {
    if (moving) {
      Serial.println("Najpierw STOP przed nowym GOTO.");
      return;
    }
    char* end;
    const char* number = gotoCommand ? line + 5 : line;
    const float value = strtof(number, &end);
    sensorOK = readEncoder();
    if (end == number || *end != '\0' || !isfinite(value) ||
        value < 0 || value > MAX_ANGLE || !calibrated || !sensorOK) {
      Serial.println("GOTO odrzucone: sprawdz kat, ZERO i AS5600.");
      return;
    }
    clearMotionFault();
    target = wrap360(value);
    startPosition = position;
    initialError = angleDifference(target, angle);
    goalPosition = position + initialError;
    motionSign = initialError >= 0 ? 1 : -1;
    profileDistance = fabsf(initialError);
    peakSpeed = fminf(maxSpeed, sqrtf(profileDistance * acceleration));
    rampTime = peakSpeed / acceleration;
    cruiseTime = peakSpeed > 0 ? fmaxf(0, (profileDistance - peakSpeed * rampTime) / peakSpeed) : 0;
    speedIntegral = 0;
    settling = correcting = correctionPulse = false;
    correctionCount = 0;
    correctionPwm = 95;
    correctionAwaitingCheck = false;
    lastProgressAngle = position;
    moveStarted = progressTime = lastControlTime = millis();
    moving = true;
    Serial.println("Start klapki.");
  } else if (strncmp(line, "SPEED ", 6) == 0 || strncmp(line, "ACCEL ", 6) == 0) {
    char* end;
    const float value = strtof(line + 6, &end);
    const bool speedSetting = strncmp(line, "SPEED ", 6) == 0;
    const float maximum = speedSetting ? 12.0f : 120.0f;
    if (moving || end == line + 6 || *end != '\0' || !isfinite(value) || value < 1 || value > maximum) {
      Serial.println("Odrzucono: wymagany postoj; SPEED 1..12, ACCEL 1..120.");
      return;
    }
    if (speedSetting) maxSpeed = value;
    else acceleration = value;
    printStatus();
  } else {
    Serial.println("Komendy: STATUS, ZERO, 10 lub GOTO 10, STOP, SPEED 12, ACCEL 12");
  }
}

