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
constexpr float MAX_ANGLE = 10.0f; // Set to actual mechanical travel before testing.
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
bool scaleZeroSaved = false, scaleCaptureActive = false;
float scaleSpan = 0;
float scaleTarget = 0;
const char* scaleMessage = "Zapisz pozycje 0, podjedz recznie do drugiego konca i zapisz 10.";
struct ScaleCalibration { uint32_t magic; uint32_t zero; float span; };
constexpr uint32_t SCALE_MAGIC = 0x53434C34;
bool manualJog = false, calibratingMotor = false;
float msPerDegree[2] = {83.333f, 83.333f};
float stopLead[2] = {0, 0};
float calStart = 0, calOffPosition = 0, calTravel = 0, calCoast = 0;
int calDirection = 1;
unsigned calPulses = 0;
bool calPowered = false;
uint32_t calPhaseAt = 0;
const char* calibrationText = "Kalibracja nieuruchomiona.";
void saveMotorCalibration() {
  float values[4] = {msPerDegree[0], msPerDegree[1], stopLead[0], stopLead[1]};
  if (settings.putBytes("motorcal", values, sizeof(values)) != sizeof(values))
    calibrationText = "Blad zapisu kalibracji; wyniki aktywne do restartu.";
  else calibrationText = "Kalibracja kierunku zakonczona i zapisana.";
}
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

// Absolute single-turn coordinate along the calibrated arc, including arcs >180 degrees.
float scalePositionDegrees(float encoderAngle) {
  const float sign = scaleSpan >= 0 ? 1.0f : -1.0f;
  const float length = fabsf(scaleSpan);
  float directed = wrap360(sign * encoderAngle);
  // Choose the representation nearest the calibrated interval; never a shortest-path target.
  if (directed > length && directed - length > 360.0f - directed) directed -= 360.0f;
  return directed * sign;
}

float scaleReading() {
  return calibrated ? position * 10.0f / scaleSpan : 0;
}

float faultScaleReading(float degrees) {
  return calibrated ? degrees * 10.0f / scaleSpan : 0;
}

void loadScaleCalibration() {
  ScaleCalibration saved{};
  calibrated = scaleZeroSaved = scaleCaptureActive = false;
  scaleSpan = 0;
  if (settings.getBytes("scale4", &saved, sizeof(saved)) != sizeof(saved) ||
      saved.magic != SCALE_MAGIC || saved.zero > 4095 || !isfinite(saved.span)) return;
  if (saved.span != 0 && (fabsf(saved.span) < 1.0f || fabsf(saved.span) > 350.0f)) return;
  zeroAngle = uint16_t(saved.zero);
  scaleZeroSaved = true;
  scaleSpan = saved.span;
  calibrated = scaleSpan != 0;
  scaleMessage = calibrated ? "Wczytano skale 0-10." :
    "Zapisano tylko 0. Po restarcie zapisz 0 ponownie przed dojazdem do 10.";
}

void recordMotionFault(uint16_t category, uint16_t reason, const char* message, const char* hint) {
  motionFault = category;
  motionFaultReason = reason;
  motionFaultText = message;
  motionFaultHint = hint;
  faultAngle = position;
  faultTarget = goalPosition;
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
  if (calibratingMotor) calibrationText = "Kalibracja przerwana.";
  calibratingMotor = false;
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

bool sampleEncoder() {
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
  if (calibrated) position = scalePositionDegrees(angle);
  previousAngle = angle;
  previousEncoderTime = sampleTime;
  return true;
}

bool readEncoder() {
  const bool ok = sampleEncoder();
  if (!ok) {
    trackingReady = false;
    if (scaleCaptureActive) {
      scaleCaptureActive = false;
      scaleMessage = "Przerwano pomiar zakresu: blad enkodera. Zapisz 0 ponownie.";
    }
  }
  return ok;
}

bool saveScaleZero() {
  if (moving || !sensorOK) return false;
  const ScaleCalibration saved{SCALE_MAGIC, rawAngle, 0};
  if (settings.putBytes("scale4", &saved, sizeof(saved)) != sizeof(saved)) {
    recordMotionFault(3, 12, "Nie zapisano punktu 0.", "Poprzednia kalibracja pozostaje aktywna.");
    return false;
  }
  zeroAngle = rawAngle;
  scaleZeroSaved = scaleCaptureActive = true;
  calibrated = false;
  scaleSpan = 0;
  scaleTarget = 0;
  angle = target = position = goalPosition = 0;
  trackingReady = false;
  clearMotionFault();
  scaleMessage = "Punkt 0 zapisany. Podjedz recznie do punktu 10 i zatrzymaj silnik.";
  return true;
}

bool saveScaleTen() {
  if (moving || !sensorOK || !scaleCaptureActive || !scaleZeroSaved) {
    scaleMessage = "Najpierw zapisz 0, podjedz recznie i zatrzymaj silnik.";
    return false;
  }
  if (!isfinite(position) || fabsf(position) < 1.0f || fabsf(position) > 350.0f) {
    scaleMessage = "Zakres zbyt maly lub obejmuje prawie pelny obrot. Zapisz 0 i 10 ponownie.";
    return false;
  }
  const ScaleCalibration saved{SCALE_MAGIC, zeroAngle, position};
  if (settings.putBytes("scale4", &saved, sizeof(saved)) != sizeof(saved)) {
    recordMotionFault(3, 12, "Nie zapisano punktu 10.", "Powtorz zapis punktu 10.");
    return false;
  }
  scaleSpan = position;
  calibrated = true;
  scaleCaptureActive = false;
  scaleTarget = 10;
  target = angle;
  goalPosition = position;
  clearMotionFault();
  scaleMessage = "Skala 0-10 zapisana. Mozesz zadawac pozycje co 0,1.";
  return true;
}

void failFlap(uint16_t reason, const char* message, const char* hint) {
  recordMotionFault(1, reason, message, hint);
  stopFlap();
  Serial.println(message);
}

void driveFull(int direction) {
  const bool forward = (direction > 0) != MOTOR_REVERSE;
  analogWrite(forward ? FLAP_B : FLAP_A, 0);
  analogWrite(forward ? FLAP_A : FLAP_B, 255);
  flapOutputPwm = 255;
}

void updateFlap(uint32_t now) {
  if (!moving) return;
  if (manualJog) {
    if (now - jogAt >= 500 || now - jogStarted >= 10000) stopFlap();
    return;
  }
  if (!sensorOK) {
    trackingReady = false;
    failFlap(encoderFault, encoderFaultText(), "Sprawdz enkoder i magnes.");
    return;
  }
  if (now - moveStarted >= MOVE_TIMEOUT_MS) {
    failFlap(7, "Przekroczono czas ruchu.", "Sprawdz naped i odczyt kata.");
    return;
  }
  if (calibratingMotor) {
    const float travel = calDirection * (position - calStart);
    if (travel < -0.5f || travel > 8.0f) {
      failFlap(5, "Kalibracja: nieprawidlowy kierunek lub zbyt duzy ruch.",
               "Sprawdz kierunek silnika i enkodera. Kalibracja wymaga wolnego miejsca.");
      return;
    }
    if (calPowered) {
      if (now - calPhaseAt >= 150) {
        calOffPosition = position;
        flapOff();
        calPowered = false;
        calPhaseAt = now;
      }
    } else if (now - calPhaseAt >= 700) {
      calCoast += fmaxf(0, calDirection * (position - calOffPosition));
      if (++calPulses >= 3) {
        if (travel < 0.3f) {
          failFlap(6, "Kalibracja: brak wystarczajacego ruchu.",
                   "Sprawdz zasilanie, L298N i polaczenie magnesu.");
          return;
        }
        const int index = calDirection > 0 ? 1 : 0;
        msPerDegree[index] = constrain(450.0f / travel, 5.0f, 5000.0f);
        stopLead[index] = constrain(calCoast / 3.0f, 0.0f, 3.0f);
        calibratingMotor = false;
        stopFlap();
        saveMotorCalibration();
        return;
      }
      driveFull(calDirection);
      calPowered = true;
      calPhaseAt = now;
    }
    return;
  }
  const float error = goalPosition - position;
  if (calibrated && ((position < fminf(0, scaleSpan) - TOLERANCE && error < 0) ||
                     (position > fmaxf(0, scaleSpan) + TOLERANCE && error > 0))) {
    failFlap(15, "Ruch poza skalibrowanym zakresem.", "Sprawdz zapis pozycji 0 i 10.");
    return;
  }
  if (settling) {
    if (now - phaseStarted < 700) return;
    settling = false;
    if (fabsf(error) <= TOLERANCE) { stopFlap(); return; }
    correcting = true;
  }
  if (correcting) {
    if (correctionPulse) {
      if (now - phaseStarted >= correctionDuration || fabsf(error) <= TOLERANCE ||
          ((error > 0 ? 1 : -1) != motionSign)) {
        flapOff();
        correctionPulse = false;
        settling = true;
        phaseStarted = now;
      }
      return;
    }
    if (++correctionCount > 80) {
      failFlap(8, "Nie osiagnieto tolerancji 0,1 stopnia.",
               "Pelna moc i luzy napedu moga ograniczac dokladnosc. Wykonaj kalibracje obu kierunkow.");
      return;
    }
    const int direction = error > 0 ? 1 : -1;
    correctionDuration = uint32_t(constrain(fabsf(error) * msPerDegree[direction > 0 ? 1 : 0] * 0.5f, 5.0f, 100.0f));
    motionSign = direction;
    driveFull(direction);
    correctionPulse = true;
    phaseStarted = now;
    return;
  }
  const int index = motionSign > 0 ? 1 : 0;
  if (fabsf(error) <= fmaxf(TOLERANCE, stopLead[index]) || error * initialError <= 0) {
    flapOff();
    settling = true;
    phaseStarted = now;
    return;
  }
  if (fabsf(error) > fabsf(initialError) + 2.0f) {
    failFlap(5, "Silnik oddala sie od celu.", "Sprawdz kierunek silnika i enkodera.");
    return;
  }
  if (fabsf(position - lastProgressAngle) >= 0.3f) {
    lastProgressAngle = position;
    progressTime = now;
  }
  if (now - progressTime >= STALL_TIMEOUT_MS) {
    failFlap(6, "Brak ruchu mimo pelnej mocy.", "Sprawdz zasilanie, ENA, IN1/IN2 i mase L298N.");
    return;
  }
  driveFull(motionSign);
}

void printStatus() {
  Serial.printf("AS5600=%s skala=%s pozycja=%.2f cel=%.1f ruch=%d rynna=%d\n",
    sensorOK ? "OK" : "BLAD", calibrated ? "0-10" : "wymaga kalibracji",
    scaleReading(), scaleTarget, moving, liftDirection);
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
  } else if (strcmp(line, "ZERO") == 0 || strcmp(line, "TEN") == 0) {
    if (moving) { Serial.println("Najpierw STOP."); return; }
    sensorOK = readEncoder();
    if (!sensorOK) { Serial.println(encoderFaultText()); return; }
    const bool ok = strcmp(line, "ZERO") == 0 ? saveScaleZero() : saveScaleTen();
    Serial.println(ok ? scaleMessage : (motionFault == 3 ? motionFaultText : scaleMessage));
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
        value < 0 || value > 10 || fabsf(value * 10 - roundf(value * 10)) > 0.001f || !calibrated || !sensorOK) {
      Serial.println("Pozycja odrzucona: skala 0-10 co 0,1; wymagane zapisane 0 i 10 oraz enkoder.");
      return;
    }
    clearMotionFault();
    scaleTarget = roundf(value * 10) / 10;
    target = wrap360(scaleSpan * scaleTarget / 10);
    startPosition = position;
    goalPosition = scaleSpan * scaleTarget / 10;
    initialError = goalPosition - position;
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
    Serial.println("Komendy: STATUS, ZERO (punkt 0), TEN (punkt 10), pozycja 0..10 co 0,1, STOP");
  }
}

