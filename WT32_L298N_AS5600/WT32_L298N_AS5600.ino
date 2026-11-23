// WT32-ETH01, L298N, AS5600. Arduino-ESP32 3.x + ArduinoJson 7.x.
#include <Arduino.h>
#include <Network.h>
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "motion.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif
#ifndef DRYER_BOOTSTRAP_WIFI
#define DRYER_BOOTSTRAP_WIFI ""
#endif
#ifndef DRYER_BOOTSTRAP_WIFI_PASSWORD
#define DRYER_BOOTSTRAP_WIFI_PASSWORD ""
#endif
#ifndef DRYER_BOOTSTRAP_OTA_PASSWORD
#define DRYER_BOOTSTRAP_OTA_PASSWORD ""
#endif
#ifndef DRYER_WIFI_NETWORKS
#define DRYER_WIFI_NETWORKS "[]"
#endif

constexpr uint16_t FIRMWARE_VERSION = 300;
constexpr unsigned CLIENT_COUNT = 3;
constexpr int NO_OWNER = -1, WEB_OWNER = 100, USB_OWNER = 101;
constexpr uint32_t LEASE_MS = 1500;
const IPAddress ethernetIP(192,168,1,13), gatewayIP(192,168,1,1), subnet(255,255,255,0);
SemaphoreHandle_t motionMutex;
bool driverReady = false, maintenance = false;
bool ethOnline = false, wifiOnline = false;
uint16_t liftCommand = 0, liftPwm = 0, liftActual = 0, previousLift = 0;
uint32_t liftAt = 0, liftOffAt = 0, flapAt = 0;
int liftOwner = NO_OWNER, flapOwner = NO_OWNER;
bool liftArmed = false, liftUsesEthernet = false, flapUsesEthernet = false;
uint16_t stagedTarget = 0, heartbeatSequence = 0;
NetworkServer modbusServer(502);
struct ModbusSession {
  NetworkClient client;
  uint8_t buffer[260] = {};
  size_t used = 0;
  uint32_t frameAt = 0;
  bool ethernet = false;
};
ModbusSession sessions[CLIENT_COUNT];
WebServer web(80);
String controlToken;
bool serverStarted = false, otaStarted = false, rebootPending = false;
bool uploadAccepted = false, uploadFailed = false, uploadSucceeded = false;
uint32_t rebootAt = 0, otaLastByte = 0;
struct WifiEntry { String ssid, password; };
WifiEntry networks[6];
unsigned networkCount = 0, networkIndex = 0;
bool wifiAttempted = false, wifiReported = false;
uint32_t wifiAttemptAt = 0;

void lockMotion() { xSemaphoreTake(motionMutex, portMAX_DELAY); }
void unlockMotion() { xSemaphoreGive(motionMutex); }

void haltLift(bool disarm) {
  if (liftDirection) {
    previousLift = liftDirection;
    liftOffAt = millis();
  }
  stopLift();
  liftActual = liftCommand = liftPwm = 0;
  if (disarm) {
    liftArmed = false;
    liftOwner = NO_OWNER;
  }
}

void haltAll() {
  stopFlap();
  flapOwner = NO_OWNER;
  haltLift(true);
}

bool ownerOnline(bool ethernet) { return ethernet ? ethOnline : wifiOnline; }

// All I2C and motor state is owned by this task or accessed under motionMutex.
// It continues monitoring motor leases while synchronous HTTP handles a slow client.
void controlTask(void*) {
  for (;;) {
    lockMotion();
    const uint32_t now = millis();
    sensorOK = readEncoder();
    if (maintenance || !driverReady) {
      haltAll();
    } else {
      if (moving && flapOwner != USB_OWNER &&
          (flapOwner == NO_OWNER || !ownerOnline(flapUsesEthernet) || now - flapAt > LEASE_MS)) {
        stopFlap();
        flapOwner = NO_OWNER;
        motionFault = 2;
      }
      updateFlap(now);
      if (!moving) flapOwner = NO_OWNER;
      if (liftOwner != NO_OWNER &&
          (!ownerOnline(liftUsesEthernet) || now - liftAt > LEASE_MS)) haltLift(true);
      if (liftArmed && liftCommand && liftPwm) {
        if (previousLift && liftCommand != previousLift && now - liftOffAt < 700) {
          stopLift();
          liftActual = 0;
        } else {
          // The existing dryer receiver already ramps the requested PWM over 2 s.
          const bool up = liftCommand == 1;
          analogWrite(up ? LIFT_B : LIFT_A, 0);
          analogWrite(up ? LIFT_A : LIFT_B, uint32_t(liftPwm) * 255 / 1000);
          liftDirection = liftCommand;
          liftActual = liftPwm;
        }
      } else {
        if (liftDirection) {
          previousLift = liftDirection;
          liftOffAt = now;
        }
        stopLift();
        liftActual = 0;
      }
    }
    unlockMotion();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void closeSession(unsigned index) {
  sessions[index].client.stop();
  sessions[index].used = 0;
  lockMotion();
  if (liftOwner == int(index)) haltLift(true);
  if (flapOwner == int(index)) {
    stopFlap();
    flapOwner = NO_OWNER;
    motionFault = 2;
  }
  unlockMotion();
}

uint16_t get16(const uint8_t* data) { return uint16_t(data[0]) << 8 | data[1]; }
void put16(uint8_t* data, uint16_t value) { data[0] = value >> 8; data[1] = value; }

uint16_t registerValue(uint16_t reg) {
  switch (reg) {
    case 0: return liftCommand;
    case 1: return liftPwm;
    case 2: return liftActual;
    case 3: return liftDirection;
    case 4: return liftArmed;
    case 5: return FIRMWARE_VERSION;
    case 6: return stagedTarget;
    case 7: return 0;
    case 8: return uint16_t(lroundf(angle * 10)) % 3600;
    case 9: return (driverReady ? 1 : 0) | (sensorOK ? 2 : 0) | (calibrated ? 4 : 0) |
                   (moving ? 8 : 0) | ((!moving && calibrated && sensorOK &&
                    fabsf(angleDifference(target, angle)) <= TOLERANCE) ? 16 : 0) |
                   (motionFault ? 32 : 0) | (maintenance ? 64 : 0);
    case 10: return lroundf(maxSpeed * 10);
    case 11: return lroundf(acceleration * 10);
    case 12: return motionFault;
    case 13: return heartbeatSequence;
    case 14: return 0;
    case 15: return rawAngle;
  }
  return 0;
}

bool saveTuning() {
  const float tuning[2] = {maxSpeed, acceleration};
  return settings.putBytes("tuning", tuning, sizeof(tuning)) == sizeof(tuning);
}

bool startFlap(float destination, int owner, bool ethernet) {
  if (!driverReady || maintenance || moving || !calibrated || !sensorOK) return false;
  char command[32];
  snprintf(command, sizeof(command), "GOTO %.2f", destination);
  motionCommand(command);
  if (!moving) return false;
  flapOwner = owner;
  flapUsesEthernet = ethernet;
  flapAt = millis();
  return true;
}

// Validate the entire write before applying it. FC16 offset 0/count 2 matches ChuteLift.
uint8_t applyRegisters(unsigned index, uint16_t address, uint16_t count, const uint16_t* values) {
  uint16_t direction = liftCommand, pwm = liftPwm, destination = stagedTarget;
  uint16_t speed = lroundf(maxSpeed * 10), accel = lroundf(acceleration * 10);
  uint16_t command = 0, globalStop = 0, heartbeat = heartbeatSequence;
  bool directionWritten = false, pwmWritten = false, heartbeatWritten = false;
  for (uint16_t i = 0; i < count; ++i) {
    const uint16_t value = values[i];
    switch (address + i) {
      case 0: if (value > 2) return 3; direction = value; directionWritten = true; break;
      case 1: if (value > 1000) return 3; pwm = value; pwmWritten = true; break;
      case 6: if (value > 3600) return 3; destination = value; break;
      case 7: if (value > 3) return 3; command = value; break;
      case 10: if (value < 10 || value > 120) return 3; speed = value; break;
      case 11: if (value < 10 || value > 1200) return 3; accel = value; break;
      case 13: heartbeat = value; heartbeatWritten = true; break;
      case 14: if (value > 1) return 3; globalStop = value; break;
      default: return 2;
    }
  }
  if (globalStop) { haltAll(); return 0; }
  if (maintenance || !driverReady) return 6;
  if (directionWritten && direction && (!liftArmed || liftOwner != int(index))) return 3;
  if (pwmWritten && !directionWritten && liftCommand && liftOwner != int(index)) return 6;
  if ((speed != lroundf(maxSpeed * 10) || accel != lroundf(acceleration * 10)) && moving) return 6;
  if (command == 1 && (moving || !sensorOK || !calibrated)) return 6;
  if (command == 2 && (moving || !sensorOK || liftCommand || (directionWritten && direction))) return 6;
  if (command == 3 && moving) return 6;
  const float oldSpeed = maxSpeed, oldAccel = acceleration;
  maxSpeed = speed / 10.0f;
  acceleration = accel / 10.0f;
  if (command == 1 && !startFlap(destination / 10.0f, index, sessions[index].ethernet)) {
    maxSpeed = oldSpeed;
    acceleration = oldAccel;
    return 6;
  }
  if (command == 2) {
    char zero[] = "ZERO";
    motionCommand(zero);
    if (!calibrated || !sensorOK || motionFault == 3) { maxSpeed = oldSpeed; acceleration = oldAccel; return 4; }
  }
  if (command == 3 && !saveTuning()) { maxSpeed = oldSpeed; acceleration = oldAccel; return 4; }
  // A write of 0 to register 7 is an explicit flap STOP.
  if (address <= 7 && uint32_t(address) + count > 7 && command == 0) {
    stopFlap();
    flapOwner = NO_OWNER;
    motionFault = 0;
  }
  if (directionWritten) {
    if (direction == 0) {
      haltLift(true);
      liftArmed = true;
      liftOwner = index;
      liftUsesEthernet = sessions[index].ethernet;
    }
    if (direction && liftDirection && direction != liftDirection) haltLift(false);
    liftCommand = direction;
    liftAt = millis();
  }
  liftPwm = pwm;
  if (directionWritten && !direction) liftPwm = 0;
  stagedTarget = destination;
  heartbeatSequence = heartbeat;
  if (heartbeatWritten && flapOwner == int(index)) flapAt = millis();
  return 0;
}

void sendReply(unsigned index, const uint8_t* payload, size_t length) {
  uint8_t reply[260];
  memcpy(reply, sessions[index].buffer, 7);
  put16(reply + 4, length + 1);
  memcpy(reply + 7, payload, length);
  if (sessions[index].client.write(reply, length + 7) != length + 7) closeSession(index);
}

void replyError(unsigned index, uint8_t function, uint8_t error) {
  const uint8_t payload[] = {uint8_t(function | 0x80), error};
  sendReply(index, payload, sizeof(payload));
}

void handleRequest(unsigned index, size_t total) {
  const uint8_t* frame = sessions[index].buffer;
  if (frame[6] != 1) return;
  const uint8_t function = frame[7];
  const size_t length = total - 7;
  if (function == 3 || function == 4) {
    if (length != 5) { replyError(index, function, 3); return; }
    const uint16_t address = get16(frame + 8), count = get16(frame + 10);
    if (!count || count > 125) { replyError(index, function, 3); return; }
    if (uint32_t(address) + count > 16) { replyError(index, function, 2); return; }
    uint8_t payload[34] = {function, uint8_t(count * 2)};
    lockMotion();
    for (unsigned i = 0; i < count; ++i) put16(payload + 2 + i * 2, registerValue(address + i));
    unlockMotion();
    sendReply(index, payload, 2 + count * 2);
    return;
  }
  if (function != 6 && function != 16) { replyError(index, function, 1); return; }
  if ((function == 6 && length != 5) || (function == 16 && length < 6)) {
    replyError(index, function, 3); return;
  }
  const uint16_t address = get16(frame + 8), count = function == 6 ? 1 : get16(frame + 10);
  if (!count || count > 16 || (function == 16 &&
      (frame[12] != count * 2 || length != size_t(6 + count * 2)))) {
    replyError(index, function, 3); return;
  }
  if (uint32_t(address) + count > 16) { replyError(index, function, 2); return; }
  uint16_t values[16];
  for (unsigned i = 0; i < count; ++i) values[i] = get16(frame + (function == 6 ? 10 : 13 + i * 2));
  lockMotion();
  const uint8_t error = applyRegisters(index, address, count, values);
  unlockMotion();
  if (error) { replyError(index, function, error); return; }
  uint8_t payload[5] = {function};
  put16(payload + 1, address);
  put16(payload + 3, function == 6 ? values[0] : count);
  sendReply(index, payload, sizeof(payload));
}

void serviceModbus() {
  if (!serverStarted) return;
  for (unsigned i = 0; i < CLIENT_COUNT; ++i) {
    if (sessions[i].client && !sessions[i].client.connected()) closeSession(i);
  }
  NetworkClient incoming = modbusServer.accept();
  if (incoming) {
    bool accepted = false;
    for (unsigned i = 0; i < CLIENT_COUNT; ++i) {
      if (!sessions[i].client || !sessions[i].client.connected()) {
        closeSession(i);
        sessions[i].client = incoming;
        sessions[i].client.setNoDelay(true);
        sessions[i].ethernet = incoming.localIP() == ethernetIP;
        accepted = true;
        break;
      }
    }
    if (!accepted) incoming.stop();
  }
  for (unsigned i = 0; i < CLIENT_COUNT; ++i) {
    ModbusSession& session = sessions[i];
    for (unsigned budget = 0; budget < 260 && session.client.available(); ++budget) {
      if (session.used == 0) session.frameAt = millis();
      const int byte = session.client.read();
      if (byte < 0) break;
      session.buffer[session.used++] = byte;
      if (session.used >= 6) {
        const uint16_t length = get16(session.buffer + 4);
        if (get16(session.buffer + 2) != 0 || length < 2 || length > 254) { closeSession(i); break; }
        if (session.used == size_t(length + 6)) {
          handleRequest(i, session.used);
          session.used = 0;
        }
      }
      if (session.used >= sizeof(session.buffer)) { closeSession(i); break; }
    }
    if (session.used && millis() - session.frameAt > 500) closeSession(i);
  }
}

bool passwordConfigured() {
  return strlen(DRYER_BOOTSTRAP_OTA_PASSWORD) > 0 && strcmp(DRYER_BOOTSTRAP_OTA_PASSWORD, "CHANGE_ME") != 0;
}

bool authenticated() { return passwordConfigured() && web.authenticate("admin", DRYER_BOOTSTRAP_OTA_PASSWORD); }

bool requireLogin() {
  if (!passwordConfigured()) {
    web.send(503, "text/plain", "Ustaw haslo DRYER_BOOTSTRAP_OTA_PASSWORD w secrets.h.");
    return false;
  }
  if (!authenticated()) { web.requestAuthentication(); return false; }
  return true;
}

bool validToken() { return web.hasArg("token") && web.arg("token") == controlToken; }

void beginMaintenance() {
  lockMotion();
  maintenance = true;
  haltAll();
  unlockMotion();
  for (unsigned i = 0; i < CLIENT_COUNT; ++i) closeSession(i);
  otaLastByte = millis();
}

void endFailedMaintenance() {
  lockMotion();
  haltAll();
  maintenance = false;
  unlockMotion();
}

const char CONTROL_PAGE[] PROGMEM = R"html(<!doctype html><html lang="pl"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Klapka suszarni</title>
<style>body{font:17px system-ui;margin:24px auto;padding:0 16px;max-width:600px;background:#f5f6f4;color:#183426}section{background:white;padding:20px;border-radius:12px;margin:16px 0}label{display:block;margin:14px 0}input,button{font:inherit;padding:10px;box-sizing:border-box;max-width:100%}input{width:150px}button{cursor:pointer;margin:4px;background:#1c6042;color:white;border:0;border-radius:6px}#stop{background:#ad302a}#state{white-space:pre-line}#message{min-height:28px}a{color:#1c6042}</style>
<h1>Klapka suszarni</h1><section><div id="state">Laczenie...</div></section>
<section><label>Kat zadany [°] <input id="angle" type="number" min="0" max="360" step="0.1" value="30"></label>
<button onclick="act('move')">Ustaw kat</button><button id="stop" onclick="act('stop')">STOP obu napedow</button>
<label>Predkosc [°/s] <input id="speed" type="number" min="1" max="12" step="0.1" value="12"></label>
<label>Przyspieszenie i hamowanie [°/s²] <input id="accel" type="number" min="1" max="120" step="0.1" value="12"></label>
<button onclick="act('tune')">Zastosuj parametry</button><button onclick="act('save')">Zapisz parametry</button>
<p>Ustaw mechanizm w pozycji zerowej przed zapisaniem odniesienia.</p><button onclick="act('zero')">Zapisz aktualna pozycje jako zero</button>
<p id="message" role="status"></p></section><p><a href="/update">Aktualizacja programu OTA</a></p>
<script>const token='__TOKEN__';let alive=false,busy=false,loaded=false;
async function post(action){const data=new URLSearchParams({token,action,angle:document.querySelector('#angle').value,speed:document.querySelector('#speed').value,accel:document.querySelector('#accel').value});const r=await fetch('/api/control',{method:'POST',body:data});const text=await r.text();if(!r.ok)throw Error(text);return text;}
async function act(action){try{document.querySelector('#message').textContent=await post(action);if(action==='move')alive=true;if(action==='stop')alive=false;}catch(e){document.querySelector('#message').textContent=e.message;}}
async function poll(){if(busy)return;busy=true;try{if(alive)await post('heartbeat');const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error('Brak odpowiedzi sterownika');const s=await r.json();alive=s.moving&&s.webOwner;document.querySelector('#state').textContent=`Kat: ${s.angle.toFixed(2)}°\nCel: ${s.target.toFixed(1)}°\nAS5600: ${s.sensor?'OK':'BLAD'} | Zero: ${s.calibrated?'zapisane':'wymagane'}\nKlapka: ${s.moving?'ruch':'postoj'} | Blad: ${s.fault}\nRynna: ${s.lift}\nEthernet: ${s.eth} | Wi-Fi: ${s.wifi}`;if(!loaded){document.querySelector('#speed').value=s.speed;document.querySelector('#accel').value=s.accel;loaded=true;}}catch(e){document.querySelector('#state').textContent=e.message;}finally{busy=false;}}
setInterval(poll,400);poll();</script></html>)html";

bool parseNumber(const String& text, float& value) {
  String normalized = text;
  normalized.replace(',', '.');
  char* end;
  value = strtof(normalized.c_str(), &end);
  return end != normalized.c_str() && *end == '\0' && isfinite(value);
}

void setupWeb() {
  controlToken = String(esp_random(), HEX) + String(esp_random(), HEX);
  web.on("/", HTTP_GET, []() {
    if (!requireLogin()) return;
    String html = FPSTR(CONTROL_PAGE);
    html.replace("__TOKEN__", controlToken);
    web.sendHeader("Cache-Control", "no-store");
    web.send(200, "text/html; charset=utf-8", html);
  });
  web.on("/api/status", HTTP_GET, []() {
    if (!requireLogin()) return;
    JsonDocument state;
    lockMotion();
    state["angle"] = angle;
    state["target"] = target;
    state["sensor"] = sensorOK;
    state["calibrated"] = calibrated;
    state["moving"] = moving;
    state["fault"] = motionFault;
    state["webOwner"] = flapOwner == WEB_OWNER;
    state["lift"] = liftDirection;
    state["speed"] = maxSpeed;
    state["accel"] = acceleration;
    unlockMotion();
    state["eth"] = ETH.localIP().toString();
    state["wifi"] = WiFi.localIP().toString();
    String json;
    serializeJson(state, json);
    web.sendHeader("Cache-Control", "no-store");
    web.send(200, "application/json", json);
  });
  web.on("/api/control", HTTP_POST, []() {
    if (!requireLogin()) return;
    if (!validToken()) { web.send(403, "text/plain", "Odswiez strone."); return; }
    const String action = web.arg("action");
    float destination = 0, speed = 0, accel = 0;
    const bool validMove = parseNumber(web.arg("angle"), destination) && destination >= 0 && destination <= 360;
    const bool validTuning = parseNumber(web.arg("speed"), speed) && speed >= 1 && speed <= 12 &&
                             parseNumber(web.arg("accel"), accel) && accel >= 1 && accel <= 120;
    bool ok = false;
    lockMotion();
    if (action == "stop") {
      haltAll();
      motionFault = 0;
      ok = true;
    } else if (action == "heartbeat") {
      if (flapOwner == WEB_OWNER) flapAt = millis();
      ok = true;
    } else if (!maintenance && driverReady) {
      if (action == "move" && validMove) {
        ok = startFlap(destination, WEB_OWNER, web.client().localIP() == ethernetIP);
      } else if (action == "tune" && validTuning && !moving) {
        maxSpeed = speed;
        acceleration = accel;
        ok = true;
      } else if (action == "save" && !moving) {
        ok = saveTuning();
      } else if (action == "zero" && !moving && !liftCommand) {
        char zero[] = "ZERO";
        motionCommand(zero);
        ok = sensorOK && calibrated && motionFault != 3;
      }
    }
    unlockMotion();
    web.send(ok ? 200 : 409, "text/plain; charset=utf-8", ok ? "OK" : "Odrzucono: sprawdz postoj, czujnik, zapis zera i wartosci.");
  });
  web.on("/update", HTTP_GET, []() {
    if (!requireLogin()) return;
    String page = F("<!doctype html><meta charset='utf-8'><meta name='viewport' content='width=device-width'><title>OTA suszarni</title><h1>Aktualizacja WT32</h1><p>Wybierz plik aplikacji .ino.bin. Rozpoczecie zatrzymuje oba napedy.</p><form method='POST' enctype='multipart/form-data' action='/update?token=");
    page += controlToken;
    page += F("'><input type='file' name='firmware' accept='.bin' required><button>Wgraj</button></form><p><a href='/'>Sterowanie</a></p>");
    web.sendHeader("Cache-Control", "no-store");
    web.send(200, "text/html; charset=utf-8", page);
  });
  web.on("/update", HTTP_POST, []() {
    if (!requireLogin()) return;
    if (!validToken()) { web.send(403, "text/plain", "Odswiez strone OTA."); return; }
    if (uploadAccepted && uploadSucceeded && !uploadFailed) {
      rebootPending = true;
      rebootAt = millis();
      web.sendHeader("Connection", "close");
      web.send(200, "text/plain", "OK. Restart sterownika.");
    } else {
      if (Update.isRunning()) Update.abort();
      endFailedMaintenance();
      web.send(500, "text/plain", "Aktualizacja nieudana. Napedy zatrzymane. Wyslij nowe polecenie po sprawdzeniu sterownika.");
    }
  }, []() {
    HTTPUpload& upload = web.upload();
    if (upload.status == UPLOAD_FILE_START) {
      uploadAccepted = authenticated() && validToken();
      uploadFailed = true;
      uploadSucceeded = false;
      if (!uploadAccepted) return;
      beginMaintenance();
      uploadFailed = !upload.filename.endsWith(".bin") || !Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
    } else if (upload.status == UPLOAD_FILE_WRITE && uploadAccepted) {
      otaLastByte = millis();
      if (!uploadFailed && Update.write(upload.buf, upload.currentSize) != upload.currentSize) uploadFailed = true;
    } else if (upload.status == UPLOAD_FILE_END && uploadAccepted) {
      otaLastByte = millis();
      uploadSucceeded = !uploadFailed && Update.end(true);
      if (!uploadSucceeded && Update.isRunning()) Update.abort();
      uploadFailed = !uploadSucceeded;
    } else if (upload.status == UPLOAD_FILE_ABORTED && uploadAccepted) {
      if (Update.isRunning()) Update.abort();
      uploadFailed = true;
      uploadSucceeded = false;
      endFailedMaintenance();
    }
  });
  web.onNotFound([]() { web.send(404, "text/plain", "Nie znaleziono."); });
}

void addWifi(const char* ssid, const char* password) {
  if (!ssid || !ssid[0] || networkCount == 6) return;
  for (unsigned i = 0; i < networkCount; ++i) if (networks[i].ssid == ssid) return;
  networks[networkCount++] = {String(ssid), String(password ? password : "")};
}

void setupWifi() {
  JsonDocument document;
  if (!deserializeJson(document, DRYER_WIFI_NETWORKS) && document.is<JsonArray>()) {
    for (JsonObject entry : document.as<JsonArray>()) addWifi(entry["ssid"] | "", entry["password"] | "");
  }
  addWifi(DRYER_BOOTSTRAP_WIFI, DRYER_BOOTSTRAP_WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("suszarnia-klapka");
  WiFi.setAutoReconnect(false);
}

void serviceWifi() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wifiReported) { Serial.print("Wi-Fi IP: "); Serial.println(WiFi.localIP()); }
  wifiReported = connected;
  if (connected || !networkCount || rebootPending) return;
  lockMotion();
  const bool idle = !moving && !liftCommand && !maintenance;
  unlockMotion();
  if (idle && (!wifiAttempted || millis() - wifiAttemptAt >= 12000)) {
    WiFi.disconnect(false);
    WiFi.begin(networks[networkIndex].ssid.c_str(), networks[networkIndex].password.c_str());
    networkIndex = (networkIndex + 1) % networkCount;
    wifiAttempted = true;
    wifiAttemptAt = millis();
  }
}

void serviceSerial() {
  for (unsigned budget = 0; budget < 32 && Serial.available(); ++budget) {
    const char value = Serial.read();
    if (value == '\r' || value == '\n') {
      if (inputOverflow) Serial.println("Za dluga komenda.");
      else if (inputLength) {
        input[inputLength] = '\0';
        char* line = input;
        while (*line && isspace(static_cast<unsigned char>(*line))) ++line;
        size_t length = strlen(line);
        while (length && isspace(static_cast<unsigned char>(line[length - 1]))) line[--length] = '\0';
        for (size_t i = 0; i < length; ++i) line[i] = toupper(static_cast<unsigned char>(line[i]));
        lockMotion();
        if (!strcmp(line, "STOP")) haltAll();
        else if (!strcmp(line, "STATUS")) printStatus();
        else if (!strcmp(line, "ZERO") && liftCommand) Serial.println("Najpierw zatrzymaj rynne.");
        else if (!maintenance && (!moving || flapOwner == USB_OWNER)) {
          if (!strcmp(line, "SAVE")) Serial.println(saveTuning() ? "Zapisano parametry." : "Blad zapisu.");
          else {
            motionCommand(line);
            if (moving) flapOwner = USB_OWNER;
          }
        } else Serial.println("Odrzucono: ruch innego klienta lub aktualizacja OTA.");
        unlockMotion();
      }
      inputLength = 0;
      inputOverflow = false;
    } else if (inputLength < sizeof(input) - 1) input[inputLength++] = value;
    else inputOverflow = true;
  }
}

void setup() {
  const int pins[] = {FLAP_A, FLAP_B, LIFT_A, LIFT_B};
  for (int pin : pins) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  Serial.begin(115200);
  motionMutex = xSemaphoreCreateMutex();
  if (!motionMutex) { Serial.println("Blad mutex. Napedy wylaczone."); for (;;) delay(1000); }
  bool pwmOK = true;
  for (int pin : pins) if (!ledcAttach(pin, 2000, 8)) pwmOK = false;
  haltAll();
  Wire.begin(17, 5);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  settings.begin("flap-test", false);
  zeroAngle = settings.getUInt("zero", 0);
  calibrated = settings.isKey("zero") && zeroAngle <= 4095;
  float tuning[2];
  if (settings.getBytes("tuning", tuning, sizeof(tuning)) == sizeof(tuning) &&
      isfinite(tuning[0]) && tuning[0] >= 1 && tuning[0] <= 12 &&
      isfinite(tuning[1]) && tuning[1] >= 1 && tuning[1] <= 120) {
    maxSpeed = tuning[0];
    acceleration = tuning[1];
  }
  sensorOK = readEncoder();
  driverReady = pwmOK;
  if (xTaskCreatePinnedToCore(controlTask, "motion", 8192, nullptr, 2, nullptr, 1) != pdPASS) {
    driverReady = false;
    haltAll();
    Serial.println("Blad zadania sterowania. Napedy wylaczone.");
  }
  Network.begin();
  if (!ETH.begin(ETH_PHY_LAN8720, 1, 23, 18, 16, ETH_CLOCK_GPIO0_IN)) Serial.println("Blad Ethernet.");
  ETH.setHostname("suszarnia-klapka");
  if (!ETH.config(ethernetIP, gatewayIP, subnet)) Serial.println("Blad statycznego IP.");
  setupWifi();
  setupWeb();
  if (passwordConfigured()) {
    ArduinoOTA.setHostname("suszarnia-klapka");
    ArduinoOTA.setPassword(DRYER_BOOTSTRAP_OTA_PASSWORD);
    ArduinoOTA.onStart([]() { beginMaintenance(); Serial.println("Arduino OTA: start."); });
    ArduinoOTA.onEnd([]() { Serial.println("Arduino OTA: gotowe."); });
    ArduinoOTA.onError([](ota_error_t) { endFailedMaintenance(); Serial.println("Arduino OTA: blad."); });
  }
  Serial.println("WT32 L298N + AS5600 v3.0 / 192.168.1.13:502 unit 1 / WWW i OTA port 80");
  Serial.println("AS5600 SDA17 SCL5; L298N IN1=32 IN2=33 IN3=14 IN4=4.");
  Serial.println("Nowy ESP wymaga ZERO. USB 115200: kat, ZERO, SPEED, ACCEL, SAVE, STOP, STATUS.");
}

void loop() {
  const bool ethernet = ETH.hasIP() && ETH.linkUp();
  const bool wireless = WiFi.status() == WL_CONNECTED;
  lockMotion();
  ethOnline = ethernet;
  wifiOnline = wireless;
  const bool busy = maintenance;
  unlockMotion();
  if ((ethernet || wireless) && !serverStarted) {
    modbusServer.begin();
    web.begin();
    serverStarted = true;
    if (passwordConfigured()) { ArduinoOTA.begin(); otaStarted = true; }
    Serial.print("Ethernet IP: "); Serial.println(ETH.localIP());
  }
  if (!busy && !rebootPending) serviceModbus();
  serviceSerial();
  serviceWifi();
  if (otaStarted && !busy && !rebootPending) ArduinoOTA.handle();
  if (serverStarted && !rebootPending) web.handleClient();
  lockMotion();
  const bool timedOut = maintenance && !rebootPending && millis() - otaLastByte > 30000;
  unlockMotion();
  if (timedOut) {
    if (Update.isRunning()) Update.abort();
    endFailedMaintenance();
  }
  if (rebootPending && millis() - rebootAt >= 1000) {
    lockMotion(); haltAll(); unlockMotion();
    ESP.restart();
  }
  delay(1);
}
