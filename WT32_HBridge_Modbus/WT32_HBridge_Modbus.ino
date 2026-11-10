// WT32-ETH01 / BTS7960 / TXS0108E v1.1. Arduino ESP32 3.x + ArduinoJson 7.x.
// Ethernet 192.168.1.13/24 gateway192.168.1.1; WiFi multi-network DHCP.
// Local browser OTA: http://192.168.1.13/update (admin / secrets OTA password).
// No internet update checks/downloads. OTA accepted only with motor stopped.
// BENCH VERSION: hardware EN default-OFF must be verified before motor power.
// TCP502 unit1. FC03, FC06, FC16. Offsets 0=direction 0/1/2, 1=PWM 0..1000.
// Write offset0 every <=500ms (also when using FC16). Timeout=1500ms.
// After boot/disconnect/timeout write direction=0 before starting.
// Read-only offsets:2 actual PWM,3 actual direction,4 armed,5 version=110.
// GPIO32->TXS A1/B1->RPWM;33->A2/B2->LPWM;14->A3/B3->R_EN+L_EN.
// GPIO4->OE, 10k OE-GND. VA3V3 VB5V driverVCC5V common GND.
// OE LOW floats TXS outputs! External default-OFF on EN required.
// PWM does not measure RPM. No software current or temperature protection.
#include <Arduino.h>
#include <ETH.h>
#include <Network.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "secrets.h"
#ifndef DRYER_WIFI_NETWORKS
#define DRYER_WIFI_NETWORKS "[]"
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
constexpr int R=32,L=33,EN=14,OE=4;
constexpr bool STATIC_IP=true;
IPAddress ip(192,168,1,13),gw(192,168,1,1),mask(255,255,255,0);
NetworkServer server(502);
NetworkClient client;
uint8_t buf[260]; size_t used=0;
uint16_t dir=0,pwm=0,actual=0,running=0,previous=0;
uint32_t last=0,tick=0,zeroAt=0,frameAt=0;
bool armed=false,ready=false,started=false,hadIP=false;
bool otaBusy=false,otaFailed=false,rebootPending=false,webStarted=false;
uint32_t otaLast=0,rebootAt=0;
WebServer web(80);
String uploadToken;
struct WifiEntry { String ssid,password; };
WifiEntry wifiEntries[12];
size_t wifiCount=0,wifiIndex=0;
uint32_t wifiAttempt=0;
bool wifiAttempted=false,hadWifi=false;
uint16_t get16(const uint8_t *p){return (uint16_t(p[0])<<8)|p[1];}
void set16(uint8_t *p,uint16_t x){p[0]=x>>8;p[1]=x;}
void drive(){
 digitalWrite(EN,LOW);
 ledcWrite(R,0);ledcWrite(L,0);
 if(armed && ready && actual){
  ledcWrite(running==1?R:L,uint32_t(actual)*1023/1000);
  digitalWrite(EN,HIGH);
 }
}
void stopMotor(bool lock){
 if(running){previous=running;zeroAt=millis();}
 actual=0;running=0;dir=0;pwm=0;
 if(lock)armed=false;
 drive();
}
void disconnect(){stopMotor(true);client.stop();used=0;}
uint16_t value(uint16_t a){
 switch(a){case 0:return dir;case 1:return pwm;case 2:return actual;
 case 3:return running;case 4:return armed;case 5:return 110;} return 0;
}
void reply(const uint8_t *p,size_t n){
 uint8_t out[260];memcpy(out,buf,7);set16(out+4,n+1);
 memcpy(out+7,p,n);client.write(out,n+7);
}
void errorReply(uint8_t f,uint8_t e){uint8_t p[2]={uint8_t(f|128),e};reply(p,2);}
void request(size_t total){
 if(buf[6]!=1)return;
 uint8_t f=buf[7];size_t n=total-7;
 if(f==3){
  if(n!=5){errorReply(f,3);return;}
  uint16_t a=get16(buf+8),c=get16(buf+10);
  if(!c||c>125){errorReply(f,3);return;}
  if(a>=6||uint32_t(a)+c>6){errorReply(f,2);return;}
  uint8_t p[14];p[0]=f;p[1]=c*2;
  for(uint16_t i=0;i<c;i++)set16(p+2+2*i,value(a+i));
  reply(p,2+c*2);return;
 }
 if(f!=6&&f!=16){errorReply(f,1);return;}
 if((f==6&&n!=5)||(f==16&&n<6)){errorReply(f,3);return;}
 uint16_t a=get16(buf+8),c=f==6?1:get16(buf+10);
 if(f==16&&(!c||c>2||buf[12]!=c*2||n!=size_t(6+c*2))){errorReply(f,3);return;}
 if(a>1||uint32_t(a)+c>2){errorReply(f,2);return;}
 uint16_t d=dir,s=pwm;
 for(uint16_t i=0;i<c;i++){
  uint16_t v=get16(buf+(f==6?10:13+2*i));
  if(a+i==0)d=v;else s=v;
 }
 if(d>2||s>1000){errorReply(f,3);return;}
 if(a==0&&d!=0&&(!armed||!ready||otaBusy||rebootPending)){errorReply(f,3);return;}
 if(a==0){
  last=millis();
  if(d==0){stopMotor(false);armed=ready&&!otaBusy&&!rebootPending;}
 }
 dir=d;pwm=s;
 uint8_t p[5];p[0]=f;set16(p+1,a);set16(p+3,f==6?get16(buf+10):c);reply(p,5);
}
void tcp(){
 if(!started)return;
 if(client&&!client.connected())disconnect();
 NetworkClient next=server.accept();
 if(next){
  if(client&&client.connected())next.stop();
  else{disconnect();client=next;client.setNoDelay(true);}
 }
 if(!client||!client.connected())return;
 for(uint16_t budget=0;budget<260&&client.available();budget++){
  if(used==0)frameAt=millis();
  int b=client.read();if(b<0)break;
  buf[used++]=uint8_t(b);
  if(used>=6){
   uint16_t len=get16(buf+4);
   if(get16(buf+2)!=0||len<2||len>254){disconnect();return;}
   if(used==size_t(len+6)){request(used);used=0;}
  }
  if(used==sizeof(buf)){disconnect();return;}
 }
 if(used&&uint32_t(millis()-frameAt)>500)disconnect();
}
void motor(){
 uint32_t now=millis();
 if(otaBusy||rebootPending||!networkOnline()||!client.connected()||
    (armed&&uint32_t(now-last)>1500)){stopMotor(true);return;}
 if(!armed||!ready||uint32_t(now-tick)<10)return;
 tick=now;
 uint16_t target=(dir&&pwm)?pwm:0;
 if(running&&dir!=running)target=0;
 if(actual<target){
  if(!running){
   if(previous&&dir!=previous&&uint32_t(now-zeroAt)<700)return;
   running=dir;
  }
  actual=min(uint16_t(actual+10),target);
 }else if(actual>target)actual=(actual-target>10)?actual-10:target;
 if(actual==0&&running){previous=running;running=0;zeroAt=now;}
 drive();
}

bool networkOnline(){
 return (ETH.hasIP()&&ETH.linkUp()) || WiFi.status()==WL_CONNECTED;
}
void addWifi(const char *ssid,const char *password){
 if(!ssid||!ssid[0]||wifiCount>=12)return;
 for(size_t i=0;i<wifiCount;i++)if(wifiEntries[i].ssid==ssid)return;
 wifiEntries[wifiCount++]={String(ssid),String(password?password:"")};
}
void prepareWifi(){
 JsonDocument doc;
 if(!deserializeJson(doc,DRYER_WIFI_NETWORKS)&&doc.is<JsonArray>()){
  for(JsonObject item:doc.as<JsonArray>())
   addWifi(item["ssid"] | "",item["password"] | "");
 }else Serial.println("WiFi JSON invalid: using bootstrap if configured");
 addWifi(DRYER_BOOTSTRAP_WIFI,DRYER_BOOTSTRAP_WIFI_PASSWORD);
 WiFi.mode(WIFI_STA);
 WiFi.setHostname("suszarnia-mostek");
 WiFi.setAutoReconnect(false);
}
void serviceWifi(){
 bool connected=WiFi.status()==WL_CONNECTED;
 if(connected&&!hadWifi){
  Serial.print("WiFi IP: ");Serial.println(WiFi.localIP());
 }
 hadWifi=connected;
 if(connected||!wifiCount||otaBusy||rebootPending)return;
 // Do not scan/reconnect while driving. Ethernet remains autonomous.
 if(actual||dir)return;
 if(!wifiAttempted||uint32_t(millis()-wifiAttempt)>=12000){
  WiFi.disconnect(false);
  WiFi.begin(wifiEntries[wifiIndex].ssid.c_str(),wifiEntries[wifiIndex].password.c_str());
  wifiIndex=(wifiIndex+1)%wifiCount;
  wifiAttempt=millis();wifiAttempted=true;
 }
}
bool otaLogin(){
 if(!strlen(DRYER_BOOTSTRAP_OTA_PASSWORD) ||
    !strcmp(DRYER_BOOTSTRAP_OTA_PASSWORD,"CHANGE_ME")){
  web.send(503,"text/plain","Set DRYER_BOOTSTRAP_OTA_PASSWORD in secrets.h first.");
  return false;
 }
 if(!web.authenticate("admin",DRYER_BOOTSTRAP_OTA_PASSWORD)){
  web.requestAuthentication();return false;
 }
 return true;
}
bool tokenOK(){
 return web.hasArg("token")&&web.arg("token")==uploadToken;
}
void setupWeb(){
 uploadToken=String(esp_random(),HEX)+String(esp_random(),HEX);
 web.on("/",HTTP_GET,[](){
  web.sendHeader("Location","/update");web.send(302,"text/plain","");
 });
 web.on("/update",HTTP_GET,[](){
  if(!otaLogin())return;
  String html=F("<!doctype html><meta charset='utf-8'><title>Mostek H OTA</title>"
   "<h2>Mostek H - aktualizacja lokalna</h2>"
   "<p>Wybierz plik aplikacji .bin. Silnik musi byc zatrzymany.</p>"
   "<form method='POST' enctype='multipart/form-data' action='/update?token=");
  html+=uploadToken;
  html+=F("'><input type='file' name='firmware' accept='.bin' required>"
   "<button>Wgraj program</button></form>");
  web.sendHeader("Cache-Control","no-store");web.send(200,"text/html",html);
 });
 web.on("/update",HTTP_POST,[](){
  if(!otaLogin())return;
  if(!tokenOK()){web.send(403,"text/plain","Invalid upload token");return;}
  bool ok=otaBusy&&!otaFailed&&!Update.hasError();
  stopMotor(true);
  if(ok){
   rebootPending=true;rebootAt=millis();
   web.sendHeader("Connection","close");
   web.send(200,"text/plain","OK. Restart. Send STOP before next motor start.");
  }else{
   if(Update.isRunning())Update.abort();
   otaBusy=false;web.send(500,"text/plain","Update failed. Motor locked; send STOP.");
  }
 },[](){
  HTTPUpload &u=web.upload();
  if(u.status==UPLOAD_FILE_START){
   otaFailed=true;
   if(!otaLogin()||!tokenOK())return;
   stopMotor(true);client.stop();used=0;
   otaBusy=true;otaLast=millis();
   otaFailed=!u.filename.endsWith(".bin")||!Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH);
  }else if(u.status==UPLOAD_FILE_WRITE){
   if(!otaBusy||!tokenOK())return;
   otaLast=millis();
   if(!otaFailed&&Update.write(u.buf,u.currentSize)!=u.currentSize)otaFailed=true;
  }else if(u.status==UPLOAD_FILE_END){
   if(!otaBusy)return;
   otaLast=millis();
   if(otaFailed||!Update.end(true)){otaFailed=true;if(Update.isRunning())Update.abort();}
  }else if(u.status==UPLOAD_FILE_ABORTED){
   if(Update.isRunning())Update.abort();
   otaBusy=false;otaFailed=true;stopMotor(true);
  }
 });
 web.onNotFound([](){web.send(404,"text/plain","Not found");});
}
void serviceWeb(){
 // Synchronous HTTP can block: process it ONLY when drive command and PWM are zero.
 // While moving, the browser waits until a Modbus STOP or communication timeout.
 if(webStarted&&((!dir&&!actual)||otaBusy||rebootPending))web.handleClient();
 if(otaBusy&&!rebootPending&&uint32_t(millis()-otaLast)>30000){
  if(Update.isRunning())Update.abort();
  otaBusy=false;otaFailed=true;stopMotor(true);
 }
 if(rebootPending&&uint32_t(millis()-rebootAt)>1000){stopMotor(true);ESP.restart();}
}
void setup(){
 pinMode(OE,OUTPUT);digitalWrite(OE,LOW);
 pinMode(EN,OUTPUT);digitalWrite(EN,LOW);
 pinMode(R,OUTPUT);digitalWrite(R,LOW);
 pinMode(L,OUTPUT);digitalWrite(L,LOW);
 Serial.begin(115200);
 bool r=ledcAttach(R,2000,10),l=ledcAttach(L,2000,10);ready=r&&l;
 if(ready){drive();digitalWrite(OE,HIGH);}
 else Serial.println("PWM ERROR: locked");
 Network.begin();
 if(!ETH.begin(ETH_PHY_LAN8720,1,23,18,16,ETH_CLOCK_GPIO0_IN))
  Serial.println("Ethernet init failed");
 ETH.setHostname("suszarnia-mostek");
 if(STATIC_IP&&!ETH.config(ip,gw,mask))Serial.println("Static IP failed");
 prepareWifi();setupWeb();
 Serial.println("Motor v1.1 / Ethernet 192.168.1.13 / local OTA / Modbus502 / STOP required");
}
void loop(){
 bool online=networkOnline();
 if(online&&!hadIP){
  if(ETH.hasIP()){Serial.print("Ethernet IP: ");Serial.println(ETH.localIP());}
  if(!started){server.begin();started=true;}
  if(!webStarted){web.begin();webStarted=true;}
 }
 if(!online&&hadIP)disconnect();
 hadIP=online;
 motor();
 if(online&&!otaBusy&&!rebootPending)tcp();
 motor();serviceWifi();serviceWeb();motor();delay(1);
}

