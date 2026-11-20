// ================================================================
// SMART IV MONITOR — FINAL VERSION (Router WiFi)
// ESP32-CAM + TFMini-S + Servo + Web Dashboard
//
// HOW TO USE:
//   1. Flash via ESP-CAM MB
//   2. Make sure your phone/laptop is on WiFi "Omer"
//   3. Open Serial Monitor at 115200 — it prints the IP
//   4. Open that IP in your browser
//      e.g. http://192.168.1.105
//
// PINS:
//   GPIO 14 -> Servo signal
//   GPIO  2 -> Buzzer
//   GPIO 15 -> TFMini TX wire (data FROM sensor)
//   GPIO 13 -> TFMini RX wire (data TO sensor)
//   GPIO  4 -> Flash LED
// ================================================================

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include <ESP32Servo.h>

// ================================================================
// WIFI — connects to your router
// ================================================================
#define WIFI_SSID     "Omer"
#define WIFI_PASSWORD "1234567qwery"

// ================================================================
// CAMERA PINS — do not change
// ================================================================
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ================================================================
// USER PINS
// ================================================================
#define FLASH_LED_PIN   4
#define SERVO_PIN      14
#define BUZZER_PIN      2
#define TFMINI_RX_PIN  15
#define TFMINI_TX_PIN  13

// ================================================================
// SETTINGS
// ================================================================
#define ANGLE_CLOSED    0
#define ANGLE_OPEN    180
#define AUTO_LEVEL      5
#define DIST_FULL_CM    5
#define DIST_EMPTY_CM  25
#define BUBBLE_SPIKE    3

// ================================================================
// GLOBALS
// ================================================================
Servo          rollerServo;
HardwareSerial tfSerial(1);

httpd_handle_t svr80 = NULL;
httpd_handle_t svr81 = NULL;

int     servoLevel  = 1;
bool    autoMode    = true;
bool    tfOk        = false;
int16_t tfDist      = 0;
int16_t tfStrength  = 0;
float   tfSmooth    = 0;
float   tfPrev      = 0;
bool    bubbleAlert = false;
bool    fluidEmpty  = false;
String  fluidLabel  = "DETECTING";
String  fluidColor  = "#aaaaaa";
int     fluidPct    = 50;

unsigned long lastSensorMs  = 0;
unsigned long lastBuzzerMs  = 0;
unsigned long lastWifiCheckMs = 0;

// ================================================================
// SERVO
// ================================================================
int levelToAngle(int lv) {
  return map(constrain(lv, 1, 10), 1, 10, ANGLE_CLOSED, ANGLE_OPEN);
}
void applyLevel(int lv) {
  servoLevel = constrain(lv, 1, 10);
  rollerServo.write(levelToAngle(servoLevel));
  Serial.printf("[SERVO] Level %d -> %d deg\n", servoLevel, levelToAngle(servoLevel));
}

// ================================================================
// BUZZER
// ================================================================
void beep(int ms) {
  digitalWrite(BUZZER_PIN, HIGH); delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}
void beepTimes(int n) {
  for (int i = 0; i < n; i++) { beep(80); delay(120); }
}

// ================================================================
// WIFI CONNECT — retries forever, reconnects if dropped
// ================================================================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.printf("\n[WIFI] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tries++;
    // After 30 seconds, restart the attempt
    if (tries > 60) {
      Serial.println("\n[WIFI] Retrying...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      tries = 0;
    }
  }

  Serial.println();
  Serial.println("================================");
  Serial.println("[WIFI] Connected!");
  Serial.printf("  Network: %s\n", WIFI_SSID);
  Serial.print("  IP:      ");
  Serial.println(WiFi.localIP());
  Serial.println("  Open this in your browser");
  Serial.printf("  (same WiFi as ESP32!)\n");
  Serial.println("================================");
}

// ================================================================
// CAMERA INIT
// ================================================================
bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_QVGA;
  cfg.jpeg_quality = 12;
  cfg.fb_count     = 2;
  if (esp_camera_init(&cfg) != ESP_OK) return false;
  sensor_t* s = esp_camera_sensor_get();
  if (s) { s->set_brightness(s, 1); s->set_contrast(s, 1); }
  return true;
}

// ================================================================
// TFMINI INIT
// ================================================================
bool initTFMini() {
  tfSerial.begin(115200, SERIAL_8N1, TFMINI_RX_PIN, TFMINI_TX_PIN);
  delay(500);
  Serial.println("[TFMINI] Waiting for data...");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (tfSerial.available() >= 9) {
      uint8_t buf[9];
      tfSerial.readBytes(buf, 9);
      if (buf[0] == 0x59 && buf[1] == 0x59) {
        Serial.println("[TFMINI] OK");
        return true;
      }
    }
    delay(10);
  }
  Serial.println("[TFMINI] Not found - check wiring");
  return false;
}

// ================================================================
// READ TFMINI
// ================================================================
bool readTFMini() {
  if (tfSerial.available() < 9) return false;
  while (tfSerial.available() >= 9) {
    if (tfSerial.peek() != 0x59) { tfSerial.read(); continue; }
    uint8_t buf[9];
    tfSerial.readBytes(buf, 9);
    if (buf[0] != 0x59 || buf[1] != 0x59) continue;
    uint8_t chk = 0;
    for (int i = 0; i < 8; i++) chk += buf[i];
    if (chk != buf[8]) continue;
    tfDist     = buf[2] | (buf[3] << 8);
    tfStrength = buf[4] | (buf[5] << 8);
    if (tfStrength < 100) return false;
    if (tfDist <= 0 || tfDist > 600) return false;
    return true;
  }
  return false;
}

// ================================================================
// ANALYSE SENSOR
// ================================================================
void analyseTFMini() {
  if (!tfOk || !readTFMini()) return;
  tfPrev   = tfSmooth;
  tfSmooth = (tfSmooth * 0.7f) + ((float)tfDist * 0.3f);
  float spike = abs(tfSmooth - tfPrev);
  bubbleAlert = (spike >= BUBBLE_SPIKE);
  float pct = map((long)(tfSmooth * 10), DIST_FULL_CM*10, DIST_EMPTY_CM*10, 100, 0);
  pct       = constrain(pct, 0, 100);
  fluidPct  = (int)pct;
  if      (fluidPct <= 5)  { fluidLabel="EMPTY";  fluidColor="#ff3b3b"; fluidEmpty=true;  }
  else if (fluidPct <= 25) { fluidLabel="LOW";    fluidColor="#ff9900"; fluidEmpty=false; }
  else if (fluidPct <= 60) { fluidLabel="MEDIUM"; fluidColor="#ffaa00"; fluidEmpty=false; }
  else                     { fluidLabel="FULL";   fluidColor="#00e676"; fluidEmpty=false; }
  Serial.printf("[TF] %.1fcm %d%% %s bubble:%s\n",
    tfSmooth, fluidPct, fluidLabel.c_str(), bubbleAlert?"YES":"no");
}

// ================================================================
// EDGE AI LOGIC
// ================================================================
void runEdgeLogic() {
  if (!autoMode) return;
  if (fluidEmpty && servoLevel > 1)  { applyLevel(1); beepTimes(5); return; }
  if (bubbleAlert && servoLevel > 3) { applyLevel(servoLevel-2); beepTimes(2); return; }
  if (!fluidEmpty && servoLevel == 1) applyLevel(AUTO_LEVEL);
}

// ================================================================
// CAMERA STREAM  port 81 /stream
// ================================================================
#define BOUNDARY "ivframe"
static esp_err_t stream_handler(httpd_req_t* req) {
  camera_fb_t* fb = NULL;
  char hdr[64];
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  while (true) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    fb = esp_camera_fb_get();
    digitalWrite(FLASH_LED_PIN, LOW);
    if (!fb) break;
    String sep = "\r\n--" BOUNDARY "\r\n";
    if (httpd_resp_send_chunk(req, sep.c_str(), sep.length()) != ESP_OK) { esp_camera_fb_return(fb); break; }
    size_t hl = snprintf(hdr, sizeof(hdr), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    if (httpd_resp_send_chunk(req, hdr, hl) != ESP_OK) { esp_camera_fb_return(fb); break; }
    if (httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) { esp_camera_fb_return(fb); break; }
    esp_camera_fb_return(fb);
    delay(33);
  }
  return ESP_OK;
}

// ================================================================
// STATUS JSON  /status
// ================================================================
static esp_err_t status_handler(httpd_req_t* req) {
  char json[300];
  snprintf(json, sizeof(json),
    "{\"fluidLabel\":\"%s\",\"fluidColor\":\"%s\","
    "\"fluidPct\":%d,\"distCm\":%.1f,"
    "\"servoLevel\":%d,\"autoMode\":%s,"
    "\"bubble\":%s,\"tfOk\":%s}",
    fluidLabel.c_str(), fluidColor.c_str(),
    fluidPct, tfSmooth,
    servoLevel, autoMode?"true":"false",
    bubbleAlert?"true":"false",
    tfOk?"true":"false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, json, strlen(json));
}

// ================================================================
// SET LEVEL  /level?v=N
// ================================================================
static esp_err_t level_handler(httpd_req_t* req) {
  char buf[32]="", param[8]="";
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  httpd_query_key_value(buf, "v", param, sizeof(param));
  int lv = atoi(param);
  if (lv >= 1 && lv <= 10) { autoMode=false; applyLevel(lv); beep(60); }
  return httpd_resp_send(req, "OK", 2);
}

// ================================================================
// AUTO MODE  /auto
// ================================================================
static esp_err_t auto_handler(httpd_req_t* req) {
  autoMode = true;
  applyLevel(AUTO_LEVEL);
  beepTimes(2);
  return httpd_resp_send(req, "AUTO", 4);
}

// ================================================================
// DASHBOARD HTML in PROGMEM — sent in 512-byte heap chunks
// Stream URL uses window.location.hostname so it always uses
// the correct IP without hardcoding anything
// ================================================================
static const char PAGE_HTML[] PROGMEM =
  "<!DOCTYPE html><html lang='en'><head>"
  "<meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>Smart IV Monitor</title>"
  "<style>"
  "*{margin:0;padding:0;box-sizing:border-box;}"
  "body{background:#04080d;color:#b8dce8;font-family:'Courier New',monospace;"
  "padding:12px;display:flex;flex-direction:column;align-items:center;gap:10px;}"
  "h1{font-size:15px;letter-spacing:4px;color:#00cfff;text-align:center;"
  "text-transform:uppercase;padding:8px 0;}"
  "p.sub{font-size:10px;color:#3a5060;letter-spacing:2px;text-align:center;}"
  ".cam-wrap{position:relative;width:100%;max-width:420px;"
  "border:1px solid #0d2535;border-radius:6px;background:#000;overflow:hidden;}"
  "#cam{display:block;width:100%;min-height:230px;object-fit:cover;}"
  ".ov{position:absolute;top:10px;left:10px;"
  "background:rgba(4,8,13,.85);border:2px solid #00cfff;"
  "border-radius:4px;padding:8px 12px;min-width:148px;transition:border-color .4s;}"
  ".ov-title{font-size:9px;letter-spacing:2px;color:#3a5060;margin-bottom:3px;}"
  ".ov-label{font-size:20px;font-weight:bold;margin-bottom:5px;}"
  ".bar-bg{background:rgba(255,255,255,.08);border-radius:2px;"
  "height:10px;width:124px;overflow:hidden;margin-bottom:4px;}"
  ".bar-fill{height:100%;border-radius:2px;transition:width .6s,background .4s;}"
  ".ov-info{font-size:10px;color:#3a5060;display:flex;justify-content:space-between;}"
  ".badge-empty{position:absolute;top:10px;right:10px;background:#ff3b3b;"
  "color:#fff;font-size:9px;font-weight:bold;padding:4px 8px;"
  "border-radius:3px;display:none;animation:blink .7s infinite;}"
  ".badge-bubble{position:absolute;bottom:10px;right:10px;background:#ff8800;"
  "color:#fff;font-size:9px;font-weight:bold;padding:4px 8px;"
  "border-radius:3px;display:none;animation:blink 1s infinite;}"
  "@keyframes blink{0%,100%{opacity:1;}50%{opacity:.1;}}"
  ".banner{width:100%;max-width:420px;padding:10px;border-radius:4px;"
  "text-align:center;display:none;animation:blink 1s infinite;}"
  ".banner p{font-size:11px;font-weight:bold;letter-spacing:2px;text-transform:uppercase;}"
  ".banner-empty{background:#3d0000;border:1px solid #ff3b3b;}"
  ".banner-empty p{color:#ff3b3b;}"
  ".banner-bubble{background:#3d2000;border:1px solid #ff8800;}"
  ".banner-bubble p{color:#ff8800;}"
  ".panel{width:100%;max-width:420px;background:#08121c;"
  "border:1px solid #0d2535;border-radius:6px;padding:14px;}"
  ".ptitle{font-size:9px;letter-spacing:3px;color:#3a5060;"
  "text-transform:uppercase;text-align:center;margin-bottom:12px;}"
  ".mode-row{display:flex;gap:8px;margin-bottom:14px;}"
  ".mbtn{flex:1;height:38px;border:1px solid #0d2535;border-radius:3px;"
  "font-family:'Courier New',monospace;font-size:11px;font-weight:bold;"
  "letter-spacing:2px;cursor:pointer;transition:all .15s;text-transform:uppercase;}"
  ".mbtn:active{transform:scale(.96);}"
  ".m-auto{background:rgba(0,207,255,.08);color:#00cfff;border-color:rgba(0,207,255,.25);}"
  ".m-auto.on{background:rgba(0,207,255,.2);border-color:#00cfff;}"
  ".m-man{background:rgba(255,170,0,.06);color:#ffaa00;border-color:rgba(255,170,0,.2);}"
  ".m-man.on{background:rgba(255,170,0,.18);border-color:#ffaa00;}"
  ".lv-row{display:flex;align-items:center;justify-content:center;gap:14px;margin-bottom:14px;}"
  ".lv-num{font-size:54px;font-weight:bold;color:#00cfff;line-height:1;"
  "min-width:68px;text-align:center;transition:color .3s;}"
  ".lv-meta span{font-size:10px;color:#3a5060;display:block;}"
  ".lv-meta strong{font-size:13px;display:block;}"
  ".flow-wrap{margin-bottom:14px;}"
  ".flow-top{display:flex;justify-content:space-between;font-size:9px;color:#3a5060;margin-bottom:4px;}"
  ".flow-bg{background:rgba(255,255,255,.06);border-radius:3px;height:12px;overflow:hidden;}"
  ".flow-fill{height:100%;border-radius:3px;background:#00cfff;transition:width .5s ease;}"
  ".lv-btns{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-bottom:12px;}"
  ".lb{height:44px;border:1px solid #0d2535;background:rgba(255,255,255,.02);"
  "color:#3a5060;font-family:'Courier New',monospace;font-size:14px;"
  "font-weight:bold;border-radius:3px;cursor:pointer;transition:all .15s;}"
  ".lb:active{transform:scale(.92);}"
  ".lb.on{border-color:#00cfff;color:#00cfff;background:rgba(0,207,255,.1);}"
  ".lb[data-l='1'].on{border-color:#ff3b3b;color:#ff3b3b;}"
  ".lb[data-l='2'].on,.lb[data-l='3'].on{border-color:#ff6b00;color:#ff6b00;}"
  ".lb[data-l='4'].on,.lb[data-l='5'].on{border-color:#ffaa00;color:#ffaa00;}"
  ".lb[data-l='6'].on,.lb[data-l='7'].on{border-color:#aadd00;color:#aadd00;}"
  ".lb[data-l='8'].on,.lb[data-l='9'].on,.lb[data-l='10'].on{border-color:#00e676;color:#00e676;}"
  ".pm-row{display:flex;gap:8px;}"
  ".pm{flex:1;height:48px;border:1px solid #0d2535;border-radius:3px;"
  "font-family:'Courier New',monospace;font-size:24px;font-weight:bold;"
  "cursor:pointer;transition:all .15s;}"
  ".pm:active{transform:scale(.94);}"
  ".pm-m{background:rgba(255,59,59,.07);color:#ff3b3b;border-color:rgba(255,59,59,.2);}"
  ".pm-p{background:rgba(0,230,118,.07);color:#00e676;border-color:rgba(0,230,118,.2);}"
  ".reads{display:grid;grid-template-columns:1fr 1fr;gap:8px;}"
  ".rcard{background:#040c14;border:1px solid #0d2535;border-radius:4px;padding:10px;}"
  ".rl{font-size:9px;color:#3a5060;letter-spacing:1px;margin-bottom:4px;}"
  ".rv{font-size:14px;font-weight:bold;color:#00cfff;}"
  ".rs{font-size:10px;color:#3a5060;margin-top:2px;}"
  "footer{font-size:9px;color:#1a3040;letter-spacing:2px;text-align:center;}"
  "</style></head><body>"
  "<h1>Smart IV Monitor</h1>"
  "<p class='sub'>Edge AI &bull; TFMini-S &bull; WiFi</p>"
  "<div class='cam-wrap'>"
  // Stream URL built dynamically from hostname in JS below
  "<img id='cam' alt='Camera loading...' style='width:100%;min-height:230px;object-fit:cover;'>"
  "<div class='ov' id='ov'>"
  "<div class='ov-title'>Fluid level</div>"
  "<div class='ov-label' id='ovLabel'>---</div>"
  "<div class='bar-bg'><div class='bar-fill' id='ovBar' style='width:50%;background:#555'></div></div>"
  "<div class='ov-info'><span id='ovPct'>--%</span><span id='ovDist'>-- cm</span></div>"
  "</div>"
  "<div class='badge-empty' id='badgeEmpty'>EMPTY</div>"
  "<div class='badge-bubble' id='badgeBubble'>BUBBLE</div>"
  "</div>"
  "<div class='banner banner-empty' id='bannerEmpty'>"
  "<p>IV fluid empty &mdash; nurse action required</p></div>"
  "<div class='banner banner-bubble' id='bannerBubble'>"
  "<p>Bubble detected in IV line</p></div>"
  "<div class='panel'>"
  "<div class='ptitle'>Flow level control</div>"
  "<div class='mode-row'>"
  "<button class='mbtn m-auto on' id='btnAuto' onclick='setAuto()'>Auto (Edge AI)</button>"
  "<button class='mbtn m-man' id='btnMan' onclick='setManual()'>Manual</button>"
  "</div>"
  "<div class='lv-row'>"
  "<div class='lv-num' id='lvNum'>5</div>"
  "<div class='lv-meta'>"
  "<span>Current level</span><strong id='lvName'>MEDIUM</strong>"
  "<span style='margin-top:4px'>Servo angle</span><strong id='lvAngle'>80&#176;</strong>"
  "</div></div>"
  "<div class='flow-wrap'>"
  "<div class='flow-top'><span>Closed</span><span id='flowPct'>44%</span><span>Full open</span></div>"
  "<div class='flow-bg'><div class='flow-fill' id='flowBar' style='width:44%'></div></div>"
  "</div>"
  "<div class='lv-btns'>"
  "<button class='lb' data-l='1' onclick='setLevel(1)'>1</button>"
  "<button class='lb' data-l='2' onclick='setLevel(2)'>2</button>"
  "<button class='lb' data-l='3' onclick='setLevel(3)'>3</button>"
  "<button class='lb' data-l='4' onclick='setLevel(4)'>4</button>"
  "<button class='lb on' data-l='5' onclick='setLevel(5)'>5</button>"
  "<button class='lb' data-l='6' onclick='setLevel(6)'>6</button>"
  "<button class='lb' data-l='7' onclick='setLevel(7)'>7</button>"
  "<button class='lb' data-l='8' onclick='setLevel(8)'>8</button>"
  "<button class='lb' data-l='9' onclick='setLevel(9)'>9</button>"
  "<button class='lb' data-l='10' onclick='setLevel(10)'>10</button>"
  "</div>"
  "<div class='pm-row'>"
  "<button class='pm pm-m' onclick='step(-1)'>&#8722;</button>"
  "<button class='pm pm-p' onclick='step(1)'>&#43;</button>"
  "</div></div>"
  "<div class='panel'>"
  "<div class='ptitle'>Live sensor readings</div>"
  "<div class='reads'>"
  "<div class='rcard'><div class='rl'>TFMini-S distance</div>"
  "<div class='rv' id='rdDist'>-- cm</div><div class='rs'>Primary sensor</div></div>"
  "<div class='rcard'><div class='rl'>Fluid level</div>"
  "<div class='rv' id='rdPct'>--%</div><div class='rs'>Calculated</div></div>"
  "<div class='rcard'><div class='rl'>Servo level</div>"
  "<div class='rv' id='rdLv'>--/10</div><div class='rs'>Flow rate</div></div>"
  "<div class='rcard'><div class='rl'>Bubble status</div>"
  "<div class='rv' id='rdBub' style='color:#00e676'>Clear</div>"
  "<div class='rs'>IR spike detect</div></div>"
  "</div></div>"
  "<footer>Smart IV Infusion System &bull; 2024</footer>"
  "<script>"
  // Use window.location.hostname — works with any IP, no hardcoding needed
  "var host=window.location.hostname;"
  "document.getElementById('cam').src='http://'+host+':81/stream';"
  "var cur=5,isAuto=true;"
  "var LN={1:'CLOSED',2:'VERY LOW',3:'LOW',4:'LOW-MED',5:'MEDIUM',"
  "6:'MED-HIGH',7:'HIGH',8:'FAST',9:'VERY FAST',10:'FULL OPEN'};"
  "var LC={1:'#ff3b3b',2:'#ff6b00',3:'#ff6b00',4:'#ffaa00',5:'#ffaa00',"
  "6:'#aadd00',7:'#aadd00',8:'#00e676',9:'#00e676',10:'#00e676'};"
  "function setLevel(lv){"
  "lv=Math.max(1,Math.min(10,lv));"
  "fetch('/level?v='+lv).catch(function(){});"
  "updateLvUI(lv);isAuto=false;updateModeUI();}"
  "function step(d){setLevel(cur+d);}"
  "function setAuto(){"
  "fetch('/auto').catch(function(){});"
  "isAuto=true;updateModeUI();}"
  "function setManual(){isAuto=false;updateModeUI();}"
  "function updateModeUI(){"
  "document.getElementById('btnAuto').classList.toggle('on',isAuto);"
  "document.getElementById('btnMan').classList.toggle('on',!isAuto);}"
  "function updateLvUI(lv){"
  "cur=lv;var c=LC[lv];"
  "document.getElementById('lvNum').textContent=lv;"
  "document.getElementById('lvNum').style.color=c;"
  "document.getElementById('lvName').textContent=LN[lv];"
  "var ang=Math.round(((lv-1)/9)*180);"
  "document.getElementById('lvAngle').textContent=ang+'\\u00b0';"
  "var pct=Math.round(((lv-1)/9)*100);"
  "document.getElementById('flowBar').style.width=pct+'%';"
  "document.getElementById('flowPct').textContent=pct+'%';"
  "document.querySelectorAll('.lb').forEach(function(b){"
  "b.classList.toggle('on',parseInt(b.dataset.l)===lv);});}"
  "function poll(){"
  "fetch('/status')"
  ".then(function(r){return r.json();})"
  ".then(function(d){"
  "var lbl=document.getElementById('ovLabel');"
  "lbl.textContent=d.fluidLabel;lbl.style.color=d.fluidColor;"
  "document.getElementById('ovBar').style.width=d.fluidPct+'%';"
  "document.getElementById('ovBar').style.background=d.fluidColor;"
  "document.getElementById('ovPct').textContent=d.fluidPct+'%';"
  "document.getElementById('ovDist').textContent=d.distCm.toFixed(1)+' cm';"
  "document.getElementById('ov').style.borderColor=d.fluidColor;"
  "var emp=d.fluidLabel==='EMPTY';var bub=d.bubble;"
  "document.getElementById('badgeEmpty').style.display=emp?'block':'none';"
  "document.getElementById('bannerEmpty').style.display=emp?'block':'none';"
  "document.getElementById('badgeBubble').style.display=bub?'block':'none';"
  "document.getElementById('bannerBubble').style.display=bub?'block':'none';"
  "document.getElementById('rdDist').textContent=d.distCm.toFixed(1)+' cm';"
  "document.getElementById('rdPct').textContent=d.fluidPct+'%';"
  "document.getElementById('rdLv').textContent=d.servoLevel+'/10';"
  "document.getElementById('rdBub').textContent=bub?'DETECTED!':'Clear';"
  "document.getElementById('rdBub').style.color=bub?'#ff8800':'#00e676';"
  "if(d.servoLevel!==cur)updateLvUI(d.servoLevel);"
  "if(d.autoMode!==isAuto){isAuto=d.autoMode;updateModeUI();}})"
  ".catch(function(){"
  "document.getElementById('ovLabel').textContent='NO SIGNAL';"
  "document.getElementById('ovLabel').style.color='#555';});}"
  "updateLvUI(5);"
  "setInterval(poll,1500);"
  "poll();"
  "</script></body></html>";

static esp_err_t dashboard_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  const size_t CHUNK = 512;
  char* buf = (char*)malloc(CHUNK);
  if (!buf) return httpd_resp_send_500(req);
  size_t total = strlen_P(PAGE_HTML);
  size_t sent  = 0;
  while (sent < total) {
    size_t n = (total - sent) > CHUNK ? CHUNK : (total - sent);
    memcpy_P(buf, PAGE_HTML + sent, n);
    if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) break;
    sent += n;
  }
  free(buf);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// ================================================================
// START SERVERS
// ================================================================
void startServers() {
  httpd_config_t c80   = HTTPD_DEFAULT_CONFIG();
  c80.server_port      = 80;
  c80.ctrl_port        = 32768;
  c80.max_uri_handlers = 8;
  if (httpd_start(&svr80, &c80) == ESP_OK) {
    httpd_uri_t u1 = {"/",       HTTP_GET, dashboard_handler, NULL};
    httpd_uri_t u2 = {"/status", HTTP_GET, status_handler,    NULL};
    httpd_uri_t u3 = {"/level",  HTTP_GET, level_handler,     NULL};
    httpd_uri_t u4 = {"/auto",   HTTP_GET, auto_handler,      NULL};
    httpd_register_uri_handler(svr80, &u1);
    httpd_register_uri_handler(svr80, &u2);
    httpd_register_uri_handler(svr80, &u3);
    httpd_register_uri_handler(svr80, &u4);
  }
  httpd_config_t c81   = HTTPD_DEFAULT_CONFIG();
  c81.server_port      = 81;
  c81.ctrl_port        = 32769;
  c81.max_uri_handlers = 4;
  if (httpd_start(&svr81, &c81) == ESP_OK) {
    httpd_uri_t s1 = {"/stream", HTTP_GET, stream_handler, NULL};
    httpd_register_uri_handler(svr81, &s1);
  }
  Serial.printf("[SERVER] Dashboard: http://%s\n",         WiFi.localIP().toString().c_str());
  Serial.printf("[SERVER] Stream:    http://%s:81/stream\n", WiFi.localIP().toString().c_str());
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SMART IV MONITOR ===");

  pinMode(FLASH_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN,    OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN,    LOW);

  for (int i = 0; i < 3; i++) { beep(100); delay(150); }

  ESP32PWM::allocateTimer(0);
  rollerServo.setPeriodHertz(50);
  rollerServo.attach(SERVO_PIN, 500, 2400);
  applyLevel(1);

  if (!initCamera()) {
    Serial.println("[CAMERA] FAILED - halting");
    while (true) {
      digitalWrite(FLASH_LED_PIN, HIGH); delay(100);
      digitalWrite(FLASH_LED_PIN, LOW);  delay(100);
    }
  }
  Serial.println("[CAMERA] OK");

  tfOk = initTFMini();

  // Connect to router WiFi
  connectWiFi();

  startServers();
  applyLevel(AUTO_LEVEL);
  beepTimes(2);
  Serial.println("[READY]");
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  unsigned long now = millis();

  // Auto-reconnect WiFi if dropped
  if (now - lastWifiCheckMs > 10000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WIFI] Dropped — reconnecting...");
      connectWiFi();
    }
    lastWifiCheckMs = now;
  }

  if (now - lastSensorMs > 100) {
    analyseTFMini();
    lastSensorMs = now;
  }

  runEdgeLogic();

  if (fluidEmpty && now - lastBuzzerMs > 4000) {
    beepTimes(4);
    lastBuzzerMs = now;
  }

  delay(20);
}
