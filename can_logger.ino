/*
  adalogger - ID-Based Table View
  Creates WiFi hotspot and serves CAN message logs grouped by ID
  Each CAN ID gets its own row showing latest data, message count, and timing
  
  Pins:
    RED_PIN   = 15 (LEDC PWM)
    GREEN_PIN = 13 (LEDC PWM)
    BLUE_PIN  = 14 (LEDC PWM)
    CAN TX    = GPIO 4
    CAN RX    = GPIO 5
  
  Status LED Indicators:
    - Booting: Pulsing blue
    - WiFi AP Ready: Solid green
    - CAN Active: Green with blue flashes on activity
    - CAN Error: Red pulsing
  
  Serial Commands:
    &CANID SET xx   (0..63 live)
    &CANID SAVE     (EEPROM + reboot)
    &CANID GET
*/

#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <map>
#include "driver/twai.h"
#include "driver/ledc.h"

// WiFi AP credentials
const char* ssid = "FRC-CAN-Logger";
const char* password = "frclogger";

// Status LED pins
#define RED_PIN    15
#define GREEN_PIN  13
#define BLUE_PIN   14

// CAN Bus pins
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

// EEPROM
#define EEPROM_BYTES 64

// LEDC (PWM) configuration
static const uint32_t LEDC_FREQ_HZ = 4000;
static const ledc_timer_bit_t LEDC_RES_BITS = LEDC_TIMER_12_BIT;
static const ledc_mode_t  LEDC_MODE  = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
static const ledc_channel_t LEDC_CH_RED   = LEDC_CHANNEL_0;
static const ledc_channel_t LEDC_CH_GREEN = LEDC_CHANNEL_1;
static const ledc_channel_t LEDC_CH_BLUE  = LEDC_CHANNEL_2;

WebServer server(80);

// FRC CAN constants
#define DEVICE_ID       0x0A
#define MANUFACTURER_ID 0x08
#define DEFAULT_DEVICE_NUMBER 10

// FRC CAN API classes
#define API_CLASS_ROBOT_CONTROL 0x00
#define API_CLASS_MOTOR_CONTROL 0x02
#define API_CLASS_RELAY_CONTROL 0x03
#define API_CLASS_GYRO_SENSOR 0x04
#define API_CLASS_ACCEL_SENSOR 0x05
#define API_CLASS_ULTRASONIC 0x06
#define API_CLASS_GEAR_TOOTH 0x07
#define API_CLASS_MISC_SENSOR 0x08
#define API_CLASS_IO_CONFIG 0x09
#define API_CLASS_POWER 0x0A
#define API_CLASS_FIRMWARE 0x1F

// Custom APIs for logger
#define API_TX_STATUS   0x1A0
#define API_TX_HEARTBEAT 0x1A1

// FRC Robot Controller Heartbeat ID
#define FRC_HEARTBEAT_ID 0x01011840

// Software version
#define SOFTWARE_VER 1

// Structure to track each unique CAN ID
struct CANIDInfo {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  unsigned long firstSeen;
  unsigned long lastSeen;
  uint32_t rxCount;
  uint32_t txCount;
  bool is_extended;
};

// FRC Robot Controller Heartbeat Data
struct FRCHeartbeat {
  uint8_t matchTime;
  uint16_t matchNumber;
  uint8_t replayNumber;
  bool redAlliance;
  bool enabled;
  bool autonomous;
  bool test;
  bool watchdog;
  uint8_t tournamentType;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  bool valid;
  unsigned long lastUpdate;
};

// Map of CAN IDs to their info
std::map<uint32_t, CANIDInfo> canIDMap;
FRCHeartbeat frcHeartbeat = {0};
unsigned long startTime = 0;
bool can_started = false;

// Device state
volatile uint8_t g_deviceNumber = DEFAULT_DEVICE_NUMBER;
volatile uint16_t g_uptimeSec = 0;

// LED state
enum LEDState {
  LED_BOOTING,
  LED_WIFI_READY,
  LED_CAN_ACTIVE,
  LED_CAN_ERROR
};
volatile LEDState g_ledState = LED_BOOTING;
volatile uint16_t g_ledR_12 = 0, g_ledG_12 = 0, g_ledB_12 = 0;
volatile uint16_t g_ledR_target = 0, g_ledG_target = 0, g_ledB_target = 0;
volatile uint32_t g_lastCANActivity = 0;
volatile uint16_t g_recentMessageCount = 0;

// FRC CAN ID helpers
static inline uint32_t encode_id(uint8_t dt, uint8_t man, uint16_t api, uint8_t dn) {
  return ((uint32_t)dt << 24) | ((uint32_t)man << 16) | ((uint32_t)api << 6) | (dn & 0x3F);
}

static inline void decode_id(uint32_t id, uint8_t &dt, uint8_t &man, uint16_t &api, uint8_t &dn) {
  dt  = (id >> 24) & 0xFF;
  man = (id >> 16) & 0xFF;
  api = (id >> 6)  & 0x3FF;
  dn  = id & 0x3F;
}

static inline uint32_t make_can_id(uint16_t api) {
  return encode_id(DEVICE_ID, MANUFACTURER_ID, api, (uint8_t)g_deviceNumber);
}

// FRC Heartbeat decoder
uint32_t getBits(const uint8_t* payload, uint8_t start, uint8_t length) {
  uint32_t result = 0;
  for (uint8_t i = 0; i < length; i++) {
    uint8_t bitPos = start + i;
    uint8_t byteIdx = bitPos / 8;
    uint8_t bitIdx = 7 - (bitPos % 8);
    if (payload[byteIdx] & (1 << bitIdx)) {
      result |= (1 << (length - 1 - i));
    }
  }
  return result;
}

void decodeFRCHeartbeat(const uint8_t* payload) {
  frcHeartbeat.matchTime = (uint8_t)getBits(payload, 56, 8);
  frcHeartbeat.matchNumber = (uint16_t)getBits(payload, 46, 10);
  frcHeartbeat.replayNumber = (uint8_t)getBits(payload, 40, 6);
  frcHeartbeat.redAlliance = getBits(payload, 39, 1);
  frcHeartbeat.enabled = getBits(payload, 38, 1);
  frcHeartbeat.autonomous = getBits(payload, 37, 1);
  frcHeartbeat.test = getBits(payload, 36, 1);
  frcHeartbeat.watchdog = getBits(payload, 35, 1);
  frcHeartbeat.tournamentType = (uint8_t)getBits(payload, 32, 3);
  frcHeartbeat.year = 2000 + getBits(payload, 26, 6) - 36;
  frcHeartbeat.month = (uint8_t)(getBits(payload, 22, 4) + 1);
  frcHeartbeat.day = (uint8_t)getBits(payload, 17, 5);
  frcHeartbeat.seconds = (uint8_t)getBits(payload, 11, 6);
  frcHeartbeat.minutes = (uint8_t)getBits(payload, 5, 6);
  frcHeartbeat.hours = (uint8_t)getBits(payload, 0, 5);
  frcHeartbeat.valid = true;
  frcHeartbeat.lastUpdate = millis();
}

// EEPROM functions
void EEPROMReadCANID() {
  uint8_t saved = EEPROM.read(0);
  g_deviceNumber = (saved <= 63) ? saved : DEFAULT_DEVICE_NUMBER;
}

void EEPROMSaveCANID() {
  EEPROM.write(0, (uint8_t)g_deviceNumber);
  EEPROM.commit();
}

// LED control functions
void LED_Init() {
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode       = LEDC_MODE;
  tcfg.duty_resolution  = LEDC_RES_BITS;
  tcfg.timer_num        = LEDC_TIMER;
  tcfg.freq_hz          = LEDC_FREQ_HZ;
  tcfg.clk_cfg          = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);

  ledc_channel_config_t chR = {};
  chR.gpio_num   = RED_PIN;
  chR.speed_mode = LEDC_MODE;
  chR.channel    = LEDC_CH_RED;
  chR.intr_type  = LEDC_INTR_DISABLE;
  chR.timer_sel  = LEDC_TIMER;
  chR.duty       = 0;
  chR.hpoint     = 0;
  ledc_channel_config(&chR);

  ledc_channel_config_t chG = chR; 
  chG.gpio_num = GREEN_PIN; 
  chG.channel = LEDC_CH_GREEN;
  ledc_channel_config(&chG);

  ledc_channel_config_t chB = chR; 
  chB.gpio_num = BLUE_PIN;  
  chB.channel = LEDC_CH_BLUE;
  ledc_channel_config(&chB);
}

void LED_SetTarget(uint16_t r, uint16_t g, uint16_t b) {
  g_ledR_target = r;
  g_ledG_target = g;
  g_ledB_target = b;
}

void LED_SetTargetRGB8(uint8_t r8, uint8_t g8, uint8_t b8) {
  LED_SetTarget(((uint16_t)r8) << 4, ((uint16_t)g8) << 4, ((uint16_t)b8) << 4);
}

// CAN functions
bool CAN_Start() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 10;
  g_config.rx_queue_len = 20;
  g_config.clkout_divider = 0;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("[CAN] Driver install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[CAN] Start failed");
    twai_driver_uninstall();
    return false;
  }
  can_started = true;
  Serial.println("[CAN] Started at 1 Mbps");
  return true;
}

void updateCANIDMap(uint32_t id, const uint8_t data[8], int dlc, bool is_tx, bool is_extended) {
  unsigned long now = millis() - startTime;
  
  // Check if this is the FRC heartbeat
  if (id == FRC_HEARTBEAT_ID && dlc == 8) {
    decodeFRCHeartbeat(data);
  }
  
  auto it = canIDMap.find(id);
  if (it == canIDMap.end()) {
    // New ID
    CANIDInfo info;
    info.id = id;
    info.len = dlc;
    info.firstSeen = now;
    info.lastSeen = now;
    info.rxCount = is_tx ? 0 : 1;
    info.txCount = is_tx ? 1 : 0;
    info.is_extended = is_extended;
    for (int i = 0; i < dlc && i < 8; ++i) {
      info.data[i] = data[i];
    }
    canIDMap[id] = info;
  } else {
    // Update existing ID
    it->second.len = dlc;
    it->second.lastSeen = now;
    if (is_tx) {
      it->second.txCount++;
    } else {
      it->second.rxCount++;
    }
    for (int i = 0; i < dlc && i < 8; ++i) {
      it->second.data[i] = data[i];
    }
  }
}

bool CAN_Send(uint32_t id, const uint8_t data[8], int dlc = 8) {
  if (!can_started) return false;
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 1;
  msg.data_length_code = dlc;
  for (int i = 0; i < dlc && i < 8; ++i) msg.data[i] = data[i];
  
  bool success = (twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK);
  
  if (success) {
    g_lastCANActivity = millis();
    g_recentMessageCount++;
    updateCANIDMap(id, data, dlc, true, true);
  }
  
  return success;
}

// LED Animation Task
void TaskLEDControl(void* parameter) {
  Serial.println("[LED] Task started");
  const uint16_t step = 64;
  uint32_t lastPulse = 0;
  uint32_t pulsePhase = 0;
  
  while (true) {
    uint32_t now = millis();
    
    switch (g_ledState) {
      case LED_BOOTING:
        if (now - lastPulse >= 20) {
          lastPulse = now;
          pulsePhase = (pulsePhase + 1) % 100;
          uint16_t brightness = (pulsePhase < 50) ? (pulsePhase * 40) : ((100 - pulsePhase) * 40);
          LED_SetTarget(0, 0, brightness);
        }
        break;
        
      case LED_WIFI_READY:
        LED_SetTargetRGB8(0, 128, 0);
        break;
        
      case LED_CAN_ACTIVE:
        if (now - g_lastCANActivity < 100) {
          uint16_t intensity = 4095 - ((now - g_lastCANActivity) * 40);
          if (intensity > 4095) intensity = 0;
          LED_SetTarget(0, 2048, intensity);
        } else {
          LED_SetTargetRGB8(0, 128, 0);
        }
        break;
        
      case LED_CAN_ERROR:
        if (now - lastPulse >= 30) {
          lastPulse = now;
          pulsePhase = (pulsePhase + 1) % 100;
          uint16_t brightness = (pulsePhase < 50) ? (pulsePhase * 40) : ((100 - pulsePhase) * 40);
          LED_SetTarget(brightness, 0, 0);
        }
        break;
    }
    
    // Smooth fade toward targets
    uint16_t curR = g_ledR_12;
    uint16_t curG = g_ledG_12;
    uint16_t curB = g_ledB_12;
    
    uint16_t tgtR = g_ledR_target;
    uint16_t tgtG = g_ledG_target;
    uint16_t tgtB = g_ledB_target;
    
    if (curR < tgtR) {
      uint32_t next = (uint32_t)curR + step;
      if (next > tgtR) next = tgtR;
      curR = (uint16_t)next;
    } else if (curR > tgtR) {
      uint16_t next = (curR > step) ? (curR - step) : 0;
      if (next < tgtR) next = tgtR;
      curR = next;
    }
    
    if (curG < tgtG) {
      uint32_t next = (uint32_t)curG + step;
      if (next > tgtG) next = tgtG;
      curG = (uint16_t)next;
    } else if (curG > tgtG) {
      uint16_t next = (curG > step) ? (curG - step) : 0;
      if (next < tgtG) next = tgtG;
      curG = next;
    }
    
    if (curB < tgtB) {
      uint32_t next = (uint32_t)curB + step;
      if (next > tgtB) next = tgtB;
      curB = (uint16_t)next;
    } else if (curB > tgtB) {
      uint16_t next = (curB > step) ? (curB - step) : 0;
      if (next < tgtB) next = tgtB;
      curB = next;
    }
    
    g_ledR_12 = curR;
    g_ledG_12 = curG;
    g_ledB_12 = curB;
    
    ledc_set_duty(LEDC_MODE, LEDC_CH_RED, curR);
    ledc_update_duty(LEDC_MODE, LEDC_CH_RED);
    
    ledc_set_duty(LEDC_MODE, LEDC_CH_GREEN, curG);
    ledc_update_duty(LEDC_MODE, LEDC_CH_GREEN);
    
    ledc_set_duty(LEDC_MODE, LEDC_CH_BLUE, curB);
    ledc_update_duty(LEDC_MODE, LEDC_CH_BLUE);
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// CAN Health Monitor Task
void TaskCANHealth(void* parameter) {
  Serial.println("[CANHEALTH] Task started");
  uint32_t lastCheck = 0;
  
  while (true) {
    uint32_t now = millis();
    
    if (now - lastCheck >= 1000) {
      lastCheck = now;
      
      if (!can_started) {
        g_ledState = LED_CAN_ERROR;
      } else {
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
          if (status.state == TWAI_STATE_BUS_OFF || status.state == TWAI_STATE_RECOVERING) {
            g_ledState = LED_CAN_ERROR;
            Serial.println("[CANHEALTH] Bus error detected");
          } else if (g_ledState != LED_CAN_ACTIVE) {
            g_ledState = LED_CAN_ACTIVE;
          }
        }
      }
      
      g_recentMessageCount = 0;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskCANTx(void* parameter) {
  Serial.println("[CANTX] Task started");
  uint32_t lastStatus = 0;
  uint32_t lastHeartbeat = 0;
  uint32_t lastSecTick = millis();

  while (true) {
    uint32_t now = millis();

    if (now - lastSecTick >= 1000) {
      lastSecTick += 1000;
      if (g_uptimeSec < 0xFFFF) g_uptimeSec++;
    }

    if (now - lastStatus >= 500) {
      lastStatus = now;
      uint8_t buf[8] = {0};
      buf[0] = SOFTWARE_VER;
      buf[1] = (uint8_t)(g_uptimeSec & 0xFF);
      buf[2] = (uint8_t)((g_uptimeSec >> 8) & 0xFF);
      uint16_t msgs = (canIDMap.size() > 0xFFFF) ? 0xFFFF : canIDMap.size();
      buf[3] = (uint8_t)(msgs & 0xFF);
      buf[4] = (uint8_t)((msgs >> 8) & 0xFF);
      CAN_Send(make_can_id(API_TX_STATUS), buf, 8);
    }

    if (now - lastHeartbeat >= 100) {
      lastHeartbeat = now;
      uint8_t buf[8] = {0};
      buf[0] = 0xAA;
      buf[1] = (uint8_t)g_deviceNumber;
      CAN_Send(make_can_id(API_TX_HEARTBEAT), buf, 8);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void TaskCANIDHelper(void* parameter) {
  Serial.println("[CANID] Helper task started. Use &CANID SET xx / SAVE / GET");
  while (true) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("&CANID SET ")) {
        int val = line.substring(11).toInt();
        if (val >= 0 && val <= 63) {
          g_deviceNumber = (uint8_t)val;
          Serial.printf("[CANID] Running DEVICE_NUMBER set to %d\n", (int)g_deviceNumber);
        } else {
          Serial.println("[CANID] Invalid value. Must be 0-63.");
        }
      } else if (line.equals("&CANID SAVE")) {
        EEPROMSaveCANID();
        Serial.println("[CANID] Saved to EEPROM. Rebooting...");
        delay(1000);
        ESP.restart();
      } else if (line.equals("&CANID GET")) {
        uint8_t eepromVal = EEPROM.read(0);
        Serial.printf("[CANID] Current=%d, EEPROM=%d, Default=%d\n",
                      (int)g_deviceNumber, (int)eepromVal, (int)DEFAULT_DEVICE_NUMBER);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== adalogger ID-Based Table View ===");
  
  startTime = millis();
  
  LED_Init();
  g_ledState = LED_BOOTING;
  
  EEPROM.begin(EEPROM_BYTES);
  EEPROMReadCANID();
  Serial.printf("[BOOT] DEVICE_NUMBER=%d\n", (int)g_deviceNumber);
  
  if (!CAN_Start()) {
    Serial.println("[BOOT] CAN init failed. Retrying in 3s...");
    g_ledState = LED_CAN_ERROR;
    delay(3000);
    CAN_Start();
  }
  
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("[WIFI] AP IP: ");
  Serial.println(IP);
  
  g_ledState = LED_WIFI_READY;
  delay(500);
  
  if (can_started) {
    g_ledState = LED_CAN_ACTIVE;
  }
  
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/clear", handleClear);
  server.on("/heartbeat", handleHeartbeat);
  
  server.begin();
  Serial.println("[BOOT] Web server started");
  
  xTaskCreatePinnedToCore(TaskLEDControl, "TaskLEDControl", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskCANHealth, "TaskCANHealth", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskCANTx, "TaskCANTx", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskCANIDHelper, "TaskCANIDHelper", 4096, nullptr, 1, nullptr, 1);
  
  Serial.println("[BOOT] Setup complete");
}

void loop() {
  server.handleClient();
  
  if (can_started) {
    twai_message_t message;
    esp_err_t r = twai_receive(&message, pdMS_TO_TICKS(10));
    if (r == ESP_OK) {
      g_lastCANActivity = millis();
      g_recentMessageCount++;
      
      updateCANIDMap(message.identifier, message.data, message.data_length_code, 
                     false, message.extd);
    }
  }
}

void handleRoot() {
  String html = "<html><head><title>adalogger</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:monospace;margin:10px;background:#1a1a1a;color:#e0e0e0;} ";
  html += "table{border-collapse:collapse;width:100%;font-size:12px;margin-bottom:20px;} ";
  html += "th,td{border:1px solid #444;padding:4px;text-align:left;} ";
  html += "th{background:#333;color:white;} ";
  html += "tr:hover{background:#252525;} ";
  html += ".tx{color:#0f0;} .rx{color:#0af;} .both{color:#fa0;}";
  html += ".hb-box{background:#2a2a2a;border:2px solid #444;border-radius:8px;padding:15px;margin:15px 0;} ";
  html += ".hb-title{font-size:16px;font-weight:bold;color:#0af;margin-bottom:10px;} ";
  html += ".hb-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;} ";
  html += ".hb-item{background:#1f1f1f;padding:8px;border-radius:4px;border-left:3px solid #0af;} ";
  html += ".hb-label{color:#888;font-size:10px;margin-bottom:2px;} ";
  html += ".hb-value{color:#fff;font-size:14px;font-weight:bold;} ";
  html += ".enabled{color:#0f0;} .disabled{color:#f44;} ";
  html += ".red-alliance{color:#f44;} .blue-alliance{color:#44f;} ";
  html += "button{background:#0af;color:#fff;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;margin-right:5px;} ";
  html += "button:hover{background:#08d;} ";
  html += ".stale{opacity:0.5;} ";
  html += "</style>";
  html += "<script>";
  html += "function refresh(){fetch('/data').then(r=>r.text()).then(d=>{document.getElementById('log').innerHTML=d;})}";
  html += "function refreshHB(){fetch('/heartbeat').then(r=>r.text()).then(d=>{document.getElementById('hb').innerHTML=d;})}";
  html += "setInterval(refresh,500);";
  html += "setInterval(refreshHB,250);";
  html += "function clear(){if(confirm('Clear all CAN ID data?')){fetch('/clear').then(()=>{document.getElementById('log').innerHTML='';document.getElementById('count').innerText='0';})}}";
  html += "</script></head><body>";
  html += "<h2>adalogger - ID Table View</h2>";
  html += "<p>Device Number: <b>" + String(g_deviceNumber) + "</b> | ";
  html += "Uptime: <b>" + String(g_uptimeSec) + "s</b> | ";
  
  html += "Status: ";
  switch (g_ledState) {
    case LED_BOOTING: html += "🔵 Booting"; break;
    case LED_WIFI_READY: html += "🟢 WiFi Ready"; break;
    case LED_CAN_ACTIVE: html += "🟢 CAN Active"; break;
    case LED_CAN_ERROR: html += "🔴 CAN Error"; break;
  }
  html += "</p>";
  
  html += "<div id='hb'></div>";
  
  html += "<p>Unique IDs: <span id='count'>0</span> | ";
  html += "<button onclick='refresh()'>Refresh</button> ";
  html += "<button onclick='clear()'>Clear</button></p><hr>";
  html += "<div id='log'></div>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleData() {
  String response = "";
  
  response += "<table><tr>";
  response += "<th>CAN ID</th><th>Type</th><th>API Class</th><th>DN</th>";
  response += "<th>Dir</th><th>Count</th><th>First (ms)</th><th>Last (ms)</th>";
  response += "<th>Δt (ms)</th><th>Latest Data</th>";
  response += "</tr>";
  
  uint32_t totalRX = 0, totalTX = 0;
  
  for (auto const& pair : canIDMap) {
    const CANIDInfo& info = pair.second;
    
    response += "<tr>";
    response += "<td>0x" + String(info.id, HEX) + "</td>";
    
    if (info.is_extended) {
      uint8_t deviceType, manufacturer, deviceNumber;
      uint16_t apiClass;
      decode_id(info.id, deviceType, manufacturer, apiClass, deviceNumber);
      
      if (deviceType == DEVICE_ID && manufacturer == MANUFACTURER_ID) {
        response += "<td>FRC</td>";
        response += "<td>" + getAPIClassName(apiClass) + "</td>";
        response += "<td>" + String(deviceNumber) + "</td>";
      } else {
        response += "<td>Ext</td>";
        response += "<td>DT:" + String(deviceType, HEX) + "/MFG:" + String(manufacturer, HEX) + "</td>";
        response += "<td>" + String(deviceNumber) + "</td>";
      }
    } else {
      response += "<td>Std</td><td>-</td><td>-</td>";
    }
    
    // Direction indicator
    response += "<td>";
    if (info.rxCount > 0 && info.txCount > 0) {
      response += "<span class='both'>RX:" + String(info.rxCount) + " TX:" + String(info.txCount) + "</span>";
    } else if (info.txCount > 0) {
      response += "<span class='tx'>TX</span>";
    } else {
      response += "<span class='rx'>RX</span>";
    }
    response += "</td>";
    
    // Message counts
    uint32_t totalCount = info.rxCount + info.txCount;
    response += "<td>" + String(totalCount) + "</td>";
    
    // Timing
    response += "<td>" + String(info.firstSeen) + "</td>";
    response += "<td>" + String(info.lastSeen) + "</td>";
    response += "<td>" + String(info.lastSeen - info.firstSeen) + "</td>";
    
    // Latest data
    response += "<td>";
    for (int j = 0; j < info.len; j++) {
      if (j > 0) response += " ";
      if (info.data[j] < 0x10) response += "0";
      response += String(info.data[j], HEX);
    }
    response += "</td></tr>";
    
    totalRX += info.rxCount;
    totalTX += info.txCount;
  }
  
  response += "</table>";
  response += "<p style='margin-top:10px;'>Total Messages: RX=" + String(totalRX) + " TX=" + String(totalTX) + "</p>";
  response += "<script>document.getElementById('count').innerText=" + String(canIDMap.size()) + ";</script>";
  
  server.send(200, "text/html", response);
}

void handleClear() {
  canIDMap.clear();
  startTime = millis();
  frcHeartbeat.valid = false;
  server.send(200, "text/plain", "OK");
}

void handleHeartbeat() {
  String response = "";
  
  if (!frcHeartbeat.valid) {
    response = "<div class='hb-box'>";
    response += "<div class='hb-title'>FRC Robot Controller Heartbeat</div>";
    response += "<p style='color:#888;'>No heartbeat detected (ID: 0x" + String(FRC_HEARTBEAT_ID, HEX) + ")</p>";
    response += "</div>";
  } else {
    unsigned long age = millis() - frcHeartbeat.lastUpdate;
    bool isStale = age > 2000;
    
    response = "<div class='hb-box" + String(isStale ? " stale" : "") + "'>";
    response += "<div class='hb-title'>FRC Robot Controller Heartbeat</div>";
    
    if (isStale) {
      response += "<p style='color:#f44;'>⚠ Stale (last seen " + String(age/1000.0, 1) + "s ago)</p>";
    }
    
    response += "<div class='hb-grid'>";
    
    // Robot state
    response += "<div class='hb-item'><div class='hb-label'>Robot State</div>";
    response += "<div class='hb-value " + String(frcHeartbeat.enabled ? "enabled" : "disabled") + "'>";
    response += frcHeartbeat.enabled ? "🟢 ENABLED" : "🔴 DISABLED";
    response += "</div></div>";
    
    // Mode
    response += "<div class='hb-item'><div class='hb-label'>Mode</div>";
    response += "<div class='hb-value'>";
    if (frcHeartbeat.test) {
      response += "TEST";
    } else if (frcHeartbeat.autonomous) {
      response += "AUTO";
    } else {
      response += "TELEOP";
    }
    response += "</div></div>";
    
    // Alliance
    response += "<div class='hb-item'><div class='hb-label'>Alliance</div>";
    response += "<div class='hb-value " + String(frcHeartbeat.redAlliance ? "red-alliance" : "blue-alliance") + "'>";
    response += frcHeartbeat.redAlliance ? "🔴 RED" : "🔵 BLUE";
    response += "</div></div>";
    
    // Match time
    response += "<div class='hb-item'><div class='hb-label'>Match Time</div>";
    response += "<div class='hb-value'>" + String(frcHeartbeat.matchTime) + "s</div></div>";
    
    // Match number
    response += "<div class='hb-item'><div class='hb-label'>Match Number</div>";
    response += "<div class='hb-value'>" + String(frcHeartbeat.matchNumber);
    if (frcHeartbeat.replayNumber > 0) {
      response += " (Replay " + String(frcHeartbeat.replayNumber) + ")";
    }
    response += "</div></div>";
    
    // Tournament type
    response += "<div class='hb-item'><div class='hb-label'>Tournament</div>";
    response += "<div class='hb-value'>";
    switch(frcHeartbeat.tournamentType) {
      case 0: response += "None"; break;
      case 1: response += "Practice"; break;
      case 2: response += "Qualification"; break;
      case 3: response += "Playoff"; break;
      default: response += "Type " + String(frcHeartbeat.tournamentType); break;
    }
    response += "</div></div>";
    
    // Watchdog
    response += "<div class='hb-item'><div class='hb-label'>Watchdog</div>";
    response += "<div class='hb-value'>";
    response += frcHeartbeat.watchdog ? "🟢 Active" : "⚪ Inactive";
    response += "</div></div>";
    
    // Date/Time
    response += "<div class='hb-item'><div class='hb-label'>Event Date</div>";
    response += "<div class='hb-value'>";
    response += String(frcHeartbeat.year) + "-";
    if (frcHeartbeat.month < 10) response += "0";
    response += String(frcHeartbeat.month) + "-";
    if (frcHeartbeat.day < 10) response += "0";
    response += String(frcHeartbeat.day);
    response += "</div></div>";
    
    response += "<div class='hb-item'><div class='hb-label'>Event Time</div>";
    response += "<div class='hb-value'>";
    if (frcHeartbeat.hours < 10) response += "0";
    response += String(frcHeartbeat.hours) + ":";
    if (frcHeartbeat.minutes < 10) response += "0";
    response += String(frcHeartbeat.minutes) + ":";
    if (frcHeartbeat.seconds < 10) response += "0";
    response += String(frcHeartbeat.seconds);
    response += "</div></div>";
    
    response += "</div></div>";
  }
  
  server.send(200, "text/html", response);
}

String getAPIClassName(uint16_t apiClass) {
  switch(apiClass) {
    case API_CLASS_ROBOT_CONTROL: return "RobotCtrl";
    case API_CLASS_MOTOR_CONTROL: return "Motor";
    case API_CLASS_RELAY_CONTROL: return "Relay";
    case API_CLASS_GYRO_SENSOR: return "Gyro";
    case API_CLASS_ACCEL_SENSOR: return "Accel";
    case API_CLASS_ULTRASONIC: return "Ultrasonic";
    case API_CLASS_GEAR_TOOTH: return "GearTooth";
    case API_CLASS_MISC_SENSOR: return "Sensor";
    case API_CLASS_IO_CONFIG: return "IOConfig";
    case API_CLASS_POWER: return "Power";
    case API_CLASS_FIRMWARE: return "Firmware";
    case 0x185: return "RX_CONTROL";
    case 0x186: return "RX_STATUS";
    case 0x195: return "TX_INPUTS";
    case 0x196: return "TX_RESET";
    case API_TX_STATUS: return "LOGGER_STATUS";
    case API_TX_HEARTBEAT: return "LOGGER_HB";
    default: return "0x" + String(apiClass, HEX);
  }
}