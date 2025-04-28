// Arduino GUI Node for CAN Bus Monitoring
// Features: touchscreen UI, alerts, WiFi upload (commented out, will need a wifi connection and to edit the ssid and password ), actuator control with feedback monitoring
// Author: Khalid Suliman

#include <Arduino_CAN.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <TouchScreen.h>
#include <vector>
#include <EEPROM.h>
#include <WiFiSSLClient.h>
#include <WiFi.h>

//UNCOMMENT FOR SHEETS AND WIFI UPLOADS
// // === WiFi Credentials ===
// const char* ssid = "iPhonekaks";
// const char* password = "123456779";

// const char* scriptURL = "https://script.google.com/macros/s/AKfycbzYbzYpoJK3oHEQxWHIXSA2JJ7tC468Ue1eb3Btz86lovDuvqpJGFBu7Twa6Vct24DF9g/exec";

// TFT SPI Pins
#define TFT_CS    6
#define TFT_DC    7
#define TFT_RST   8
#define TFT_MOSI  11
#define TFT_CLK   5
//software spi
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST);

//highest alert active level
enum AlertLevel { ALERT_NONE, ALERT_YELLOW, ALERT_RED };
AlertLevel currentAlertLevel = ALERT_NONE;
AlertLevel lastAlertLevel = ALERT_NONE;

// Touchscreen pins
#define YP A3
#define XM A2
#define YM 2
#define XP 9
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

//calibration of screen
#define TS_MINX 200
#define TS_MAXX 860
#define TS_MINY 140
#define TS_MAXY 875

//min and max pressures for touch
#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define BTN_W 60
#define BTN_H 40

#define BUZZER_PIN 3
// CAN IDs
#define CONTROLLER_ID 0x00
const uint32_t DHT22_ID = 0x20;
const uint32_t BMP280_ID = 0x21;
const uint32_t CONTROL_CMD_ID = 0x10;
const uint32_t STATUS_ACK_ID = 0x31;
const uint32_t STATUS_REQUEST_ID = 0x11;
const uint32_t HEARTBEAT_REQUEST_ID = 0x12;
const uint32_t HEARTBEAT_REPLY_ID   = 0x13;
const uint32_t ALERT_ID = 0x09;
const uint32_t AUTO_CONFIG_ID = 0x40;
const uint32_t FEEDBACK_ID = 0x35;

//config values defaults
bool configScreen = false;
float configTempThreshold = 22.0;
float configHumThreshold = 26.0;

const int FEEDBACK_RESOLVE_THRESHOLD = 5;
int dhtValidUpdates = 0;
int bmpValidUpdates = 0;
//last displayed sensor vals used to optimise redraws
float lastTempIn = -999.0;
float lastHumIn = -999.0;
float lastTempOut = -999.0;
float lastPresOut = -999.0;

//used to stage sensor vals when in alerts/config screens
float stagedTempIn = -999.0;
float stagedHumIn = -999.0;
float stagedTempOut = -999.0;
float stagedPresOut = -999.0;

//used for wifi uploading
float uploadTempIn = -999.0;
float uploadHumIn  = -999.0;
float uploadTempOut = -999.0;
float uploadPresOut = -999.0;

//flags indicating new or updated sensor vals
bool newTempIn = false;
bool newHumIn = false;
bool newTempOut = false;
bool newPresOut = false;

//last known toggle states
bool lastFan = false;
bool lastVent = false;
bool lastAuto = false;

bool alertLayoutChanged = true; //redraw alert popup
bool senderOfflineFlag = false;

unsigned long lastAlertsButtonUpdate = 0;
const unsigned long ALERT_BUTTON_UPDATE_INTERVAL = 500;  
bool alert200ReceivedSinceBoot = false;
unsigned long bootTime = 0;

// States
bool fan = false, vent = false, autoMode = false, alarmPopup = false;
unsigned long lastDHTTime = 0, lastBMPTime = 0;
unsigned long alarmPopupOpenedAt = 0;
unsigned long lastHeartbeatReplyTime = 0;
unsigned long lastHeartbeatSent = 0;

//alerts page, used for pagination etc
int alertPage = 0;
const int ALERTS_PER_PAGE = 5; 

unsigned long lastCmdSentAt = 0;
bool awaitingAck = false;
uint8_t lastCmdState[3];  // fan, vent, auto

unsigned long buzzerStartTime = 0;
bool buzzerActive = false;
const unsigned long BUZZ_DURATION = 5000;
#define ALERT_MAX 10
#define ALERT_UPDATE_INTERVAL 10000  // 10 seconds minimum


struct Alert {
  String msgPresent;
  String msgPast;
  unsigned long triggeredAt;
  bool active;   // true = red (ongoing), false = yellow (resolved)
  uint8_t type;  // custom type so no duplicates
};

std::vector<Alert> alerts;
std::vector<uint8_t> suppressedResolvedTypes;
unsigned long lastAlertUpdate = 0;

//feedback
#define ALERT_CONTROL_MISMATCH_VENT 200
bool ventFeedbackAlertActive = false;
float lastVentFeedbackVoltage = -1.0;

// Types
#define ALERT_SENDER_OFFLINE 100
#define ALERT_DHT_TIMEOUT 1
#define ALERT_BMP_TIMEOUT 2
#define ALERT_HIGH_TEMP_IN 3
#define ALERT_HIGH_TEMP_OUT 4

//not used anymoer as handled already by others, some can be  implemented in future as 3rd node helps to identify point of failure, not doable for 2 nodes
//#define ALERT_CAN_WRITE_FAIL 5
//#define ALERT_CONTROL_SEND_FAIL 103
//#define ALERT_ACK_TIMEOUT       104
//#define ALERT_CAN_WRITE_FREEZE  105
//#define ALERT_GUI_TRANSMIT_FAIL 106

bool isAlertActive(uint8_t type) {
  for (Alert& a : alerts) {
    if (a.type == type && a.active) return true;
  }
  return false;
}

//toggles
void drawButton(int x, int y, const char* label, bool state, bool disabled = false) {
  uint16_t colour = disabled ? ILI9341_DARKGREY : (state ? ILI9341_GREEN : ILI9341_RED);
  tft.fillRoundRect(x, y, BTN_W, BTN_H, 8, colour);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
  tft.setCursor(x + 10, y + 12); tft.print(label);
}

//buzzer trigers when red alert occurs
void triggerBuzzer() {
  buzzerStartTime = millis();
  buzzerActive = true;
  digitalWrite(BUZZER_PIN, HIGH);
  Serial.println("BUZZER");
}

void setup() {
  Serial.begin(115200);
  //uncomment for WIFI CONNECTION AND SHEETS UPLOAD
  // Serial.print("Connecting to WiFi...");
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("Connected!");
  // Serial.print(" IP: ");
  // Serial.println(WiFi.localIP());
  delay(500);
  delay(1000);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  bootTime = millis();
  uint16_t temp10 = (EEPROM.read(0) << 8) | EEPROM.read(1);
  uint16_t hum10  = (EEPROM.read(2) << 8) | EEPROM.read(3);
  configTempThreshold = temp10 / 10.0;
  configHumThreshold  = hum10 / 10.0;
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);

  drawUI();

  // Redraw after UI init
  drawButton(20, 240, "FAN", fan);
  drawButton(90, 240, "VENT", vent);
  drawButton(160, 240, "AUTO", autoMode);
  lastTempIn = lastHumIn = lastTempOut = lastPresOut = -999;

  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("CAN Init Failed!");
    while (1);
  }

  // Request current toggle states on start
  CanMsg request(CanStandardId(STATUS_REQUEST_ID), 0, nullptr);
  CAN.write(request);

}

void loop() {
 
  checkTouch();
  receiveCAN();
  checkTimeouts();
  updateAlertsButtonIfNeeded();
  //tryUploadToSheets();
  if (millis() - lastHeartbeatSent > 20000) {
  CanMsg hb(CanStandardId(HEARTBEAT_REQUEST_ID), 0, nullptr);
  CAN.write(hb);
  Serial.println("Sent heartbeat request");
  lastHeartbeatSent = millis();
  }
  if (buzzerActive && millis() - buzzerStartTime >= BUZZ_DURATION) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }
}

// ---------------- UI ----------------
void drawUI() {
  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0, 0, 240, 40, ILI9341_CYAN);
  tft.setTextColor(ILI9341_BLACK); tft.setTextSize(2);
  tft.setCursor(10, 10); tft.print("DATA CENTRE");

  drawAlertsButton(getAlertLevel());
  tft.fillRect(20, 190, 200, 40, ILI9341_BLUE);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
  tft.setCursor(55, 200); tft.print("Auto Config");

  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);  
  updateSensorText();
  drawToggles();
}

void updateSensorText() {
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
  tft.setCursor(10, 50); tft.print("In Temp : ");
  tft.setCursor(10, 75); tft.print("Out Temp: ");
  tft.setCursor(10, 100); tft.print("In Hum  : ");
  tft.setCursor(10, 125); tft.print("Out Pres: ");
  tft.setCursor(10, 150); tft.print("Fan Curr: 0.00A");
}

void drawToggles() {
  bool disableFanVent = autoMode;
  drawButton(20, 240, "FAN", fan, disableFanVent);
  drawButton(90, 240, "VENT", vent, disableFanVent);
  drawButton(160, 240, "AUTO", autoMode, false);
}

// ---------------- TOUCH ----------------
void checkTouch() {
  TSPoint p = ts.getPoint();
  pinMode(XM, OUTPUT); pinMode(YP, OUTPUT);
  if (millis() - bootTime < 1000) return;
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
    int x = map(p.x, TS_MINX, TS_MAXX, 0, 240);
    int y = map(p.y, TS_MINY, TS_MAXY, 0, 320);

    // -------- PICK AUTO VALUES SCREEN --------
    if (configScreen && millis() - alarmPopupOpenedAt > 300) {

      // TEMP +
      if (x > 30 && x < 70 && y > 65 && y < 95) configTempThreshold += 1;
      // TEMP -
      if (x > 30 && x < 70 && y > 135 && y < 165) configTempThreshold -= 1;
      // HUM +
      if (x > 145 && x < 185 && y > 65 && y < 95) configHumThreshold += 1;
      // HUM -
      if (x > 145 && x < 185 && y > 135 && y < 165) configHumThreshold -= 1;

      configTempThreshold = constrain(configTempThreshold, 15.0, 35.0);
      configHumThreshold  = constrain(configHumThreshold, 20.0, 60.0);

      updateThresholdValuesOnly();

      // Save
      if (x > 20 && x < 110 && y > 190 && y < 220) {
        uint16_t temp10 = round(configTempThreshold * 10);
        uint16_t hum10  = round(configHumThreshold * 10);
        uint8_t data[4] = { temp10 >> 8, temp10 & 0xFF, hum10 >> 8, hum10 & 0xFF };
        CanMsg msg(CanStandardId(AUTO_CONFIG_ID), 4, data);
        CAN.write(msg);
        EEPROM.update(0, temp10 >> 8);
        EEPROM.update(1, temp10 & 0xFF);
        EEPROM.update(2, hum10 >> 8);
        EEPROM.update(3, hum10 & 0xFF);
        Serial.println("Sent Auto thresholds");

        configScreen = false;
        drawUI();

        // Force sensor rerender
        lastTempIn = lastHumIn = lastTempOut = lastPresOut = -999;

        if (stagedTempIn != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 50, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 50); tft.print(stagedTempIn, 1); tft.print("C");
          lastTempIn = stagedTempIn;
        }
        if (stagedHumIn != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 100, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 100); tft.print(stagedHumIn, 1); tft.print("%");
          lastHumIn = stagedHumIn;
        }
        if (stagedTempOut != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 75, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 75); tft.print(stagedTempOut, 1); tft.print("C");
          lastTempOut = stagedTempOut;
        }
        if (stagedPresOut != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 125, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 125); tft.print(stagedPresOut, 1); tft.print("hPa");
          lastPresOut = stagedPresOut;
        }

        return;

      }

      // Back
      if (x > 130 && x < 220 && y > 190 && y < 220) {
        configScreen = false;
        drawUI();

        // Force sensor rerender
        lastTempIn = lastHumIn = lastTempOut = lastPresOut = -999;

        if (stagedTempIn != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 50, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 50); tft.print(stagedTempIn, 1); tft.print("C");
          lastTempIn = stagedTempIn;
        }
        if (stagedHumIn != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 100, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 100); tft.print(stagedHumIn, 1); tft.print("%");
          lastHumIn = stagedHumIn;
        }
        if (stagedTempOut != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 75, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 75); tft.print(stagedTempOut, 1); tft.print("C");
          lastTempOut = stagedTempOut;
        }
        if (stagedPresOut != -999.0) {
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.fillRect(130, 125, 80, 20, ILI9341_BLACK);
          tft.setCursor(130, 125); tft.print(stagedPresOut, 1); tft.print("hPa");
          lastPresOut = stagedPresOut;
        }

        return;
      }return;}
//check in main screen
    if (!alarmPopup && !configScreen) {
      if (x > 20 && x < 20 + BTN_W && y > 240 && y < 240 + BTN_H && !autoMode) {
        fan = !fan; sendControl();
        drawToggles();  // Redraw all buttons together
      }
      if (x > 90 && x < 90 + BTN_W && y > 240 && y < 240 + BTN_H && !autoMode && !isAlertActive(ALERT_CONTROL_MISMATCH_VENT)) {
        vent = !vent; sendControl();
        drawToggles();
      }
      if (x > 160 && x < 160 + BTN_W && y > 240 && y < 240 + BTN_H) {
        autoMode = !autoMode; sendControl();
        drawToggles();
      }
      if (x > 160 && x < 240 && y > 0 && y < 40) {
        alarmPopup = true;
        alarmPopupOpenedAt = millis();
        showAlarmPopup();
      }
      if (x > 20 && x < 220 && y > 190 && y < 230) {
        configScreen = true;
        drawConfigScreen();
        return;
      }
    } else {
        if (millis() - alarmPopupOpenedAt > 500) {
          int x = map(p.x, TS_MINX, TS_MAXX, 0, 240);
          int y = map(p.y, TS_MINY, TS_MAXY, 0, 320);

          if (x > 10 && x < 110 && y > 270 && y < 300) {
            for (auto it = alerts.begin(); it != alerts.end(); ) {
              if (!it->active) {
                suppressedResolvedTypes.push_back(it->type);
                it = alerts.erase(it);
                alertLayoutChanged = true;
              } else {
                ++it;
              }
            }

            alertLayoutChanged = true;
            showAlarmPopup();
            return;
          }

          // Next Page
          if (alerts.size() > ALERTS_PER_PAGE && x > 130 && x < 230 && y > 270 && y < 295) {
            alertPage = (alertPage + 1) % ((alerts.size() + ALERTS_PER_PAGE - 1) / ALERTS_PER_PAGE);
            alertLayoutChanged = true;
            showAlarmPopup();
            return;
          }

          // Prev Page
          if (alerts.size() > ALERTS_PER_PAGE && x > 130 && x < 230 && y > 240 && y < 265) {
            int pageCount = (alerts.size() + ALERTS_PER_PAGE - 1) / ALERTS_PER_PAGE;
            alertPage = (alertPage - 1 + pageCount) % pageCount;
            alertLayoutChanged = true;
            showAlarmPopup();
            return;
          }

          alarmPopup = false;
          alertPage = 0;
          drawUI();
          drawToggles(); 
          delay(100);
          lastTempIn = lastHumIn = lastTempOut = lastPresOut = -999;

          if (newTempIn) {
            tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
            tft.fillRect(130, 50, 80, 20, ILI9341_BLACK);
            tft.setCursor(130, 50); tft.print(stagedTempIn, 1); tft.print("C");
            lastTempIn = stagedTempIn; newTempIn = false;
          }
          if (newHumIn) {
            tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
            tft.fillRect(130, 100, 80, 20, ILI9341_BLACK);
            tft.setCursor(130, 100); tft.print(stagedHumIn, 1); tft.print("%");
            lastHumIn = stagedHumIn; newHumIn = false;
          }
          if (newTempOut) {
            tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
            tft.fillRect(130, 75, 80, 20, ILI9341_BLACK);
            tft.setCursor(130, 75); tft.print(stagedTempOut, 1); tft.print("C");
            lastTempOut = stagedTempOut; newTempOut = false;
          }
          if (newPresOut) {
            tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
            tft.fillRect(130, 125, 80, 20, ILI9341_BLACK);
            tft.setCursor(130, 125); tft.print(stagedPresOut, 1); tft.print("hPa");
            lastPresOut = stagedPresOut; newPresOut = false;
          }
        }
      }

    delay(300);
  }
}

// ---------------- CAN ----------------
void sendControl() {
  uint8_t data[4] = { CONTROLLER_ID, fan, vent, autoMode };
  CanMsg cmd(CanStandardId(CONTROL_CMD_ID), sizeof(data), data);

  if (CAN.write(cmd) >= 0) {
    lastCmdState[0] = fan;
    lastCmdState[1] = vent;
    lastCmdState[2] = autoMode;
    awaitingAck = true;
    lastCmdSentAt = millis();
  } else {
    Serial.println("CAN write failed for control");

    // Revert toggle state
    fan = lastCmdState[0];
    vent = lastCmdState[1];
    autoMode = lastCmdState[2];
    drawToggles();

    // No alert silent failure
  }
}



void receiveCAN() {
  while (CAN.available()) {
    CanMsg msg = CAN.read();

    if (msg.id == ALERT_ID && msg.data_length == 3) {
      uint8_t nodeID = msg.data[0];
      uint8_t type = msg.data[1];
      bool isActive = msg.data[2];
        if (type == ALERT_CONTROL_MISMATCH_VENT) {
          bool wasPreviouslyActive = isAlertActive(type);
          setAlert(type,
                  "VENT mismatch (no response)",
                  "Previous Vent mismatch", isActive);

          // Revert even if alert was already active but first time received
          if (isActive && (!wasPreviouslyActive || !alert200ReceivedSinceBoot)) {
            alert200ReceivedSinceBoot = true;

            bool expected = lastCmdState[1];
            vent = !expected;
            lastCmdState[1] = vent;
            Serial.println("ALERT 200 active → forcing VENT mismatch display");
            if (!alarmPopup && !configScreen) drawToggles();
          }

          // Resolve logic unchanged
          if (!isActive && wasPreviouslyActive) {
            vent = lastCmdState[1] = !vent;
            Serial.println("ALERT 200 resolved → restoring VENT toggle");
            if (!alarmPopup && !configScreen) drawToggles();
          }
        }

        if (type == ALERT_HIGH_TEMP_IN) {
          setAlert(type,
                  "Inside Temp too high!",
                  "Inside Temp was high", isActive);
        }

        if (type == ALERT_HIGH_TEMP_OUT) {
          setAlert(type,
                  "Outside Temp too high!",
                  "Outside Temp was high", isActive);
        }
    }


    if (msg.id == STATUS_ACK_ID && msg.data_length == 3) {
      awaitingAck = false; 
      bool newFan = msg.data[0];
      bool newVent = msg.data[1];
      bool newAuto = msg.data[2];

      if (!alarmPopup && !configScreen) {
        fan = newFan;

        // Defer vent update if alert 200 active
        if (!isAlertActive(ALERT_CONTROL_MISMATCH_VENT)) {
          vent = newVent;
          lastCmdState[1] = newVent;
        } else {
          // Just update lastCmdState so revert still works
          lastCmdState[1] = newVent;
        }

        autoMode = newAuto;

        lastFan = newFan;
        lastVent = newVent;
        lastAuto = newAuto;

        lastCmdState[0] = fan;
        lastCmdState[1] = vent;
        lastCmdState[2] = autoMode;

        drawToggles();
      }
      Serial.println("Sync received");
    }

    //handle indoor sensor
    if (msg.id == DHT22_ID && msg.data_length == 6) {
      uint8_t nodeID = msg.data[0];
      float temp = (int16_t)((msg.data[2] << 8) | msg.data[3]) / 10.0;
      float hum = ((msg.data[4] << 8) | msg.data[5]) / 10.0;
      uploadTempIn = temp;
      uploadHumIn  = hum;
      lastDHTTime = millis();

      if (alarmPopup) {
        stagedTempIn = temp; stagedHumIn = hum;
        newTempIn = true; newHumIn = true;
      } else if (!configScreen) {
        if (temp != lastTempIn) {
          tft.fillRect(130, 50, 80, 20, ILI9341_BLACK);
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.setCursor(130, 50); tft.print(temp, 1); tft.print("C");
          lastTempIn = temp;
        }
        if (hum != lastHumIn) {
          tft.fillRect(130, 100, 80, 20, ILI9341_BLACK);
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.setCursor(130, 100); tft.print(hum, 1); tft.print("%");
          lastHumIn = hum;
        }
      }
      dhtValidUpdates++;
    }

    //handle outdoor sensor
    if (msg.id == BMP280_ID && msg.data_length == 6) {
      uint8_t nodeID = msg.data[0];
      float temp = (int16_t)((msg.data[2] << 8) | msg.data[3]) / 10.0;
      float pres = ((msg.data[4] << 8) | msg.data[5]) / 10.0;
      uploadTempOut = temp;
      lastBMPTime = millis();

      if (alarmPopup) {
        stagedTempOut = temp; stagedPresOut = pres;
        newTempOut = true; newPresOut = true;
      } else if (!configScreen) {
        if (temp != lastTempOut) {
          tft.fillRect(130, 75, 80, 20, ILI9341_BLACK);
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.setCursor(130, 75); tft.print(temp, 1); tft.print("C");
          lastTempOut = temp;
        }
        if (pres != lastPresOut) {
          tft.fillRect(130, 125, 80, 20, ILI9341_BLACK);
          tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
          tft.setCursor(130, 125); tft.print(pres, 1); tft.print("hPa");
          lastPresOut = pres;
        }
      }
      bmpValidUpdates++;
    }

  //voltage from vent, can be extended in future to monitor fan current as nice addition
    if (msg.id == FEEDBACK_ID && msg.data_length == 2) {
      uint16_t raw = (msg.data[0] << 8) | msg.data[1];
      lastVentFeedbackVoltage = raw / 100.0;  // for example 456 → 4.56 V
      Serial.print("Vent feedback received: ");
      Serial.println(lastVentFeedbackVoltage);
    }

  //send heartbeats and can be extended to check for multiple node responses
    if (msg.id == HEARTBEAT_REPLY_ID && msg.data_length == 2) {
      uint8_t nodeID = msg.data[0];
      uint8_t seq = msg.data[1];

      static int lastSeq = -1;
      if (lastSeq >= 0 && seq != (lastSeq + 1) % 256) {
        int missed = (256 + seq - lastSeq - 1) % 256;
        Serial.print(" Missed ");
        Serial.print(missed); Serial.println(" heartbeat(s)");
      }
      lastSeq = seq;

      lastHeartbeatReplyTime = millis();
      Serial.print("Heartbeat reply from node ");
      Serial.print(nodeID);
      Serial.print(" (seq ");
      Serial.print(seq);
      Serial.println(")");
    }
  }
}

unsigned long lastUpload = 0;

//UNCOMMENT FOR SHEETS AND WIFI UPLOADS
// void tryUploadToSheets() {
//   unsigned long now = millis();
//   if (now - lastUpload < 15UL * 60UL * 1000UL) return;  // 15 min interval
////   //if (now - lastUpload < 30UL * 1000UL) return;  // 30 second interval for testing 

//   float tempIn = stagedTempIn;
//   float humIn = stagedHumIn;
//   float tempOut = stagedTempOut;
//   float presOut = stagedPresOut;
//   String alertStr= "";

//   for (Alert& a : alerts) {
//     if (a.active) alertStr += String(a.type) + ",";
//   }

//   String postData = "tempIn=" + String(uploadTempIn, 1) +
//                     "&humIn=" + String(uploadHumIn, 1) +
//                     "&tempOut=" + String(uploadTempOut, 1) +
//                     "&fan=" + String(fan) +
//                     "&vent=" + String(vent) +
//                     "&auto=" + String(autoMode) +
//                     "&tThreshold=" + String(configTempThreshold, 1) +
//                     "&hThreshold=" + String(configHumThreshold, 1) +
//                     "&alerts=" + alertStr +
//                     "&voltage=" + String(lastVentFeedbackVoltage, 2);

//   WiFiSSLClient client;
//   //client.setInsecure();  //  skip certificate check

//   Serial.println("Uploading to Google Sheets...");
//   if (!client.connect("script.google.com", 443)) {
//     Serial.println(" Connection failed!");
//     return;
//   }

//   // Send POST request
//   client.println("POST " + String(scriptURL) + " HTTP/1.1");
//   client.println("Host: script.google.com");
//   client.println("Content-Type: application/x-www-form-urlencoded");
//   client.print("Content-Length: ");
//   client.println(postData.length());
//   client.println();  // end headers
//   client.print(postData);

//   // Wait for response
//   while (client.connected()) {
//     String line = client.readStringUntil('\n');
//     if (line == "\r") break;
//   }

//   String result = client.readStringUntil('\n');
//   Serial.print("Response: ");
//   Serial.println(result);

//   client.stop();
//   lastUpload = millis();
// }

//dynamic timestamps for alerts
String getTimeAgo(unsigned long timestamp) {
  unsigned long seconds = (millis() - timestamp) / 1000;

  if (seconds < 60) return String(seconds) + "s ago";
  if (seconds < 1800) return String(seconds / 60) + "m ago";
  if (seconds < 3600) return String(seconds / 60) + "m ago";
  return String(seconds / 3600) + "h ago";
}

void checkTimeouts() {
  unsigned long now = millis();
  bool allowTimeoutChecks = now > 30000;

  bool dhtTimeout = allowTimeoutChecks && (now - lastDHTTime > 60000);
  bool bmpTimeout = allowTimeoutChecks && (now - lastBMPTime > 60000);
  bool noSensors = dhtTimeout && bmpTimeout;
  bool noHB = (now - lastHeartbeatReplyTime > 30000);

  bool overTimeout = (now - lastHeartbeatReplyTime > 60000);  // Wait full 60s before allowing 106
  bool dhtStillFresh = (now - lastDHTTime < 70000);
  bool bmpStillFresh = (now - lastBMPTime < 70000);
  bool sensorsStillFresh = dhtStillFresh || bmpStillFresh;
  bool enoughValidSensorUpdates = (dhtValidUpdates >= 2 || bmpValidUpdates >= 2);

  senderOfflineFlag = false;

  // ---------- CASE 1: FULL DISCONNECTION ----------
  if (noSensors && noHB) {
    setAlert(ALERT_SENDER_OFFLINE, "CAN ISSUE, SENDER OR GUI NOT ON BUS",
      "SENDER OR GUI WAS DISCONNECTED", true);
    senderOfflineFlag = true;

    setAlert(ALERT_DHT_TIMEOUT, "", "", false);
    setAlert(ALERT_BMP_TIMEOUT, "", "", false);

    for (auto it = alerts.begin(); it != alerts.end();) {
      if (!it->active && (it->type == ALERT_DHT_TIMEOUT || it->type == ALERT_BMP_TIMEOUT)) {
        it = alerts.erase(it);
        alertLayoutChanged = true;
      } else {
        ++it;
      }
    }
  }

  // CASE 2: GUI not sending heartbeats (but sender alive) 
  else if (noHB && overTimeout && sensorsStillFresh && enoughValidSensorUpdates && !isAlertActive(ALERT_SENDER_OFFLINE)) {
    setAlert(ALERT_SENDER_OFFLINE, "", "", false);
  }

  // CASE 3: Not enough time has passed or sensors not valid yet
  else if (noHB) {
    setAlert(ALERT_SENDER_OFFLINE, "", "", false);
  }


  // CASE 4: Everything normal (maybe just individual sensor failures) -
  else {
    setAlert(ALERT_SENDER_OFFLINE, "", "", false);
    if (!senderOfflineFlag) {
      setAlert(ALERT_DHT_TIMEOUT,
               "DHT22 sensor not updating", "DHT22 had no update", dhtTimeout);
      setAlert(ALERT_BMP_TIMEOUT,
               "BMP280 sensor not updating", "BMP280 had no update", bmpTimeout);
    }
  }

  //  ACK Timeout 
  //bool guiTransmitOkay = (now - lastHeartbeatReplyTime < 30000) && !isAlertActive(ALERT_GUI_TRANSMIT_FAIL);


  //  Alert Popup Refresh 
  if (alarmPopup && now - lastAlertUpdate > ALERT_UPDATE_INTERVAL) {
    if (alertLayoutChanged) {
      showAlarmPopup();
      alertLayoutChanged = false;
    } else {
      updateAlertTimestamps();
    }
    lastAlertUpdate = now;
  }
}

void setAlert(uint8_t type, String present, String past, bool isActive) {
  // If already in alerts list
  for (Alert& a : alerts) {
    if (a.type == type) {
      if (a.active != isActive) {
        a.active = isActive;
        a.triggeredAt = millis();
        alertLayoutChanged = true;

        // Buzz only if new and red 
        if (isActive && (
              type == ALERT_SENDER_OFFLINE ||
              type == ALERT_CONTROL_MISMATCH_VENT ||
              type == ALERT_HIGH_TEMP_IN ||
              type == ALERT_HIGH_TEMP_OUT || type == ALERT_DHT_TIMEOUT || type == ALERT_BMP_TIMEOUT)){
          triggerBuzzer();
        }
      }
      return;
    }
  }

  // If resolved and previously cleared manually dont re add
  if (!isActive) {
    for (uint8_t t : suppressedResolvedTypes) {
      if (t == type) return;
    }
  }

  // Add new alert
  if (isActive && alerts.size() < ALERT_MAX) {
    alerts.push_back({ present, past, millis(), isActive, type });
    alertLayoutChanged = true;

    //  buzz for new red alerts
    if (type == ALERT_SENDER_OFFLINE ||
        type == ALERT_CONTROL_MISMATCH_VENT ||
        type == ALERT_HIGH_TEMP_IN ||
        type == ALERT_HIGH_TEMP_OUT || type == ALERT_DHT_TIMEOUT || type == ALERT_BMP_TIMEOUT) {
      triggerBuzzer();
    }
  }
}

void showAlarmPopup() {
  tft.fillScreen(ILI9341_DARKGREY);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
  tft.setCursor(10, 10); tft.print("ALERTS");

  tft.setTextSize(1);
  int rowHeight = 45;
  int y = 40;
  int startIdx = alertPage * ALERTS_PER_PAGE;
  int endIdx = min((int)alerts.size(), startIdx + ALERTS_PER_PAGE);

  for (int i = startIdx; i < endIdx; i++) {
    Alert& a = alerts[i];
    if (a.msgPresent == "" && a.msgPast == "") continue;

    uint16_t color = a.active ? ILI9341_RED : ILI9341_YELLOW;
    tft.fillRect(10, y, 220, rowHeight - 5, ILI9341_BLACK);

    // Message
    tft.setTextColor(color);
    tft.setCursor(12, y + 3); 
    tft.print(a.active ? a.msgPresent : a.msgPast);

    // Timestamp (on next line)
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(12, y + 15);
    tft.print(getTimeAgo(a.triggeredAt));

    y += rowHeight;
  }

  bool hasYellow = false;
for (Alert& a : alerts) {
  if (!a.active) {
    hasYellow = true;
    break;
  }
}

if (hasYellow) {
    tft.fillRect(10, 270, 100, 30, ILI9341_RED);
    tft.setTextColor(ILI9341_WHITE); 
    tft.setCursor(20, 280); 
    tft.print("[ Clear All ]");
  } else {
    // Clear the area where button would be (in case it was drawn previously)
    tft.fillRect(10, 270, 100, 30, ILI9341_DARKGREY);
  }

  // Prev & Next buttons (right side)
  if (alerts.size() > ALERTS_PER_PAGE) {
    tft.fillRect(130, 240, 100, 25, ILI9341_ORANGE);
    tft.setCursor(140, 247); tft.print("[ Prev Page ]");

    tft.fillRect(130, 270, 100, 25, ILI9341_BLUE);
    tft.setCursor(140, 277); tft.print("[ Next Page ]");
  }
}

void updateAlertTimestamps() {
  int rowHeight = 45;
  int y = 40;
  int startIdx = alertPage * ALERTS_PER_PAGE;
  int endIdx = min((int)alerts.size(), startIdx + ALERTS_PER_PAGE);

  for (int i = startIdx; i < endIdx; i++) {
    tft.fillRect(12, y + 15, 150, 10, ILI9341_BLACK); // Clear timestamp area only
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(12, y + 15);
    tft.print(getTimeAgo(alerts[i].triggeredAt));
    y += rowHeight;
  }
}

AlertLevel getAlertLevel() {
  bool hasRed = false;
  bool hasYellow = false;

  for (Alert& a : alerts) {
    if (a.active) hasRed = true;
    else hasYellow = true;
  }

  if (hasRed) return ALERT_RED;
  if (hasYellow) return ALERT_YELLOW;
  return ALERT_NONE;
}

void updateAlertsButtonIfNeeded() {
  if (alarmPopup || configScreen) return;
  if (millis() - lastAlertsButtonUpdate < ALERT_BUTTON_UPDATE_INTERVAL) return;

  currentAlertLevel = getAlertLevel();
  if (currentAlertLevel != lastAlertLevel) {
    drawAlertsButton(currentAlertLevel);
    lastAlertLevel = currentAlertLevel;
  }

  lastAlertsButtonUpdate = millis();
}

void drawAlertsButton(AlertLevel level) {
  // Choose button background color based on alert level
  uint16_t color = (level == ALERT_RED) ? ILI9341_RED :
                   (level == ALERT_YELLOW) ? ILI9341_YELLOW :
                   ILI9341_WHITE;

  // Fill full button area
  tft.fillRect(160, 0, 80, 40, color);

  // Label always black for contrast
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setCursor(180, 14);
  tft.print("ALERTS");

  // Reset global text settings as the color affects ALL GUI
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
}

void drawConfigScreen() {
  tft.fillScreen(ILI9341_NAVY);
  tft.setTextColor(ILI9341_WHITE); 
  tft.setTextSize(2);
  tft.setCursor(10, 10); 
  tft.print("Auto Mode Settings");

  // TEMP column
  tft.setCursor(35, 40); 
  tft.print("TEMP");

  tft.fillRect(30, 65, 40, 30, ILI9341_GREEN); // Up
  tft.setCursor(42, 72); tft.print("^");

  tft.fillRect(30, 100, 40, 30, ILI9341_BLACK); // Value box
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(35, 108); 
  tft.print(configTempThreshold, 0);

  tft.fillRect(30, 135, 40, 30, ILI9341_RED); // Down
  tft.setCursor(42, 142); tft.print("v");

  // HUM column
  tft.setTextColor(ILI9341_WHITE); 
  tft.setCursor(150, 40); 
  tft.print("HUM");

  tft.fillRect(145, 65, 40, 30, ILI9341_GREEN); // Up
  tft.setCursor(157, 72); tft.print("^");

  tft.fillRect(145, 100, 40, 30, ILI9341_BLACK); // Value box
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(150, 108); 
  tft.print(configHumThreshold, 0);

  tft.fillRect(145, 135, 40, 30, ILI9341_RED); // Down
  tft.setCursor(157, 142); tft.print("v");

  // SAVE button
  tft.fillRect(20, 190, 90, 30, ILI9341_GREEN);
  tft.setCursor(40, 198); tft.print("SAVE");

  // BACK button
  tft.fillRect(130, 190, 90, 30, ILI9341_RED);
  tft.setCursor(150, 198); tft.print("BACK");

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
}

//avoid refreshing full pagfe so only redraw numbers
void updateThresholdValuesOnly() {
  tft.fillRect(30, 100, 40, 30, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
  tft.setCursor(35, 108); tft.print(configTempThreshold, 0);

  tft.fillRect(145, 100, 40, 30, ILI9341_BLACK);
  tft.setCursor(150, 108); tft.print(configHumThreshold, 0);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
}