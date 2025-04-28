// Arduino Sender Node for CAN Bus Monitoring
// Features: Sensing and periodic data transmission, alerts, controlling fan and vent
// Author: Khalid Suliman

#include <Arduino_CAN.h>
#include <DHT.h>
#include <Wire.h>
#include <RTC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <EEPROM.h>

#define NODE_ID 1

//EEPROM adresses
#define EEPROM_ADDR           0
#define EEPROM_ADDR_FAN       EEPROM_ADDR
#define EEPROM_ADDR_VENT      (EEPROM_ADDR + 1)
#define EEPROM_ADDR_AUTO      (EEPROM_ADDR + 2)
#define EEPROM_ADDR_TEMP_HI   (EEPROM_ADDR + 3) // uint16_t (2 bytes)
#define EEPROM_ADDR_HUM_HI    (EEPROM_ADDR + 5) // uint16_t (2 bytes)

//pins
#define DHTPIN 2
#define FAN_SSR_PIN 5
// Feedback control
const int controlPin = A0;   // DAC output to op-amp for actuator
const int feedbackPin = A1;  // Analogue input from voltage divider

#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;


// CAN IDs
const uint32_t DHT22_ID = 0x20;
const uint32_t BMP280_ID = 0x21;
const uint32_t CONTROL_CMD_ID = 0x10;
const uint32_t STATUS_ACK_ID = 0x31;
const uint32_t STATUS_REQUEST_ID = 0x11;
const uint32_t ALERT_ID = 0x09;
const uint32_t HEARTBEAT_REQUEST_ID = 0x12;
const uint32_t HEARTBEAT_REPLY_ID   = 0x13;
const uint32_t AUTO_CONFIG_ID = 0x40;
const uint32_t FEEDBACK_ID = 0x35;  //actuator feedback

// Alert types, first 2 not acted upon
#define ALERT_DHT_READ_FAIL     301
#define ALERT_BMP_READ_FAIL     302
#define ALERT_HIGH_TEMP_IN      3
#define ALERT_HIGH_TEMP_OUT     4

//sequences
uint8_t heartbeatSeq = 0;
uint8_t dhtSeq = 0, bmpSeq = 0;

//state booleans
bool fanOn = false, ventOn = false, autoMode = false;
bool suppressVentAlert = true;
unsigned long startupTime = 0;//used for vent

// sensor booleans
bool dhtInitFail = false;
bool bmpInitFail = false;
bool dhtFail = false;
bool bmpFail = false;
bool highIn = false;
bool highOut = false;
int bmpBadReadCount = 0;  //used as I2C can read garbage if not properly connected 
const int BMP_BAD_READ_THRESHOLD = 2;

//sensor error detects
int highInCounter = 0;
int highOutCounter = 0;
const int HIGH_TEMP_TRIGGER_THRESHOLD = 3;

//ASHRAE recommended condditions for data centres
float autoTempThreshold = 27.0;
float autoHumThreshold  = 60.0;
//values used for testing
// float autoTempThreshold = 22.0; 
// float autoHumThreshold  = 26.0;
//hysteresis for indoor params
const float TEMP_HYST = 3.0;
const float HUM_HYST  = 7.0;

//vent voltage mismatch variables
int ventMismatchCounter = 0;
const int MISMATCH_THRESHOLD = 15; //vent needs time to reach ON/OFF thresholds
bool feedbackAlertActive = false;

void saveStateToEEPROM() {
  EEPROM.update(EEPROM_ADDR_FAN, fanOn);
  EEPROM.update(EEPROM_ADDR_VENT, ventOn);
  EEPROM.update(EEPROM_ADDR_AUTO, autoMode);

  EEPROM.update(EEPROM_ADDR_TEMP_HI,   (uint8_t)((uint16_t)(autoTempThreshold * 10) >> 8));
  EEPROM.update(EEPROM_ADDR_TEMP_HI+1, (uint8_t)((uint16_t)(autoTempThreshold * 10) & 0xFF));
  EEPROM.update(EEPROM_ADDR_HUM_HI,    (uint8_t)((uint16_t)(autoHumThreshold * 10) >> 8));
  EEPROM.update(EEPROM_ADDR_HUM_HI+1,  (uint8_t)((uint16_t)(autoHumThreshold * 10) & 0xFF));

  Serial.println("saved states to EEPROM:");
  Serial.print("Fan: "); Serial.println(fanOn ? "ON" : "OFF");
  Serial.print("Vent: "); Serial.println(ventOn ? "ON" : "OFF");
  Serial.print("Auto: "); Serial.println(autoMode ? "ON" : "OFF");
  Serial.print("Auto Temp Threshold: "); Serial.println(autoTempThreshold);
  Serial.print("Auto Hum Threshold: "); Serial.println(autoHumThreshold);
}


void loadStateFromEEPROM() {
  fanOn = EEPROM.read(EEPROM_ADDR_FAN);
  ventOn = EEPROM.read(EEPROM_ADDR_VENT);
  autoMode = EEPROM.read(EEPROM_ADDR_AUTO);

  uint16_t tempRaw = (EEPROM.read(EEPROM_ADDR_TEMP_HI) << 8) | EEPROM.read(EEPROM_ADDR_TEMP_HI + 1);
  uint16_t humRaw  = (EEPROM.read(EEPROM_ADDR_HUM_HI) << 8)  | EEPROM.read(EEPROM_ADDR_HUM_HI + 1);

  // Validate values
  if (tempRaw >= 100 && tempRaw <= 400) autoTempThreshold = tempRaw / 10.0;
  if (humRaw  >= 100 && humRaw  <= 900) autoHumThreshold  = humRaw  / 10.0;
}

bool retryBMPInit(uint8_t retries = 5) {
  for (uint8_t i = 0; i < retries; i++) {
    if (bmp.begin(0x77)) return true;
    delay(500);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  analogWriteResolution(10);
  pinMode(feedbackPin, INPUT);
  analogWrite(controlPin, 0); 
  pinMode(FAN_SSR_PIN, OUTPUT);

  startupTime = millis();
  if (!CAN.begin(CanBitRate::BR_125k)) {
    Serial.println("CAN Init Failed!");
    while (1);
  }
  Serial.println("CAN Ready!");

  loadStateFromEEPROM();
  updateFanOutput(); 
  sendStatusAck(); // Notify GUI of restored state

  dht.begin();
  delay(1000);  // Give DHT time
  float dummyT = dht.readTemperature();
  dhtInitFail = isnan(dummyT);

  if (dhtInitFail) {
    Serial.println("DHT22 init failed");
    sendAlert(ALERT_DHT_READ_FAIL, 1);
  } else {
    sendAlert(ALERT_DHT_READ_FAIL, 0);
  }

  bmpInitFail = !retryBMPInit();

  if (bmpInitFail) {
    Serial.println("BMP280 init failed after retries");
    sendAlert(ALERT_BMP_READ_FAIL, 1);
  } else {
    sendAlert(ALERT_BMP_READ_FAIL, 0);
  }

  
}

void loop() {
  //process any can messages that are control 
  handleControlMessages();

  //check if recovered from failure and clear alert
  if (dhtInitFail) {
    float dummyT = dht.readTemperature();
    if (!isnan(dummyT)) {
      Serial.println("DHT22 recovered");
      dhtInitFail = false;
      sendAlert(ALERT_DHT_READ_FAIL, 0);
    }
  }

  //check if recovered from failure and clear alert
  if (bmpInitFail) {
    if (bmp.begin(0x77)) {
      Serial.println("BMP280 recovered");
      bmpInitFail = false;
      sendAlert(ALERT_BMP_READ_FAIL, 0);
    }
  }

  //send sensor data if not failing
  if (!dhtInitFail) sendDHT22Data();
  if (!bmpInitFail) sendBMP280Data();

  bool previousFan = fanOn;
  bool previousVent = ventOn;
  //check if states need toggling based on dht (indoor)
  if (autoMode && !dhtFail) {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  bool newFan = fanOn;
  bool newVent = ventOn;

  // Activation: if either crosses threshold turn both on
  if ((!isnan(t) && t >= autoTempThreshold) || (!isnan(h) && h >= autoHumThreshold)) {
    newFan = true;
    newVent = true;
  }

  // Deactivation: if both are below thresholds & hysteresis turn  Off
  if (fanOn && ventOn &&
      (!isnan(t) && t < (autoTempThreshold - TEMP_HYST)) &&
      (!isnan(h) && h < (autoHumThreshold - HUM_HYST))) {
    newFan = false;
    newVent = false;
  }

  // Apply new states
  if (newFan != fanOn || newVent != ventOn) {
    fanOn = newFan;
    ventOn = newVent;
    Serial.print("Auto triggered fan="); Serial.print(fanOn);
    Serial.print(" vent="); Serial.println(ventOn);
    sendStatusAck();
    updateFanOutput();
  }
}

  sendVentFeedback();  //  send vent feedback

  delay(5000);
}

//handle incoming can control messages
void handleControlMessages() {
  while (CAN.available()) {
    CanMsg cmd = CAN.read();

    //update config values set by the GUI and save to EEPROM
    if (cmd.id == AUTO_CONFIG_ID && cmd.data_length == 4) {
      uint16_t tempRaw = (cmd.data[0] << 8) | cmd.data[1];
      uint16_t humRaw  = (cmd.data[2] << 8) | cmd.data[3];

      autoTempThreshold = tempRaw / 10.0;
      autoHumThreshold  = humRaw / 10.0;

      //printing thresholds
      Serial.print("Received Auto config: Temp=");
      Serial.print(autoTempThreshold); Serial.print("°C, Hum=");
      Serial.print(autoHumThreshold); Serial.println("%");
      saveStateToEEPROM();
    }

    //handle toggle commands
    if (cmd.id == CONTROL_CMD_ID && cmd.data_length >= 4) {

      bool newFan = cmd.data[1];
      bool newVent = cmd.data[2];
      bool newAuto = cmd.data[3];

      if (!autoMode) {
        fanOn = newFan;
        ventOn = newVent;
        updateFanOutput();
        //printing states
        Serial.print("Fan: "); Serial.println(fanOn ? "ON" : "OFF");
        Serial.print("Vent: "); Serial.println(ventOn ? "ON" : "OFF");
        Serial.print("Auto: "); Serial.println(autoMode ? "ON" : "OFF");
      } else {
        Serial.println("Ignored fan/vent toggle due to AUTO mode");
      }
      //auto mode changed, reevaluate
      if (newAuto != autoMode) {
        autoMode = newAuto;
        Serial.print("Auto mode changed to: "); Serial.println(autoMode ? "ON" : "OFF");

        if (autoMode) {
          float t = dht.readTemperature();
          float h = dht.readHumidity();
          bool activate = (!isnan(t) && t >= autoTempThreshold) || (!isnan(h) && h >= autoHumThreshold);
          bool deactivate = (t < (autoTempThreshold - TEMP_HYST)) && (h < (autoHumThreshold - HUM_HYST));

          if (activate) {
            fanOn = true;
            ventOn = true;
          } else if (deactivate) {
            fanOn = false;
            ventOn = false;
          }

          updateFanOutput();
          sendStatusAck();
        }
      }

      //send and save
      saveStateToEEPROM();
      sendStatusAck(); 
    }
    //handle status requests from GUI
    if (cmd.id == STATUS_REQUEST_ID) {
      feedbackAlertActive = false;  
      ventMismatchCounter = 0;      // Restart counter on boot

      Serial.println("Resetting vent alert status on STATUS_REQUEST");
      sendStatusAck();
    }

    //reply to heartbeat request
    if (cmd.id == HEARTBEAT_REQUEST_ID) {
      uint8_t data[2] = { NODE_ID, heartbeatSeq++ };
      CanMsg reply(CanStandardId(HEARTBEAT_REPLY_ID), sizeof(data), data);
      CAN.write(reply);
      Serial.print("Replied to heartbeat (seq ");
      Serial.print(heartbeatSeq - 1); Serial.println(")");
    }
  }
}

//send states to GUI
void sendStatusAck() {
  uint8_t status[3] = { fanOn, ventOn, autoMode };
  CanMsg ack(CanStandardId(STATUS_ACK_ID), sizeof(status), status);
  CAN.write(ack);
}

void sendDHT22Data() {
  float temp = dht.readTemperature(), hum = dht.readHumidity();
  bool readFail = isnan(temp) || isnan(hum);

  // Alert on fail, not currently acted on in GUI, can be implemented in future to differentiate between Nan and broken sensor
  dhtFail = readFail;

  if (readFail) return;

  // Alert on high temp with hysteresis
  if (temp > 30.0) {
    highInCounter++;
    if (highInCounter >= HIGH_TEMP_TRIGGER_THRESHOLD && !highIn) {
      sendAlert(ALERT_HIGH_TEMP_IN, 1);
      highIn = true;
    }
  } else if (temp < 29.0 && highIn) {
    highInCounter = 0;
    sendAlert(ALERT_HIGH_TEMP_IN, 0);
    highIn = false;
  } else {
    highInCounter = 0;
  }

  int16_t t = round(temp * 10);
  uint16_t h = round(hum * 10);
  uint8_t data[6] = { NODE_ID, dhtSeq++, t >> 8, t & 0xFF, h >> 8, h & 0xFF };
  CanMsg msg(CanStandardId(DHT22_ID), sizeof(data), data);
  Serial.print("DHT22: "); Serial.print(temp); Serial.print("°C, "); Serial.print(hum); Serial.println("%");

  retrySend(msg, "DHT22");
}

void sendBMP280Data() {
  float temp = bmp.readTemperature(), pres = bmp.readPressure() / 100.0;
  bool readFail = isnan(temp) || isnan(pres);

  // avoiding garbage values
  if (temp < -40 || temp > 85 || pres < 700 || pres > 1500) {
    Serial.print("BMP280: Invalid values -> Temp: ");
    Serial.print(temp); Serial.print(" °C, Pressure: "); Serial.println(pres);
    readFail = true;
    bmpBadReadCount++;
  } else {
    bmpBadReadCount = 0;
  }

  if (bmpBadReadCount >= BMP_BAD_READ_THRESHOLD) {
    Serial.println("Reinitializing BMP280 due to repeated bad data...");
    bmp.begin(0x77);
    bmpBadReadCount = 0;
  }

  //Alert on fail, also not currently acted on in GUI, can be implemented in future to differentiate between Nan and broken sensor
  bmpFail = readFail;

  if (readFail) return;

  // Alert on high temp with hysteresis
  if (temp > 30.0) {
    highOutCounter++;
    if (highOutCounter >= HIGH_TEMP_TRIGGER_THRESHOLD && !highOut) {
      sendAlert(ALERT_HIGH_TEMP_OUT, 1);
      highOut = true;
    }
  } else if (temp < 29.0 && highOut) {
    highOutCounter = 0;
    sendAlert(ALERT_HIGH_TEMP_OUT, 0);
    highOut = false;
  } else {
    highOutCounter = 0;
  }

  int16_t t = round(temp * 10);
  uint16_t p = round(pres * 10);
  uint8_t data[6] = { NODE_ID, bmpSeq++, t >> 8, t & 0xFF, p >> 8, p & 0xFF };

  CanMsg msg(CanStandardId(BMP280_ID), sizeof(data), data);
  Serial.print("BMP280: "); Serial.print(temp); Serial.print("°C, "); Serial.print(pres); Serial.println(" hPa");

  retrySend(msg, "BMP280");
}

//retry message if needed
void retrySend(CanMsg &msg, const char* label) {
  for (int i = 0; i < 5; i++) {
    if (CAN.write(msg) >= 0) {
      Serial.print("Sent "); Serial.println(label);
      return;
    }
    Serial.print("Retry "); Serial.println(label);
    delay(200);
  }
  Serial.print("Failed to send "); Serial.println(label);
  // No alert sent handled by GUI via heartbeat plus timeout logic
}

//send alerts to gui
void sendAlert(uint8_t type, uint8_t status) {
  uint8_t data[3] = { NODE_ID, type, status };
  CanMsg alert(CanStandardId(ALERT_ID), sizeof(data), data);
  CAN.write(alert);
}

//send vent actuator voltage
void sendVentFeedback() {

  // suppress Vent alert during first 10s
  if (millis() - startupTime > 10000) {
    suppressVentAlert = false;
  }

  // Drive actuator
  if (ventOn) {
    analogWrite(controlPin, 1023);  // 5V becomes 10V after op-amp
  } else {
    analogWrite(controlPin, 0);     // 0V
  }

  // Read and scale feedback voltage
  int raw = analogRead(feedbackPin);           // 0–1023
  float scaled = (raw / 1023.0) * 5.0;         // 0–5V
  float actual = scaled * 2.0;                 

  // Send feedback voltage over CAN
  uint16_t voltage_mV = (uint16_t)(actual * 100);  // for example 480 = 4.8V
  uint8_t data[2] = { voltage_mV >> 8, voltage_mV & 0xFF };

  CanMsg msg(CanStandardId(FEEDBACK_ID), sizeof(data), data);
  CAN.write(msg);

  // Compare to expected state
  bool expectedHigh = ventOn;
  bool isHigh = actual >= 4.5;
  bool isLow = actual <= 2;
  bool matched = (expectedHigh && isHigh) || (!expectedHigh && isLow);

  Serial.print("SENT Vent Feedback Voltage: ");
  Serial.print(actual, 2); Serial.println(" V");

  if (!matched) {
    ventMismatchCounter++;

    if (ventMismatchCounter >= MISMATCH_THRESHOLD && !feedbackAlertActive) {
      if (!suppressVentAlert) {
        sendAlert(200, 1);  // 200 = ALERT_CONTROL_MISMATCH_VENT
        feedbackAlertActive = true;
        Serial.println("Alert 200: Vent mismatch detected");
      } else {
        Serial.println("Suppressed alert 200 during startup grace period");
      }
    }

  } else {
    ventMismatchCounter = 0;

    if (feedbackAlertActive) {
      sendAlert(200, 0);  // resolved
      feedbackAlertActive = false;
      Serial.println("Alert 200 resolved: Vent feedback matched");
    }
  }
}

void updateFanOutput() {
  digitalWrite(FAN_SSR_PIN, fanOn ? HIGH : LOW);
}


  