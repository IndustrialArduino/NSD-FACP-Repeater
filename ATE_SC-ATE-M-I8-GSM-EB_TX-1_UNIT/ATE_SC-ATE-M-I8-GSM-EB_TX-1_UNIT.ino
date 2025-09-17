#include <Wire.h>
#include <WiFi.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <PCA9536D.h>
#include "RTClib.h"
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_PCA9538.h>
#include <ClosedCube_HDC1080.h>
#include <ClosedCube_OPT3001.h>
#include <Update.h>
#include "Secret.h"
#include "Configurations.h"
#include "Message_Content.h"
#include "datacake.h"
#include "github.h"

//I2C PINS
#define SDA   8
#define SCL   9

// I2C address of I2C LCD Display
#define PCA9538_ADDR 0x73

//RS485 LINE-1 PINS
#define RS485_RXD1 16
#define RS485_TXD1 15
#define RS485_FC1  47

//RS485 LINE-2 PINS
#define RS485_RXD2 41
#define RS485_TXD2 40
#define RS485_FC2  48

//ETHERNET LINE PINS
#define ETHERNET_RESET 2
#define ETH_CS 1
#define ETH_INT 4

//SPI LINE PINS
#define MISO 37
#define MOSI 35
#define SCLK 36

//ON BOARD BUZZER PIN
#define BUZZER 3

//ONBOARD DIGITAL INPUTS IN MIDDDLE OF BOARD
#define DR_INPUT1  5
#define DR_INPUT2  6

// DIGITAL INPUTS
#define INPUT1  12     // FIRE ALARM INPUT
#define INPUT2  11     // FAULT ALRAM INPUT
#define INPUT3  10     // MAINS_FAIL_ALARM INPUT
#define INPUT4  7
#define INPUT5  13
#define INPUT6  14

//ON BOARD MOTION SENSOR INPUT
#define INPUT_MOTION  42

//ON BOARD LEDS
#define LED_OUT1  38
#define LED_OUT2  39

//MODEM PINS
#define GSM_TX   18
#define GSM_RX   17
#define GSM_RESET  21

//RELAY OUTPUTS
#define IO_RL1  0
#define IO_RL2  1

//TRANSISTOR OUTPUTS
#define IO_TR1  2
#define IO_TR2  3

#define SerialMon Serial
#define SerialAT Serial1
#define GSM_PIN ""

PCA9538_LCD lcd(PCA9538_ADDR);

PCA9536 io;
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define DEBOUNCE_DELAY 50

bool FEA_Status = false;
bool FTA_Status = false;
bool MSA_Status = false ;

bool inputStatus[3] = {false, false, false}; // FEA, FTA, MSA

//Debouncing and states
unsigned long lastDebounceTime[3] = {0, 0, 0};
bool lastStableState[3] = {HIGH, LOW, LOW};
bool currentState[3] = {HIGH, LOW, LOW};
bool processing = false;

String firmware_url = "https://raw.githubusercontent.com/IndustrialArduino/NSD-FACP-Updates/release/Transmitter_Panel.bin";

unsigned long lastAliveSentTime = 0;
const unsigned long aliveInterval = 900000; // 15 minutes in milliseconds
bool aliveMessageSentOnce = false;

String MQTTconnection = "";
String GPRSconnection = "";

unsigned long lastStatusMQTTSentTime = 0;
const unsigned long statusMQTTInterval = 60000;

// Tracking activation timing
unsigned long activationStart[3] = {0, 0, 0};
bool pendingAlarm[3] = {false, false, false};

// Input names
const char* inputNames[3] = {"Fire", "Fault", "Mains"};

// Status text abbreviations
const char* statusText[2] = {"Normal", "Alarm"}; // 0 = inactive, 1 = active

unsigned long lastScrollTime = 0;
const unsigned long scrollDelay = 3000; // 3s per screen
int scrollIndex = 0; // Which screen is currently displayed

unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000;

void setup() {
  Serial.begin(115200);
  delay(2000);
  SerialAT.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(1000);

  digitalWrite(GSM_RESET, LOW);
  delay(1000);
  digitalWrite(GSM_RESET, HIGH);

  pinMode(LED_OUT1, OUTPUT);
  pinMode(LED_OUT2, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  pinMode(DR_INPUT1,  INPUT);
  pinMode(DR_INPUT2,  INPUT);
  pinMode(INPUT1,  INPUT);
  pinMode(INPUT2,  INPUT);
  pinMode(INPUT3,  INPUT);
  pinMode(INPUT4,  INPUT);
  pinMode(INPUT5,  INPUT);
  pinMode(INPUT6,  INPUT);
  pinMode(INPUT_MOTION,  INPUT);

  Wire.begin(SDA, SCL);
  delay(100);

  lcd.begin(16, 4);

//  if (io.begin() == false)
//  {
//    Serial.println("PCA9536 not detected. Please check wiring. Freezing...");
//    while (1);
//  }

  io.pinMode(IO_RL1, OUTPUT);
  io.pinMode(IO_RL2, OUTPUT);
  io.pinMode(IO_TR1, OUTPUT);
  io.pinMode(IO_TR2, OUTPUT);

  Init();
  connectToGPRS();
  RTC_Check();
  connectToMQTT();
  updateLCD16x4();

}

void loop() {
  if (processing) return;
  processing = true;

  readInputsAndCheckAlarms();
  checkForMessage();
  isGPRSConnected();
  maintainMQTTConnection();

  // Send periodic status update every 1 minute
  unsigned long now = millis();
  if (now - lastStatusMQTTSentTime >= statusMQTTInterval) {
    //sendMQTTStatusUpdate();  // Custom function
    lastStatusMQTTSentTime = now;
  }

  unsigned long currentAliveMillis = millis();
  if (!aliveMessageSentOnce) {
    sendAliveMessage();
    lastAliveSentTime = currentAliveMillis;
    aliveMessageSentOnce = true;
  } else if (currentAliveMillis - lastAliveSentTime >= aliveInterval) {
    sendAliveMessage();
    lastAliveSentTime = currentAliveMillis;
  }

  if (millis() - lastDisplayUpdate > displayInterval) {
    updateLCD16x4();
    lastDisplayUpdate = millis();
  }

  processing = false;

}

void sendMQTTStatusUpdate() {
  // Read raw digital states
  currentState[0] = digitalRead(INPUT1);  // Fire
  currentState[1] = digitalRead(INPUT2);  // Fault
  currentState[2] = digitalRead(INPUT3);  // MAINS_FAIL_ALARM

  // Interpret logic as per your alarm rules
  bool fireStatus  = interpretAlarmStatus(0, currentState[0]);
  bool faultStatus = interpretAlarmStatus(1, currentState[1]);
  bool mainpowerStatus = interpretAlarmStatus(2, currentState[2]);

  // Publish interpreted statuses
  publishToMQTT(mqttConfig.fireTopic, String(fireStatus));
  publishToMQTT(mqttConfig.faultTopic, String(faultStatus));
  publishToMQTT(mqttConfig.mainsFailTopic, String(mainpowerStatus));

  Serial.println("[STATUS] Periodic MQTT status sent: FIRE=" + String(fireStatus) +
                 ", FAULT=" + String(faultStatus) + ", MAIN POWER=" + String(mainpowerStatus));

}


void readInputsAndCheckAlarms() {
  unsigned long now = millis();
  currentState[0] = digitalRead(INPUT1);
  currentState[1] = digitalRead(INPUT2);
  currentState[2] = digitalRead(INPUT3);

  for (int i = 0; i < 3; i++) {
    if (currentState[i] != lastStableState[i]) {
      if ((now - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
        lastDebounceTime[i] = now;
        lastStableState[i] = currentState[i];

        bool interpreted = interpretAlarmStatus(i, currentState[i]);

        unsigned long delayMs = (unsigned long)inputDelayMinutes[i] * 60000UL;

        if (interpreted) {
          if (delayMs == 0) {
            // Instant alarm
            inputStatus[i] = true;
            handleAlarmStateChange(i, true);
          } else {
            // Start pending timer
            activationStart[i] = now;
            pendingAlarm[i] = true;
          }
        } else {
          // Input cleared
          if (pendingAlarm[i]) {
            // Cleared before delay → cancel pending
            pendingAlarm[i] = false;
            activationStart[i] = 0;
          }
          if (inputStatus[i]) {
            // Alarm was already active → send restore immediately
            inputStatus[i] = false;
            handleAlarmStateChange(i, false);
          }
        }
      }
    }

    // Check if pending timer expired
    if (pendingAlarm[i]) {
      unsigned long delayMs = (unsigned long)inputDelayMinutes[i] * 60000UL;
      if (now - activationStart[i] >= delayMs) {
        inputStatus[i] = true;
        pendingAlarm[i] = false;
        handleAlarmStateChange(i, true);
      }
    }
  }
}


bool interpretAlarmStatus(int idx, bool state) {
  if (idx == 0) return state == LOW;  // Fire
  else return state == HIGH;          // Fault / MAIN POWER
}

void handleAlarmStateChange(int idx, bool status) {
  String topic;
  String payload = String(status);

  String smsMessage = smsMessages[idx][status ? 0 : 1]; // Get proper message
  NotificationMode mode = inputNotificationSettings[idx];

  Serial.println("[STATE CHANGE] Alarm " + String(idx) + ": " + (status ? "ACTIVATED" : "CLEARED") + " -> " + smsMessage);

  // Send SMS
  if (mode == NOTIFY_SMS || mode == NOTIFY_BOTH) {
    sendSMSForInput(idx, status, smsMessage);
  }

  switch (idx) {
    case 0:
      topic = mqttConfig.fireTopic;
      Serial.println(status ? "[ALARM] FIRE Active" : "[NORMAL] FIRE Normal");
      break;
    case 1:
      topic = mqttConfig.faultTopic;
      Serial.println(status ? "[ALARM] FAULT Active" : "[NORMAL] FAULT Normal");
      break;
    case 2:
      topic = mqttConfig.mainsFailTopic;
      Serial.println(status ? "[ALARM] MAINS_FAIL_ALARM" : "[NORMAL] MAIN POWER OK");
      break;
  }

  publishToMQTT(topic, payload);

  if (!status) return; // Only act on activation

  if (mode == NOTIFY_CALL || mode == NOTIFY_BOTH) {
    if (idx == 0) { // Fire
      makeAlarmCalls(Fire_Alarm_Text);
    } else if (idx == 1) { // Fault
      makeAlarmCalls(Fault_Alarm_Text);
    }
  }

}

void sendSMSForInput(int idx, bool isAlarm, String message) {
  InputNotificationConfig &cfg = inputConfigs[idx];

  // 1. Always send to RX number
  gsm_send_serial("AT+CMGF=1", 1000);
  gsm_send_serial("AT+CSCS=\"GSM\"", 500);
  String cmd = String("AT+CMGS=\"") + rxSimNumber + "\"";
  gsm_send_serial(cmd, 1000);
  gsm_send_serial(message + "\x1A", 5000);

  // 2. Apply rules from configuration
  for (int r = 0; r < cfg.ruleCount; r++) {
    NotificationRule &rule = cfg.rules[r];

    // Check if this rule applies (Alarm or Restore)
    if ((isAlarm && rule.sendOnAlarm) || (!isAlarm && rule.sendOnRestore)) {
      for (int n = 0; n < 6; n++) {
        if (rule.routingMask[n] && smsRoutingNumbers[n][0] != '\0') {
          gsm_send_serial("AT+CMGF=1", 1000);
          gsm_send_serial("AT+CSCS=\"GSM\"", 500);
          String cmd2 = String("AT+CMGS=\"") + smsRoutingNumbers[n] + "\"";
          gsm_send_serial(cmd2, 1000);
          gsm_send_serial(message + "\x1A", 5000);
        }
      }
    }
  }
}


void checkForMessage() {
  String response = gsm_send_serial("AT+CMGL=\"REC UNREAD\"", 2000); // List unread messages
}

void handleSMSLine(String metaLine) {
  int indexEnd = metaLine.indexOf("\n");
  int bodyStart = indexEnd + 1;
  String messageBody = "";

  if (SerialAT.available()) {
    messageBody = SerialAT.readStringUntil('\r');
    messageBody.trim();
  }

  // Extract SMS index
  int smsIndex = metaLine.substring(6, metaLine.indexOf(',', 6)).toInt();

  Serial.println("[SMS] Received: " + messageBody);
  handleSMS(messageBody);

  // Delete processed message
  gsm_send_serial("AT+CMGD=" + String(smsIndex), 1000);
}

void handleSMS(String message) {
  message.trim();
  message.toLowerCase();

  if (message.indexOf("update the TX-1 panel") != -1) {
    Serial.println("[SMS Action] OTA UPDATE");
    performOTA();
  }
}


void sendAliveMessage() {
  String message = "1";
  Serial.println("[HEARTBEAT MESAAGE] Sending alive message");

  publishToMQTT(mqttConfig.aliveTopic, message);
}


void publishToMQTT(String topic, String payload) {
  String command = "AT+QMTPUBEX=0,1,1,0,\"" + topic + "\"," + String(payload.length());
  gsm_send_serial(command, 1000);
  gsm_send_serial(payload + "\x1A", 1000);
}

void maintainMQTTConnection() {
  String status = gsm_send_serial("AT+QMTCONN?", 1000);
  if (status.indexOf("+QMTCONN: 0,3") == -1) {
    Serial.println("[WARNING] MQTT Disconnected. Reconnecting...");
    MQTTconnection = "Disconnect";
    connectToMQTT();
  } else {
    MQTTconnection = "Connect";
  }
}

void makeAlarmCalls(String message) {
  for (int i = 0; i < 1; i++) {
    SerialMon.println("[CALL] Dialing " + phonecall_phoneNumbers[i]);

    // 1. Enable COLP
    gsm_send_serial("AT+COLP=1", 1000);

    // 2. Dial number
    gsm_send_serial("ATD" + phonecall_phoneNumbers[i] + ";", 1000);

    unsigned long callStart = millis();
    bool callAnswered = false;
    bool callEnded = false;

    // 3. Wait for call status (answered, rejected, timeout)
    while (millis() - callStart < 30000) {
      while (SerialAT.available()) {
        String line = SerialAT.readStringUntil('\n');
        line.trim();
        SerialMon.println("URC: " + line);

        if (line.startsWith("+COLP:")) {
          SerialMon.println("[CALL] Call answered via +COLP");
          callAnswered = true;
          break;
        }
        if (line.indexOf("NO CARRIER") != -1 || line.indexOf("BUSY") != -1 || line.indexOf("NO ANSWER") != -1) {
          SerialMon.println("[CALL] Call ended or rejected");
          callEnded = true;
          break;
        }
      }
      if (callAnswered || callEnded) break;
      delay(100);
    }

    // 4. Handle call result
    if (callAnswered) {
      delay(3000);

      for (int playCount = 0; playCount < 2; playCount++) {
        SerialMon.println("[CALL] Playing message via TTS...");
        gsm_send_serial("AT+QWTTS=1,1,2,\"" + message + "\"", 1000); // Start TTS

        // 5. Wait for TTS to finish or call to end early
        unsigned long ttsStart = millis();
        bool ttsEndedEarly = false;

        while (millis() - ttsStart < 5000) {
          while (SerialAT.available()) {
            String ttsLine = SerialAT.readStringUntil('\n');
            ttsLine.trim();
            SerialMon.println("TTS URC: " + ttsLine);

            if (ttsLine.indexOf("NO CARRIER") != -1 || ttsLine.indexOf("BUSY") != -1) {
              SerialMon.println("[CALL] Call ended during TTS.");
              callEnded = true;
              ttsEndedEarly = true;
              break;
            }
          }
          if (callEnded || ttsEndedEarly) break;
          delay(100);
        }
        if (callEnded || ttsEndedEarly) break;

        if (playCount == 0) {
          SerialMon.println("[CALL] Waiting before playing message again...");
          delay(500); // Delay before second TTS
        }
      }
    } else {
      SerialMon.println("[CALL] Not answered or ended early.");
    }

    // 6. Ensure hang-up
    SerialMon.println("[CALL] Hanging up.");
    gsm_send_serial("ATH", 1000);

    // 7. Wait before calling next number
    delay(3000);

  }
}

void Init(void) {                        // Connecting with the network and GPRS
  delay(5000);
  gsm_send_serial("AT+CFUN=1", 10000);
  gsm_send_serial("AT+CPIN?", 10000);
  gsm_send_serial("AT+CSQ", 1000);
  gsm_send_serial("AT+CREG?", 1000);
  gsm_send_serial("AT+COPS?", 1000);
  gsm_send_serial("AT+CGATT?", 1000);
  gsm_send_serial("AT+CPSI?", 500);
  String cmd = "AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"";
  gsm_send_serial(cmd, 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGATT?", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
  gsm_send_serial("AT+CMGF=1", 1000); // Set SMS to text mode
  gsm_send_serial("AT+CNMI=2,1,0,0,0", 1000); // Immediate notification when SMS is received
}

void connectToGPRS(void) {
  gsm_send_serial("AT+CGATT=1", 1000);
  String cmd = "AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"";
  gsm_send_serial(cmd, 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
}

void connectToMQTT(void) {
  // Initialize MQTT configurations
  gsm_send_serial("AT+QMTCLOSE=0", 1000);
  gsm_send_serial("AT+QMTCFG=\"recv/mode\",0,0,1", 1000);
  gsm_send_serial("AT+QMTCFG=\"SSL\",0,1,4", 1000);
  int cert_length = mqtt_ca_cert.length(); // Get the length of the CA certificate
  String ca_cert = "AT+QFUPL=\"RAM:datacake_ca.pem\"," + String(cert_length) + ",100";
  gsm_send_serial(ca_cert, 1000); // Send the command
  delay(1000);
  gsm_send_serial(mqtt_ca_cert, 1000); // Send the command to upload CA singned certificate
  delay(1000);
  gsm_send_serial("AT+QSSLCFG=\"cacert\",2,\"RAM:datacake_ca.pem\"", 1000);
  gsm_send_serial("AT+QMTOPEN=0,\"159.89.214.202\",8883", 1000);
  delay(2000); // Wait for the connection to establish
  gsm_send_serial("AT+QMTDISC=0", 1000);
  delay(1000);
  String mqtt_conn = "AT+QMTCONN=0,\"TX1\",\"" + username + "\",\"" + password + "\"";
  gsm_send_serial(mqtt_conn, 1000);
  delay(2000); // Wait for the connection to establish

  // Debug: Print MQTT connection status
  String connStatus = gsm_send_serial("AT+QMTCONN?", 1000);
  SerialMon.print("MQTT Connection Status: ");
  SerialMon.println(connStatus);
}

//16X4 LCD DISPLAY UPDATES
void updateLCD16x4() {
  DateTime now = rtc.now();
  lcd.clear();

  // --- Row 0: Node + Date/Time ---
  char row0[21];
  snprintf(row0, sizeof(row0), "%s %02d/%02d %02d:%02d:%02d",
           nodeName, now.day(), now.month(), now.hour(), now.minute(), now.second());
  lcd.setCursor(0, 0);
  lcd.print(row0);

  // --- Rows 1-3: Depending on scrollIndex ---
  switch (scrollIndex) {
    case 0:
      // Show input statuses
      for (int i = 0; i < 3; i++) {
        lcd.setCursor(0, i + 1);
        char rowMsg[17];
        snprintf(rowMsg, 17, "%-6s %s", inputNames[i], statusText[inputStatus[i] ? 1 : 0]);
        lcd.print(rowMsg);
      }
      break;
    case 1:
      // Show GSM Signal Strength
      lcd.setCursor(0, 1);
      lcd.print("Signal: ");
      lcd.print(String(getGSMSignalStrength()));

      // Show GPRS
      lcd.setCursor(0, 2);
      lcd.print("GPRS: ");
      lcd.print(GPRSconnection);

      // Show MQTT
      lcd.setCursor(0, 3);
      lcd.print("MQTT: ");
      lcd.print(MQTTconnection);
      break;
  }

  // Handle automatic scrolling
  unsigned long nowMillis = millis();
  if (nowMillis - lastScrollTime >= scrollDelay) {
    scrollIndex = (scrollIndex + 1) % 2; // toggle between screens
    lastScrollTime = nowMillis;
  }
}

void displayTime(void) {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);

  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  delay(1000);

}

void RTC_Check() {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }
  else {
    if (rtc.lostPower()) {

      Serial.println("RTC lost power, lets set the time!");
      //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      syncRTCWithNetworkTime();

    }
    syncRTCWithNetworkTime();

    int a = 1;
    while (a < 6)
    {
      displayTime();   // printing time function for oled
      a = a + 1;
    }
  }
}

String getNetworkTime() {
  String response = gsm_send_serial("AT+QLTS=1", 2000);
  // Expected response: +QLTS: "2025/09/02,15:48:30+08"
  int idx = response.indexOf("+QLTS: \"");
  if (idx >= 0) {
    String t = response.substring(idx + 8, response.indexOf("\"", idx + 8)); // Extract "YYYY/MM/DD,HH:MM:SS+TZ"
    t.trim();
    Serial.println("Network Time: " + t);
    return t;
  }
  return "";
}

void syncRTCWithNetworkTime() {
  String t = getNetworkTime(); // +QLTS response
  if (t.length() == 0) return;

  int yyyy = t.substring(0, 4).toInt();
  int mm   = t.substring(5, 7).toInt();
  int dd   = t.substring(8, 10).toInt();
  int hh   = t.substring(11, 13).toInt();
  int mi   = t.substring(14, 16).toInt();
  int ss   = t.substring(17, 19).toInt();

  // --- Timezone parsing ---
  int tzStart = 19;
  int tzSign = (t.charAt(tzStart) == '-') ? -1 : 1;
  int tzRaw  = t.substring(tzStart + 1, t.indexOf(',', tzStart)).toInt();
  int tzMinutes = tzRaw * 15 * tzSign;


  // Apply timezone shift
  long totalSeconds = (hh * 3600L) + (mi * 60L) + ss;
  totalSeconds += tzMinutes * 60L;

  // Handle overflow/underflow by using DateTime
  DateTime dt(yyyy, mm, dd, 0, 0, 0);
  dt = dt + TimeSpan(totalSeconds);

  rtc.adjust(dt);

  Serial.print("RTC synced with network time: ");
  Serial.print(dt.year()); Serial.print('/');
  Serial.print(dt.month()); Serial.print('/');
  Serial.print(dt.day()); Serial.print(" ");
  Serial.print(dt.hour()); Serial.print(':');
  Serial.print(dt.minute()); Serial.print(':');
  Serial.println(dt.second());
}

int getGSMSignalStrength() {
  String response = gsm_send_serial("AT+CSQ", 500);
  int rssi = -1;

  int index = response.indexOf("+CSQ:");
  if (index != -1) {
    int commaIndex = response.indexOf(",", index);
    if (commaIndex != -1) {
      String rssiStr = response.substring(index + 6, commaIndex);
      rssiStr.trim();
      rssi = rssiStr.toInt();
    }
  }

  return rssi;
}

void performOTA() {
  int cert_length = root_ca.length();
  String ca_cert = "AT+QFUPL=\"RAM:github_ca.pem\"," + String(cert_length) + ",100";
  gsm_send_serial(ca_cert, 1000);
  delay(1000);
  gsm_send_serial(root_ca, 1000);
  delay(1000);
  gsm_send_serial("AT+QHTTPCFG=\"contextid\",1", 1000);
  gsm_send_serial("AT+QHTTPCFG=\"responseheader\",1", 1000);
  gsm_send_serial("AT+QHTTPCFG=\"sslctxid\",1", 1000);
  gsm_send_serial("AT+QSSLCFG=\"sslversion\",1,4", 1000);
  gsm_send_serial("AT+QSSLCFG=\"ciphersuite\",1,0xC02F", 1000);
  gsm_send_serial("AT+QSSLCFG=\"seclevel\",1,1", 1000);
  gsm_send_serial("AT+QSSLCFG=\"sni\",1,1", 1000);
  gsm_send_serial("AT+QSSLCFG=\"cacert\",1,\"RAM:github_ca.pem\"", 1000);

  gsm_send_serial("AT+QHTTPURL=" + String(firmware_url.length()) + ",80", 1000);
  delay(100);
  gsm_send_serial(firmware_url, 2000);

  gsm_send_serial("AT+QHTTPGET=80", 1000);
  Serial.println("[OTA] Waiting for +QHTTPGET response...");


  long contentLength = -1;
  unsigned long timeout = millis();
  while (millis() - timeout < 5000) {
    if (SerialAT.available()) {
      String line = SerialAT.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      Serial.println("[Modem Line] " + line);
      if (line.startsWith("+QHTTPGET:")) {
        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
          String lenStr = line.substring(secondComma + 1);
          contentLength = lenStr.toInt();
          Serial.print("[OTA] Content-Length: ");
          Serial.println(contentLength);
        }
      }
      if (line == "OK") break;
    }
    delay(10);
  }

  Serial.println("[OTA] HTTPS GET sent");

  // Save response to RAM file
  gsm_send_serial("AT+QHTTPREADFILE=\"RAM:firmware.bin\",80", 1000);

  // Wait for final confirmation and avoid overlap
  unsigned long readfileTimeout = millis();
  while (millis() - readfileTimeout < 5000) {
    if (SerialAT.available()) {
      String line = SerialAT.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      Serial.println("[READFILE] " + line);
      if (line.startsWith("+QHTTPREADFILE:")) break;
    }
    delay(10);
  }

  // Clear SerialAT buffer
  while (SerialAT.available()) SerialAT.read();

  // Send QFLST directly
  SerialAT.println("AT+QFLST=\"RAM:firmware.bin\"");

  long ramFileSize = 0;
  timeout = millis();
  while (millis() - timeout < 5000) {
    if (SerialAT.available()) {
      String line = SerialAT.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;

      Serial.println("[OTA Raw] " + line);

      // Find +QFLST line
      if (line.startsWith("+QFLST:")) {
        int commaIdx = line.lastIndexOf(',');
        if (commaIdx != -1) {
          String sizeStr = line.substring(commaIdx + 1);
          sizeStr.trim();
          ramFileSize = sizeStr.toInt();
          break;
        }
      }
    }
    delay(10);
  }

  Serial.println("[OTA] File size: " + String(ramFileSize));

  if (ramFileSize <= 0) {
    Serial.println("[OTA] ERROR: Invalid file size.");
    return;
  }



  int headerSize = ramFileSize - contentLength;
  if (headerSize <= 0 || headerSize > ramFileSize) {
    Serial.println("[OTA] Invalid header size!");
    return;
  }
  Serial.println("[OTA] Header size: " + String(headerSize));

  // Clear SerialAT buffer before command
  while (SerialAT.available()) SerialAT.read();

  // Send QFOPEN directly
  SerialAT.println("AT+QFOPEN=\"RAM:firmware.bin\",0");

  int fileHandle = -1;
  unsigned long handleTimeout = millis();

  while (millis() - handleTimeout < 5000) {
    if (SerialAT.available()) {
      String line = SerialAT.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;

      Serial.println("[OTA Raw] " + line);

      if (line.startsWith("+QFOPEN:")) {
        String handleStr = line.substring(line.indexOf(":") + 1);
        handleStr.trim();
        fileHandle = handleStr.toInt();
        break;
      }
    }
    delay(10);
  }

  Serial.println("[OTA] File handle: " + String(fileHandle));

  if (fileHandle <= 0) {
    Serial.println("[OTA] ERROR: Invalid file handle.");
    return;
  }

  // Seek to payload
  gsm_send_serial("AT+QFSEEK=" + String(fileHandle) + "," + String(headerSize) + ",0", 1000);
  delay(300);
  // Step 7: Begin OTA
  if (!Update.begin(contentLength)) {
    Serial.println("[OTA] Update.begin failed");
    return;
  }

  Serial.println("[OTA] Start writing...");


  size_t chunkSize = 1024;
  size_t totalWritten = 0;
  uint8_t buffer[1024];

  while (totalWritten < contentLength) {
    size_t bytesToRead = min(chunkSize, (size_t)(contentLength - totalWritten));
    SerialAT.println("AT+QFREAD=" + String(fileHandle) + "," + String(bytesToRead));

    // Wait for CONNECT (start of binary data)
    bool gotConnect = false;
    unsigned long startWait = millis();
    while (millis() - startWait < 2000) {
      if (SerialAT.available()) {
        String line = SerialAT.readStringUntil('\n');
        line.trim();
        if (line.startsWith("CONNECT")) {
          gotConnect = true;
          break;
        }
      }
      delay(1);
    }
    if (!gotConnect) {
      Serial.println("[OTA] Failed to get CONNECT");
      Update.abort();
      return;
    }

    // Read exactly bytesToRead bytes of binary data
    size_t readCount = 0;
    unsigned long lastReadTime = millis();
    while (readCount < bytesToRead && millis() - lastReadTime < 3000) {
      if (SerialAT.available()) {
        buffer[readCount++] = (uint8_t)SerialAT.read();
        lastReadTime = millis();
      } else {
        delay(1);
      }
    }
    if (readCount != bytesToRead) {
      Serial.println("[OTA] Incomplete read from modem");
      Update.abort();
      return;
    }

    // After reading data, wait for the final OK
    bool gotOK = false;
    startWait = millis();
    while (millis() - startWait < 2000) {
      if (SerialAT.available()) {
        String line = SerialAT.readStringUntil('\n');
        line.trim();
        if (line == "OK") {
          gotOK = true;
          break;
        }
      }
      delay(1);
    }
    if (!gotOK) {
      Serial.println("[OTA] Did not receive final OK after data");
      Update.abort();
      return;
    }

    // Write to flash
    size_t written = Update.write(buffer, readCount);
    if (written != readCount) {
      Serial.println("[OTA] Flash write mismatch");
      Update.abort();
      return;
    }

    totalWritten += written;
    Serial.printf("\r[OTA] Progress: %u / %u bytes", (unsigned)totalWritten, (unsigned)contentLength);
  }

  Serial.println("\n[OTA] Firmware write complete.");

  // Close the file
  SerialAT.println("AT+QFCLOSE=" + String(fileHandle));
  delay(500);

  // Finalize OTA update
  if (Update.end()) {
    Serial.println("[OTA] Update successful!");
    if (Update.isFinished()) {
      //sendSMS("FACP-Transmitter Panel Updated Successfully");
      delay(300);
      Serial.println("[OTA] Rebooting...");
      delay(300);
      ESP.restart();
    } else {
      Serial.println("[OTA] Update not finished!");
    }
  } else {
    Serial.println("[OTA] Update failed with error: " + String(Update.getError()));
  }
}

bool isNetworkConnected() {
  String response = gsm_send_serial("AT+CREG?", 3000);
  return (response.indexOf("+CREG: 0,1") != -1 || response.indexOf("+CREG: 0,5") != -1);
}

void isGPRSConnected() {
  String status = gsm_send_serial("AT+CGATT?", 3000);
  if (status.indexOf("+CGATT: 1") == -1) {
    Serial.println("[WARNING] GPRS Disconnected. Reconnecting...");
    GPRSconnection = "Deactive";
    connectToGPRS();
  } else {
    GPRSconnection = "Active";
  }
}

String gsm_send_serial(String command, int timeout) {
  String buff_resp = "";
  Serial.println("Send ->: " + command);
  SerialAT.println(command);
  unsigned long startMillis = millis();

  while (millis() - startMillis < timeout) {
    while (SerialAT.available()) {
      //      char c = SerialAT.read();
      //      buff_resp += c;
      //    }
      String line = SerialAT.readStringUntil('\n');
      line.trim();

      if (line.length() == 0) continue;

      Serial.println("[Modem Line] " + line);

      // Handle SMS lines
      if (line.startsWith("+CMGL:")) {
        handleSMSLine(line); // Extract and forward SMS
        continue; // Do not append to buffer
      }

      // Accumulate only unhandled lines
      buff_resp += line + "\n";
    }
    delay(10); // Small delay to allow for incoming data to accumulate
  }

  Serial.println("Response ->: " + buff_resp);
  return buff_resp;
}
