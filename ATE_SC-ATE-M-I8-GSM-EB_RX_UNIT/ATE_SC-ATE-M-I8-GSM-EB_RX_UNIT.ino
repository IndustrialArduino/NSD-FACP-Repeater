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
#define INPUT1  12     // ALARM SILENCE
#define INPUT2  11
#define INPUT3  10     // MAINS FAULT SIGNAL
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
#define IO_RL1  0     // EXTERNEL BELL
#define IO_RL2  1     // FIRE ALARM INDICATOR

//TRANSISTOR OUTPUTS
#define IO_TR1  2
#define IO_TR2  3

#define SerialMon Serial
#define SerialAT Serial1
#define GSM_PIN ""

PCA9538_LCD lcd(PCA9538_ADDR);

PCA9536 io;
RTC_DS3231 rtc;

bool RMSA_Status = false ;
bool inputStatus = false; // MSA

#define DEBOUNCE_DELAY 50
//Debouncing and states
unsigned long lastDebounceTime = 0;
bool lastStableState =  LOW;
bool currentState    =  LOW;

volatile bool d1InterruptTriggered = false;
unsigned long lastD1InterruptTime = 0;
const unsigned long debounceDelay = 100;  // 100 ms debounce
bool SilenceEnabled = false;

bool faultActive = false;
unsigned long lastBuzzerToggleTime = 0;
bool buzzerState = false;
const unsigned long buzzerOnTime = 2000;   // 2 second ON
const unsigned long buzzerOffTime = 10000;  // 10 seconds OFF

String lastReceivedMessage = "";
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000;

unsigned long lastSignalCheckTime = 0;
const unsigned long signalCheckInterval = 10UL * 60UL * 1000UL; // 10 minutes
int cachedSignalStrength = -1;

unsigned long lastTX1AliveTime = 0;
unsigned long lastTX2AliveTime = 0;
const unsigned long transmitterTimeout = 15UL * 60UL * 1000UL;  // 15 minutes
bool TX1_DeadShown = false;
bool TX2_DeadShown = false;

String MQTTconnection = "";
String GPRSconnection = "";

bool Buzzersilence[4] = {false, false, false, false};
bool ActivatedBuzzerInput[4] = {false, false, false, false};

//IDENTFY TX1 AND TX2 NODES ALARMS SEPERATELY
bool TX1_Fire = false;
bool TX2_Fire = false;
bool TX1_Fault = false;
bool TX2_Fault = false;
bool TX1_Alive = true;
bool TX2_Alive = true;

int txID = 0; // 0 = unknown, 1 = TX1, 2 = TX2

unsigned long lastScrollTime = 0;
const unsigned long scrollDelay = 3000; // 3s per screen
int scrollIndex = 0; // Which screen is currently displayed

String firmware_url = "https://raw.githubusercontent.com/IndustrialArduino/NSD-FACP-Updates/release/Receiver_Panel.bin";

void IRAM_ATTR onD1FallingEdge() {
  unsigned long now = millis();
  if (now - lastD1InterruptTime > debounceDelay) {
    d1InterruptTriggered = true;
    lastD1InterruptTime = now;
  }
}

void mqttCallback(char* topic, String payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  String topicStr = String(topic);

  // Identify TX source
  if (topicStr.indexOf("TX1") != -1) {
    txID = 1;
  }
  else if (topicStr.indexOf("TX2") != -1) {
    txID = 2;
  }

  if (txID == 0) {
    SerialMon.println("[MQTT Action] Unknown TX source");
    return;
  }

  // Determine which type of alarm it is
  if (topicStr.endsWith("/FIRE_ALARM")) {
    SerialMon.println("Received a FIRE ALARM SIGNAL!");
    handleFireAlarm(payload);
  } else if (topicStr.endsWith("/FAULT_ALARM")) {
    SerialMon.println("Received a FAULT ALARM SIGNAL!");
    handleFaultAlarm(payload);
  } else if (topicStr.endsWith("/TRANSMITTER_ALIVE_STATUS")) {
    SerialMon.println("Received a TX ALIVE SIGNAL!");
    handleAliveSignal(payload);
  } else {
    SerialMon.println("Unknown topic type.");
  }
}

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
  pinMode(INPUT1,  INPUT_PULLUP);
  pinMode(INPUT2,  INPUT);
  pinMode(INPUT3,  INPUT);
  pinMode(INPUT4,  INPUT);
  pinMode(INPUT5,  INPUT);
  pinMode(INPUT6,  INPUT);
  pinMode(INPUT_MOTION,  INPUT);

  attachInterrupt(digitalPinToInterrupt(INPUT1), onD1FallingEdge, FALLING);

  Wire.begin(SDA, SCL);
  delay(100);

  lcd.begin(16, 4);

  if (io.begin() == false)
  {
    Serial.println("PCA9536 not detected. Please check wiring. Freezing...");
    while (1);
  }

  io.pinMode(IO_RL1, OUTPUT);
  io.pinMode(IO_RL2, OUTPUT);
  io.pinMode(IO_TR1, OUTPUT);
  io.pinMode(IO_TR2, OUTPUT);

  Init();
  connectToGPRS();
  connectToMQTT();

  xTaskCreatePinnedToCore(
    Buzzer_task,
    "Buzzer_task",
    10000,
    NULL,
    0,   //piority
    NULL,
    0  // Core 0
  );

}

void Buzzer_task(void *parameter) {
  for (;;) {
    handleBuzzerPulse();
  }
}

void loop() {
  checkForMessage();
  readInputsAndCheckAlarms();
  checkSilenceButton();
  checkTransmitterAlive();
  isGPRSConnected();
  maintainMQTTConnection();

  if (millis() - lastDisplayUpdate > displayInterval) {
    updateLCD16x4();
    lastDisplayUpdate = millis();
  }

}

void readInputsAndCheckAlarms() {
  unsigned long now = millis();
  currentState = digitalRead(INPUT3);

  if (currentState != lastStableState) {
    if ((now - lastDebounceTime) > DEBOUNCE_DELAY) {
      lastDebounceTime = now;
      lastStableState = currentState;

      inputStatus = interpretAlarmStatus(currentState);
      handleAlarmStateChange(inputStatus);
    }
  }

}

bool interpretAlarmStatus( bool state) {
  return state == HIGH;          //  Mains
}

void handleAlarmStateChange(bool status) {
  String topic;
  String payload = String(status);

  String smsMessage = smsMessages[status ? 0 : 1]; // Get proper message
  Serial.println(String("[STATE CHANGE] Alarm ") + (status ? "ACTIVATED" : "CLEARED") + " -> " + smsMessage);

  // Send SMS
  sendSMS(smsMessage);

  Serial.println(status ? "[ALARM] MAINS Fail" : "[NORMAL] MAINS OK");

  publishToMQTT(mqttConfig.mainsFailTopic, payload);

}

void publishToMQTT(String topic, String payload) {
  String command = "AT+QMTPUBEX=0,1,1,0,\"" + topic + "\"," + String(payload.length());
  gsm_send_serial(command, 1000);
  gsm_send_serial(payload + "\x1A", 1000);
}

void sendSMS(String message) {
  for (int i = 0; i < 1; i++) {
    gsm_send_serial("AT+CMGF=1", 1000); // Set SMS text mode
    gsm_send_serial("AT+CSCS=\"GSM\"", 500); // Use GSM character set

    String cmd = String("AT+CMGS=\"") + phoneNumbers[i] + "\"";
    gsm_send_serial(cmd, 1000);
    gsm_send_serial(message + "\x1A", 5000); // Send message with Ctrl+Z
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
  lastReceivedMessage = messageBody;
  handleSMS(messageBody);

  // Delete processed message
  gsm_send_serial("AT+CMGD=" + String(smsIndex), 1000);
}

void handleSMS(String message) {
  message.trim();
  message.toLowerCase();

  // Special case: OTA update (no TX number)
  if (message.indexOf("update the receiver panel") != -1) {
    Serial.println("[SMS Action] OTA UPDATE");
    performOTA();
    return;
  }

  // Identify TX source
  if (message.indexOf("tx1") != -1) {
    txID = 1;
  }
  else if (message.indexOf("tx2") != -1) {
    txID = 2;
  }

  if (txID == 0) {
    Serial.println("[SMS Action] Unknown TX source");
    return;
  }

  // Fire Alarm
  if (message.indexOf("fire alarm activated") != -1) {
    Serial.print("[SMS Action] TX");
    Serial.print(txID);
    Serial.println(" FIRE ALARM ACTIVATED");

    if (txID == 1) {
      TX1_Fire = true;
    } else {
      TX2_Fire = true;
    }

    io.digitalWrite(IO_RL2, HIGH); // Local relay
    if (!Buzzersilence[3]) {
      io.digitalWrite(IO_RL1, HIGH); // Local bell
      Serial.println("LOCAL BELL ON");
      ActivatedBuzzerInput[3] = true;
    }
  }
  else if (message.indexOf("fire alarm cleared") != -1) {
    Serial.print("[SMS Action] TX");
    Serial.print(txID);
    Serial.println(" FIRE ALARM CLEARED");

    if (txID == 1) {
      TX1_Fire = false;
    } else {
      TX2_Fire = false;
    }

    io.digitalWrite(IO_RL2, LOW);
    io.digitalWrite(IO_RL1, LOW);
    ActivatedBuzzerInput[3] = false;
  }

  // Fault Alarm
  else if (message.indexOf("fault reported") != -1) {
    Serial.print("[SMS Action] TX");
    Serial.print(txID);
    Serial.println(" FAULT ACTIVATED");

    if (txID == 1) {
      TX1_Fault = true;
    } else {
      TX2_Fault = true;
    }

    if (!Buzzersilence[0]) {
      faultActive = true;
      lastBuzzerToggleTime = millis();
      buzzerState = false;
      digitalWrite(BUZZER, LOW); // Start from OFF
      ActivatedBuzzerInput[0] = true;
    }
  }
  else if (message.indexOf("fault cleared") != -1) {
    Serial.print("[SMS Action] TX");
    Serial.print(txID);
    Serial.println(" FAULT CLEARED");

    if (txID == 1) {
      TX1_Fault = false;
    } else {
      TX2_Fault = false;
    }

    Buzzersilence[0] = false;
    digitalWrite(BUZZER, LOW);
    faultActive = false;
    ActivatedBuzzerInput[0] = false;
  }

  if (txID == 1) {
    TX1_Alive = true;
    lastTX1AliveTime = millis();
    TX1_DeadShown = false;
  } else {
    TX2_Alive = true;
    lastTX2AliveTime = millis();
    TX2_DeadShown = false;
  }
}

void checkTransmitterAlive() {
  unsigned long now = millis();

  // Check TX1
  if ((now - lastTX1AliveTime > transmitterTimeout) && !TX1_DeadShown) {
    Serial.println("[Warning] TX1 Transmitter is dead!");
    lastReceivedMessage = "TX1 Transmitter is dead!";
    TX1_Alive = false;
    TX1_DeadShown = true;
  }

  // Check TX2
  if ((now - lastTX2AliveTime > transmitterTimeout) && !TX2_DeadShown) {
    Serial.println("[Warning] TX2 Transmitter is dead!");
    lastReceivedMessage = "TX2 Transmitter is dead!";
    TX2_Alive = false;
    TX2_DeadShown = true;
  }
}

void checkSilenceButton() {
  if (d1InterruptTriggered) {
    d1InterruptTriggered = false;

    Serial.println("[INTERRUPT] D1 pressed. Silencing bell.");
    io.digitalWrite(IO_RL1, LOW);    // Turn off Local Bell
    digitalWrite(BUZZER, LOW);
    faultActive = false;     // Stop buzzer pulse if any
    if (ActivatedBuzzerInput[0])Buzzersilence[0] = true;
    if (ActivatedBuzzerInput[1])Buzzersilence[1] = true;
    if (ActivatedBuzzerInput[2])Buzzersilence[2] = true;
    if (ActivatedBuzzerInput[3])Buzzersilence[3] = true;

  }
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

void handleIncomingMessages(String line) {

  line.trim();
  if (!line.startsWith("+QMTRECV:")) return;

  // Parse MQTT message
  String messagePart = line.substring(line.indexOf(':') + 1);
  int firstComma = messagePart.indexOf(',');
  int secondComma = messagePart.indexOf(',', firstComma + 1);
  String client_idx = messagePart.substring(0, firstComma);
  String msg_id = messagePart.substring(firstComma + 1, secondComma);

  int firstQuote = messagePart.indexOf('"', secondComma + 1);
  int secondQuote = messagePart.indexOf('"', firstQuote + 1);
  String topic = messagePart.substring(firstQuote + 1, secondQuote);

  int thirdComma = messagePart.indexOf(',', secondQuote + 1);
  int fourthComma = messagePart.indexOf(',', thirdComma + 1);
  int payloadLength = messagePart.substring(thirdComma + 1, fourthComma).toInt();

  int thirdQuote = messagePart.indexOf('"', fourthComma + 1);
  int fourthQuote = messagePart.indexOf('"', thirdQuote + 1);
  String payload = messagePart.substring(thirdQuote + 1, fourthQuote);

  // Forward to MQTT callback
  char topicArr[topic.length() + 1];
  topic.toCharArray(topicArr, topic.length() + 1);

  mqttCallback(topicArr, payload, payload.length());
}

void handleFireAlarm(String payload) {
  int state = payload.toInt();
  SerialMon.print("FIRE_ALARM STATE: ");
  SerialMon.println(state);

  if (state) {
    SerialMon.println("Fire Alarm Activated at Cooling Plant");
    if (txID == 1) {
      TX1_Fire = true;
    } else {
      TX2_Fire = true;
    }
    lastReceivedMessage = "FACP - Fire Alarm Activated at Cooling Plant";
    io.digitalWrite(IO_RL2, HIGH);
    if (!Buzzersilence[3]) {
      io.digitalWrite(IO_RL1, HIGH);;
      Serial.println("LOCAL BELL ON");
      ActivatedBuzzerInput[3] = true;
    }
  } else {
    SerialMon.println("Fire Alarm Cleared at Cooling Plant");
    if (txID == 1) {
      TX1_Fire = false;
    } else {
      TX2_Fire = false;
    }
    lastReceivedMessage = "FACP - Fire Alarm CLEARED at Cooling Plant";
    Buzzersilence[3] = false;
    io.digitalWrite(IO_RL2, LOW);
    io.digitalWrite(IO_RL1, LOW);
    SilenceEnabled = false;
    ActivatedBuzzerInput[3] = false;
  }
  if (txID == 1) {
    TX1_Alive = true;
    lastTX1AliveTime = millis();
    TX1_DeadShown = false;
  } else {
    TX2_Alive = true;
    lastTX2AliveTime = millis();
    TX2_DeadShown = false;
  }
}

void handleFaultAlarm(String payload) {
  int state = payload.toInt();
  SerialMon.print("FAULT_ALARM STATE: ");
  SerialMon.println(state);

  if (state) {
    SerialMon.println("Fault reported at Cooling Plant");
    if (txID == 1) {
      TX1_Fault = true;
    } else {
      TX2_Fault = true;
    }
    lastReceivedMessage = "FACP - Fault reported at Cooling Plant";
    if (!Buzzersilence[0]) {
      faultActive = true;
      lastBuzzerToggleTime = millis();
      buzzerState = false;
      digitalWrite(BUZZER, LOW); // Start from OFF
      ActivatedBuzzerInput[0] = true;
    }
  } else {
    SerialMon.println("Fault cleared at Cooling Plant");
    if (txID == 1) {
      TX1_Fault = false;
    } else {
      TX2_Fault = false;
    }
    lastReceivedMessage = "FACP - Fault cleared at Cooling Plant";
    Buzzersilence[0] = false;
    digitalWrite(BUZZER, LOW);
    faultActive = false;
    ActivatedBuzzerInput[0] = false;
  }
  if (txID == 1) {
    TX1_Alive = true;
    lastTX1AliveTime = millis();
    TX1_DeadShown = false;
  } else {
    TX2_Alive = true;
    lastTX2AliveTime = millis();
    TX2_DeadShown = false;
  }
}

void handleAliveSignal(String payload) {
  int state = payload.toInt();
  SerialMon.print("ALIVE_STATUS ");
  SerialMon.println(state);

  if (txID == 1) {
    TX1_Alive = (state == 1);
    lastTX1AliveTime = millis();
    TX1_DeadShown = false;
    Serial.println("TX1 ALIVE update received");
  }
  else if (txID == 2) {
    TX2_Alive = (state == 1);
    lastTX2AliveTime = millis();
    TX2_DeadShown = false;
    Serial.println("TX2 ALIVE update received");
  }
}

void handleBuzzerPulse() {
  if (!faultActive) {
    digitalWrite(BUZZER, LOW);  // Make sure buzzer is off
    buzzerState = false;
    return;
  }

  unsigned long now = millis();
  unsigned long interval = buzzerState ? buzzerOnTime : buzzerOffTime;

  if (now - lastBuzzerToggleTime >= interval) {
    buzzerState = !buzzerState;
    digitalWrite(BUZZER, buzzerState ? HIGH : LOW);
    lastBuzzerToggleTime = now;
  }
}

void Init(void) {                        // Connecting with the network and GPRS
  delay(5000);
  gsm_send_serial("AT+CFUN=1", 10000);
  gsm_send_serial("AT+CPIN?", 10000);
  gsm_send_serial("AT+GMR", 1000);
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
  // gsm_send_serial("AT+CGDCONT=1,\"IP\",\"dialogbb\"", 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
}

void connectToMQTT(void) {
  // Initialize MQTT configurations
  gsm_send_serial("AT+QMTCLOSE=0", 1000);
  gsm_send_serial("AT+QMTCFG=\"recv/mode\",0,0,1", 1000);
  gsm_send_serial("AT+QMTCFG=\"SSL\",0,1,2", 1000);
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
  String mqtt_conn = "AT+QMTCONN=0,\"Receiver_panel\",\"" + username + "\",\"" + password + "\"";
  gsm_send_serial(mqtt_conn, 1000);
  delay(2000); // Wait for the connection to establish


  String sub1 = "AT+QMTSUB=0,0,\"" + String(mqttConfig.TX1_fireTopic) + "\",0";
  gsm_send_serial(sub1, 1000);
  delay(1000);

  String sub2 = "AT+QMTSUB=0,1,\"" + String(mqttConfig.TX1_faultTopic) + "\",0";
  gsm_send_serial(sub2, 1000);
  delay(1000);

  String sub3 = "AT+QMTSUB=0,2,\"" + String(mqttConfig.TX1_aliveTopic) + "\",0";
  gsm_send_serial(sub3, 1000);
  delay(1000);

  String sub4 = "AT+QMTSUB=0,3,\"" + String(mqttConfig.TX2_fireTopic) + "\",0";
  gsm_send_serial(sub4, 1000);
  delay(1000);

  String sub5 = "AT+QMTSUB=0,4,\"" + String(mqttConfig.TX2_faultTopic) + "\",0";
  gsm_send_serial(sub5, 1000);
  delay(1000);

  String sub6 = "AT+QMTSUB=0,5,\"" + String(mqttConfig.TX2_aliveTopic) + "\",0";
  gsm_send_serial(sub6, 1000);
  delay(1000);


  // Debug: Print MQTT connection status
  String connStatus = gsm_send_serial("AT+QMTCONN?", 1000);
  SerialMon.print("MQTT Connection Status: ");
  SerialMon.println(connStatus);
}

//16X4 LCD DISPLAY UPDATES
void updateLCD16x4() {
  DateTime now = rtc.now();
  lcd.clear();

  switch (scrollIndex) {
    case 0: // --- TX1 status ---
      // Row 0: Date/Time
      char row0[17];
      snprintf(row0, 17, "Node:%s %02d/%02d %02d:%02d", nodeName,
               now.day(), now.month(), now.hour(), now.minute());
      lcd.setCursor(0, 0);
      lcd.print(row0);

      lcd.setCursor(0, 1);
      lcd.print(String("TX1 Fire ") + (TX1_Fire ? "Alarm" : "Normal"));

      lcd.setCursor(0, 2);
      lcd.print(String("TX1 Fault ") + (TX1_Fault ? "Alarm" : "Normal"));

      lcd.setCursor(0, 3);
      lcd.print(String("TX1 Alive ") + (TX1_Alive ? "Yes" : "No"));
      break;

    case 1: // --- TX2 status ---
      // Row 0: Date/Time
      snprintf(row0, 17, "Node:%s %02d/%02d %02d:%02d", nodeName,
               now.day(), now.month(), now.hour(), now.minute());
      lcd.setCursor(0, 0);
      lcd.print(row0);

      lcd.setCursor(0, 1);
      lcd.print(String("TX2 Fire ") + (TX2_Fire ? "Alarm" : "Normal"));

      lcd.setCursor(0, 2);
      lcd.print(String("TX2 Fault ") + (TX2_Fault ? "Alarm" : "Normal"));

      lcd.setCursor(0, 3);
      lcd.print(String("TX2 Alive ") + (TX2_Alive ? "Yes" : "No"));
      break;

    case 2: // --- RX + GSM/GPRS/MQTT ---
      lcd.setCursor(0, 0);
      lcd.print(String("RX Mains ") + (inputStatus ? "Alarm" : "Normal"));

      lcd.setCursor(0, 1);
      lcd.print("Signal: " + String(getGSMSignalStrength()));

      lcd.setCursor(0, 2);
      lcd.print("GPRS: " + String(GPRSconnection));

      lcd.setCursor(0, 3);
      lcd.print("MQTT: " + String(MQTTconnection));
      break;
  }

  // --- Automatic scrolling ---
  unsigned long nowMillis = millis();
  if (nowMillis - lastScrollTime >= scrollDelay) {
    scrollIndex = (scrollIndex + 1) % 3; // rotate through 3 screens
    lastScrollTime = nowMillis;
  }
}

String getNetworkTime() {
  String response = gsm_send_serial("AT+QLTS=1", 2000);
  // Expected response: +QLTS: "2025/09/02,15:48:30+08"
  int idx = response.indexOf("+QLTS: \"");
  if (idx >= 0) {
    String t = response.substring(idx + 8, idx + 27); // Extract "YYYY/MM/DD,HH:MM:SS+TZ"
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

  // Parse timezone in 15-min intervals
  int tzIndex = t.indexOf(',', 20); // find first comma after time
  int tzRaw = t.substring(20, tzIndex).toInt();
  int tzMinutes = tzRaw * 15; // convert to minutes
  int tzHours = tzMinutes / 60;
  int tzMins = tzMinutes % 60;

  // Adjust hh/mi to local time
  hh += tzHours;
  mi += tzMins;
  if (mi >= 60) {
    mi -= 60;
    hh += 1;
  }
  if (hh >= 24) {
    hh -= 24;  // Simplified: ignore month overflow
    dd += 1;
  }

  rtc.adjust(DateTime(yyyy, mm, dd, hh, mi, ss));
  Serial.println("RTC synced with network time (adjusted for TZ).");
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
      sendSMS("FACP-Receiver Panel Updated Successfully");
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
    Serial.println("[WARNING] MQTT Disconnected. Reconnecting...");
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

      // Handle MQTT lines
      if (line.startsWith("+QMTRECV:")) {
        handleIncomingMessages(line); // Reuse MQTT message parser
        continue; // Do not append to buffer
      }

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
