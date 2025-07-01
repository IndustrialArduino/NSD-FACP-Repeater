#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  // Include the WiFi library for MAC address
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <Update.h>
#include "Secret.h" // Include file to get the username and password of MQTT server
#include "Configurations.h"
#include "Message_Content.h"
#include "datacake.h"
#include "github.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

String gsm_send_serial(String command, int delay);

#define SerialMon Serial
#define SerialAT Serial1
#define GSM_PIN ""

// MQTT details
String broker = "mqtt.datacake.co";
String MQTTport = "8883";

#define UART_BAUD 115200

#define MODEM_TX 32
#define MODEM_RX 33
#define GSM_RESET 21

#define I2C_SDA 16
#define I2C_SCL 17

#define D1 34 // Silence Bell
#define D2 35 // Mains_Fails_Alarm_Input
#define D3 14
#define D4 13
#define D5 5
#define R0 12  // RED Indicator
#define R1 2   // Local Bell
#define R2 27  // Yellow indicator
#define R3 4   // Buzzer
#define R4 23  //TX ALIVE INDICATOR
#define R5 18

bool RMSA_Status = false ;
bool inputStatus = false; // FEA, FTA, MSA

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
const unsigned long buzzerOnTime = 1000;   // 1 second ON
const unsigned long buzzerOffTime = 5000;  // 5 seconds OFF

String lastReceivedMessage = "";
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000;

unsigned long lastSignalCheckTime = 0;
const unsigned long signalCheckInterval = 10UL * 60UL * 1000UL; // 10 minutes
int cachedSignalStrength = -1;

unsigned long lastTransmitterAliveTime = 0;
const unsigned long transmitterTimeout = 30UL * 60UL * 1000UL;  // 30 minutes
bool transmitterDeadShown = false;

String MQTTconnection = "";
String GPRSconnection = "";

bool TX_Fault = false;
bool Tx_Main_Fails = false;
bool Rx_Main_Fails = false;
bool Buzzersilence[4] = {false, false, false};
bool ActivatedBuzzerInput[4] = {false, false, false};

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

  // Determine which type of alarm it is
  if (topicStr.endsWith("/FIRE_ALARM")) {
    SerialMon.println("Received a FIRE ALARM SIGNAL!");
    handleFireAlarm(payload);
  } else if (topicStr.endsWith("/FAULT_ALARM")) {
    SerialMon.println("Received a FAULT ALARM SIGNAL!");
    handleFaultAlarm(payload);
  } else if (topicStr.endsWith("/MAINS_FAIL_ALARM")) {
    SerialMon.println("Received a MAINS POWER FAILS ALARM SIGNAL!");
    handleMainFailsAlarm(payload);
  }
  else if (topicStr.endsWith("/TRANSMITTER_ALIVE_STATUS")) {
    SerialMon.println("Received a TX ALIVE SIGNAL!");
    handleAliveSignal(payload);
  } else {
    SerialMon.println("Unknown topic type.");
  }
}

void setup() {
  // Set console baud rate
  Serial.begin(115200);
  delay(10);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX, false, 2048);


  delay(2000);
  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);  // RS-485
  delay(2000);

  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(R0, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(R3, OUTPUT);
  pinMode(R4, OUTPUT);
  pinMode(R5, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(D1), onD1FallingEdge, FALLING);

  digitalWrite(R0, LOW); digitalWrite(R1, LOW); digitalWrite(R2, LOW); digitalWrite(R3, LOW); digitalWrite(R4, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.display();
  updateOLED();
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
    updateOLED();
    lastDisplayUpdate = millis();
  }

}

void readInputsAndCheckAlarms() {
  unsigned long now = millis();
  currentState = digitalRead(D2);

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

  topic = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/RECEIVE_MAINS_FAIL_ALARM";
  Serial.println(status ? "[ALARM] MAINS Fail" : "[NORMAL] MAINS OK");

  publishToMQTT(topic, payload);
  handleRXMainFailsAlarm(payload);

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

  if (message.indexOf("fire alarm activated") != -1) {
    Serial.println("[SMS Action] FIRE ALARM ON");
    digitalWrite(R0, HIGH);
    if (!Buzzersilence[3]) {
      digitalWrite(R1, HIGH);
      Serial.println("LOCAL BELL ON");
      ActivatedBuzzerInput[3] = true;
    }
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false;
  }
  else if (message.indexOf("fire alarm cleared") != -1) {
    Serial.println("[SMS Action] FIRE ALARM OFF");
    Buzzersilence[3] = false;
    digitalWrite(R0, LOW);
    digitalWrite(R1, LOW);
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false;
    ActivatedBuzzerInput[3] = false;
  }
  else if (message.indexOf("fault reported") != -1) {
    Serial.println("[SMS Action] FAULT ON");
    digitalWrite(R2, HIGH);
    TX_Fault = true;
    if (!Buzzersilence[0]) {
      faultActive = true;
      lastBuzzerToggleTime = millis();
      buzzerState = false;
      digitalWrite(R3, LOW); // Start from OFF
      ActivatedBuzzerInput[0] = true;
    }
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false;
  }
  else if (message.indexOf("fault cleared") != -1) {
    Serial.println("[SMS Action] FAULT OFF");
    TX_Fault = false;
    if ((!TX_Fault) && (!Tx_Main_Fails) && (!Rx_Main_Fails)) {
      digitalWrite(R2, LOW);
    }
    Buzzersilence[0] = false;
    digitalWrite(R3, LOW);
    faultActive = false;
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false;
    ActivatedBuzzerInput[0] = false;
  }
  else if (message.indexOf("mains power fail reported") != -1) {
    Serial.println("[SMS Action] MAINS FAIL ON");
    digitalWrite(R2, HIGH);
    Tx_Main_Fails = true;
    if (!Buzzersilence[1]) {
      faultActive = true;
      lastBuzzerToggleTime = millis();
      buzzerState = false;
      digitalWrite(R3, LOW); // Start from OFF
      ActivatedBuzzerInput[1] = true;
    }
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false;
  }
  else if (message.indexOf("mains power fail cleared") != -1) {
    Serial.println("[SMS Action] MAINS FAIL OFF");
    Tx_Main_Fails = false;
    if ((!TX_Fault) && (!Tx_Main_Fails) && (!Rx_Main_Fails)) {
      digitalWrite(R2, LOW);
    }
    Buzzersilence[1] = false;
    digitalWrite(R3, LOW);
    faultActive = false;
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false;
    ActivatedBuzzerInput[1] = false;
  }
  else if (message.indexOf("update the receiver panel") != -1) {
    Serial.println("[SMS Action] OTA UPDATE");
    performOTA();
  }
}

void checkTransmitterAlive() {
  if (millis() - lastTransmitterAliveTime > transmitterTimeout && !transmitterDeadShown) {
    Serial.println("[Warning] Transmitter is dead!");
    digitalWrite(R4, HIGH);
    lastReceivedMessage = "Transmitter is dead!";
    transmitterDeadShown = true;
  }
}


void checkSilenceButton() {
  if (d1InterruptTriggered) {
    d1InterruptTriggered = false;

    Serial.println("[INTERRUPT] D1 pressed. Silencing bell.");
    digitalWrite(R1, LOW);   // Turn off Local Bell
    digitalWrite(R3, LOW);
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
    MQTTconnection = "Disconnected";
    connectToMQTT();
  } else {
    MQTTconnection = "Connected";
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
    lastReceivedMessage = "FACP - Fire Alarm Activated at Cooling Plant";
    digitalWrite(R0, HIGH);
    if (!Buzzersilence[3]) {
      digitalWrite(R1, HIGH);
      Serial.println("LOCAL BELL ON");
      ActivatedBuzzerInput[3] = true;
    }
  } else {
    SerialMon.println("Fire Alarm Cleared at Cooling Plant");
    lastReceivedMessage = "FACP - Fire Alarm CLEARED at Cooling Plant";
    Buzzersilence[3] = false;
    digitalWrite(R0, LOW);
    digitalWrite(R1, LOW);
    SilenceEnabled = false;
    ActivatedBuzzerInput[3] = false;
  }
  lastTransmitterAliveTime = millis();
  transmitterDeadShown = false;
}

void handleFaultAlarm(String payload) {
  int state = payload.toInt();
  SerialMon.print("FAULT_ALARM STATE: ");
  SerialMon.println(state);

  if (state) {
    SerialMon.println("Fault reported at Cooling Plant");
    lastReceivedMessage = "FACP - Fault reported at Cooling Plant";
    digitalWrite(R2, HIGH);
    TX_Fault = true;
    if (!Buzzersilence[0]) {
      faultActive = true;
      lastBuzzerToggleTime = millis();
      buzzerState = false;
      digitalWrite(R3, LOW); // Start from OFF
      ActivatedBuzzerInput[0] = true;
    }
  } else {
    SerialMon.println("Fault cleared at Cooling Plant");
    lastReceivedMessage = "FACP - Fault cleared at Cooling Plant";
    TX_Fault = false;
    if ((!TX_Fault) && (!Tx_Main_Fails) && (!Rx_Main_Fails)) {
      digitalWrite(R2, LOW);
    }
    Buzzersilence[0] = false;
    digitalWrite(R3, LOW);
    faultActive = false;
    ActivatedBuzzerInput[0] = false;
  }
  lastTransmitterAliveTime = millis();
  transmitterDeadShown = false;
}

void handleMainFailsAlarm(String payload) {
  int state = payload.toInt();
  SerialMon.print("MAIN_FAILS_ALARM STATE: ");
  SerialMon.println(state);

  if (state) {
    SerialMon.println("Main power fails reported at Cooling Plant");
    lastReceivedMessage = "FACP - Main Power Fails reported at Cooling Plant";
    digitalWrite(R2, HIGH);
    Tx_Main_Fails = true;
    if (!Buzzersilence[1]) {
      faultActive = true;
      lastBuzzerToggleTime = millis();
      buzzerState = false;
      digitalWrite(R3, LOW); // Start from OFF
      ActivatedBuzzerInput[1] = true;
    }
  } else {
    SerialMon.println("Main power fails cleared at Cooling Plant");
    lastReceivedMessage = "FACP - Main Power Fails cleared at Cooling Plant";
    Tx_Main_Fails = false;
    if ((!TX_Fault) && (!Tx_Main_Fails) && (!Rx_Main_Fails)) {
      digitalWrite(R2, LOW);
    }
    Buzzersilence[1] = false;
    digitalWrite(R3, LOW);
    faultActive = false;
    ActivatedBuzzerInput[1] = false;
  }
  lastTransmitterAliveTime = millis();
  transmitterDeadShown = false;
}

void handleRXMainFailsAlarm(String payload) {
  int state = payload.toInt();
  SerialMon.print("RX_MAIN_FAILS_ALARM STATE: ");
  SerialMon.println(state);

  if (state) {
    SerialMon.println("Receiver Main power fails reported at Cooling Plant");
    lastReceivedMessage = "FACP - Receiver Main Power Fails reported at Cooling Plant";
    digitalWrite(R2, HIGH);
    Rx_Main_Fails = true;
    if (!Buzzersilence[2]) {
      faultActive = true;
      lastBuzzerToggleTime = millis();
      buzzerState = false;
      digitalWrite(R3, LOW); // Start from OFF
      ActivatedBuzzerInput[2] = true;
    }
  } else {
    SerialMon.println("Receiver Main power fails cleared at Cooling Plant");
    lastReceivedMessage = "FACP - Receiver Main Power Fails cleared at Cooling Plant";
    Rx_Main_Fails = false;
    if ((!TX_Fault) && (!Tx_Main_Fails) && (!Rx_Main_Fails)) {
      digitalWrite(R2, LOW);
    }
    Buzzersilence[2] = false;
    digitalWrite(R3, LOW);
    faultActive = false;
    ActivatedBuzzerInput[2] = false;
  }
  lastTransmitterAliveTime = millis();
  transmitterDeadShown = false;
}

void handleAliveSignal(String payload) {
  int state = payload.toInt();
  SerialMon.print("ALIVE_STATUS ");
  SerialMon.println(state);

  if (state) {
    Serial.println("TRANSMITTER ALIVE RECEIVED");
    digitalWrite(R4, LOW);
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false; // Reset flag to allow showing again if needed
  }
}

void handleBuzzerPulse() {
  if (!faultActive) {
    digitalWrite(R3, LOW);  // Make sure buzzer is off
    buzzerState = false;
    return;
  }

  unsigned long now = millis();
  unsigned long interval = buzzerState ? buzzerOnTime : buzzerOffTime;

  if (now - lastBuzzerToggleTime >= interval) {
    buzzerState = !buzzerState;
    digitalWrite(R3, buzzerState ? HIGH : LOW);
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

  // Subscribe to both topics
  String topic1 = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/FIRE_ALARM";
  String topic2 = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/FAULT_ALARM";
  String topic3 = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/MAINS_FAIL_ALARM";
  String topic4 = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/TRANSMITTER_ALIVE_STATUS";

  String sub1 = "AT+QMTSUB=0,0,\"" + topic1 + "\",0";
  gsm_send_serial(sub1, 1000);
  delay(1000);

  String sub2 = "AT+QMTSUB=0,1,\"" + topic2 + "\",0";
  gsm_send_serial(sub2, 1000);
  delay(1000);

  String sub3 = "AT+QMTSUB=0,2,\"" + topic3 + "\",0";
  gsm_send_serial(sub3, 1000);
  delay(1000);

  String sub4 = "AT+QMTSUB=0,3,\"" + topic4 + "\",0";
  gsm_send_serial(sub4, 1000);
  delay(2000);

  // Debug: Print MQTT connection status
  String connStatus = gsm_send_serial("AT+QMTCONN?", 1000);
  SerialMon.print("MQTT Connection Status: ");
  SerialMon.println(connStatus);
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.println("Receiver Panel");

  // Fire Alarm Status
  display.setCursor(0, 12);
  display.print("I1:");
  display.print(inputStatus ? "H " : "L ");

  display.print("R0:");
  display.print(digitalRead(R0) ? "On " : "Off ");

  display.print("R1:");
  display.print(digitalRead(R1) ? "On " : "Off ");

  display.setCursor(0, 24);
  display.print("R2:");
  display.print(digitalRead(R2) ? "On " : "Off ");

  display.print("R3:");
  display.print(digitalRead(R3) ? "On " : "Off ");

  display.print("R4:");
  display.print(digitalRead(R4) ? "On" : "Off");

  display.setCursor(0, 36);
  display.print("Signal: ");
  // Check if 10 minutes have passed
  unsigned long now = millis();
  if (now - lastSignalCheckTime >= signalCheckInterval || lastSignalCheckTime == 0) {
    lastSignalCheckTime = now;
    cachedSignalStrength = getGSMSignalStrength();
  }

  display.print(cachedSignalStrength);

  display.setCursor(0, 48);
  display.print("GPRS: ");
  display.print(GPRSconnection);

  display.setCursor(0, 56);
  display.print("MQTT: ");
  display.print(MQTTconnection);

  display.display();
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
