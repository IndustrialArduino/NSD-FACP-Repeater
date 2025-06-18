#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  // Include the WiFi library for MAC address
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include "Secret.h" // Include file to get the username and password of MQTT server
#include "Configurations.h"
#include"datacake.h"

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

bool silenceButtonPressed = false;
bool lastSilenceButtonState = HIGH;
unsigned long lastSilenceDebounceTime = 0;
const unsigned long silenceDebounceDelay = 50;

bool faultActive = false;
unsigned long lastBuzzerToggleTime = 0;
bool buzzerState = false;
const unsigned long buzzerOnTime = 1000;   // 1 second ON
const unsigned long buzzerOffTime = 5000;  // 5 seconds OFF

// Alarm messages
const char* smsMessages[2] = {
  "FACP Remote monitoring panel – Mains Fail Alarm", "FACP Remote monitoring panel – Mains Fail Alarm cleared"
};

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
  } else {
    SerialMon.println("Unknown topic type.");
  }
}

void setup() {
  // Set console baud rate
  Serial.begin(115200);
  delay(10);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

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

  digitalWrite(R0, LOW);digitalWrite(R1, LOW);digitalWrite(R2, LOW);digitalWrite(R3, LOW);digitalWrite(R4, LOW);

  Wire.begin(I2C_SDA,I2C_SCL);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  updateOLED();
  Init();
  connectToGPRS();
  connectToMQTT();
}

void loop() {

  checkForMessage();
  readInputsAndCheckAlarms();
  checkSilenceButton(); 
  handleBuzzerPulse();
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

    if (currentState!= lastStableState) {
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

}

void publishToMQTT(String topic, String payload) {
  String command = "AT+QMTPUBEX=0,1,1,1,\"" + topic + "\"," + String(payload.length());
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
    digitalWrite(R1, HIGH);
  }
  else if (message.indexOf("fire alarm cleared") != -1) {
    Serial.println("[SMS Action] FIRE ALARM OFF");
    digitalWrite(R0, LOW);
    digitalWrite(R1, LOW);
  }
  else if (message.indexOf("fault reported") != -1) {
    Serial.println("[SMS Action] FAULT ON");
    digitalWrite(R2, HIGH);
    faultActive = true;
    lastBuzzerToggleTime = millis();
    buzzerState = false;
    digitalWrite(R3, LOW); // Start from OFF
  }
  else if (message.indexOf("fault cleared") != -1) {
    Serial.println("[SMS Action] FAULT OFF");
    digitalWrite(R2, LOW);
    digitalWrite(R3, LOW);
    faultActive = false;
  }else if (message.indexOf("transmitter is alive") != -1) {
    Serial.println("[SMS Action] TRANSMITTER ALIVE RECEIVED");
    digitalWrite(R4, LOW);
    lastTransmitterAliveTime = millis();
    transmitterDeadShown = false; // Reset flag to allow showing again if needed
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
  bool reading = digitalRead(D1);

  if (reading != lastSilenceButtonState) {
    lastSilenceDebounceTime = millis();
    lastSilenceButtonState = reading;
  }

  if ((millis() - lastSilenceDebounceTime) > silenceDebounceDelay) {
    if (reading == LOW && !silenceButtonPressed) { // Button was just pressed
      silenceButtonPressed = true;

      Serial.println("[ACTION] Silence button pressed. Turning OFF local bell.");
      digitalWrite(R1, LOW); // Turn off Relay Output-2 (Local Bell)
    }

    if (reading == HIGH) {
      silenceButtonPressed = false; // Reset to allow next press
    }
  }
}

void maintainMQTTConnection() {
  String status = gsm_send_serial("AT+QMTCONN?", 1000);
  if (status.indexOf("+QMTCONN: 0,3") == -1) {
    Serial.println("[WARNING] MQTT Disconnected. Reconnecting...");
    MQTTconnection = "Disconnected";
    connectToMQTT();
  }else{
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

  if(state){
    SerialMon.println("Fire Alarm Activated at Cooling Plant");
    lastReceivedMessage = "FACP - Fire Alarm Activated at Cooling Plant";
    digitalWrite(R0, HIGH);
    digitalWrite(R1, HIGH);
  }else{
    SerialMon.println("Fire Alarm Cleared at Cooling Plant");
    lastReceivedMessage = "FACP - Fire Alarm CLEARED at Cooling Plant";
    digitalWrite(R0, LOW);
    digitalWrite(R1, LOW);   
  }
   
   }

void handleFaultAlarm(String payload) {
  int state = payload.toInt();
  SerialMon.print("FAULT_ALARM STATE: ");
  SerialMon.println(state);

  if(state){
    SerialMon.println("Fault reported at Cooling Plant");
    lastReceivedMessage = "FACP - Fault reported at Cooling Plant";
    digitalWrite(R2, HIGH);
    faultActive = true;         // Enable pulsing
    lastBuzzerToggleTime = millis();  // Reset buzzer timer
    buzzerState = false;
    digitalWrite(R3, LOW);      // Start from buzzer OFF
  }else{
    SerialMon.println("Fault cleared at Cooling Plant");
    lastReceivedMessage = "FACP - Fault cleared at Cooling Plant";
    digitalWrite(R2, LOW);
    digitalWrite(R3, LOW); 
    faultActive = false;  
  }
}

void handleBuzzerPulse() {
  if (!faultActive) return;

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
  gsm_send_serial("AT+QMTDISC=0",1000);
  delay(1000);
  String mqtt_conn = "AT+QMTCONN=0,\"Receiver_panel\",\"" + username + "\",\"" + password + "\"";
  gsm_send_serial(mqtt_conn, 1000);
  delay(2000); // Wait for the connection to establish

  // Subscribe to both topics
  String topic1 = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/FIRE_ALARM";
  String topic2 = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/FAULT_ALARM";
  
  String sub1 = "AT+QMTSUB=0,0,\"" + topic1 + "\",0";
  gsm_send_serial(sub1, 1000);
  delay(1000);

  String sub2 = "AT+QMTSUB=0,1,\"" + topic2 + "\",0";
  gsm_send_serial(sub2, 1000);
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

bool isNetworkConnected() {
  String response = gsm_send_serial("AT+CREG?", 3000);
  return (response.indexOf("+CREG: 0,1") != -1 || response.indexOf("+CREG: 0,5") != -1);
}

void isGPRSConnected() {
  String status = gsm_send_serial("AT+CGATT?", 3000);
  if (status.indexOf("+CGATT: 1") == -1){
    Serial.println("[WARNING] MQTT Disconnected. Reconnecting...");
    GPRSconnection = "Deactive"; 
    connectToGPRS();
  }else{
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
