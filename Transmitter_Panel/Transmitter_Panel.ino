#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h> 
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include "Secret.h" // Include file to get the username and password of MQTT server
#include "datacake.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

String gsm_send_serial(String command, int delay);

#define SerialMon Serial
#define SerialAT Serial1
#define GSM_PIN ""

// Your GPRS credentials
const char apn[] = "dialogbb";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
String broker = "mqtt.datacake.co";
String MQTTport = "8883";

#define UART_BAUD 115200

#define MODEM_TX 32
#define MODEM_RX 33
#define GSM_RESET 21

#define I2C_SDA 16
#define I2C_SCL 17

#define D1 34 // Fire_Alarm_Input
#define D2 35 // Fault_Alarm_Input
#define D3 14 // Mains_Fails_Alarm_Input
#define D4 13
#define D5 5
#define R0 12
#define R1 2
#define R2 27
#define R3 4
#define R4 23
#define R5 18

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

String phonecall_phoneNumbers[3] = {"+94769164662", "+94741111111", "+94771111111"};

// Predefined phone numbers
const char* phoneNumbers[5] = {
   "+94769164662",
   "+94740432001",
   "+94701111111",
   "+94741111111",
   "+94781111111"
  
};

// Alarm messages
const char* smsMessages[3][2] = {
  { "FACP - Fire Alarm Activated at Cooling Plant", "FACP - Fire Alarm CLEARED at Cooling Plant" },
  { "FACP - Fault reported at Cooling Plant", "FACP - Fault cleared at Cooling Plant" },
  { "FACP – Mains Power Fail reported at Cooling Plant", "FACP - Mains Power Fail cleared at Cooling Plant" }
};

unsigned long lastAliveSMSSentTime = 0;
const unsigned long aliveSMSInterval = 1800000; // 30 minutes in milliseconds

const char* ReceiverPanelAlive = {"+94761111111" };
String MQTTconnection = "";
String GPRSconnection = "";

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
  pinMode(D3, INPUT_PULLUP); 
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(R0, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(R3, OUTPUT);
  pinMode(R4, OUTPUT);
  pinMode(R5, OUTPUT);

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
  if (processing) return;
  processing = true;

  readInputsAndCheckAlarms();
  isGPRSConnected();
  maintainMQTTConnection();

  processing = false;
  updateOLED();

  unsigned long currentAliveMillis = millis();
  if (currentAliveMillis - lastAliveSMSSentTime >= aliveSMSInterval) {
      sendAliveSMS();
      lastAliveSMSSentTime = currentAliveMillis;
  }
  
  delay(1000); 

}

void readInputsAndCheckAlarms() {
  unsigned long now = millis();
  currentState[0] = digitalRead(D1);
  currentState[1] = digitalRead(D2);
  currentState[2] = digitalRead(D3);

  for (int i = 0; i < 3; i++) {
    if (currentState[i] != lastStableState[i]) {
      if ((now - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
        lastDebounceTime[i] = now;
        lastStableState[i] = currentState[i];

        inputStatus[i] = interpretAlarmStatus(i, currentState[i]);
        handleAlarmStateChange(i, inputStatus[i]);
        break;
      }
    }
  }
}

bool interpretAlarmStatus(int idx, bool state) {
  if (idx == 0) return state == LOW;  // Fire
  else return state == HIGH;          // Fault / Mains
}

void handleAlarmStateChange(int idx, bool status) {
  String topic;
  String payload = String(status);

  String smsMessage = smsMessages[idx][status ? 0 : 1]; // Get proper message
  Serial.println("[STATE CHANGE] Alarm " + String(idx) + ": " + (status ? "ACTIVATED" : "CLEARED") + " -> " + smsMessage);

  // Send SMS
  sendSMS(smsMessage);

  switch (idx) {
    case 0:
      topic = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/FIRE_ALARM";
      Serial.println(status ? "[ALARM] FIRE Active" : "[NORMAL] FIRE Normal");
      break;
    case 1:
      topic = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/FAULT_ALARM";
      Serial.println(status ? "[ALARM] FAULT Active" : "[NORMAL] FAULT Normal");
      break;
    case 2:
      topic = "dtck-pub/nsd-facp-repeater-1/997a8398-0323-4c19-9633-7ecdd259d7e0/MAINS_FAIL_ALARM";
      Serial.println(status ? "[ALARM] MAINS Fail" : "[NORMAL] MAINS OK");
      break;
  }

  publishToMQTT(topic, payload);

  if (!status) return; // Only act on activation

  if (idx == 0) { // Fire
    makeAlarmCalls("Fire Alarm Activated at Cooling Plant");
  } else if (idx == 1) { // Fault
    makeAlarmCalls("Fault Activated at Cooling Plant");
  }

}

void sendSMS(String message) {
  for (int i = 0; i < 2; i++) {
    gsm_send_serial("AT+CMGF=1", 1000); // Set SMS text mode
    gsm_send_serial("AT+CSCS=\"GSM\"", 500); // Use GSM character set

    String cmd = String("AT+CMGS=\"") + phoneNumbers[i] + "\"";
    gsm_send_serial(cmd, 1000);
    gsm_send_serial(message + "\x1A", 5000); // Send message with Ctrl+Z
  }
}

void sendAliveSMS() {
  String message = "FACP - Transmitter is alive ";
  Serial.println("[HEARTBEAT SMS] Sending alive message");

    gsm_send_serial("AT+CMGF=1", 1000); // Text mode
    gsm_send_serial("AT+CSCS=\"GSM\"", 500); // Use GSM charset

    String cmd = String("AT+CMGS=\"") + ReceiverPanelAlive + "\"";
    gsm_send_serial(cmd, 1000);
    gsm_send_serial(message + "\x1A", 5000); // Send message with Ctrl+Z

}


void publishToMQTT(String topic, String payload) {
  String command = "AT+QMTPUBEX=0,1,1,1,\"" + topic + "\"," + String(payload.length());
  gsm_send_serial(command, 1000);
  gsm_send_serial(payload + "\x1A", 1000);
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
        if (callEnded|| ttsEndedEarly) break;
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
  gsm_send_serial("AT+QMTDISC=0",1000);
  delay(1000);
  String mqtt_conn = "AT+QMTCONN=0,\"Transmitter_panel\",\"" + username + "\",\"" + password + "\"";
  gsm_send_serial(mqtt_conn, 1000);
  delay(2000); // Wait for the connection to establish

  // Debug: Print MQTT connection status
  String connStatus = gsm_send_serial("AT+QMTCONN?", 1000);
  SerialMon.print("MQTT Connection Status: ");
  SerialMon.println(connStatus);
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Title
  display.println("Transmitter Panel");

  // I1, I2, I3 Inputs in one line
  display.setCursor(0, 12);
  display.print("I1:");
  display.print(inputStatus[0] ? "H " : "L ");
  display.print("I2:");
  display.print(inputStatus[1] ? "H " : "L ");
  display.print("I3:");
  display.println(inputStatus[2] ? "H" : "L");

  // GSM Signal Strength
  int signalStrength = getGSMSignalStrength();
  display.setCursor(0, 24);
  display.print("Signal: ");
  display.print(signalStrength);
  //display.println(" dBm");

  display.setCursor(0, 36);
  display.print("GPRS: ");
  display.print(GPRSconnection);

  display.setCursor(0, 48);
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
    Serial.println("[WARNING] GPRS Disconnected. Reconnecting...");
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
      char c = SerialAT.read();
      buff_resp += c;
    }
    delay(10); // Small delay to allow for incoming data to accumulate
  }
  
  Serial.println("Response ->: " + buff_resp);
  return buff_resp;
}
