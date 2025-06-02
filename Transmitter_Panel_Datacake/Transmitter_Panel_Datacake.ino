#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  // Include the WiFi library for MAC address
#include <ArduinoJson.h>
#include "Secret.h" // Include the file to get the username and password of MQTT server
#include"datacake.h"

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

  Init();
  connectToGPRS();
  connectToMQTT();
}

void loop() {
  if (processing) return;
  processing = true;

  readInputsAndCheckAlarms();
  maintainMQTTConnection();

  processing = false;
  delay(50); 

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
    connectToMQTT();
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
  //gsm_send_serial("AT+CGDCONT=1,\"IP\",\"dialogbb\"", 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGATT?", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
}

void connectToGPRS(void) {
  gsm_send_serial("AT+CGATT=1", 1000);
  String cmd = "AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"";
  gsm_send_serial(cmd, 1000);
  //gsm_send_serial("AT+CGDCONT=1,\"IP\",\"dialogbb\"", 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
}

void connectToMQTT(void) {
  // Initialize MQTT configurations
  gsm_send_serial("AT+QMTCFG=\"recv/mode\",0,0,1", 1000);
  gsm_send_serial("AT+QMTCFG=\"SSL\",0,1,2", 1000);
  int cert_length = mqtt_ca_cert.length(); // Get the length of the CA certificate
  String ca_cert = "AT+QFUPL=\"RAM:datacake_ca.pem\"," + String(cert_length) + ",100";
  gsm_send_serial(ca_cert, 1000); // Send the command
  delay(1000);
  gsm_send_serial(mqtt_ca_cert, 1000); // Send the command to upload CA singned certificate
  delay(1000);
  gsm_send_serial("AT+QSSLCFG=\"cacert\",2,\"RAM:datacake_ca.pem\"", 1000);
  gsm_send_serial("AT+QMTOPEN=0,\"mqtt.datacake.co\",8883", 1000);
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

bool isNetworkConnected() {
  String response = gsm_send_serial("AT+CREG?", 3000);
  return (response.indexOf("+CREG: 0,1") != -1 || response.indexOf("+CREG: 0,5") != -1);
}

bool isGPRSConnected() {
  String response = gsm_send_serial("AT+CGATT?", 3000);
  return (response.indexOf("+CGATT: 1") != -1);
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
