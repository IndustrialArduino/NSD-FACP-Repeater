#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  // Include the WiFi library for MAC address
#include <ArduinoJson.h>
#include "Secret.h" // Include the file to get the username and password of MQTT server
#include"datacake.h"

String gsm_send_serial(String command, int delay);

//#define TINY_GSM_MODEM_SIM7600
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

  unsigned long now = millis();

  // Read raw input states
  currentState[0] = digitalRead(D1); // Fire
  currentState[1] = digitalRead(D2); // Fault
  currentState[2] = digitalRead(D3); // Mains

  for (int i = 0; i < 3; i++) {
    if (currentState[i] != lastStableState[i]) {
      if ((now - lastDebounceTime[i]) > DEBOUNCE_DELAY && !processing) {
        lastDebounceTime[i] = now;
        processing = true;

        String topic;
        String payload;

        switch (i) {
          case 0: // FIRE
            FEA_Status = (currentState[i] == LOW);  // Active when LOW
            topic = "dtck-pub/nsd-facp-repeater/7d165773-3625-4c82-9412-5df17217c656/FIRE_ALARM";
            payload = String(FEA_Status);
            Serial.println(FEA_Status ? "[ALARM] FIRE Active" : "[NORMAL] FIRE Normal");
            break;
          case 1: // FAULT
            FTA_Status = (currentState[i] == HIGH); // Active when HIGH
            topic = "dtck-pub/nsd-facp-repeater/7d165773-3625-4c82-9412-5df17217c656/FAULT_ALARM";
            payload = String(FTA_Status);
            Serial.println(FTA_Status ? "[ALARM] FAULT Active" : "[NORMAL] FAULT Normal");
            break;
          case 2: // MAINS
            MSA_Status = (currentState[i] == HIGH); // Active when HIGH
            topic = "dtck-pub/nsd-facp-repeater/7d165773-3625-4c82-9412-5df17217c656/MAINS_FAIL_ALARM";
            payload = String(MSA_Status);
            Serial.println(MSA_Status ? "[ALARM] MAINS Fail" : "[NORMAL] MAINS OK");
            break;
        }

        // Send MQTT publish command
        String command = "AT+QMTPUBEX=0,1,1,0,\"" + topic + "\"," + String(payload.length());
        String publishResponse = gsm_send_serial(command, 1000);
        publishResponse += gsm_send_serial(payload + "\x1A", 1000);

                // Check if publish was successful
        if (!(publishResponse.indexOf("OK") != -1 && publishResponse.indexOf("+QMTPUBEX: 0,1,0") != -1)) {
          Serial.println("[ERROR] MQTT publish failed. Starting step-by-step recovery...");

          // Step 1: Check network connection
          if (!isNetworkConnected()) {
            Serial.println("[RECOVERY] Network not connected. Resetting network...");
            gsm_send_serial("AT+CFUN=1", 2000);
            delay(1000);
            gsm_send_serial("AT+CPIN?", 1000);
            gsm_send_serial("AT+CREG?", 1000);
            if (!isNetworkConnected()) {
              Serial.println("[ERROR] Network still unavailable. Aborting publish.");
              processing = false;
              return;
            }
          }

          // Step 2: Check GPRS connection
          if (!isGPRSConnected()) {
            Serial.println("[RECOVERY] GPRS not connected. Reconnecting GPRS...");
            connectToGPRS();
            delay(1000);
            if (!isGPRSConnected()) {
              Serial.println("[ERROR] GPRS still unavailable. Aborting publish.");
              processing = false;
              return;
            }
          }

          // Step 3: Check MQTT connection
          String status = gsm_send_serial("AT+QMTCONN?", 1000);
          if (status.indexOf("+QMTCONN: 0,0") == -1) {
            Serial.println("[RECOVERY] MQTT not connected. Reconnecting MQTT...");
            connectToMQTT();
            delay(1000);
            status = gsm_send_serial("AT+QMTCONN?", 1000);
            if (status.indexOf("+QMTCONN: 0,0") == -1) {
              Serial.println("[ERROR] MQTT still disconnected. Aborting publish.");
              processing = false;
              return;
            }
          }

          // Retry MQTT publish after recovery
          Serial.println("[INFO] Retrying MQTT publish...");
          publishResponse = gsm_send_serial(command, 1000);
          publishResponse += gsm_send_serial(payload + "\x1A", 1000);
          if (publishResponse.indexOf("OK") != -1 && publishResponse.indexOf("+QMTPUBEX: 0,0,0") != -1) {
            Serial.println("[SUCCESS] MQTT publish successful after recovery.");
          } else {
            Serial.println("[FAILURE] MQTT publish still failed after recovery.");
          }
        }

        // Update state
        lastStableState[i] = currentState[i];
        processing = false;
      }
    }
  }

  // Check MQTT connection
  String status = gsm_send_serial("AT+QMTCONN?", 1000);
  if (status.indexOf("+QMTCONN: 0,0") == -1) {
    Serial.println("[WARNING] MQTT Disconnected. Reconnecting...");
    connectToMQTT();
  }

  delay(50); // small delay to reduce CPU churn

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
  gsm_send_serial("AT+CGDCONT=1,\"IP\",\"hologram\"", 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGATT?", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
}

void connectToGPRS(void) {
  gsm_send_serial("AT+CGATT=1", 1000);
  gsm_send_serial("AT+CGDCONT=1,\"IP\",\"hologram\"", 1000);
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
