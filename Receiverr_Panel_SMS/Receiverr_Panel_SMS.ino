#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  // Include the WiFi library for MAC address
#include <ArduinoJson.h>


String gsm_send_serial(String command, int delay);

#define SerialMon Serial
#define SerialAT Serial1
#define GSM_PIN ""

// Your GPRS credentials
const char apn[] = "dialogbb";
const char gprsUser[] = "";
const char gprsPass[] = "";

#define UART_BAUD 115200

#define MODEM_TX 32
#define MODEM_RX 33
#define GSM_RESET 21

#define D1 34 // Silence Bell
#define D2 35 // Mains_Fails_Alarm_Input
#define D3 14 
#define D4 13
#define D5 5
#define R0 12  // RED Indicator
#define R1 2   // Local Bell
#define R2 27  // Yellow indicator
#define R3 4   // Buzzer
#define R4 23
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

// Predefined phone numbers
const char* phoneNumbers[5] = {
   "+94761111111",
   "+94771111111",
   "+94701111111",
   "+94741111111",
   "+94711111111"
  
};

// Alarm messages
const char* smsMessages[2] = {
  "FACP Remote monitoring panel – Mains Fail Alarm", "FACP Remote monitoring panel – Mains Fail Alarm cleared"
};


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

  Init();
  connectToGPRS();
}

void loop() {
  
  readInputsAndCheckAlarms();
  checkSilenceButton(); 
  handleBuzzerPulse();
  checkForSMS();
  
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
}

void sendSMS(String message) {
  for (int i = 0; i < 5; i++) {
    gsm_send_serial("AT+CMGF=1", 1000); // Set SMS text mode
    gsm_send_serial("AT+CSCS=\"GSM\"", 500); // Use GSM character set

    String cmd = String("AT+CMGS=\"") + phoneNumbers[i] + "\"";
    gsm_send_serial(cmd, 1000);
    gsm_send_serial(message + "\x1A", 5000); // Send message with Ctrl+Z
  }
}

void checkForSMS() {
  String response = gsm_send_serial("AT+CMGL=\"REC UNREAD\"", 2000); // List unread messages

  int indexStart = response.indexOf("+CMGL:");
  while (indexStart != -1) {
    int indexEnd = response.indexOf("\n", indexStart);
    String meta = response.substring(indexStart, indexEnd);
    int bodyStart = indexEnd + 1;
    int bodyEnd = response.indexOf("\r", bodyStart);
    String messageBody = response.substring(bodyStart, bodyEnd);

    // Extract SMS index
    int smsIndex = meta.substring(6, meta.indexOf(',', 6)).toInt();

    Serial.println("[SMS] Received: " + messageBody);
    handleSMS(messageBody); // Process SMS content

    // Delete processed message
    gsm_send_serial("AT+CMGD=" + String(smsIndex), 1000);

    // Check if more messages exist
    indexStart = response.indexOf("+CMGL:", bodyEnd);
  }
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
