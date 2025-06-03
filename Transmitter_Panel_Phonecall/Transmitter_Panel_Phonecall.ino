#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>

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

String phoneNumbers[3] = {"+94761111111", "+94741111111", "+94771111111"};

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
}

void loop() {
  if (processing) return;
  processing = true;

  readInputsAndCheckAlarms();

  processing = false;
  updateOLED();
  delay(1000); // refresh every 1 second

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
  if (!status) return; // Only act on activation

  if (idx == 0) { // Fire
    makeAlarmCalls("Fire Alarm Activated at Cooling Plant");
  } else if (idx == 1) { // Fault
    makeAlarmCalls("Fault Activated at Cooling Plant");
  }
}


void makeAlarmCalls(String message) {
  for (int i = 0; i < 3; i++) {
    SerialMon.println("[CALL] Dialing " + phoneNumbers[i]);

    // 1. Enable COLP
    gsm_send_serial("AT+COLP=1", 1000);
    
    // 2. Dial number
    gsm_send_serial("ATD" + phoneNumbers[i] + ";", 1000);
    
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
      SerialMon.println("[CALL] Playing message via TTS...");
      gsm_send_serial("AT+QWTTS=1,1,2,\"" + message + "\"", 2000); // Start TTS

      // 5. Wait for TTS to finish or call to end early
      unsigned long ttsStart = millis();
      while (millis() - ttsStart < 15000) {
        while (SerialAT.available()) {
          String ttsLine = SerialAT.readStringUntil('\n');
          ttsLine.trim();
          SerialMon.println("TTS URC: " + ttsLine);

          if (ttsLine.indexOf("NO CARRIER") != -1 || ttsLine.indexOf("BUSY") != -1) {
            SerialMon.println("[CALL] Call ended during TTS.");
            callEnded = true;
            break;
          }
        }
        if (callEnded) break;
        delay(100);
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

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Title
  display.println("Transmitter Panel");

  // Fire Alarm Status
  display.setCursor(0, 12);
  display.print("FIRE: ");
  display.println(inputStatus[0] ? "Activated" : "Cleared");

  // Fault Alarm Status
  display.setCursor(0, 24);
  display.print("FAULT: ");
  display.println(inputStatus[1] ? "Activated" : "Cleared");

  // Mains Fail Alarm Status
  display.setCursor(0, 36);
  display.print("MAINS: ");
  display.println(inputStatus[2] ? "Activated" : "Cleared");

  // GSM Signal Strength
  int signalStrength = getGSMSignalStrength();
  display.setCursor(0, 48);
  display.print("Signal: ");
  display.print(signalStrength);
  //display.println(" dBm");

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
  gsm_send_serial("AT+QTTSETUP=1,1,-5000", 1000); // Speed
  gsm_send_serial("AT+QTTSETUP=1,2,8000", 1000);  // Volume
  gsm_send_serial("AT+QTTSETUP=1,3,1", 1000);     // Language

}

void connectToGPRS(void) {
  gsm_send_serial("AT+CGATT=1", 1000);
  String cmd = "AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"";
  gsm_send_serial(cmd, 1000);
  //gsm_send_serial("AT+CGDCONT=1,\"IP\",\"dialogbb\"", 1000);
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
