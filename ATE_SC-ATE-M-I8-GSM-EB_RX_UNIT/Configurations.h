#ifndef Configurations_H
#define Configurations_H

// Node name
const char* nodeName = "RX";

// Your GPRS credentials
const char apn[] = "dialogbb";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Delay settings in minutes (user configurable, 0–30)
uint8_t inputDelayMinutes_RX = 5;  //RX MAINS FAILS ALARM TRIGGERING DELAY

// User configurable: how many numbers to send SMS (0-5)
uint8_t numPhoneNumbers = 3;  // Example: send to first 3 numbers

//Your phone numbers for SMS
const char* phoneNumbers[5] = {
   "+94761111111",
   "+94771111111",
   "+94701111111",
   "+94741111111",
   "+94711111111"
  
};

//MQQT TOPIC STRUCT
struct MQTTConfig {
  const char* mainsFailTopic;
  const char* TX1_fireTopic;
  const char* TX1_faultTopic;
  const char* TX1_aliveTopic;
  const char* TX2_fireTopic;
  const char* TX2_faultTopic;
  const char* TX2_aliveTopic;
};

//CHANGE THE MQQT TOPIC ACCORDING TO YOUR MQQT DEVICE'S FIELD'S MQQT TOPICS
MQTTConfig mqttConfig = {
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/RX_MAINS_FAIL_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX1_FIRE_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX1_FAULT_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX1_ALIVE_STATUS",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX2_FIRE_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX2_FAULT_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX2_ALIVE_STATUS"
}; 

#endif 
