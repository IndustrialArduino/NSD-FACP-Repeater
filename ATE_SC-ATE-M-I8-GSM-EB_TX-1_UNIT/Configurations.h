#ifndef Configurations_H
#define Configurations_H

// Node name
const char* nodeName = "TX-1";

// Your GPRS credentials
const char apn[] = "etisalat.ae";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Delay settings in minutes (user configurable, 0–30)(to avoid nuisance alerts)
uint8_t inputDelayMinutes[3] = {
  0,   // DI-1 FIRE → instant
  10,  // DI-2 FAULT → 10 min
  5    // DI-3 MAINS_POWER_FAULT → 5 min
};

//Your phone numbers for phone calls
String phonecall_phoneNumbers[5] = {
  "+971506168293"   // #1

};

// User configurable: how many numbers to send SMS OF REMOTE SUCCESSFULL UPDATE MESSAGE (0-6)
uint8_t numPhoneNumbers = 1;  // Example: send to first number ONLY

//RX number is Always included in SMS (do not remove)
String rxSimNumber = "+971501608741";    

//Pool of up to 6 numbers for routing (can be empty "")
String smsRoutingNumbers[6] = {
  "+971506168293"   // #1

};

struct NotificationRule {
  bool sendOnAlarm;                // Whether to send on alarm
  bool sendOnRestore;              // Whether to send on restore
  bool routingMask[6];             // Which numbers to use
};

struct InputNotificationConfig {
  const char* name;                // e.g. "FIRE", "FAULT" , "MAINS_POWER_FAULT"
  NotificationRule rules[4];       // Allow up to 4 different rules per input
  uint8_t ruleCount;               // How many rules are actually used
};

InputNotificationConfig inputConfigs[3] = {
    {
    "FIRE",
    {
      { true, true, { true, true, true, true, true, true } }       // Both Alarm + Restore → all numbers
    },
    1 // total rules for FIRE
  },
  {
    "FAULT",
    {
      { true, false, { true, true, false, false, false, false } }, // Alarm → only first 2 numbers
      { false, true, { true, false, true, false, true, false } }   // Restore → 1st, 3rd, 5th
    },
    2  // total rules for FAULT
  },
  {
    "MAINS_POWER_FAULT",
    {
      { true, true, { true, true, true, true, true, true } }       // Both Alarm + Restore → all numbers
    },
    1 // total rules for MAINS_POWER_FAULT
  }

};

//PLEASE DON'T CHANGE THIS. THESE ARE NOTIFICATION MODES
enum NotificationMode {
  NOTIFY_NONE = 0,
  NOTIFY_SMS = 1,
  NOTIFY_CALL = 2,
  NOTIFY_BOTH = 3
};

//CHOOSE YOUR PREFERED METHOD TO SEND NOTIFICATIONS ON EACH INPUT
NotificationMode inputNotificationSettings[3] = {
  NOTIFY_BOTH,  // Input 0 (Fire): both SMS + Call
  NOTIFY_BOTH,   // Input 1 (Fault): only SMS
  NOTIFY_SMS   // Input 2 (Mains): only SMS
};

//MQQT TOPIC STRUCT
struct MQTTConfig {
  const char* fireTopic;
  const char* faultTopic;
  const char* mainsFailTopic;
  const char* aliveTopic;
};

//CHANGE THE MQQT TOPIC ACCORDING TO YOUR MQQT DEVICE'S FIELD'S MQQT TOPICS
MQTTConfig mqttConfig = {
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX1_FIRE_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX1_FAULT_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX1_MAINS_FAIL_ALARM",
  "dtck-pub/facp-repeater-ate-sc-ate-m-i8-gsm-eb/5ae3a817-a1e3-4f38-8aaf-9ee88f26b21e/TX1_ALIVE_STATUS"
};   

#endif 
