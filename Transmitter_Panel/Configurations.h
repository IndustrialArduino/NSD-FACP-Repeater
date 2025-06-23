#ifndef Configurations_H
#define Configurations_H

// Your GPRS credentials
const char apn[] = "dialogbb";
const char gprsUser[] = "";
const char gprsPass[] = "";

//Your phone numbers for phone calls
String phonecall_phoneNumbers[3] = {"+94761111111", "+94741111111", "+94771111111"};

//Your phone numbers for SMS
const char* phoneNumbers[5] = {
   "+94761111111",
   "+94741111111",
   "+94701111111",
   "+94741111111",
   "+94781111111"
  
};

//Phone number of the RX to Send alive message
const char* ReceiverPanelAlive = {"+94761111111" };  

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

   

#endif 
