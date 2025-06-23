#ifndef Message_Content_H
#define Message_Content_H

// SMS MESSAGE TEXT (Alarm messages)
const char* smsMessages[3][2] = {
  { "FACP - Fire Alarm Activated at Cooling Plant", "FACP - Fire Alarm CLEARED at Cooling Plant" },
  { "FACP - Fault reported at Cooling Plant", "FACP - Fault cleared at Cooling Plant" },
  { "FACP – Mains Power Fail reported at Cooling Plant", "FACP - Mains Power Fail cleared at Cooling Plant" }
};

// VOICE MESSAGE CONTENT

String Fire_Alarm_Text = "Fire Alarm Activated at Cooling Plant";
String Fault_Alarm_Text = "Fault Activated at Cooling Plant";



#endif
