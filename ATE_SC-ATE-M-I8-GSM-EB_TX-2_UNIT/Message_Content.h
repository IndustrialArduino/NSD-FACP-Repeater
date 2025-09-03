#ifndef Message_Content_H
#define Message_Content_H

// SMS MESSAGE TEXT (Alarm messages) //USER CAN EDIT MESSAGE CONTENTS BUT KEEP TX1 AS IT IS TO IDENTIFY TWO TX NODES SEPERATELY
const char* smsMessages[3][2] = {        
             // ALARM ACTIVE MESSAGE                      // ALARM RESTORE MESSAGE 
  { "TX2 - Fire Alarm Activated at Cooling Plant", "TX2 - Fire Alarm CLEARED at Cooling Plant" },
  { "TX2 - Fault reported at Cooling Plant", "TX2 - Fault cleared at Cooling Plant" },
  { "TX2 – Mains Power Fail reported at Cooling Plant", "TX2 - Mains Power Fail cleared at Cooling Plant" }
};

// VOICE MESSAGE CONTENT

String Fire_Alarm_Text = "Fire Alarm Activated at Cooling Plant";
String Fault_Alarm_Text = "Fault Activated at Cooling Plant";



#endif
