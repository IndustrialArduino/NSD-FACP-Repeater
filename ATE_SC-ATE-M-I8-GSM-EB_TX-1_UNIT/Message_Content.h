#ifndef Message_Content_H
#define Message_Content_H

// SMS MESSAGE TEXT (Alarm messages) //USER CAN EDIT MESSAGE CONTENTS BUT KEEP TX1 AS IT IS TO IDENTIFY TWO TX NODES SEPERATELY
const char* smsMessages[3][2] = {        
             // ALARM ACTIVE MESSAGE                      // ALARM RESTORE MESSAGE 
  { " TX1 Fire Alarm ACTIVATED at : ETIHAD HQ ", "TX1 Fire Alarm CLEARED at : ETIHAD HQ " },
  { " TX1 Fault reported at: ETIHAD HQ ", " TX1 Fault cleared at : ETIHAD HQ " },
  { " Mains Power Fail reported at : ETIHAD HQ ", " Mains Power Fail cleared at : ETIHAD HQ " }
};

// VOICE MESSAGE CONTENT

String Fire_Alarm_Text = " Fire Alarm Activated at ETIHAD HQ";
String Fault_Alarm_Text = " Fault Activated at ETIHAD HQ ";



#endif
