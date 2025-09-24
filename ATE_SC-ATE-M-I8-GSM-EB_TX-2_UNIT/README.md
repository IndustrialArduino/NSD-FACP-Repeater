
# ATE_SC-ATE-M-I8-GSM-EB_TX-2_UNIT

## Functionalities

### Inputs and Alarm Conditions

| Input | Signal Name       | Contact Type                  | Alarm Condition           |
|-------|-----------------|------------------------------|---------------------------|
| DI-1  | FIRE ALARM       | Normally Open (NO)           | Alarm when contact CLOSED |
| DI-2  | FAULT ALARM      | Normally Closed (NC)         | Alarm when contact OPEN   |
| DI-3  | NODE POWER FAULT | Normally Closed (NC)         | Alarm when contact OPEN   |

### Features

1. Monitor **INPUT1, INPUT2, INPUT3** in real-time.
2. Send notifications on **Alarm/Restore events** via both **SMS** and **Datacake**.
3. LCD display automatically scrolls every 3 seconds through three interfaces:

#### Display 1

```bash

Node: TX-2 03/09 15:25
Fire Activated
Fault Normal
Mains Normal

 ```

#### Display 2

```bash

Node: TX-2 03/09 15:25
Signal: 22
GPRS: Active
MQTT: Connect

 ```

4. **SMS Routing:**
   - First to **RX SIM number**
   - Then up to **6 additional SIM numbers** (configurable)

5. **User-Configurable Numbers:**
   - Independent receiver number entry per input
   - Selectable for **Alarm, Restore, or Both**
   - Clearly labeled in **Parameter Tab**

6. Configure **Alarm notifications** via SMS, Phone Calls, or Both.
7. **Delay Configuration (Input1, Input2, Input3):**
   - Adjustable **0–30 minutes** before triggering SMS
   - Prevents nuisance alerts
   - Configurable independently for each input

8. **Custom SMS Message Content:**
   - User-editable for **Alarm Active / Alarm Restore**
   - Configurable per input

9. **Voice Call Notifications:**
   - Up to **5 calls per input**
   - Configurable SIM numbers
   - Pre-recorded, user-editable voice content

10. **Watchdog Mechanism:**
    - TX-2 send **heartbeat every 15 minutes** via MQTT
    - RX monitors heartbeat loss and reports missing TX-2 

11. **OTA Functionality:**
     - Binary file should be named as "TX2.bin"
    - Triggered via SMS: `"update the TX-2 panel"`

---

## Configurations

### Configurations Tab

1. Configure **GPRS credentials**.
2. Define phone numbers for **SMS** (including RX mains fail and OTA messages).
3. Configure **delay settings** (minutes) to avoid nuisance alerts.
4. Configure phone numbers for **voice calls**.
5. Configure **RX number** and up to 6 additional numbers for SMS.
6. Set **notification rules** per user preference (examples provided in code).
7. Choose preferred **notification method** per input.
8. Configure **MQTT topics** for Datacake device publishing.

### Message_Content Tab

1. Create SMS messages according to **user preference**.
2. Create **voice content** according to user preference.

### Secrets Tab

1. Set **MQTT server username and password**, if needed for Datacake.

---

## Notes

- All configurations are **user-editable**.
- Delays and notification rules are independent per input.
- OTA updates can be triggered remotely via SMS.
- LCD scrolling cycles automatically every **3 seconds**.


