# ATE_SC-ATE-M-I8-GSM-EB_RX_UNIT

## Functionalities

1. **Receive Alerts**  
   - Receives SMS and MQTT alerts from **TX-1** and **TX-2**.

2. **Main Fails Input**  
   - `INPUT3` monitors the RX mains fail condition.

3. **LCD Display**  
   - The LCD automatically scrolls every **3 seconds** through **three display interfaces**:

  **Display 1 – TX1 Status**

Node: FACP01 03/09 15:25
TX1 Fire Alarm
TX1 Fault Normal
TX1 Alive Yes

**Display 2 – TX2 Status**

Node: FACP01 03/09 15:25
TX2 Fire Normal
TX2 Fault Alarm
TX2 Alive No


**Display 3 – RX Status**

RX Mains Normal
Signal: 22
GPRS: Active
MQTT: Connect


4. **Relay Activation Logic**
- **Fire Alarm Handling**
  - If **DI-1 Fire Alarm** occurs on **TX-1 OR TX-2**:
    - `IO_RL1` → **Sounder ON**
    - `IO_RL2` → **Indicator ON**
- **Silencing Logic**
  - `IO_RL1` stays ON until **INPUT1 (silence switch)** is pressed for 1s.
  - Once silenced, `IO_RL1` remains OFF.
  - If a new Fire Alarm occurs → `IO_RL1` turns ON again.
- **Indicator Reset**
  - `IO_RL2` automatically turns OFF once RX receives confirmation that all Fire alarms are cleared from **TX-1 and TX-2**.

5. **Onboard Buzzer Function**
- If **DI-1 (Fire)** or **DI-2 (Fault)** is triggered at TX:
  - RX buzzer beeps every **10 seconds** for **2 seconds**.
- Pressing **local DI-1 silence switch for 1s** stops the buzzer.
- New alarm/fault → buzzer re-activates.

6. **OTA Functionality**
- SMS command: `"update the RX panel"` triggers OTA update.

---

## Configurations

### Configurations Tab
1. Configure **GPRS credentials**.
2. Define **phone numbers** to send SMS (for RX mains fail alarms and OTA updates).
3. Configure **MQTT topics** for **publish** and **subscribe** (Datacake device).

### Message_Content Tab
- Create SMS messages according to user preference.

### Secrets Tab
- Set MQTT **username** and **password** if changes are needed on Datacake.

---






