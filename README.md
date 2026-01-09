# CPC357 Air Quality Monitor (ESP32 + Blynk)

A simple IoT air-quality monitoring and ventilation demo for **CPC357 – IoT Architecture and Smart Applications (SDG 11: Sustainable Cities & Communities)**.

This project measures:
- **Gas concentration (MQ2)** → `gas_raw` + normalized `gas_index`
- **Temperature & humidity (DHT11)** → `temp_c`, `hum_pct`

It visualizes all data in **Blynk IoT mobile dashboard** and controls a **fan via relay** using:
- Manual control (Blynk switch)
- Auto control (threshold + hysteresis + off-delay)

---

## 1. System Overview

### Components
- **ESP32 Dev Board** (NodeMCU-ESP32S / ESP32 Dev Module)
- **MQ2 gas sensor module** (analog output AO)
- **DHT11 temperature/humidity module**
- **1-channel relay module** (DC+ / DC- / IN1; COM/NO/NC)
- **DC motor fan** (red/black wires)
- Breadboard + jumper wires + resistors (used for MQ2 analog divider)

### Data Flow (High-Level)
Sensors → ESP32 → WiFi → **Blynk Cloud** → Blynk Mobile Dashboard  
ESP32 also receives manual commands from Blynk and controls the relay/fan.

---

## 2. Blynk Template Setup

### Template
Create a Blynk Template in **Blynk.Console** and add these Datastreams:

| Name | Pin | Type | Range | Description |
|---|---:|---|---|---|
| gas_raw | V0 | Integer | 0–4095 | Smoothed MQ2 ADC raw value |
| gas_index | V1 | Integer | 0–100 | Normalized index for UI/control |
| temp_c | V2 | Double | 0–60 | DHT11 temperature (°C) |
| hum_pct | V3 | Double | 0–100 | DHT11 humidity (%) |
| fan_state | V4 | Integer | 0–1 | Actual relay/fan state (device output) |
| fan_manual | V5 | Integer | 0–1 | Manual fan switch (app input) |

**Dashboard widgets (recommended):**
- Gauge/Value: `hum_pct (V3)`, `temp_c (V2)`, `gas_index (V1)`, `gas_raw (V0)`
- Switch: `fan_manual (V5)`
- LED/Indicator: `fan_state (V4)`

> Optional: create an Event (e.g., `air_poor`) if you want push notifications when `gas_index` is high.

---

## 3. Arduino / ESP32 Firmware Setup

### 3.1 Prerequisites
- **Arduino IDE**
- ESP32 board support: **“ESP32 by Espressif Systems”**
- Select board: **Tools → Board → ESP32 Arduino → ESP32 Dev Module**

### 3.2 Required Libraries
Install via Arduino Library Manager:
- **Blynk**
- **DHT sensor library** (Adafruit)
- **Adafruit Unified Sensor**

### 3.3 Configure Credentials
In the `.ino` file, set:
- `BLYNK_TEMPLATE_ID`
- `BLYNK_TEMPLATE_NAME`
- `BLYNK_AUTH_TOKEN`
- `ssid` / `pass`

**Security note:** do NOT commit real tokens/passwords to a public repo.

### 3.4 Upload
- Connect ESP32 via USB
- Select correct **Port**
- Upload the sketch
- Open Serial Monitor at **115200 baud** for logs

---

## 4. Control Logic (Key Parameters)

### 4.1 Gas Index (0–100)
- MQ2 analog output is read by ESP32 ADC (0–4095).
- Startup baseline calibration is performed in clean air.
- Gas Index is computed from `(raw - baseline)` and normalized to 0–100.

### 4.2 Smoothing
- EMA (Exponential Moving Average) is used to reduce noise and stabilize control.

### 4.3 Auto Control (Hysteresis + Off-Delay)
- **ON threshold:** if `gas_index >= setpointOn`, fan turns ON.
- **OFF threshold:** if `gas_index <= setpointOff`, start counting “good air time”.
- **Hysteresis gap = 20:** `setpointOff = setpointOn - 20`
- **Off-delay = 20 seconds:** fan turns OFF only if air remains good continuously for 20 seconds.

This prevents relay chattering and makes behavior stable for demo.

---

## 5. Pin Mapping (Firmware)

| Module | Signal | ESP32 Pin |
|---|---|---:|
| DHT11 | Data (S) | GPIO4 |
| MQ2 | Analog input (AO via divider) | GPIO34 |
| Relay | IN1 | GPIO26 |

Power:
- ESP32 powered by USB
- MQ2/Relay typically use **5V**, DHT11 uses **3.3V**
- All grounds must be **common GND**

---

## 6. Hardware Wiring Notes (Important)

### 6.1 MQ2 Analog Divider (Required)
MQ2 AO can be up to ~5V, but ESP32 ADC is 3.3V max.
Use a resistor divider:
- AO → R → (midpoint) → R → GND
- ESP32 ADC (GPIO34) reads the midpoint

### 6.2 Relay Trigger Mode
This relay module supports **High/Low trigger** (yellow jumper).
- Set jumper to **LOW trigger** if your firmware uses active-low control.

### 6.3 Fan Wiring (COM/NO)
Use COM + NO for “normally OFF” behavior:
- Power + (5V) → COM
- NO → Motor red (+)
- Motor black (-) → GND (direct)

If ESP32 resets when motor starts:
- use an external 5V supply for the motor
- **share GND** with ESP32 (common ground)

---

## 7. How to Demo (Recommended Script)

1. Show Blynk dashboard with live values (Temp/Humidity/Gas).
2. Turn **fan_manual ON** → relay clicks, fan spins.
3. Bring a lighter close to MQ2 (release gas without flame) → `gas_index` rises.
4. Turn **fan_manual OFF** (or show auto behavior if enabled in your firmware).
5. Remove the gas source → after **20s**, fan automatically turns off (stable off-delay).

---

## 8. License
For academic use (CPC357 course project).  
Third-party libraries follow their own licenses.