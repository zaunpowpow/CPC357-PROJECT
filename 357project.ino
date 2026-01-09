#define BLYNK_TEMPLATE_ID "TMPL618UL4x0n"
#define BLYNK_TEMPLATE_NAME "CPC357 Air Quality Monitor"
#define BLYNK_AUTH_TOKEN "iBhc_meG3wpUodE4PJfTrF_aOLWUmE8R"

// Blynk cloud credentials/config.
// Tip: avoid committing auth tokens and Wi-Fi passwords to public repos.

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

// Hardware/libraries used:
// - ESP32 WiFi + Blynk for IoT dashboard/control
// - DHT11 for temperature/humidity
// - MQ-2 analog output (via ADC) for gas/smoke indication

char ssid[] = "Dadike";
char pass[] = "Lzcmsmde0987";

// Wi-Fi network credentials (2.4 GHz typically required for ESP32).

// ===== Pins =====
// Board pin mapping (adjust to match your wiring).
static const int PIN_DHT   = 4;
static const int PIN_MQ2   = 34;   // ADC input (after divider)
static const int PIN_RELAY = 26;

#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
BlynkTimer timer;

// ===== State =====
// Blynk virtual pins used by this sketch:
// - V0: MQ2 filtered raw (ADC-like)
// - V1: Gas index (0..100)
// - V2: Temperature (°C)
// - V3: Humidity (%)
// - V4: Fan state (0/1)
// - V5: Manual fan switch (0/1)
// - V6: Auto/manual mode (1=auto)
// - V7: Gas setpoint (0..100)
bool autoMode = true;      // V6
bool fanOn = false;        // real state
int setpointOn = 70;       // V7 (gas_index >= setpointOn -> ON)
int setpointOff = 50;      // hysteresis OFF threshold (can be derived)

// delay to turn off after good air
const unsigned long FAN_OFF_DELAY_MS = 20000; // 2 min
// NOTE: 20000 ms = 20 seconds. Increase this value if you want ~2 minutes.
unsigned long goodSinceMs = 0;

// event cooldown
unsigned long lastAlertMs = 0;
const unsigned long ALERT_COOLDOWN_MS = 60000;

// Cooldown prevents spamming Blynk event notifications.

// ===== Gas processing =====
// Exponential moving average (EMA) is used to smooth noisy ADC readings.
float ema = 0;
const float EMA_ALPHA = 0.12f;

// Baseline calibration for MQ-2 (raw ADC value in clean air).
// `span` controls how aggressively raw values map into a 0..100 index.
int baseline = 0;
int span = 1200;

int toGasIndex(int raw) {
  // Convert a filtered ADC value into a bounded 0..100 "gas index".
  int v = raw - baseline;
  if (v < 0) v = 0;
  int idx = (int)((float)v * 100.0f / (float)span);
  if (idx > 100) idx = 100;
  return idx;
}

// ===== Relay helper =====
// Active-low relay: LOW=ON. If yours is reversed, swap LOW/HIGH here.
void setFan(bool on) {
  // Single place to change the real relay output and keep the UI in sync.
  fanOn = on;
  digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  Blynk.virtualWrite(V4, fanOn ? 1 : 0); // fan_state
}

void calibrateBaseline() {
  // Sample MQ-2 multiple times at boot to estimate a clean-air baseline.
  long sum = 0;
  const int N = 60;
  for (int i = 0; i < N; i++) {
    sum += analogRead(PIN_MQ2);
    delay(50);
  }
  baseline = (int)(sum / N);
  span = 1200;

  Serial.print("Baseline raw=");
  Serial.println(baseline);
}

// ===== Handlers =====
// V6: auto_mode (1=auto, 0=manual)
BLYNK_WRITE(V6) {
  // Called whenever the Blynk app/dashboard writes to V6.
  int v = param.asInt();
  autoMode = (v == 1);

  Serial.print("autoMode=");
  Serial.println(autoMode);

  if (autoMode) {
    // When switching to auto, optionally force manual switch off in UI
    Blynk.virtualWrite(V5, 0);
    // do not change fan immediately; auto loop will manage
  } else {
    // When switching to manual, stop auto timers
    goodSinceMs = 0;
  }
}

// V5: fan_manual (only works in manual mode)
BLYNK_WRITE(V5) {
  // Manual on/off switch is honored only when autoMode is disabled.
  int v = param.asInt();

  Serial.print("fan_manual=");
  Serial.println(v);

  if (!autoMode) {
    setFan(v == 1);
  } else {
    // In auto mode, ignore manual switch and push UI back to OFF
    Blynk.virtualWrite(V5, 0);
  }
}

// V7: setpoint (0..100)
BLYNK_WRITE(V7) {
  // Setpoint controls when the fan turns ON; OFF uses a hysteresis threshold.
  int v = param.asInt();
  if (v < 0) v = 0;
  if (v > 100) v = 100;

  setpointOn = v;
  // keep some hysteresis gap, e.g. 20
  setpointOff = max(0, setpointOn - 20);

  Serial.print("setpointOn=");
  Serial.print(setpointOn);
  Serial.print(" setpointOff=");
  Serial.println(setpointOff);
}

void readAndSend() {
  // Periodic task (timer-driven):
  // 1) read sensors, 2) update Blynk widgets, 3) run auto-control logic.
  // MQ2
  int raw = analogRead(PIN_MQ2);
  if (ema == 0) ema = raw;
  ema = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema;

  int gas_raw = (int)(ema + 0.5f);
  int gas_index = toGasIndex(gas_raw);

  Blynk.virtualWrite(V0, gas_raw);
  Blynk.virtualWrite(V1, gas_index);

  // DHT
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  // DHT reads can fail; NaN checks prevent pushing invalid values to Blynk.
  if (!isnan(t)) Blynk.virtualWrite(V2, t);
  if (!isnan(h)) Blynk.virtualWrite(V3, h);

  // Auto control
  if (autoMode) {
    // Simple hysteresis control:
    // - Turn ON when gas_index >= setpointOn
    // - Turn OFF only after gas_index <= setpointOff for FAN_OFF_DELAY_MS
    unsigned long now = millis();

    if (gas_index >= setpointOn) {
      goodSinceMs = 0;
      if (!fanOn) setFan(true);

      if (now - lastAlertMs > ALERT_COOLDOWN_MS) {
        // Send a Blynk event (push notification / event log) with a cooldown.
        Blynk.logEvent("air_poor", String("GasIndex=") + gas_index);
        lastAlertMs = now;
      }
    } else if (gas_index <= setpointOff) {
      if (goodSinceMs == 0) goodSinceMs = now;
      if (fanOn && (now - goodSinceMs > FAN_OFF_DELAY_MS)) {
        setFan(false);
      }
    } else {
      // hysteresis zone: do nothing
      goodSinceMs = 0;
    }
  }

  Serial.print("idx=");
  Serial.print(gas_index);
  Serial.print(" fan=");
  Serial.print(fanOn);
  Serial.print(" mode=");
  Serial.println(autoMode ? "AUTO" : "MANUAL");
}

void setup() {
  // One-time initialization.
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);

  // Relay default state is OFF (active-low relay means HIGH=OFF).

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH); // OFF for active-low relay

  dht.begin();
  // Connect to Wi-Fi and Blynk cloud.
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  calibrateBaseline();

  // init UI states
  // Set initial widget values on the Blynk dashboard.
  Blynk.virtualWrite(V6, 1); // auto on
  Blynk.virtualWrite(V5, 0); // manual off
  Blynk.virtualWrite(V7, setpointOn);

  timer.setInterval(1000L, readAndSend);
}

void loop() {
  // Blynk + timer housekeeping.
  Blynk.run();
  timer.run();
}
