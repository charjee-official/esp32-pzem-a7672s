// main.cpp
// ESP32 + PZEM-004T v3 + A7672S (TinyGSM) + PubSubClient (MQTT)
// Make sure to edit src/secrets.h before building.

/*
  Important:
  - Provide modem VBAT a dedicated regulator capable of current peaks (up to ~1-2A)
  - Connect grounds together (ESP32, modem power supply, PZEM)
  - Adjust pins below if your board layout differs
*/

#define TINY_GSM_MODEM_SIM900   // included per your requirement

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <PZEM004Tv30.h>
#include "secrets.h"

// ----------------- Pin / Serial config -----------------
/*
 * Serial mapping used in this project:
 * - Serial   : USB / debug (115200)
 * - Serial1  : Modem AT (UART1) -> GPIO 26 (RX1), GPIO 27 (TX1)
 * - Serial2  : PZEM UART  -> GPIO 16 (RX2), GPIO 17 (TX2)
 *
 * Change pins if your board requires different ones.
 */

HardwareSerial SerialAT(1);   // Serial1 -> modem
HardwareSerial SerialPZEM(2); // Serial2 -> PZEM

const int MODEM_RX_PIN = 26; // connect to A7672S TX
const int MODEM_TX_PIN = 27; // connect to A7672S RX

const int PZEM_RX_PIN = 16;  // connect to PZEM TX
const int PZEM_TX_PIN = 17;  // connect to PZEM RX

// ----------------- Objects -----------------
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);
PZEM004Tv30 pzem(SerialPZEM, PZEM_RX_PIN, PZEM_TX_PIN);


// ----------------- Timing -----------------
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL_MS = 15000; // 15s default

// ----------------- Helpers -----------------
void debugPrintModemInfo() {
  String info = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(info);
}

bool connectGPRS() {
  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork(60000L)) { // wait up to 60s
    Serial.println(" failed");
    return false;
  }
  Serial.println(" ok");

  Serial.print("Connecting to GPRS APN...");
  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) {
    Serial.println(" failed");
    return false;
  }
  Serial.println(" ok (GPRS connected)");
  return true;
}

bool initModemAndNetwork() {
  Serial.println("Restarting modem...");
  if (!modem.restart()) {
    Serial.println("modem.restart() failed");
    // try a soft "power on" or continue to try
  } else {
    Serial.println("modem restarted");
  }

  delay(1000);
  debugPrintModemInfo();

  if (!connectGPRS()) return false;

  return true;
}

bool mqttConnect() {
  Serial.print("Connecting to MQTT broker...");
  if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
    Serial.println(" connected");
    // subscribe here if needed, e.g. mqtt.subscribe("pzem/cmd");
    return true;
  } else {
    Serial.print(" failed, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}

void publishPZEM() {
  float voltage = pzem.voltage();
  float current = pzem.current();
  float power   = pzem.power();
  float energy  = pzem.energy();
  float pf      = pzem.pf();
  float freq    = pzem.frequency();

  // Handle sensor read failures (PZEM returns NAN or very large values)
  if (isnan(voltage) || isnan(current)) {
    Serial.println("PZEM read error (NaN), skipping publish");
    return;
  }

  char payload[256];
  int n = snprintf(payload, sizeof(payload),
    "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.2f,\"energy\":%.3f,\"pf\":%.3f,\"freq\":%.2f}",
    voltage, current, power, energy, pf, freq);

  Serial.print("Publishing: ");
  Serial.println(payload);

  if (!mqtt.connected()) {
    Serial.println("MQTT not connected, attempting reconnect...");
    mqttConnect();
  }

  if (mqtt.connected()) {
    bool ok = mqtt.publish(MQTT_TOPIC, payload);
    Serial.print("Publish result: ");
    Serial.println(ok ? "OK" : "FAIL");
  } else {
    Serial.println("MQTT offline - publish skipped");
  }
}

// optional: handle incoming MQTT messages if subscribed
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT msg [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; i++) Serial.write(payload[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 + PZEM + A7672S MQTT Example ===");
  // Start modem serial (Serial1)
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(200);

  // Start PZEM serial (Serial2)
  SerialPZEM.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  delay(200);

  // small stabilization wait
  delay(500);
  delay(500);
  // Initialize modem and attach network
  if (!initModemAndNetwork()) {
    Serial.println("Modem init or network attach failed. Retrying in loop.");
  }

  // Configure MQTT client
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  Serial.println("Setup complete");
}

void loop() {
  // keep mqtt loop running
  if (mqtt.connected()) mqtt.loop();

  // try to ensure connectivity
  if (!modem.isNetworkConnected()) {
    Serial.println("Network lost. Trying to reconnect GPRS...");
    connectGPRS();
  }

  if (!mqtt.connected()) {
    Serial.println("MQTT disconnected. Attempting connect...");
    mqttConnect();
  }

  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = now;
    publishPZEM();
  }

  delay(100);
}
