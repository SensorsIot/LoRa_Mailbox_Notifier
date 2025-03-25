#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <secrets.h>  // Contains definitions for: ssid, password, mqtt_server

// --- Pin Definitions ---
// Module control pins (for setting parameter vs. transparent mode)
#define M0 21    // Mode control pin 0
#define M1 19    // Mode control pin 1
#define AUX 4    // Module status indicator (used to synchronize operations)

// UART configuration for module communication
#define TXD2 17  // UART TX for module commands/data
#define RXD2 16  // UART RX for module responses

// --- Mailbox Command Definitions ---
#define ARRIVED 0x55      // Mail has arrived
#define EMPTY 0xAA        // Mailbox is empty
#define ACKNOWLEDGE 0x25  // Acknowledge received command

// --- Module Configuration ---
// Expected parameter configurations for the two module types
byte expected_30dBm[6] = { 0xC0, 0x00, 0x01, 0x1A, 0x17, 0x47 };
byte expected_20dBm[6] = { 0xC0, 0x00, 0x01, 0x1A, 0x17, 0x44 };

// --- Global Variables for MQTT and Device Identification ---
String macAddr;
String uniqueID;
String messageTopic;
String discoveryTopic;

// --- Mailbox Status Enumeration ---
enum boxStatus {
  emptyStatus,
  fullStatus
} mailBoxStatus = emptyStatus;

// --- WiFi and MQTT Client Objects ---
WiFiClient espClient;
PubSubClient client(espClient);

//
// ======================== MODULE CONTROL FUNCTIONS ========================
//

// Wait until the AUX pin indicates that the module is ready
void wait_aux() {
  while (digitalRead(AUX) == LOW) {
    delay(1);
  }
  delay(5);  // Additional delay for stability
}

// Set the module to programming (parameter configuration) mode
void set_programming() {
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(20);
  wait_aux();
}

// Set the module to transparent (normal operation) mode
void set_transparent() {
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(20);
  wait_aux();
}

// Send a command over Serial2 using a String
void sendCommand(const String& command) {
  wait_aux();
  Serial2.write(command.c_str(), command.length());
  Serial2.flush();
  delay(30);
}

// Send a command over Serial2 using a byte array
void sendCommand(const byte* command, size_t len) {
  wait_aux();
  Serial2.write(command, len);
  Serial2.flush();
  delay(30);
}

// Request the stored configuration and compare it with the expected configuration.
// If a mismatch is detected, program the module with the expected parameters.
void sendCommandIfMismatch(const byte* expected, size_t len) {
  // Request the stored configuration from the module
  sendCommand("\xC1\xC1\xC1");  

  // Wait until 6 bytes are available on Serial2
  while (Serial2.available() < 6) {
    delay(10);
  }
  
  // Read the 6-byte current configuration from the module
  byte currentConfig[6];
  Serial2.readBytes(currentConfig, 6);

  // Compare each byte with the expected configuration
  bool mismatch = false;
  for (size_t i = 0; i < len; i++) {
    if (currentConfig[i] != expected[i]) {
      mismatch = true;
      break;
    }
  }

  // Only program the module if a configuration mismatch is detected
  if (mismatch) {
    Serial.println("Configuration mismatch detected, programming module...");
    sendCommand(expected, len);
  } else {
    Serial.println("Module configuration is already correct.");
  }
}

// Program the E32 module: switch to programming mode, request configuration,
// determine module type from the response, and program the module only if needed.
bool programming_E32() {
  Serial.println("--------------------Program E32 Module---------------------");

  // Switch module to programming mode
  set_programming();

  // Send command to request configuration
  sendCommand("\xC3\xC3\xC3");

  // Read the 4-byte response from the module
  byte received[4];
  Serial2.readBytes(received, 4);

  // Determine module type based on the last byte of the response
  const byte* expectedConfig;
  if (received[3] == 0x1E) {
    Serial.println("30dBm module detected.");
    expectedConfig = expected_30dBm;
  } else {
    Serial.println("20dBm module detected.");
    expectedConfig = expected_20dBm;
  }

  // Compare the current configuration and program if necessary
  sendCommandIfMismatch(expectedConfig, 6);

  // Return the module to transparent mode
  set_transparent();
  return true;
}

//
// ======================== WIFI & MQTT FUNCTIONS ========================
//

// Initialize WiFi connection
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait until connected to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT callback: handle incoming messages (currently just prints the message)
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Ensure a persistent MQTT connection by reconnecting if disconnected
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "Mailbox-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Publish an MQTT discovery message for Home Assistant autodiscovery
void mqtt_discovery() {
  if (!client.connected()) {
    reconnect();
  }

  StaticJsonDocument<256> doc;
  doc["name"] = "Mailbox";
  doc["device_class"] = "occupancy";
  doc["state_topic"] = "homeassistant/binary_sensor/" + uniqueID + "/state";
  doc["unique_id"] = uniqueID;

  // Device metadata for Home Assistant
  JsonObject device = doc.createNestedObject("device");
  device["ids"] = macAddr;
  device["name"] = "Mailbox";
  device["mf"] = "Sensorsiot";
  device["mdl"] = "Notifier";
  device["sw"] = "1.0";
  device["hw"] = "0.9";

  char buffer[256];
  serializeJson(doc, buffer);
  Serial.println(discoveryTopic);
  Serial.println(buffer);

  // Publish retained discovery message
  client.publish(discoveryTopic.c_str(), buffer, true);
}

//
// ======================== ARDUINO SETUP & LOOP ========================
//

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start program");

  // Initialize Serial2 for module communication
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.flush();

  // Configure module control pins
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT);

  // Program the E32 module (only reprograms if necessary)
  programming_E32();

  // Initialize WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setBufferSize(512);
  client.setCallback(callback);

  // Generate a unique device ID based on the MAC address for MQTT topics
  macAddr = WiFi.macAddress();
  Serial.println(macAddr);
  String macLower = macAddr;
  macLower.toLowerCase();
  macLower.replace(":", "");
  uniqueID = "mailbox" + macLower.substring(macLower.length() - 4);
  messageTopic = "homeassistant/binary_sensor/" + uniqueID + "/state";
  discoveryTopic = "homeassistant/binary_sensor/" + uniqueID + "/config";

  // Publish MQTT discovery information
  mqtt_discovery();

  Serial.println("-----------------------------");
  Serial.println("Initialization finished.");
  Serial.println("-----------------------------");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Process incoming signals from the module on Serial2
  if (Serial2.available() > 0) {
    byte receivedCode = Serial2.read();

    // Print the received code (if not 0xFF)
    Serial.print("Received: 0x");
    Serial.println(receivedCode, HEX);

    // Process mailbox signals
    if (receivedCode == ARRIVED) {
      sendCommand(String((char)ACKNOWLEDGE));
      mailBoxStatus = fullStatus;
      Serial.println("Mailbox Full");
      client.publish(messageTopic.c_str(), "ON", true);
    } else if (receivedCode == EMPTY) {
      sendCommand(String((char)ACKNOWLEDGE));
      mailBoxStatus = emptyStatus;
      Serial.println("Mailbox Empty");
      client.publish(messageTopic.c_str(), "OFF", true);
    }
  }
}