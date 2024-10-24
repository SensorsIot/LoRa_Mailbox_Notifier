#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets1.h"

#define M0 21
#define M1 19
#define AUX 2

#define TXD2 17
#define RXD2 16

#define FULL 0x55
#define EMPTY 0xAA
#define ACKNOWLEDGE 0x25

byte receivedCode = 0;
bool transmissionSuccess = 0;
String macAddr;
String uniqueID;
String stateTopicName;
String discoveryTopicName;

enum boxStatus {
  empty,
  full
} mailBoxStatus = empty;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if a '1' was received as the first character
  if ((char)payload[0] == '1') {
    digitalWrite(2, LOW);  // Turn the LED on (active low)
  } else {
    digitalWrite(2, HIGH);  // Turn the LED off
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "Mailbox-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      //client.publish(stateTopicName.c_str(), "reconnected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Function to send MQTT discovery message
void mqtt_discovery() {
  if (!client.connected()) {
    reconnect();
  }

  StaticJsonDocument<256> doc;  // Allocate memory for the JSON document

  doc["name"] = "Mailbox";
  doc["device_class"] = "occupancy";
  doc["state_topic"] = "homeassistant/binary_sensor/" + uniqueID + "/state";
  doc["unique_id"] = uniqueID;  // Use MAC-based unique ID

  JsonObject device = doc.createNestedObject("device");
  device["ids"] = macAddr;      // Use the full MAC address as identifier
  device["name"] = "Mailbox";   // Device name
  device["mf"] = "Sensorsiot";  // Include supplier info
  device["mdl"] = "Notifier";
  device["sw"] = "1.0";
  device["hw"] = "0.9";
  
  char buffer[256];
  serializeJson(doc, buffer);  // Serialize JSON object to buffer

  Serial.println(discoveryTopicName.c_str());
  Serial.println(buffer);                                // Print the JSON payload to Serial Monitor
  client.publish(discoveryTopicName.c_str(), buffer, true);  // Publish to MQTT with retain flag set
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT_PULLUP);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setBufferSize(512);
  client.setCallback(callback);

  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(7);

  byte data[] = { 0xC0, 0x0, 0x1, 0x1A, 0x17, 0x44 };
  for (int i = 0; i < sizeof(data); i++) {
    Serial2.write(data[i]);
    Serial.println(data[i], HEX);
  }
  delay(10);

  while (digitalRead(AUX) == LOW)
    ;
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(100);
  Serial2.flush();

  // Get the MAC address of the board
  macAddr = WiFi.macAddress();
  Serial.println(macAddr);
  String hi = macAddr;
  hi.toLowerCase();
  hi.replace(":", "");  // Remove colons from MAC address to make it topic-friendly
  // Extract the last 6 characters of the MAC address (ignoring colons)
  uniqueID = "mailbox" + hi.substring(hi.length() - 4);  // Use last 4 byte

  stateTopicName = "homeassistant/binary_sensor/" + uniqueID + "/state";
  discoveryTopicName = "homeassistant/binary_sensor/" + uniqueID + "/config";

  // Send MQTT discovery message
  mqtt_discovery();
  Serial.println("Setup finished");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (Serial2.available() > 0) {
    while (Serial2.available() > 0) {
      receivedCode = Serial2.read();
      Serial.print(receivedCode, HEX);
      if (receivedCode == FULL) {
        transmissionSuccess = true;
        mailBoxStatus = full;
        Serial.println("Mailbox Full");
        client.publish(stateTopicName.c_str(), "ON", true);  // Update mailbox status
      }

      if (receivedCode == EMPTY) {
        transmissionSuccess = true;
        mailBoxStatus = empty;
        Serial.println("Mailbox empty");
        client.publish(stateTopicName.c_str(), "OFF", true);  // Update mailbox status
      }
    }
  }

  if (transmissionSuccess) {
    Serial2.write(ACKNOWLEDGE);
    Serial.println(mailBoxStatus);
    transmissionSuccess = false;
    Serial.println("Transmission acknowledged");
  }
}
