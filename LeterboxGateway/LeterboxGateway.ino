#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets1.h"

#define M0 21
#define M1 19
#define AUX 15


#define TXD2 17
#define RXD2 16

#define ARRIVED 0x55
#define EMPTY 0xAA
#define ACKNOWLEDGE 0x25

byte receivedCode = 0;
bool transmissionSuccess = 0;

enum boxStatus {
  empty,
  full
} mailBoxStatus = empty;

// Update these with values suitable for your network.



WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
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

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(2, LOW);  // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("MailboxStatus", "hello world");
      // ... and resubscribe
      // client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}




void setup() {
  // Initialize Serial2 at 9600 baud rate
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT_PULLUP);
  // Array of hexadecimal values
  byte data[] = { 0xC0, 0x0, 0x1, 0x1A, 0x17, 0x44 };

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(7);
  for (int i = 0; i < sizeof(data); i++) {
    Serial2.write(data[i]);
    Serial.println(data[i], HEX);
  }
  delay(10);

  while (digitalRead(AUX) == LOW)
    digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(100);
  Serial2.flush();
  Serial.println("Init finished");
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (Serial2.available() > 0) {
    while (Serial2.available() > 0) {
      receivedCode = Serial2.read();
      //Serial.print(receivedCode, HEX);
      if (receivedCode == 0x55) {
        Serial2.write(ACKNOWLEDGE);
        Serial.println("Transmission acknowledged");
        mailBoxStatus = full;
        Serial.println("Mailbox Full");
        snprintf(msg, MSG_BUFFER_SIZE, "Mailbox #%ld", mailBoxStatus);
        client.publish("outTopic", msg);
      }
      if (receivedCode == 0xAA) {
        Serial2.write(ACKNOWLEDGE);
        Serial.println("Transmission acknowledged");
        mailBoxStatus = empty;
        Serial.println("Mailbox empty");
        snprintf(msg, MSG_BUFFER_SIZE, "Mailbox #%ld", mailBoxStatus);
        client.publish("outTopic", msg);
      }
    }
  }
}
