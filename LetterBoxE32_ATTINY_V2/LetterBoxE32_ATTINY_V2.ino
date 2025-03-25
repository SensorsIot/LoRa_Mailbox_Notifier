#include <avr/sleep.h>
#include <avr/interrupt.h>

//---------------------------------
// Pin Definitions
#define SWITCH_OPENING 10  // Opening switch (e.g. mailbox door opening)
#define SWITCH_DOOR 0      // Door switch (e.g. physical door press)
#define M0 6               // Module control: M0 pin
#define M1 7               // Module control: M1 pin
#define AUX 3              // Module ready indicator (AUX pin)
#define LED 1              // LED for testing (optional)

// Mailbox signal definitions
#define FULL 0x55        // Signal: mailbox filled (mail deposited)
#define EMPTY 0xAA       // Signal: mailbox empty (mail removed)
#define ACKNOWLEDGE 0x25  // Signal: acknowledgment received

//---------------------------------
// Global Variables & Program State
bool acknowledged = false;
unsigned long transmissionTime = 0;  // Time when a transmission was sent
int retransmissions = 0;
volatile bool opening_pressed = false;
volatile bool door_pressed = false;
byte signal;  // Signal to send (FULL or EMPTY)

// Program states for mailbox events
enum ProgramState {
  boxfilled,    // Mailbox has been filled (mail deposited)
  boxemptied,   // Mailbox has been emptied (mail removed)
  boxfull,      // Mailbox remains full; waiting for door action
  boxempty,     // Mailbox remains empty; waiting for opening action
  waitackfull,  // Waiting for ACK after sending FULL
  waitackempty  // Waiting for ACK after sending EMPTY
} programStatus = boxempty;  // Start with an empty mailbox

//---------------------------------
// Module Programming Configuration
// Expected parameter configurations for the two module types
byte expected_30dBm[6] = { 0xC0, 0x00, 0x01, 0x1A, 0x17, 0x47 };
byte expected_20dBm[6] = { 0xC0, 0x00, 0x01, 0x1A, 0x17, 0x44 };

//
// ======================== MODULE CONTROL FUNCTIONS ========================
//

// Wait until the AUX pin indicates that the module is ready.
void wait_aux() {
  while (digitalRead(AUX) == LOW) {
    delay(1);
  }
  delay(5);  // Additional delay for stability
}

// Set the module to programming (configuration) mode.
void set_programming() {
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(20);
  wait_aux();
}

// Set the module to transparent (normal operation) mode.
void set_transparent() {
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(20);
  wait_aux();
}

// Send a command over Serial using a String.
void sendCommand(const String& command) {
  wait_aux();
  Serial.write(command.c_str(), command.length());
  Serial.flush();
  delay(30);
}

// Send a command over Serial using a byte array.
void sendCommand(const byte* command, size_t len) {
  wait_aux();
  Serial.write(command, len);
  Serial.flush();
  delay(30);
}

// Request the stored configuration and compare it with the expected configuration.
// If a mismatch is detected, reprogram the module.
void sendCommandIfMismatch(const byte* expected, size_t len) {
  // Request the stored configuration from the module.
  sendCommand("\xC1\xC1\xC1");

  // Wait until 6 bytes are available on Serial.
  while (Serial.available() < 6) {
    delay(10);
  }
  
  // Read the 6-byte current configuration from the module.
  byte currentConfig[6];
  Serial.readBytes(currentConfig, 6);

  // Compare each byte with the expected configuration.
  bool mismatch = false;
  for (size_t i = 0; i < len; i++) {
    if (currentConfig[i] != expected[i]) {
      mismatch = true;
      break;
    }
  }

  // If a mismatch is detected, program the module with the expected parameters.
  if (mismatch) {
    sendCommand(expected, len);
  }
}

// Program the E32 module: switch to programming mode, request configuration,
// determine module type from the response, and program the module if needed.
bool programming_E32() {
  set_programming();
  sendCommand("\xC3\xC3\xC3");  // Request module configuration
  
  // Wait until 4 bytes are available.
  while (Serial.available() < 4) {
    delay(10);
  }
  
  byte received[4];
  Serial.readBytes(received, 4);

  // Determine module type based on the last byte of the response.
  const byte* expectedConfig;
  if (received[3] == 0x1E) {
    expectedConfig = expected_30dBm;
  } else {
    expectedConfig = expected_20dBm;
  }

  // Compare the current configuration and program if necessary.
  sendCommandIfMismatch(expectedConfig, 6);

  // Return the module to transparent mode.
  set_transparent();
  return true;
}

//
// ======================== INTERRUPT & SLEEP FUNCTIONS ========================
//

// ISR: Wake up when the door switch is activated.
void wakeUpDoor() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(SWITCH_DOOR));
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  door_pressed = true;
  opening_pressed = false;
}

// ISR: Wake up when the opening switch is activated.
void wakeUpOpening() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(SWITCH_OPENING));
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  opening_pressed = true;
  door_pressed = false;
}

// Put the module into sleep mode with interrupts enabled on the switch pins.
void goToSleep() {
  Serial.flush();
  // Set the module to a state that minimizes power consumption.
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  attachInterrupt(digitalPinToInterrupt(SWITCH_OPENING), wakeUpOpening, LOW);
  attachInterrupt(digitalPinToInterrupt(SWITCH_DOOR), wakeUpDoor, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

//
// ======================== COMMUNICATION FUNCTIONS ========================
//

// Check for incoming data on Serial and update acknowledgment flag if ACK is received.
void receive() {
  if (Serial.available() > 0) {
    byte incoming = Serial.read();
    if (incoming == ACKNOWLEDGE) {
      acknowledged = true;
    }
  }
}

//
// ======================== SETUP & LOOP FUNCTIONS ========================
//

void setup() {
  // Configure pins 0 to 11 as INPUT_PULLUP to reduce power consumption during sleep.
  for (byte i = 0; i <= 11; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  Serial.begin(9600);
  
  // Configure control and switch pins.
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(AUX, INPUT_PULLUP);
  pinMode(SWITCH_OPENING, INPUT_PULLUP);
  pinMode(SWITCH_DOOR, INPUT_PULLUP);

  // Program the module (reprogramming occurs only if configuration mismatches).
  programming_E32();

  // Start with the mailbox in the empty state and send the EMPTY signal.
  signal = EMPTY;
  sendCommand(&signal, 1);

  // Enter sleep mode until an external event (switch activation) occurs.
  goToSleep();
}

void loop() {
  switch (programStatus) {
    case boxfilled:
      signal = FULL;
      sendCommand(&signal, 1);
      transmissionTime = millis();
      retransmissions = 0;
      programStatus = waitackfull;
      break;

    case boxemptied:
      signal = EMPTY;
      sendCommand(&signal, 1);
      transmissionTime = millis();
      retransmissions = 0;
      programStatus = waitackempty;
      break;

    case boxfull:
      // Wait for a door action (to empty the mailbox).
      goToSleep();
      if (door_pressed) {
        programStatus = boxemptied;
      }
      break;

    case boxempty:
      // Wait for an opening action (to fill the mailbox).
      goToSleep();
      if (opening_pressed) {
        programStatus = boxfilled;
      }
      break;

    case waitackfull:
      if (acknowledged) {
        acknowledged = false;
        programStatus = boxfull;
      } else {
        if ((millis() - transmissionTime > 1000) && (retransmissions < 5)) {
          signal = FULL;
          sendCommand(&signal, 1);
          Serial.flush();
          retransmissions++;
          transmissionTime = millis();
        }
        if (retransmissions >= 5) {
          programStatus = boxfull;
        }
      }
      break;

    case waitackempty:
      if (acknowledged) {
        acknowledged = false;
        programStatus = boxempty;
      } else {
        if ((millis() - transmissionTime > 1000) && (retransmissions < 5)) {
          signal = EMPTY;
          sendCommand(&signal, 1);
          retransmissions++;
          transmissionTime = millis();
        }
        if (retransmissions >= 5) {
          programStatus = boxempty;
        }
      }
      break;
      
    default:
      break;
  }

  // Continuously check for incoming data to update the acknowledgment status.
  receive();
}
