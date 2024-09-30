#include <avr/sleep.h>
#include <avr/interrupt.h>

#define SWITCH_OPENING 0
#define SWITCH_DOOR 1
#define LORA_DEVICE 2
#define PIN3 3
#define PIN4 4  // Serial
#define PIN5 5  // Serial
#define M0 6
#define M1 7
#define AUX 8
#define TEST0 9
#define TEST1 10
#define PIN11 11

#define FULL 0x55
#define EMPTY 0xAA
#define ACKNOWLEDGE 0x25

bool transmitted = false;
bool acknowledged = false;
unsigned long transmissionTime = millis();
int retransmissions = 0;

enum boxStatus {
  empty,
  full
} mailBoxStatus = empty;

enum programstat {
  boxinit,
  boxfilled,
  boxemptied,
  boxfull,
  boxempty,
  waitackfull,
  waitackempty
} programStatus;

void receive() {
  byte _receivedCode = 0;
  if (Serial.available() > 0) {
    while (Serial.available()) {
      _receivedCode = Serial.read();
      if (_receivedCode == ACKNOWLEDGE) {
        acknowledged = true;
      }
    }
  }
}

// needs one ISR for each interrupt pin
void wakeUpDoor() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(SWITCH_DOOR));
  // needed (I do not kow why)
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
}

void wakeUpOpening() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(SWITCH_OPENING));
  // needed (I do not kow why)
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
}

void goToSleep() {
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  attachInterrupt(digitalPinToInterrupt(SWITCH_OPENING), wakeUpOpening, LOW);
  attachInterrupt(digitalPinToInterrupt(SWITCH_DOOR), wakeUpDoor, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Choose sleep mode
  sleep_enable();                       // Enable sleep mode
  sleep_cpu();                          // Go to sleep
}

void setup() {
  // Initialize Serial2 at 9600 baud rate
  for (byte i = 0; i <= 11; i++) pinMode(i, INPUT_PULLUP);  // to save deepsleep current
  Serial.begin(9600);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  pinMode(AUX, INPUT_PULLUP);
  pinMode(SWITCH_OPENING, INPUT_PULLUP);
  pinMode(SWITCH_DOOR, INPUT_PULLUP);
  pinMode(TEST0, OUTPUT);
  pinMode(TEST1, OUTPUT);

  // all unused pins to save energy
  pinMode(PIN3, INPUT_PULLUP);
  pinMode(PIN11, INPUT_PULLUP);

  byte data[] = { 0xC0, 0x0, 0x1, 0x1A, 0x17, 0x44 };
  for (unsigned int i = 0; i < sizeof(data); i++) {
    Serial.write(data[i]);
  }
  delay(10);
  //while (digitalRead(AUX) == LOW)
  goToSleep();
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(10);
  Serial.write(FULL);
}


void loop() {
  switch (programStatus) {
    case boxinit:
      //exit:
      if (digitalRead(SWITCH_OPENING) == LOW) programStatus = boxfilled;
      if (digitalRead(SWITCH_DOOR) == LOW) programStatus = boxemptied;
      break;
    case boxfilled:
      Serial.write(FULL);
      transmissionTime = millis();
      //exit:
      retransmissions = 0;
      programStatus = waitackfull;
      break;
    case boxemptied:
      Serial.write(EMPTY);
      transmissionTime = millis();
      //exit:
      retransmissions = 0;
      programStatus = waitackempty;
      break;
    case boxfull:
      goToSleep();
      //exit:
      if (digitalRead(SWITCH_DOOR) == LOW) programStatus = boxemptied;
      break;
    case boxempty:
      goToSleep();
      //exit:
      if (digitalRead(SWITCH_OPENING) == LOW) programStatus = boxfilled;
      break;
    case waitackfull:
      if (acknowledged) {
        acknowledged = false;
        programStatus = boxfull;
      } else {
        if ((millis() - transmissionTime > 1000) && (retransmissions < 5)) {  // 5 retransmissions max every second
          Serial.write(FULL);
          retransmissions++;
          transmissionTime = millis();
        }
        if (retransmissions >= 5) programStatus = boxfull;  // go to status anyway
      }
      break;
    case waitackempty:
      if (acknowledged) {
        acknowledged = false;
        programStatus = boxempty;
      } else {
        if ((millis() - transmissionTime > 1000) && (retransmissions < 5)) {  // 5 retransmissions max every second
          Serial.write(EMPTY);
          retransmissions++;
          transmissionTime = millis();
        }
        if (retransmissions >= 5) programStatus = boxempty;  // go to status anyway
      }
      break;
    default:
      break;
  }

  receive();
}
