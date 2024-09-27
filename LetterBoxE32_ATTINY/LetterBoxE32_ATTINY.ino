#include <avr/sleep.h>
#include <avr/interrupt.h>

#define M0 6
#define M1 7
#define AUX 8
#define SWITCH_OPENING 0
#define SWITCH_DOOR 1
#define LORA_DEVICE 2

#define FULL 0x55
#define EMPTY 0xAA
#define ACKNOWLEDGE 0x25

bool transmitted = false;
bool acknowledged = false;

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

void test() {
  digitalWrite(3, HIGH);
  delay(100);
  digitalWrite(3, LOW);
}

void receive() {
  byte _receivedCode = 0;
  if (Serial.available() > 0) {
    while (Serial.available()) {
      _receivedCode = Serial.read();
      //Serial.print("Received: ");
      //Serial.println(_receivedCode, HEX);
      if (_receivedCode == ACKNOWLEDGE) {
        acknowledged = true;
      }
    }
  }
}

void setup() {
  // Initialize Serial2 at 9600 baud rate
  Serial.begin(9600);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT_PULLUP);
  pinMode(SWITCH_OPENING, INPUT_PULLUP);
  pinMode(SWITCH_DOOR, INPUT_PULLUP);


  byte data[] = { 0xC0, 0x0, 0x1, 0x1A, 0x17, 0x44 };
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(7);
  for (unsigned int i = 0; i < sizeof(data); i++) {
    Serial.write(data[i]);
  }
  delay(10);
  while (digitalRead(AUX) == LOW)
    digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  Serial.write(EMPTY);

  //Serial.println("Init finished");
}


void loop() {
  //Serial.print(digitalRead(SWITCH_OPENING));
  //Serial.println(digitalRead(SWITCH_DOOR));

  switch (programStatus) {
    case boxinit:
      //Serial.println("Init");
      //exit:
      if (digitalRead(SWITCH_OPENING) == LOW) programStatus = boxfilled;
      if (digitalRead(SWITCH_DOOR) == LOW) programStatus = boxemptied;
      break;
    case boxfilled:
      //Serial.println("Filling");
      Serial.write(FULL);
      //exit:
      programStatus = waitackfull;
      break;
    case boxemptied:
      //Serial.println("Emptiing");
      Serial.write(EMPTY);
      //exit:
      programStatus = waitackempty;
      break;
    case boxfull:
      //Serial.println("Full");
      acknowledged = false;
      //exit:
      if (digitalRead(SWITCH_DOOR) == LOW) programStatus = boxemptied;
      break;
    case boxempty:
      //Serial.println("Empty");
      acknowledged = false;
      //exit:
      if (digitalRead(SWITCH_OPENING) == LOW) programStatus = boxfilled;
      break;
    case waitackfull:
      //Serial.println("waitackfull");
      if (acknowledged) {
        programStatus = boxfull;
      }
      break;
    case waitackempty:
      //Serial.println("Waitackempty");
      if (acknowledged) {
          programStatus = boxempty;
      }
      break;
    default:
      break;
  }

  receive();
}
