#include <avr/sleep.h>
#include <avr/interrupt.h>

// #define SWITCH_OPENING 1   //PA5 for prototype
#define SWITCH_OPENING 10  //PA3
#define SWITCH_DOOR 0      //PA4
#define M0 7               //PB0
#define M1 6               //PB1
#define AUX 3              // PA7
#define LED 1              // PA5  for testing only

#define FULL 0x55
#define EMPTY 0xAA
#define ACKNOWLEDGE 0x25

bool transmitted = false;
bool acknowledged = false;
unsigned long transmissionTime = millis();
int retransmissions = 0;
bool opening_pressed = false;
bool door_pressed = false;

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

// --- Battery Voltage Reading ---
// This function uses the internal 1.1V reference to measure Vcc in millivolts.
// (Adapted from common Arduino techniques.)
uint16_t readBatteryVoltage() {
  // set the ADC to measure the internal 1.1V bandgap voltage against Vcc
  // ADMUX: REFS0=1 (use Vcc as reference), MUX[3:1]=111 to select 1.1V (see datasheet)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);  // Wait for the reference to settle
  ADCSRA |= _BV(ADSC);  // Start conversion
  while (ADCSRA & _BV(ADSC)) ;  // Wait until conversion is complete
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  uint16_t result = (high << 8) | low;
  // The typical constant is 1.1 (in volts) * 1023 * 1000 = 1125300.
  uint32_t vcc = 1125300UL / result; // Vcc in millivolts
  return (uint16_t)vcc;
}

// This helper function sends the status byte plus the two bytes of battery voltage.
void transmitWithBattery(byte status) {
  Serial.write(status);
  uint16_t batt = readBatteryVoltage();
  // Send the high byte then the low byte.
  Serial.write(highByte(batt));
  Serial.write(lowByte(batt));
  Serial.flush();
}

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
  // needed (I do not know why)
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  opening_pressed = false;
  door_pressed = true;
}

void wakeUpOpening() {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(SWITCH_OPENING));
  // needed (I do not know why)
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  opening_pressed = true;
  door_pressed = false;
}

void goToSleep() {
  /*
  Serial.print("opening_pressed ");
  Serial.print(opening_pressed);
  Serial.print(" door_pressed ");
  Serial.println(door_pressed);
  while (digitalRead(SWITCH_OPENING) != HIGH && digitalRead(SWITCH_DOOR != HIGH)) {  // debounce
    delay(100);
  }
  Serial.println(" go to sleep");
  */
  Serial.flush();
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  attachInterrupt(digitalPinToInterrupt(SWITCH_OPENING), wakeUpOpening, LOW);
  attachInterrupt(digitalPinToInterrupt(SWITCH_DOOR), wakeUpDoor, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Choose sleep mode
  sleep_enable();                       // Enable sleep mode
  sleep_cpu();                          // Go to sleep
}

void setup() {
  // Initialize pins 0 to 11 as INPUT_PULLUP to save deepsleep current
  for (byte i = 0; i <= 11; i++) pinMode(i, INPUT_PULLUP);
  Serial.begin(9600);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  pinMode(AUX, INPUT_PULLUP);
  pinMode(SWITCH_OPENING, INPUT_PULLUP);
  pinMode(SWITCH_DOOR, INPUT_PULLUP);

  delay(100);
  // Initialize the module (for 20dBm Module)
  byte data[] = { 0xC0, 0x0, 0x1, 0x1A, 0x17, 0x44 };
  for (unsigned int i = 0; i < sizeof(data); i++) {
    Serial.write(data[i]);
  }
  delay(100);
  // Enter sleep until an event occurs.
  goToSleep();
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  // On wakeup, transmit the initial EMPTY status along with battery voltage.
  transmitWithBattery(EMPTY);
}

void loop() {
  switch (programStatus) {
    case boxinit:
      // exit:
      if (digitalRead(SWITCH_OPENING) == LOW) programStatus = boxfilled;
      if (digitalRead(SWITCH_DOOR) == LOW) programStatus = boxemptied;
      break;
    case boxfilled:
      // Transmit FULL status plus battery voltage.
      transmitWithBattery(FULL);
      transmissionTime = millis();
      retransmissions = 0;
      programStatus = waitackfull;
      break;
    case boxemptied:
      // Transmit EMPTY status plus battery voltage.
      transmitWithBattery(EMPTY);
      transmissionTime = millis();
      retransmissions = 0;
      programStatus = waitackempty;
      break;
    case boxfull:
      goToSleep();
      // exit:
      if (door_pressed) programStatus = boxemptied;
      break;
    case boxempty:
      goToSleep();
      // exit:
      if (opening_pressed) programStatus = boxfilled;
      break;
    case waitackfull:
      if (acknowledged) {
        acknowledged = false;
        programStatus = boxfull;
      } else {
        if ((millis() - transmissionTime > 1000) && (retransmissions < 5)) {  // 5 retransmissions max every second
          transmitWithBattery(FULL);
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
          transmitWithBattery(EMPTY);
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

