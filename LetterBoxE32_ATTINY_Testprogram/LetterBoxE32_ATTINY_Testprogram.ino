
// This program generates full and empty messages on 433MHz


#define M0 6   //PB0
#define M1 7   //PB1
#define AUX 3  // PA7

#define TXD2 17
#define RXD2 16


#define FULL 0x55
#define EMPTY 0xAA
#define ACKNOWLEDGE 0x25

bool transmitted = false;
bool acknowledged = false;
unsigned long transmissionTime = millis();
int retransmissions = 0;
bool opening_pressed = false;
bool door_pressed = false;

void waitForAUX() {
  while (digitalRead(AUX) == LOW)
    delay(10);
}




void setup() {
  delay(1000);
  Serial.begin(9600);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT_PULLUP);

  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(100);
  byte data1[] = { 0xC4, 0xC4, 0xC4 };
  Serial.write(data1, sizeof(data1));
  Serial.flush();
delay(100);
  //byte data[] = { 0xC0, 0x0, 0x1, 0x1A, 0x17, 0x47 }; // for 30dBm Module
  byte data[] = { 0xC0, 0x0, 0x1, 0x1A, 0x17, 0x44 };  // for 20dBm Module
  Serial.write(data, sizeof(data));
  delay(100);
  waitForAUX();
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
}


void loop() {
  waitForAUX();
  Serial.write(FULL);
  //Serial.println("Full");
  delay(5000);
  waitForAUX();
  Serial.write(EMPTY);
  //Serial.println("empty");
  delay(2000);
}
