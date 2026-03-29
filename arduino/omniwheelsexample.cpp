#include <Arduino.h>
/*-----------------------------------------------*/
// Pin assignment (Motors)
/*-----------------------------------------------*/
const int motor1A = 8;  // Motor 1 VL
const int motor1B = 9;
const int motor2A = 10;  // Motor 2 HL
const int motor2B = 11;
const int motor3A = 4;  // Motor 3 VR
const int motor3B = 5;
const int motor4A = 6;  // Motor 4 HR
const int motor4B = 7;

/*-----------------------------------------------*/
// Pin assignment (Encoders)
/*-----------------------------------------------*/
const int encoder1A = 12;  // Encoder 1 VL
const int encoder1B = 2;
const int encoder2A = 13;  // Encoder 2 HL
const int encoder2B = 3;
const int encoder3A = 14;  // Encoder 3 VR
const int encoder3B = 18;
const int encoder4A = 15;  // Encoder 4 HR
const int encoder4B = 19;

/*-----------------------------------------------*/
// Global Support variables
/*-----------------------------------------------*/
int speed = 100;

int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;

long ENC1T1 = 0;
long ENC1T2 = 0;
long ENC2T1 = 0;
long ENC2T2 = 0;
long ENC3T1 = 0;
long ENC3T2 = 0;
long ENC4T1 = 0;
long ENC4T2 = 0;

float rpm1 = 0;
float rpm2 = 0;
float rpm3 = 0;
float rpm4 = 0;

boolean cw1;
boolean cw2;
boolean cw3;
boolean cw4;

/*-----------------------------------------------*/
// Configuring the pins and the event detection
/*-----------------------------------------------*/
void setup() {
  Serial.begin(9600);

  // Motor pins setup
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(motor3A, OUTPUT);
  pinMode(motor3B, OUTPUT);
  pinMode(motor4A, OUTPUT);
  pinMode(motor4B, OUTPUT);

  // Encoder pins setup
  pinMode(encoder1A, INPUT_PULLUP);
  pinMode(encoder1B, INPUT_PULLUP);
  pinMode(encoder2A, INPUT_PULLUP);
  pinMode(encoder2B, INPUT_PULLUP);
  pinMode(encoder3A, INPUT_PULLUP);
  pinMode(encoder3B, INPUT_PULLUP);
  pinMode(encoder4A, INPUT_PULLUP);
  pinMode(encoder4B, INPUT_PULLUP);

  // Event detection setup
  attachInterrupt(digitalPinToInterrupt(encoder1B), pulse1, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder2B), pulse2, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder3B), pulse3, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder4B), pulse4, FALLING);
}

/*-----------------------------------------------*/
// Main loop for driving the Motors forwards or backwards
/*-----------------------------------------------*/
void loop() {
  //a) Forward
  Serial.println("Forward");
  analogWrite(motor1A, speed);  // Motor 1 VL
  analogWrite(motor1B, 0);
  analogWrite(motor2A, speed);  // Motor 2 HL
  analogWrite(motor2B, 0);
  analogWrite(motor3A, speed);  // Motor 3 VR
  analogWrite(motor3B, 0);
  analogWrite(motor4A, speed);  // Motor 4 HR
  analogWrite(motor4B, 0);
  for (int i = 0; i < 10; i++) {
    Encoder1();
    Encoder2();
    Encoder3();
    Encoder4();
    writeText();
    delay(1000);
  }
  Serial.println("--------------------------------------------------");

  //b) Right
  Serial.println("Right");
  analogWrite(motor1A, speed);  // Motor 1 VL
  analogWrite(motor1B, 0);
  analogWrite(motor2A, 0);  // Motor 2 HL
  analogWrite(motor2B, speed);
  analogWrite(motor3A, 0);  // Motor 3 VR
  analogWrite(motor3B, speed);
  analogWrite(motor4A, speed);  // Motor 4 HR
  analogWrite(motor4B, 0);
  for (int i = 0; i < 5; i++) {
    Encoder1();
    Encoder2();
    Encoder3();
    Encoder4();
    writeText();
    delay(1000);
  }
  Serial.println("--------------------------------------------------");

  //c) Diagonally right
  Serial.println("Diagonally right");
  analogWrite(motor1A, 0);  // Motor 1 VL
  analogWrite(motor1B, 0);
  analogWrite(motor2A, speed);  // Motor 2 HL
  analogWrite(motor2B, 0);
  analogWrite(motor3A, speed);  // Motor 3 VR
  analogWrite(motor3B, 0);
  analogWrite(motor4A, 0);  // Motor 4 HR
  analogWrite(motor4B, 0);
  for (int i = 0; i < 5; i++) {
    Encoder1();
    Encoder2();
    Encoder3();
    Encoder4();
    writeText();
    delay(1000);
  }
  Serial.println("--------------------------------------------------");

  //d) Rotate around the rear right wheel
  Serial.println("Rotate around the rear right wheel");
  analogWrite(motor1A, speed);  // Motor 1 VL
  analogWrite(motor1B, 0);
  analogWrite(motor2A, speed);  // Motor 2 HL
  analogWrite(motor2B, 0);
  analogWrite(motor3A, 0);  // Motor 3 VR
  analogWrite(motor3B, 0);
  analogWrite(motor4A, 0);  // Motor 4 HR
  analogWrite(motor4B, 0);
  for (int i = 0; i < 5; i++) {
    Encoder1();
    Encoder2();
    Encoder3();
    Encoder4();
    writeText();
    delay(1000);
  }
  Serial.println("--------------------------------------------------");

  //e) Rotate around center
  Serial.println("Rotate around center");
  analogWrite(motor1A, speed);  // Motor 1 VL
  analogWrite(motor1B, 0);
  analogWrite(motor2A, speed);  // Motor 2 HL
  analogWrite(motor2B, 0);
  analogWrite(motor3A, 0);  // Motor 3 VR
  analogWrite(motor3B, speed);
  analogWrite(motor4A, 0);  // Motor 4 HR
  analogWrite(motor4B, speed);
  for (int i = 0; i < 5; i++) {
    Encoder1();
    Encoder2();
    Encoder3();
    Encoder4();
    writeText();
    delay(1000);
  }
  Serial.println("--------------------------------------------------");

  //f) Rotate around the center of the rear axle
  Serial.println("Rotate around the center of the rear axle");
  analogWrite(motor1A, 0);  // Motor 1 VL
  analogWrite(motor1B, 0);
  analogWrite(motor2A, speed);  // Motor 2 HL
  analogWrite(motor2B, 0);
  analogWrite(motor3A, 0);  // Motor 3 VR
  analogWrite(motor3B, 0);
  analogWrite(motor4A, 0);  // Motor 4 HR
  analogWrite(motor4B, speed);
  for (int i = 0; i < 5; i++) {
    Encoder1();
    Encoder2();
    Encoder3();
    Encoder4();
    writeText();
    delay(1000);
  }
  Serial.println("--------------------------------------------------");
}

/*-----------------------------------------------*/
// Interrupt callback function to decide the rotational direction
/*-----------------------------------------------*/
void pulse1() {
  if (digitalRead(encoder1A) == LOW) {
    count1++;
    cw1 = false;
  } else {
    count1++;
    cw1 = true;
  }
  ENC1T2 = millis();
}

void pulse2() {
  if (digitalRead(encoder2A) == LOW) {
    count2++;
    cw2 = false;
  } else {
    count2++;
    cw2 = true;
  }
  ENC2T2 = millis();
}

void pulse3() {
  if (digitalRead(encoder3A) == LOW) {
    count3++;
    cw3 = false;
  } else {
    count3++;
    cw3 = true;
  }
  ENC3T2 = millis();
}

void pulse4() {
  if (digitalRead(encoder4A) == LOW) {
    count4++;
    cw4 = false;
  } else {
    count4++;
    cw4 = true;
  }
  ENC4T2 = millis();
}

/*-----------------------------------------------*/
// Reading the Motors built in Encoder
/*-----------------------------------------------*/
void Encoder1() {
  if (ENC1T2 > ENC1T1) {
    // Calculating the rpm of the motor
    rpm1 = 60000 * count1 / (ENC1T2 - ENC1T1);
    rpm1 = rpm1 / 234.3;  // Gear ratio
    ENC1T1 = ENC1T2;
    count1 = 0;
  }
}

void Encoder2() {
  if (ENC2T2 > ENC2T1) {
    rpm2 = 60000 * count2 / (ENC2T2 - ENC2T1);
    rpm2 = rpm2 / 234.3;  // Gear ratio
    ENC2T1 = ENC2T2;
    count2 = 0;
  }
}

void Encoder3() {
  if (ENC3T2 > ENC3T1) {
    rpm3 = 60000 * count3 / (ENC3T2 - ENC3T1);
    rpm3 = rpm3 / 234.3;  // Gear ratio
    ENC3T1 = ENC3T2;
    count3 = 0;
  }
}

void Encoder4() {
  if (ENC4T2 > ENC4T1) {
    rpm4 = 60000 * count4 / (ENC4T2 - ENC4T1);
    rpm4 = rpm4 / 234.3;  // Gear ratio
    ENC4T1 = ENC4T2;
    count4 = 0;
  }
}

/*-----------------------------------------------*/
// Writing text to putput Motor RPM
/*-----------------------------------------------*/
void writeText() {
  if (cw1 == true) {
    Serial.print("Motor1 (VL) forward RPM = ");
    Serial.println(rpm4);
  } else if (cw4 == false) {
    Serial.print("Motor1 (VL) backwards RPM = ");
    Serial.println(rpm1);
  }
  if (cw2 == true) {
    Serial.print("Motor2 (HL) forward RPM = ");
    Serial.println(rpm2);
  } else if (cw2 == false) {
    Serial.print("Motor2 (HL) backwards RPM = ");
    Serial.println(rpm2);
  }
  if (cw3 == true) {
    Serial.print("Motor3 (VR) forward RPM = ");
    Serial.println(rpm3);
  } else if (cw3 == false) {
    Serial.print("Motor3 (VR) backwards RPM = ");
    Serial.println(rpm3);
  }
  if (cw4 == true) {
    Serial.print("Motor4 (HR) forward RPM = ");
    Serial.println(rpm4);
  } else if (cw4 == false) {
    Serial.print("Motor4 (HR) backwards RPM = ");
    Serial.println(rpm4);
  }
}