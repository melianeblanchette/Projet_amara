/*
  Example sketch for the Xbox ONE USB library - by guruthree, based on work by
  Kristian Lauszus.
*/

#include <XBOXONE.h>

#define PULSE 9
#define DIR 8
#define ENABLE 10
#define GEARBOX 90.25 // Change that for the right one.
#define MICROSTEP 2   // 400 setting on the driver.
#define STEPBYREV 200 // 360deg / 1.8deg = 200
#define EFFICIENCY 0.5 // Gearbox efficiency
#define RATIO 9.53 // Ratio engrenage droit/demi-lune
#define ANGLE_AB 45 // Angle de rotation de l'avant-bras
#define TOUR 0.25

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXONE Xbox(&Usb);
int Etat = 0;
int out = 3;


void setup() {
  pinMode(7, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(PULSE, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  // Startup state of your pins.
  digitalWrite(ENABLE, 1);
  digitalWrite(DIR, 0);
  digitalWrite(PULSE, 0);


  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));

  pinMode(out, OUTPUT);
}
void loop() {
  Usb.Task();
  Etat = 237;
  if (Xbox.XboxOneConnected) {
    if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500 || Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500 || Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500) {
        Serial.print(F("LeftHatX: "));
        Serial.print(Xbox.getAnalogHat(LeftHatX));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500) {
        Serial.print(F("LeftHatY: "));
        Serial.print(Xbox.getAnalogHat(LeftHatY));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
        Serial.print(F("RightHatX: "));
        Serial.print(Xbox.getAnalogHat(RightHatX));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
        Serial.print(F("RightHatY: "));
        Serial.print(Xbox.getAnalogHat(RightHatY));
      }
      Serial.println();
    }

    if (Xbox.getButtonPress(L2) > 0 || Xbox.getButtonPress(R2) > 0) {
      if (Xbox.getButtonPress(L2) > 0) {
        Serial.print(F("L2: "));
        Serial.print(Xbox.getButtonPress(L2));
        Serial.print("\t");
      }
      if (Xbox.getButtonPress(R2) > 0) {
        Serial.print(F("R2: "));
        Serial.print(Xbox.getButtonPress(R2));
        Serial.print("\t");
      }
      Serial.println();
    }

    // Set rumble effect
    static uint16_t oldL2Value, oldR2Value;
    if (Xbox.getButtonPress(L2) != oldL2Value || Xbox.getButtonPress(R2) != oldR2Value) {
      oldL2Value = Xbox.getButtonPress(L2);
      oldR2Value = Xbox.getButtonPress(R2);
      uint8_t leftRumble = map(oldL2Value, 0, 1023, 0, 255); // Map the trigger values into a byte
      uint8_t rightRumble = map(oldR2Value, 0, 1023, 0, 255);
      if (leftRumble > 0 || rightRumble > 0)
        Xbox.setRumbleOn(leftRumble, rightRumble, leftRumble, rightRumble);
      else
        Xbox.setRumbleOff();
    }

    if (Xbox.getButtonPress(UP)) {    //Press peut etre remplacer par click
      Serial.println(F("Up"));
      Etat = 11;
    }
    if (Xbox.getButtonPress(DOWN)) {
      Serial.println(F("Down"));
      Etat = 12;
    }
    if (Xbox.getButtonPress(LEFT)) {
      Serial.println(F("Left"));
      Etat = 13;
    }
    if (Xbox.getButtonPress(RIGHT)) {
      Serial.println(F("Right"));
      Etat = 14;
    }

    if (Xbox.getButtonPress(START)) {
      Serial.println(F("Start"));
      Etat = 100;
    }
    if (Xbox.getButtonPress(BACK)) {
      Serial.println(F("Back"));
      Etat = 200;
    }
    if (Xbox.getButtonPress(XBOX)) {
      Serial.println(F("Xbox"));
      Etat = 500;
    }
    if (Xbox.getButtonPress(SYNC)) {
      Serial.println(F("Sync"));
      Etat = 300;
    }

    if (Xbox.getButtonPress(L1)) {
      Serial.println(F("L1"));
      Etat = 21;
    }
    if (Xbox.getButtonPress(R1)) {
      Serial.println(F("R1"));
      Etat = 22;
    }
    if (Xbox.getButtonPress(L2)) {
      Serial.println(F("L2"));
      Etat = 23;
    }
    if (Xbox.getButtonPress(R2)) {
      Serial.println(F("R2"));
      Etat = 24;
    }
    if (Xbox.getButtonPress(L3)) {
      Serial.println(F("L3"));
      Etat = 25;
    }
    if (Xbox.getButtonPress(R3)) {
      Serial.println(F("R3"));
      Etat = 26;
    }


    if (Xbox.getButtonPress(A)) {
      Serial.println(F("A"));
      Etat = 1;
    }
    if (Xbox.getButtonPress(B)) {
      Serial.println(F("B"));
      Etat = 2;
    }
    if (Xbox.getButtonPress(X)) {
      Serial.println(F("X"));
      Etat = 3;
    }
    if (Xbox.getButtonPress(Y)) {
      Serial.println(F("Y"));
      Etat = 4;

    }
    analogWrite(out, Etat * 2);
  }
  Serial.println(Etat);
  delay(1);
}
void POIGNET() {
  static int pulseState = 0;  // pulse state variable.
  //static int dir = 0;         // Direction variable
  digitalWrite(DIR, 0);
  for (unsigned long i = 0; i < TOUR * 1 / EFFICIENCY * STEPBYREV * MICROSTEP * GEARBOX; i++) {
    pulseState ++;
    digitalWrite(PULSE, (pulseState & 0x01)); // toggle the pin. Write 1 or 0.
    delayMicroseconds(200);                 // datasheet page 10
  }

  while (digitalRead(6) == LOW) {
    pulseState ++;
    digitalWrite(PULSE, (pulseState & 0x01)); //Output high
    delayMicroseconds(200);   //Set rotate speed
  }

  digitalWrite(DIR, 1);
  while (digitalRead(7) == LOW) {
    pulseState ++;
    digitalWrite(PULSE, (pulseState & 0x01)); //Output high
    delayMicroseconds(200);   //Set rotate speed
  }
}
