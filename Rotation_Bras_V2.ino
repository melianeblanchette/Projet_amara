#define PULSE 9
#define DIR 8
#define ENABLE 10

//int joystick;


int val1 = 0;      // variable to store the read value
int val2 = 0;      // variable to store the read value
int x;
void setup() {
  // Define the pins you'll be using.
  pinMode(PULSE, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(4, INPUT); //Comm Mega: Enable
  pinMode(5, INPUT); //Comme Mega: Direction
  pinMode(7, INPUT_PULLUP);   // Limit switch dessous
  pinMode(6, INPUT_PULLUP);   // Limit switch dessus

  // Startup state of your pins.
  digitalWrite(ENABLE, 1);
  digitalWrite(DIR, 0);
  digitalWrite(PULSE, 0);

  //affichage
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {

  val1 = digitalRead(4);   // read enable pin Mega Comm
  val2 = digitalRead(5);   // read direction pin Mega Comm
  Serial.print(val1);
  Serial.println(val2);
  int LSdown = digitalRead(7); //1=pressed, 0=unpressed
  int LSup = digitalRead(6);   //1=pressed, 0=unpressed

  //Serial.println(LSdown);
  //Serial.println(LSup);
  int dis = 1;
  if (val1 == 1) { //Enabled

    if (val2 == 1 && LSup == 0) { //Supination
      Serial.println("Sup");
      Serial.println(val2);
      Serial.println(LSup);
      digitalWrite(DIR, HIGH);    //Set high level direction
      dis = 0;
      while (LSup == 0) {
        LSup = digitalRead(6);
        digitalWrite(PULSE, HIGH);   //Output high
        delayMicroseconds(50);   //Set rotate speed
        digitalWrite(PULSE, LOW);     //Output low
        delayMicroseconds(50);    //Set rotate speed
      }
    }
    else if (val2 == 0 && LSdown == 0) { //Pronation
      Serial.println("Pro");
      Serial.println(val2);
      Serial.println(LSdown);
      digitalWrite(DIR, LOW);    //Set low level direction
      dis = 0;
      double Time = millis() + 7000;
      while ((Time - millis() > 0)) {
        if (digitalRead(7)==1){
          break;
        }
        for (x = 0; x < 500; x++)   //Repeat 400 times a revolution when setting is 400 on driver
        {
          //        LSup = digitalRead(6);
          digitalWrite(PULSE, HIGH);   //Output high
          delayMicroseconds(50);   //Set rotate speed
          digitalWrite(PULSE, LOW);     //Output low
          delayMicroseconds(50);    //Set rotate speed
        }
      }
      delay(500);
    }
    //delay(5); // Delay after changing direction.

    //    if (dis == 0) {
    //
    //      for (x = 0; x < 500; x++)   //Repeat 400 times a revolution when setting is 400 on driver
    //      {
    //
    //        digitalWrite(PULSE, HIGH);   //Output high
    //        delayMicroseconds(50);   //Set rotate speed
    //        digitalWrite(PULSE, LOW);     //Output low
    //        delayMicroseconds(50);    //Set rotate speed
    //      }
    //    }
  }

}
