//Pour la rotation
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

static int pulseState = 0;  // pulse state variable.

int PWM;
int Tik = 0;
int Tok = 0;
long Time;
//Pour la communication
int In = A1;

void setup() {
  Serial.begin(115200);
  pinMode(7, INPUT_PULLUP);   // Celui du dessous
  pinMode(6, INPUT_PULLUP);   // Celus du dessus
  pinMode(PULSE, OUTPUT);
  pinMode(DIR, OUTPUT);       // 1 fait de la supination et 0 de la pronation
  pinMode(ENABLE, OUTPUT);

  // Startup state of your pins.
  digitalWrite(ENABLE, 1);
  digitalWrite(DIR, 1);
  digitalWrite(PULSE, 0);
  digitalWrite(DIR, 1);
  Time = millis();
}


void loop() {
  // COmpteur de temps
  //  Tok = micros() - Tik;
  //  Tik = micros();
//  Serial.println(PWM);
  //  Serial.print(" ");
  //  Serial.println(Tok);
  

  PWM = pulseIn(In, HIGH, 0.001) / 8;
  static int pulseState = 0;

  Time = millis();
  if ((PWM) >= 25 && (PWM) < 75) { //Supination
    digitalWrite(DIR, 1);
    Serial.println("Up");
    delay(10);
  }
  else if ((PWM) >= 75 && (PWM) < 125) { //Pronation
    digitalWrite(DIR, 0);
    Serial.print(digitalRead(DIR));
    Serial.println("Down");
    delay(10);
  }


  if ((PWM) >= 25 && (PWM) < 75) { //Supination
    //    digitalWrite(DIR, 1);
        digitalWrite(DIR, 1);
    while ((millis() - Time) < 2000) {
      digitalWrite(DIR, 1);
      if (digitalRead(7) == HIGH) {
        pulseState ++;
        digitalWrite(PULSE, (pulseState & 0x01)); //Output high
        delayMicroseconds(150);   //Set rotate speed   }
      }
      else {
        break;
      }
    }
  }
  else if ((PWM) >= 75 && (PWM) < 125) { //Pronation
    //    digitalWrite(DIR, 0);
        digitalWrite(DIR, 0);
    while ((millis() - Time) < 2000) {
      digitalWrite(DIR, 0);
      if (digitalRead(6) == LOW) {

        pulseState ++;
        digitalWrite(PULSE, (pulseState & 0x01)); //Output high
        delayMicroseconds(150);   //Set rotate speed
      }
      else {
        break;
      }
    }
  }
}

//Mettre des if avec des break et on lit les commadnes dans les boucles while
