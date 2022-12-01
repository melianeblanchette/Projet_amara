//------------Librairies----------------------
#include <SoftwareSerial.h>
//---------------PINS-------------------------

const int EMG1 = A2; // Pin pour le signal du EMG 2
const int EMG2 = A3; // Pin pour le signal du EMG 3
const int OutPut = 7;


//---------------- Variables pour utilisateur-------------

const int Memory1 = 50; //Nombre de valeurs EMG utilisé pour faire la moyenne
const int Memory2 = 200; //Nombre de valeurs de difference utilisé pour faire la moyenne
const int Variation = 10;
const int NbEMG = 2;
const int Threshold = 200;
//--------------Variables de calculs-----------------

const int NbVariable = NbEMG + 1;
const int NbPosMatrice = NbVariable - 1;
const int Memoire1 = Memory1 - 1;
const int Memoire2 = Memory2 - 1;

int Sig_EMG_Last_Moyen[Memoire1 + 1];


long Somme;
int Moyenne;
int Signal[NbVariable];
int Difference[Memoire1 + 1];
long Decallage[Memoire1 + 1];
long SommeDecallage[Memoire2 + 1];
long TotalSommeDecallage = 0;
double MoyenneDecallage = 0;
double MoyenneDecallageLente = 0;
int activation = 0;
int Safety = 0;
double SafetyStart = 0;
double SafetyEnd = 0;
int Info = 0;

static long lastMillis = 0;
//--------------Variables de debuggage-----------------

int Loupe = 0;
int Bam = 0;

//---------------CONSTANTES-------------------------
//int Sig_EMG1 = 0; // Valeur analogique pour le signal du EMG1
//int Sig_EMG2 = 0; // Valeur analogique pour le signal du EMG2

SoftwareSerial serialNextion (5, 3); // 5-rx (sert à rien)  / 3-tx
//------------------------------------------------------------
void setup() {

  Serial.begin(57600);
  serialNextion.begin(57600);
  pinMode(OutPut, OUTPUT);
}


//------------------------------------------------------------
void loop() {

  //delay(500);

  Signal[1] = analogRead(EMG2);          //Valeur du EMG petit
  Signal[2] = analogRead(EMG2);          //Valeur du EMG gros

  Serial.print(Signal[1]); // Petit EMG
  Serial.print(",");
  Serial.print(Signal[2]); //Gros EMG


  //------Calcul du signal moyen------------------
  Signal[0] = (Signal[1] + Signal[2]) / NbEMG;

  Serial.print(",");
  Serial.print(Signal[0]);

  //----- Décallage des valeur dans la matrice pour laisser la position 0 à la nouvelle valeur-------

  for (int j = 0; j <= Memoire1 - 1; j++) {
    Sig_EMG_Last_Moyen[Memoire1 - j] = Sig_EMG_Last_Moyen[Memoire1 - 1 - j];
    Decallage[Memoire1 - j] = Decallage[Memoire1 - 1 - j];
  }
  for (int j = 0; j <= Memoire2 - 1; j++) {
    SommeDecallage[Memoire2 - j] = SommeDecallage[Memoire2 - 1 - j];
  }


  //----- Ajout nouvelle donnee avec filtre du peak-------


  if (Signal[0] > Sig_EMG_Last_Moyen[0] + Variation) {    //Ajout de la nouvelle donnée avec Variation pour éviter d'avoir des peaks
    Sig_EMG_Last_Moyen[0] = Sig_EMG_Last_Moyen[0] + Variation;

  }
  else if (Signal[0] < Sig_EMG_Last_Moyen[0] - Variation) {
    Sig_EMG_Last_Moyen[0] = Sig_EMG_Last_Moyen[0] - Variation;

  }
  else {
    Sig_EMG_Last_Moyen[0] = Signal[0];

  }



  //----- Calcul decalage-------
  Decallage[0] = abs(Signal[0] - Moyenne);


  //----- Calcul de la somme du signal et du decallage-------

  Somme = 0;
  SommeDecallage[0] = 0;
  TotalSommeDecallage = 0;
  for (int i = 0; i <= Memoire1; i++) { //calcul somme et print des valeurs
    Somme = Sig_EMG_Last_Moyen[i] + Somme;
    SommeDecallage[0] = Decallage[i] + SommeDecallage[0]; //On stock le decallage entre la moyenne et le signal pour les 50 derniere valeurs
  }
  for (int i = 0; i <= Memoire2; i++) {
    TotalSommeDecallage = SommeDecallage[i] + TotalSommeDecallage;
  }
  //----- Calcul de la moyenne du signal et du decallage-------
  Moyenne = Somme / (Memoire1 + 1);
  MoyenneDecallage = TotalSommeDecallage / (Memoire2 + 1);
  if (Signal[0] > 950) { // pour dire go si on a un signal de base eleve
    MoyenneDecallage = 10000;
  }

  if ((MoyenneDecallage - 50) > MoyenneDecallageLente) { // Si on a une variation de plus de 50 on le limite a ce point pour ne pas varier trop vite
    MoyenneDecallageLente = MoyenneDecallageLente + 50;
  }
  else if (MoyenneDecallage + 50 < MoyenneDecallageLente) {
    MoyenneDecallageLente = MoyenneDecallageLente - 50;
  }
  else {
    MoyenneDecallageLente = MoyenneDecallage;
  }
  //  Serial.print(",");
  //  Serial.print(Moyenne);
  //  Serial.print(",");
  //  Serial.print(SommeDecallage[0]);
  //  Serial.print(",");
  //  Serial.print(MoyenneDecallage);
  //  Serial.print(",");
  //  Serial.print(MoyenneDecallageLente);

  //----- Activation-------
  Info = 0;
  if (Moyenne > Threshold) {
    activation = 500;
    if (Safety == 0) {
      SafetyStart = millis();
    }
    Safety = 1;
    SafetyEnd = millis();
    if ((SafetyEnd - SafetyStart) > 500) {
      //      digitalWrite(OutPut, HIGH);
      Info = 100;
      Serial.print(",");
      Serial.print("Go");
    }
    else {
      //      digitalWrite(OutPut, LOW);
      Serial.print(",");
      Serial.print("No");
    }
  }
  else {
    Safety = 0;
    activation = 0;
    digitalWrite(OutPut, HIGH);
  }
  Serial.print(",");
  Serial.print(digitalRead(OutPut) * 1000);

  if (millis() - lastMillis >= 80)
  {
    lastMillis = millis();


    if (Moyenne < Threshold * 0.1)
    {
      Serial.print(", Serial: 10");
      Info = Info+15;
    }
    else if (Moyenne < Threshold * 0.2)
    {
      Serial.print(", Serial: 20");
      Info = Info+30;
    }
    else if (Moyenne < Threshold * 0.4)
    {
      Serial.print(", Serial: 40");
      Info = Info+45;
    }
    else if (Moyenne < Threshold * 0.6)
    {
      Serial.print(", Serial: 60");
      Info = Info+60;
    }
    else if (Moyenne < Threshold * 0.8)
    {
      Serial.print(", Serial: 80");
      Info = Info+75;
    }
    else if (Moyenne < Threshold)
    {
      Serial.print(", Serial: 90");
      Info = Info+90;
    }
    else if (Moyenne > Threshold)
    {
      Serial.print(", Serial: 100");
      Info = Info+99;
    }

    // serialNextion.write(Moyenne);
    // serialNextion.write('\n');
    Serial.print(",");
    Serial.print(Moyenne);
    serialNextion.write(Info);
  }
  //delay(50);
  Serial.println("");
  //  delay(100);
}
