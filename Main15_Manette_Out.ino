
// X est en avant
// Y est sur le côté gauche
// Z est dans les air
// Valeur en cm

//----------LIBRARIES-------------------------------

#include <math.h>
//Pour les moteurs
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>


#include<Wire.h>
//Pour l'écran
#include<LiquidCrystal_I2C.h>

//Pour la manette
#include <XBOXONE.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXONE Xbox(&Usb);
//----------PINS------------------------------------
const int Sig1 = 2; //Signal provenant du Nano des EMG indicant si on a une activation ou non
const int Sig2 = 4; //Signal provenant du Nano des EMG indicant si on a une activation ou non
const int Sig3 = 7; //Signal provenant de l ordi oculaire indicant si on a une activation ou non
const int Sig4 = 8;
const int Sig5 = 12;
const int Sig6 = 41;
const int LedR = 51;
const int LedB = 53;
const int LedG = 52;
const int LedY = 50;
const int LedW = 49;
const int SwitchOn = 29;
const int PotSpeed = A1;
const int ComRotMain = 11;
const int ValRotMain = A2;


//-------------VARIABLES UTILISATEUR-------------------
const int Lo1 = 20;
const int Lo2 = 40;
const int Ra1 = 9.5; //Distance entre le centre du pivot de la base et le début du pivot des tige du haut du bras
const int Ra2 = 9.5; //Distance entre le centre du pivot du coude et la fin du pivot des tige du haut du bras
int DManuel = 2; //Déplacement manuel demandé en mode proto
double Precision = 1; // Precision en cm
double PRepos[3] = {38, 40, -10}; //X = avant Y = gauche Z= Up
double PBouche[3] = {18, 50, 10};
double PFunny[3] = {50, 50, 10};
double PInit[3] = {Lo1 + Lo2 + Ra1 + Ra2 - 1, 0 , 0};
int TAgain = 5; //Temps qu'on doit appuye sur le bouton pour continuer le mouvement si celui-ci a ete lache
//int Speed = 70; //Valeur arbitraire qui détermine la vitesse de mouvement, plus le chiffre est petit, plus on va vite (de 0 à l'infini)
int Speed2 = 20; //Longueur max ue peut avoir une iteration de calcul (plus le chiffre est petit plus le deplacement sera rapide mais moins clean)

double DecaRot0 = -60;    //-30 Difference en degrés entre la position attendue et la position réel (positif si la valeur est à gauche ou en bas de ce u'on veut)
double DecaRot1 = 30;  //-33;    //Difference en degrés entre la position attendue et la position réel (positif si la valeur est à gauche ou en bas de ce u'on veut)
double DecaRot2 = 20;    //Difference en degrés entre la position attendue et la position réel (positif si la valeur est à gauche ou en bas de ce u'on veut)

//Limite d'angle des moteurs
int Lim0D = -45;
int Lim0G = 45;
int Lim1H = 35;
int Lim1B = -35;
int Lim2D = -3;
int Lim2G = 130;


//-------------VARIABLES DE CALCULS--------------------
int NbI = 1;
double PAb[3] = {PRepos[0], PRepos[1], PRepos[2]};    //Position de l'avant bras
double PAbF[3] = {PRepos[0], PRepos[1], PRepos[2]};   //Position de l'avant bras desire
double PAbFT[3] = {PRepos[0], PRepos[1], PRepos[2]};  // PAbF Temporelle
double PDAbX = 0; //Deplacement demande de l'avant bras en X
double PDAbY = 0; //Deplacement demande de l'avant bras en Y
double PDAbZ = 0; //Deplacement demande de l'avant bras en Z

double PM0[3] = {0, 0, 0};                     //Position du moteur de la base
double PM2[3] = {Lo1, 0, 0};                    //Position du moteur du coude
double PMF2[3];                                //Position desire du moteur du coude
double LHautReel = 0;
double Distance = 0;


float Angle[3];   //Valeurs des angles des moteurs en temps reel
float AReal[3];

int S1A = 0;  //Switch Ordi 1
int S1B = 0;  //Switch Ordi 2

int S2A = 2;  //Switch Ordi 1
int S2B = 2;  //Switch Ordi 2

int S3A = 0;  //Switch Ordi 1
int S3B = 0;  //Switch Ordi 2

int S6A = 0;  //Switch Ordi 1
int S6B = 0;  //Switch Ordi 2

int Start = 0;
int Continue = 0;
int Stop = 0;

int NM = 0; //Numéro (identifiant) des moteurs

int ModeTest = 0;

float RManuel0 = 0;
float RManuel1 = 0;
float RManuel2 = 0;

int Etat = 0;
int Switch69A = 0;
int Switch69B = 0;

int Index = 0;

float Amp0;
float Amp1;
float Amp2;

float T0;
float T1;
float T2;

int EMG = 0;
double EMGStart = 0;
double EMGEnd = 0;
int EMGSwitch = 0;

int ERREUR = 0;

int Condition = 0;
int AFini[3];

int Mode = 0;

int ActivMain = 0;
LiquidCrystal_I2C lod(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

int In = A10;
int Compteur = 20;
int Somme = 0;
int Moy = 0;
int k = 0;
int Val = 0;

int Message = 0;
float Tic = 0;
float Toc = 0;

int Demo = 0;


//--------------ODrive object------------------------

HardwareSerial& odrive1_serial = Serial2;
HardwareSerial& odrive2_serial = Serial1;
ODriveArduino odrive1(odrive1_serial);
ODriveArduino odrive2(odrive2_serial);





void setup() {
  // put your setup code here, to run once:
  digitalWrite(LedG, HIGH);
  digitalWrite(LedB, HIGH);
  digitalWrite(LedR, HIGH);
  digitalWrite(LedY, HIGH);
  digitalWrite(LedW, HIGH);

  pinMode(Sig1, INPUT_PULLUP);
  pinMode(Sig2, INPUT_PULLUP);
  pinMode(Sig3, INPUT_PULLUP);
  pinMode(Sig4, INPUT_PULLUP);
  pinMode(Sig5, INPUT_PULLUP);
  pinMode(Sig6, INPUT_PULLUP);
  pinMode(SwitchOn, INPUT_PULLUP);
  pinMode(LedR, OUTPUT);
  pinMode(LedB, OUTPUT);
  pinMode(LedG, OUTPUT);
  pinMode(LedY, OUTPUT);
  pinMode(LedW, OUTPUT);
  pinMode(ComRotMain, OUTPUT);
  odrive1_serial.begin(115200);
  odrive2_serial.begin(115200);
  Serial.begin(115200);




  lod.begin(20, 4);
  lod.clear();
  lod.print("Initialisation");
  lod.setCursor(0, 1);
  lod.print("des moteurs");
  while (!Serial);    //Pour attendre que le Serial ouvre

  //----------Variable a reset lors de la mise en tention----------------------------
  for (int i = 0; i  <= 2; i++) {
    PAb[i] = PRepos[i];    //Position de l'avant bras
    PAbF[i] = PRepos[i];   //Position de l'avant bras desire
    PAbFT[i] = PRepos[i];  // PAbF Temporelle
  }
  PDAbX = 0; //Deplacement demande de l'avant bras en X
  PDAbY = 0; //Deplacement demande de l'avant bras en Y
  PDAbZ = 0; //Deplacement demande de l'avant bras en Z

  //----------Set Up des Moteurs----------------------------

  Serial.println("s1");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  int requested_state;

  // Attente de la connexion des moteurs
  odrive1_serial.write("r axis0.motor.current_control.Iq_measured\n");
  Amp0 = odrive1.readFloat();
  odrive1_serial.write("r axis1.motor.current_control.Iq_measured\n");
  Amp1 = odrive1.readFloat();
  if ((Amp0 == 0) && (Amp1 == 0)) {
    Serial.println("Attente de connexion du moteur");
    odrive1.run_state(1, AXIS_STATE_ENCODER_INDEX_SEARCH, true);
    setup();
  }




  Serial.println("Moteur connecté, recherche des index");




  for (float i = 0; i <= 2; i++) {
    digitalWrite(LedW, LOW);
    //Index est il deja fait?
    if (i == 1) {
      odrive1_serial.write("r axis0.encoder.index_found\n");
      Index = odrive1.readFloat();
    }
    if (i == 0) {
      odrive1_serial.write("r axis1.encoder.index_found\n");
      Index = odrive1.readFloat();
    }
    if (i == 2) {
      odrive2_serial.write("r axis1.encoder.index_found\n");
      Index = odrive2.readFloat();
    }
    //Recherche de l'index si necessaire
    if (!Index) {
      requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
      if (i == 1) {
        odrive1.run_state(0, requested_state, true);
      }
      else if (i == 0) {
        odrive1.run_state(1, requested_state, true);
      }
      else if (i == 2) {
        odrive2.run_state(1, requested_state, true);
      }
    }
    //Met en close loop
    requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    if (i == 1) {
      odrive1.run_state(0, requested_state, false);
    }
    else if (i == 0) {
      odrive1.run_state(1, requested_state, false);
    }
    else if (i == 2) {
      odrive2.run_state(1, requested_state, false);
    }

    // Correction du decallage
    if (i == 1) {
      odrive1.SetPosition(0, -1 * DecaRot0 / 360 * 6);
    }
    else if (i == 0) {
      odrive1.SetPosition(1, DecaRot1 / 360 * 6);
    }
    else if (i == 2) {
      odrive2.SetPosition(1, DecaRot2 / 360 * 6);
    }
    delay(500);
  }

  Serial.println("Index trouvé et moteurs en position");


  //Mise du bras aux angles 0



}
void loop() {


  //----------Cas en fonction des input-------------------------------------------------------------------------------------------------

  Start = 0;
  Continue = 0;
  ActivMain = 0;
  //  Usb.Task();
  //  MANETTE();
  MANETTE2();
  if (Etat == 99) { //(Xbox.getButtonClick(START)) {
    MENU_MODE();
    MANETTE2();
  }

  switch (Mode) {
    //Mode normal
    case 1:
      if (Etat == 1) {
        Start = 1;
        PAbF[0] = PBouche[0];
        PAbF[1] = PBouche[1];
        PAbF[2] = PBouche[2];
        COMMANDES_MOTEURS();
        delay(5000);
        PAbF[0] = PRepos[0];
        PAbF[1] = PRepos[1];
        PAbF[2] = PRepos[2];
      }
      break;

    //Mode prototypage
    case 2:
      if (Etat == 1) {
        Start = 1;
        PAbF[0] = PBouche[0];
        PAbF[1] = PBouche[1];
        PAbF[2] = PBouche[2];
      }
      else if (Etat == 2) {
        Start = 1;
        PAbF[0] = PRepos[0];
        PAbF[1] = PRepos[1];
        PAbF[2] = PRepos[2];
      }
      //Deplacement avant arriere (X)
      else if (Etat == 11) {
        Start = 1;
        PAbF[0] = PAbF[0] + DManuel;
      }
      else if (Etat == 12) {
        Start = 1;
        PAbF[0] = PAbF[0] - DManuel;
      }
      //Deplacement Droite gauche (Y)
      else if (Etat == 13) {
        Start = 1;
        PAbF[1] = PAbF[1] + DManuel;
      }
      else if (Etat == 14) {
        Start = 1;
        PAbF[1] = PAbF[1] - DManuel;
      }
      //D/placement up down (Z)
      else if (Etat == 4) {
        Start = 1;
        PAbF[2] = PAbF[2] + DManuel;
      }
      else if (Etat == 3) {
        Start = 1;
        PAbF[2] = PAbF[2] - DManuel;
      }

      break;
    //      Demo
    case 3:
      Start = 1;
      if (Demo == 0) {
        MANETTE2();
        if (Etat == 99) { //(Xbox.getButtonClick(START)) {
          MENU_MODE();
          MANETTE2();
        }
        Demo = Demo + 1;
        delay(5000);
        PAbF[0] = PBouche[0];
        PAbF[1] = PBouche[1];
        PAbF[2] = PBouche[2];
      }
      else if (Demo == 1) {
        MANETTE2();
        if (Etat == 99) { //(Xbox.getButtonClick(START)) {
          MENU_MODE();
          MANETTE2();
        }
        Demo = Demo + 1;
        delay(5000);
        PAbF[0] = PRepos[0];
        PAbF[1] = PRepos[1];
        PAbF[2] = PRepos[2];
      }
      else if (Demo == 2) {
        MANETTE2();
        if (Etat == 99) { //(Xbox.getButtonClick(START)) {
          MENU_MODE();
          MANETTE2();
        }
        Demo = Demo + 1;
        delay(5000);
        PAbF[0] = PInit[0];
        PAbF[1] = PInit[1];
        PAbF[2] = PInit[2];
      }
      else if (Demo >= 3) {
        MANETTE();
        Demo = 0;
        delay(5000);
        PAbF[0] = PFunny[0];
        PAbF[1] = PFunny[1];
        PAbF[2] = PFunny[2];
      }
      break;
  }


  COMMANDES_MOTEURS();
  ROTATION_MAIN();
  IMPRESSION();
  ECRAN();
  VERIF_ON();
}
//----------------FIN DU CODE PRINCIPALE-------------------------------------------------------------------------------------------------
//----------------LA SUITE SONT DES FONCTIONS UTILISEES----------------------------------------------------------------------------------------



//-----------Envoit de la commande au moteurs---------------------------------------------------------------------------------------
void COMMANDES_MOTEURS() {
  //Detection position pour calcul
  POSITION();

  ERREUR = 0;

  //Detection du changement de commande
  if ((abs(PAbF[0] - PAb[0]) < Precision ) && (abs(PAbF[1] - PAb[1]) < Precision) && (abs(PAbF[2] - PAb[2]) < Precision)) { //Si la position du bout du bras respecte les tolerances, le bras ne cherchera pas a bouger
    PAbFT[0] = PAbF[0];
    PAbFT[1] = PAbF[1];
    PAbFT[2] = PAbF[2];
  }
  else if (Start == 1) { //Si le bras ne respecte pas les tolerances, il cherchera a s'ajuster
    //Calcul du nombre d'iteration necessaire pour parcourir la distance desire
    Distance = sqrt(pow(PAbF[0], 2) + pow(PAbF[1], 2) + pow(PAbF[2], 2));
    NbI = Distance / Speed2 + 1;
    Serial.println("lets gooo");

    //Separation du trajet a effectuer en plusieurs petits trajet. Cela permet de garder un parcours lineaire
    double PasX = (PAbF[0] - PAb[0]) / NbI; //Step du parcous lineaire
    double PasY = (PAbF[1] - PAb[1]) / NbI;
    double PasZ = (PAbF[2] - PAb[2]) / NbI;

    double PAbIX = PAb[0];
    double PAbIY = PAb[1];
    double PAbIZ = PAb[2];

    float Theta0[NbI]; //Angle moteur Base
    float Theta1[NbI]; //Angle moteur Up
    float Theta2[NbI]; //Angle moteur Coude


    //Boucle qui va calculer les deplacements des moteurs pas par pas jusqu'a ce qu'il atteigne la position desire
    for (int i = 0; i < NbI; i++) {

      // Valeur que le bout du bras doit se rendre pour l'increment ou on est rendu
      PDAbX = PAbIX + PasX * (i + 1);
      PDAbY = PAbIY + PasY * (i + 1);
      PDAbZ = PAbIZ + PasZ * (i + 1);

      //----------Valeur de la position desire par rapport a l'origine-------------------
      double DDAbH = sqrt(pow(PDAbX, 2) + pow(PDAbY, 2)); //Distance sur le plan horizontal entre la base du système et le bout du bras
      double ThetaDAbH = atan(PDAbY / PDAbX); //Angle lors du deplacement de l'avant bras sur le plan horizontale
      double ThetaDAbV = asin(PDAbZ / Lo1);    //Angle vertical lors du deplacement de l'avant bras

      //----------Valeur des differents joints------------------------------------------
      //Angle des joint
      Theta1[i] = ThetaDAbV;
      LHautReel = Lo1 * cos(Theta1[i]) + Ra1 + Ra2;
      Theta0[i] = ThetaDAbH - acos((pow(Lo2, 2) - pow(DDAbH, 2) - pow(LHautReel, 2)) / (-2 * DDAbH * LHautReel)); //Joint de la base
      Theta2[i] = 3.1416 - acos((pow(DDAbH, 2) - pow(Lo2, 2) - pow(LHautReel, 2)) / (-2 * LHautReel * Lo2)); //Joint du coude (PEut etre garder pour mettre par rapport a la tige -Theta0)

      //Si une position ne peut pas etre effectue on empeche le mouvement--------------
      if (isnan(Theta0[i]) || isnan(Theta1[i]) || isnan(Theta2[i]) ||
          (Theta0[i] / (2 * 3.1416) * 360 > Lim0G) || (Theta0[i] / (2 * 3.1416) * 360 < Lim0D) || (Theta1[i] / (2 * 3.1416) * 360 > Lim1H) || // Ensemble des depassement des limites du syteme
          (Theta1[i] / (2 * 3.1416) * 360 < Lim1B) || (Theta2[i] / (2 * 3.1416) * 360 > Lim2G) || (Theta2[i] / (2 * 3.1416) * 360 < Lim2D) ) {

        ERREUR = 1;
        lod.clear();
        lod.setCursor(5, 1);
        lod.print("Commande");
        lod.setCursor(4, 2);
        lod.print("impossible");
        PAbF[0] = PAbFT[0] ;
        PAbF[1] = PAbFT[1] ;
        PAbF[2] = PAbFT[2];
        Message = 1;
        break;
      }
    }

    //-----------Envoie des commandes aux moteurs-------------------------------------
    for (int i = 0; i < NbI; i++) {
      if (ERREUR == 1) {
        break;
      }

      ROTATION_MAIN();


      //      int Debut = millis();
      odrive1.SetPosition(1, (Theta1[i] * 6 / (2 * 3.1416)) + (DecaRot1 * 6 / 360));
      odrive1.SetPosition(0, (( Theta0[i] * 6 / (2 * 3.1416))  + (DecaRot0 * 6 / 360)) * -1); // Multiplie par -1 car le sens du moteur est inverse
      odrive2.SetPosition(1, ((Theta2[i] * 6 / (2 * 3.1416) - ((DecaRot2) * 6 / 360)) * -1)); //Vérifier l'identifiant du moteur (2)

      Angle[0] = Theta0[i];
      Angle[1] = Theta1[i];
      Angle[2] = Theta2[i];

      //      if (i > 1) {
      //        Condition = 0;
      //
      //        while (Condition == 0) {
      //          AReal[0] = odrive1.GetPosition(0);
      //          AReal[1] = odrive1.GetPosition(1);
      //          AReal[2] = odrive2.GetPosition(1);
      //
      //          AFini[0] = 1 - abs((AReal[0] - Angle[0]) / (Angle[0] - Theta0[i - 1]));
      //          AFini[1] = 1 - abs((AReal[1] - Angle[1]) / (Angle[1] - Theta1[i - 1]));
      //          AFini[2] = 1 - abs((AReal[2] - Angle[2]) / (Angle[2] - Theta2[i - 1]));
      //          if (((AFini[0] + AFini[1] + AFini[2]) / 3) >= 0.75) {
      //            Condition = 1;
      //          }
      //        }
      //        //      int Fin = millis();
      //        //      int Delai = Fin - Debut;
      //
      //        //      if (Delai < Speed && i != NbI - 1) {
      //        //        delay(Speed - Delai);
      //        //      }
      //        IMPRESSION();
      //      }
      IMPRESSION();
      PAbFT[0] = PAb[0];
      PAbFT[1] = PAb[1];
      PAbFT[2] = PAb[2];

      RManuel0 = Angle[0];
      RManuel1 = Angle[1];

    }
  }
  Etat = 0;
}
//-------------------Détection des positions du bras----------------------------------------------------------------------------------
void POSITION() {
  //Position réel des differents joints
  PM2[0] = cos((odrive1.GetPosition(0) / 6 * (2 * 3.1416) + (DecaRot0 / 360 * (2 * 3.1416))) * -1) * ((cos(odrive1.GetPosition(1) / 6 * (2 * 3.1416) - (DecaRot1 / 360 * (2 * 3.1416))) * Lo1) + Ra1 + Ra2); //Position coude
  PM2[1] = sin((odrive1.GetPosition(0) / 6 * (2 * 3.1416) + (DecaRot0 / 360 * (2 * 3.1416))) * -1) * ((cos(odrive1.GetPosition(1) / 6 * (2 * 3.1416) - (DecaRot1 / 360 * (2 * 3.1416))) * Lo1) + Ra1 + Ra2); //Position coude
  PM2[2] = sin(odrive1.GetPosition(1) / 6 * (2 * 3.1416) - (DecaRot1 / 360 * (2 * 3.1416))) * Lo1;       //Position coude
  PAb[0] = cos((odrive2.GetPosition(1) / 6 * (2 * 3.1416) - (DecaRot2 / 360 * (2 * 3.1416))) * -1 + ((odrive1.GetPosition(0) / 6 * (2 * 3.1416) + (DecaRot0 / 360 * (2 * 3.1416))) * -1)) * Lo2 + PM2[0]; //Position bout
  PAb[1] = sin((odrive2.GetPosition(1) / 6 * (2 * 3.1416) - (DecaRot2 / 360 * (2 * 3.1416))) * -1 + ((odrive1.GetPosition(0) / 6 * (2 * 3.1416) + (DecaRot0 / 360 * (2 * 3.1416))) * -1)) * Lo2 + PM2[1];
  PAb[2] = PM2[2];
}

//-------------------Commande pour rotation de la main--------------------------------------------------------------------------
void ROTATION_MAIN() {
  if (ActivMain == 1) {
    if (abs(digitalRead(Sig2) - 1) == 1) {
      analogWrite(ComRotMain, 185);
    }
    else if ((abs(digitalRead(Sig3) - 1) == 1)) {
      analogWrite(ComRotMain, 35);
    }
    else if ((abs(digitalRead(Sig6) - 1) == 1)) {
      analogWrite(ComRotMain, 105);
    }
  }
  //  int Rot_Main = pulseIn(ValRotMain, HIGH) / 80;

}

//-----------Verifie si les moteurs sont allume---------------------------------------------------------------------------------------
void VERIF_ON() {
  // Restart du systeme si l'appareil a ete mis a l'arret
  odrive1_serial.write("r axis0.motor.current_control.Iq_measured\n");
  Amp0 = odrive1.readFloat();
  odrive1_serial.write("r axis1.motor.current_control.Iq_measured\n");
  Amp1 = odrive1.readFloat();
  if ( (Amp0 == 0) && (Amp1 == 0)) { //(!digitalRead(SwitchOn)) &&
    Serial.println("Les moteurs sont eteints");
    lod.clear();
    lod.print("Moteurs MIA");
    lod.setCursor(0, 1);
    lod.print("Connexion...");
    Index = 0;
    while (!Index) {
      odrive1.run_state(0, AXIS_STATE_ENCODER_INDEX_SEARCH, true);
      odrive1_serial.write("r axis0.encoder.index_found\n");
      Index = odrive1.readFloat();
    }
    setup();
  }
}


//-------------------Fonction menu Mode----------------------------------------------------------------------------------
void MENU_MODE() {
  lod.clear();
  lod.setCursor(2, 0);
  lod.print("Choix du mode:");
  lod.setCursor(0, 1);
  lod.print("A: Normal");
  lod.setCursor(0, 2);
  lod.print("B: Prototypage");
  lod.setCursor(0, 3);
  lod.print("X: Demo");
  Etat = 0;
  double Temps = millis();
  double Compteur = 0;
  while ((Etat != 99) || (Etat != 1) || (Etat != 2) || (Etat != 3)) {
    MANETTE2();
    if (Etat == 1) {
      break;
    }
    if (Etat == 2) {
      break;
    }
    if (Etat == 3) {
      break;
    }
    if (((millis() - Temps) >= 15000) || (Etat == 198)) {
      lod.clear();
      lod.setCursor(7, 0);
      lod.print("Aucun");
      lod.setCursor(4, 1);
      lod.print("mode choisi");
      Serial.print("why");
      delay(5000);
      break;
    }
  }
  lod.clear();
  lod.setCursor(4, 0);
  lod.print("Mode choisi:");
  Serial.print("in");
  if (Etat == 1) {
    Mode = 1;
    lod.setCursor(0, 1);
    lod.print("A: Normal");
  }
  else if (Etat == 2) {
    Mode = 2;
    lod.setCursor(0, 2);
    lod.print("B: Prototypage");
  }
  else if (Etat == 3) {
    Mode = 3;
    lod.setCursor(0, 3);
    lod.print("X: Demo");
  }
  Serial.print("out");
}



//-----------Fonction pour afficher les valeurs---------------------------------------------------------------------------------------
void IMPRESSION() {
  POSITION();
  //Print des angles des moteurs
  Serial.print("Angle: ");
  Serial.print((odrive1.GetPosition(0) / 6 * 360  + (DecaRot0)) * -1);
  Serial.print(",");
  Serial.print(odrive1.GetPosition(1) / 6 * 360 - (DecaRot1));
  Serial.print(",");
  Serial.print((odrive2.GetPosition(1) / 6 * 360  - (DecaRot2)) * -1);
  Serial.print(",");
  //Print des commandes des angles des moteurs
  Serial.print(" Com Theta: ");
  Serial.print(Angle[0] / (2 * 3.1416) * 360);
  Serial.print(",");
  Serial.print(Angle[1] / (2 * 3.1416) * 360);
  Serial.print(",");
  Serial.print(Angle[2] / (2 * 3.1416) * 360);
  Serial.print(",");
  // Position du coude
  Serial.print(" PCoude: ");
  Serial.print(PM2[0]);
  Serial.print(",");
  Serial.print(PM2[1]);
  Serial.print(",");
  Serial.print(PM2[2]);
  Serial.print(",");
  //Print des Positions de l'avant bras
  Serial.print(" PAb: ");
  Serial.print(PAb[0]);
  Serial.print(",");
  Serial.print(PAb[1]);
  Serial.print(",");
  Serial.print(PAb[2]);
  Serial.print(",");
  //Print de la commande a l'avant bras
  Serial.print(" Com: ");
  Serial.print(PAbF[0]);
  Serial.print(",");
  Serial.print(PAbF[1]);
  Serial.print(",");
  Serial.print(PAbF[2]);
  Serial.print(",");

  //Print de l<amperage des moteurs
  AMPERAGE();
  Serial.print(" Amp: ");
  Serial.print(Amp0);
  Serial.print (',');
  Serial.print(Amp1);
  Serial.print (',');
  Serial.println(Amp2);

}

void AMPERAGE() {
  //Lecture des données
  odrive1_serial.write("r axis0.motor.current_control.Iq_measured\n");
  Amp0 = (odrive1.readFloat());
  odrive1_serial.write("r axis1.motor.current_control.Iq_measured\n");
  Amp1 = (odrive1.readFloat());
  odrive2_serial.write("r axis1.motor.current_control.Iq_measured\n");
  Amp2 = (odrive2.readFloat());
}
//--------------------Print Écran-----------------------------
void ECRAN() {
  Tic = millis();
  if (Message == 1) {
    Toc = Tic + 2000;
    Message = 0;
  }
  if (Toc > Tic) {
  }
  else {
    Toc = Tic + 1000;
    lod.clear();
    //Premiere rangee
    lod.setCursor(0, 0);
    lod.print("Mode: ");
    if (Mode == 1) {
      lod.print("Normal");
    }
    else if (Mode == 2) {
      lod.print("Prototypage");
    }
    else if (Mode == 3) {
      lod.print("Demonstration");
    }
    else {
      lod.print("Non choisi");
    }

    //Deuxieme range (Nom moteur)
    lod.setCursor(0, 1);
    lod.print("Mote: ");

    lod.setCursor(7, 1);
    lod.print("M0");
    lod.setCursor(12, 1);
    lod.print("M1");
    lod.setCursor(17, 1);
    lod.print("M2");

    //Troisieme range (Temperature moteur)
    lod.setCursor(0, 2);
    lod.print("Temp: ");

    lod.setCursor(7, 2);
    lod.print(T0, 0);
    lod.setCursor(12, 2);
    lod.print(T1, 0);
    lod.setCursor(17, 2);
    lod.print(T2, 0);

    //4ieme range (Amperage moteur)
    lod.setCursor(0, 3);
    lod.print("Amps: ");
    //  AMPERAGE();
    if ( Amp1 >= 0) {
      lod.setCursor(7, 3);
    }
    else {
      lod.setCursor(6, 3);
    }
    lod.print(Amp0, 1);


    if ( Amp1 >= 0) {
      lod.setCursor(12, 3);
    }
    else {
      lod.setCursor(11, 3);
    }
    lod.print(Amp1, 1);


    if ( Amp2 >= 0) {
      lod.setCursor(17, 3);
    }
    else {
      lod.setCursor(16, 3);
    }
    lod.print(Amp2, 1);
  }
}
//-----------Fonction pour utiliser manette xbox one---------------------------------------------------------------------------------------
void MANETTE2() {
  for (int k = 0; k <= Compteur; k++) {
    Val = (pulseIn(In, HIGH));
    if (k < Compteur) {
      Somme = Somme + Val + 4;
      k = k + 1;
      //      Serial.print(pulseIn(In, HIGH) + 8);
      //      Serial.print(" ");
    }
    else {
      //      Serial.println(" ");
      Etat = (Somme / (16 * Compteur));
      Serial.println(Etat);
      Somme = 0;
      k = 0;
    }
  }
}
void MANETTE() {
  Usb.Task();
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

    //    if (Xbox.getButtonPress(L2) > 0 || Xbox.getButtonPress(R2) > 0) {
    //      if (Xbox.getButtonPress(L2) > 0) {
    //        Serial.print(F("L2: "));
    //        Serial.print(Xbox.getButtonPress(L2));
    //        Serial.print("\t");
    //      }
    //      if (Xbox.getButtonPress(R2) > 0) {
    //        Serial.print(F("R2: "));
    //        Serial.print(Xbox.getButtonPress(R2));
    //        Serial.print("\t");
    //      }
    //      Serial.println();
    //    }

    // Set rumble effect
    //    static uint16_t oldL2Value, oldR2Value;
    //    if (Xbox.getButtonPress(L2) != oldL2Value || Xbox.getButtonPress(R2) != oldR2Value) {
    //      oldL2Value = Xbox.getButtonPress(L2);
    //      oldR2Value = Xbox.getButtonPress(R2);
    //      uint8_t leftRumble = map(oldL2Value, 0, 1023, 0, 255); // Map the trigger values into a byte
    //      uint8_t rightRumble = map(oldR2Value, 0, 1023, 0, 255);
    //      if (leftRumble > 0 || rightRumble > 0)
    //        Xbox.setRumbleOn(leftRumble, rightRumble, leftRumble, rightRumble);
    //      else
    //        Xbox.setRumbleOff();
  }

  if (Xbox.getButtonClick(UP)) {
    Serial.println(F("Up"));
    Etat = 11;
  }
  if (Xbox.getButtonClick(DOWN)) {
    Serial.println(F("Down"));
    Etat = 12;
  }
  if (Xbox.getButtonClick(LEFT)) {
    Serial.println(F("Left"));
    Etat = 13;
  }
  if (Xbox.getButtonClick(RIGHT)) {
    Serial.println(F("Right"));
    Etat = 14;
  }

  if (Xbox.getButtonClick(START)) {
    Serial.println(F("Start"));
    Etat = 100;
  }
  if (Xbox.getButtonClick(BACK)) {
    Serial.println(F("Back"));
    Etat = 200;
  }
  if (Xbox.getButtonClick(XBOX)) {
    Serial.println(F("Xbox"));
    Etat = 500;
  }
  if (Xbox.getButtonClick(SYNC)) {
    Serial.println(F("Sync"));
    Etat = 300;
  }

  if (Xbox.getButtonClick(L1)) {
    Serial.println(F("L1"));
    Etat = 21;
  }
  if (Xbox.getButtonClick(R1)) {
    Serial.println(F("R1"));
    Etat = 22;
  }
  if (Xbox.getButtonClick(L2)) {
    Serial.println(F("L2"));
    Etat = 23;
  }
  if (Xbox.getButtonClick(R2)) {
    Serial.println(F("R2"));
    Etat = 24;
  }
  if (Xbox.getButtonClick(L3)) {
    Serial.println(F("L3"));
    Etat = 25;
  }
  if (Xbox.getButtonClick(R3)) {
    Serial.println(F("R3"));
    Etat = 26;
  }


  if (Xbox.getButtonClick(A)) {
    Serial.println(F("A"));
    Etat = 1;
  }
  if (Xbox.getButtonClick(B)) {
    Serial.println(F("B"));
    Etat = 2;
  }
  if (Xbox.getButtonClick(X)) {
    Serial.println(F("X"));
    Etat = 3;
  }
  if (Xbox.getButtonClick(Y)) {
    Serial.println(F("Y"));
    Etat = 4;
  }

  delay(1);
}
