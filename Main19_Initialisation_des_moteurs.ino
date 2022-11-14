
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
#include <PS2X_lib.h>  //for v1.6
PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

//int SetManette = 40;

//----------PINS------------------------------------
const int SigEMG = 2; //Signal provenant du Nano des EMG indicant si on a une activation ou non
const int CRM = 11;       //Commande de Rotation de la Main envoyé à l'autre ARduino
const int ValRotMain = A2;


//-------------VARIABLES UTILISATEUR-------------------

int Moteur_ON = 1; // Si on fait fonctionner les moteurs ou non

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
int Speed2 = 30; //Longueur max ue peut avoir une iteration de calcul (plus le chiffre est petit plus le deplacement sera rapide mais moins clean)

double DecaRot0 = 0;//-60;    //-30 Difference en degrés entre la position attendue et la position réel (positif si la valeur est à gauche ou en bas de ce u'on veut)
double DecaRot1 = -30;  //-33;    //Difference en degrés entre la position attendue et la position réel (positif si on veut monter le bras)
double DecaRot2 = 0;//20;    //Difference en degrés entre la position attendue et la position réel (positif si la valeur est à gauche ou en bas de ce u'on veut)

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
float Tic2 = 0;
float Toc2 = 0;

int Wait = 0;

int Demo = 0;

int VRM; //Valeur rotation main

int Pos = 0;


//--------------ODrive object------------------------

HardwareSerial& odrive1_serial = Serial2;
HardwareSerial& odrive2_serial = Serial1;
ODriveArduino odrive1(odrive1_serial);
ODriveArduino odrive2(odrive2_serial);





void setup() {
  // put your setup code here, to run once:
  pinMode(SigEMG, INPUT_PULLUP);
  pinMode(CRM, OUTPUT);
  odrive1_serial.begin(115200);
  odrive2_serial.begin(115200);
  Serial.begin(115200);




  lod.begin(20, 4);
  lod.clear();
  lod.setCursor(3, 0);
  lod.print("Initialisation");
  lod.setCursor(4, 1);
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
  //----------Set Up de la manette-------------------------
  error = ps2x.config_gamepad(22, 23, 24, 25, true, true);
  if (error == 0) {
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  // Serial.print(ps2x.Analog(1), HEX);
  type = ps2x.readType();
  switch (type)
  {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }

  //----------Set Up des Moteurs----------------------------

  Serial.println("s1");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  int requested_state;

  // Attente de la connexion des moteurs
  if (Moteur_ON == 1) {
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
      //Index est il deja fait?

      if (i == 0) {
        lod.setCursor(6, 2);
        lod.print("Moteur 0");

        odrive1_serial.write("r axis0.encoder.index_found\n");
        Index = odrive1.readFloat();

        if (!Index) {
          odrive1_serial.write("odrv0.axis0.error=0\n");
          requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
          odrive1.run_state(0, requested_state, true);
          delay(1000);
          odrive1_serial.write("odrv0.axis0.error=0\n");
          delay(50);
          requested_state = AXIS_STATE_HOMING;
          odrive1.run_state(0, requested_state, true);
        }
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        odrive1.run_state(0, requested_state, false);
        odrive1.SetPosition(0, -1 * DecaRot0 / 360 * 6);
      }
      if (i == 1) {
        lod.setCursor(6, 2);
        lod.print("Moteur 1");
        odrive1_serial.write("r axis1.encoder.index_found\n");
        Index = odrive1.readFloat();
        if (!Index) {
          requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
          odrive1.run_state(1, requested_state, true);
          //          delay(1000);
          //          odrive1_serial.write("odrv0.axis0.error=0\n");
          //          delay(50);
          //          requested_state = AXIS_STATE_HOMING;
          //          odrive1.run_state(0, requested_state, true);
        }
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        odrive1.run_state(1, requested_state, false);
        odrive1.SetPosition(1, DecaRot1 / 360 * 6);
      }
      if (i == 2) {
        lod.setCursor(6, 2);
        lod.print("Moteur 2");
        odrive2_serial.write("r axis0.encoder.index_found\n");
        Index = odrive2.readFloat();
        if (!Index) {
          requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
          odrive2.run_state(1, requested_state, true);
          //          delay(1000);
          //          odrive1_serial.write("odrv0.axis0.error=0\n");
          //          delay(50);
          //          requested_state = AXIS_STATE_HOMING;
          //          odrive1.run_state(0, requested_state, true);
        }
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        odrive2.run_state(1, requested_state, false);
        odrive2.SetPosition(1, DecaRot2 / 360 * 6);
      }
      
      delay(3000);
    }
    lod.clear();
    lod.setCursor(3, 1);
    lod.print("MOTEURS PRETS");
    delay(3000);
  }
  else {
    Message = 1;
    delay(5);
    lod.clear();
    lod.setCursor(2, 1);
    lod.print("MOTEURS ETEINTS");
  }



  //Mise du bras aux angles 0



}
void loop() {

lod.clear();
    lod.setCursor(3, 1);
    lod.print("GO");
    delay(3000);
  //----------Cas en fonction des input-------------------------------------------------------------------------------------------------

  Start = 0;
  Continue = 0;
  ActivMain = 0;
  VRM = 150;
  MANETTE();
  if (Etat == 100) {
    MENU_MODE();
  }

  switch (Mode) {
    //Mode normal
    case 1:
      Start = 1;
      if (Wait == 1) {
        Tic2 = millis();
        if (Tic2 >= (Toc2 + 5000)) {
          Wait = 0;
        }
      }
      if (((Etat == 1) || (digitalRead(SigEMG) == 1)) && (Pos == 0) && (Wait == 0)) {
        VRM = 10; //position a determiner
        PAbF[0] = PBouche[0];
        PAbF[1] = PBouche[1];
        PAbF[2] = PBouche[2];
        Wait = 1;
        Pos = 1;
        Toc2 = millis();
      }
      else if ((Pos == 1) && (Wait == 0)) {
        VRM = 20; //position a derterminer
        PAbF[0] = PRepos[0];
        PAbF[1] = PRepos[1];
        PAbF[2] = PRepos[2];
        Wait = 1;
        Pos = 0;
        Toc2 = millis();
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
      else if (Etat == 22) {
        Start = 1;
        PAbF[2] = PAbF[2] + DManuel;
      }
      else if (Etat == 24) {
        Start = 1;
        PAbF[2] = PAbF[2] - DManuel;
      }
      //---Rotation avant bras
      else if (Etat == 23) {  //Supination
        VRM = 50;
      }
      else if (Etat == 21) {  //Pronation
        VRM = 100;
      }


      break;
    //      Demo
    case 3:
      Start = 1;

      if (Wait == 1) {
        Tic2 = millis();
        if (Tic2 >= (Toc2 + 5000)) {
          Wait = 0;
        }
      }
      else if ((Demo == 0) && (Wait == 0)) {
        //        MANETTE();
        //
        //        if ((Etat == 100) { //(Xbox.getButtonClick(START)) {
        //        MENU_MODE();
        //          MANETTE();
        //        }
        Demo = Demo + 1;
        //               delay(5000);
        PAbF[0] = PBouche[0];
        PAbF[1] = PBouche[1];
        PAbF[2] = PBouche[2];
        Toc2 = millis();
        Wait = 1;
      }
      else if ((Demo == 1) && (Wait == 0)) {
        Demo = Demo + 1;
        PAbF[0] = PRepos[0];
        PAbF[1] = PRepos[1];
        PAbF[2] = PRepos[2];
        Toc2 = millis();
        Wait = 1;
      }
      else if ((Demo == 2) && (Wait == 0)) {
        Demo = Demo + 1;
        PAbF[0] = PInit[0];
        PAbF[1] = PInit[1];
        PAbF[2] = PInit[2];
        Toc2 = millis();
        Wait = 1;
      }
      else if ((Demo >= 3) && (Wait == 0)) {
        Demo = 0;
        PAbF[0] = PFunny[0];
        PAbF[1] = PFunny[1];
        PAbF[2] = PFunny[2];
        Toc2 = millis();
        Wait = 1;
      }
      break;
  }

  ROTATION_MAIN();
  COMMANDES_MOTEURS();
  IMPRESSION();
  ECRAN();
//  VERIF_ON();
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
  analogWrite(CRM, VRM);

}

//-----------Verifie si les moteurs sont allume---------------------------------------------------------------------------------------
void VERIF_ON() {
  if (Moteur_ON == 1) {
    // Restart du systeme si l'appareil a ete mis a l'arret
    odrive1_serial.write("r axis0.motor.current_control.Iq_measured\n");
    Amp0 = odrive1.readFloat();
    odrive1_serial.write("r axis1.motor.current_control.Iq_measured\n");
    Amp1 = odrive1.readFloat();
    if ( (Amp0 == 0) && (Amp1 == 0)) {
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
  while ((Etat != 100) || (Etat != 1) || (Etat != 2) || (Etat != 3)) {
    MANETTE();
    if (Etat == 1) {
      break;
    }
    if (Etat == 2) {
      break;
    }
    if (Etat == 3) {
      break;
    }
    if (((millis() - Temps) >= 15000) || (Etat == 4)) {
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
    Toc = Tic + 5000;
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

void MANETTE() {
  if (error != 0) {
    error = ps2x.config_gamepad(22, 23, 24, 25, true, true);
    if (error == 0) {
      Serial.print("Code 0  : ");
    }
    else if (error == 1)
      Serial.print("Code 1  : ");
    else if (error == 2)
      Serial.print("Code 2  : ");
    else if (error == 3)
      Serial.print("Code 3  : ");
  }

  else {
    ps2x.read_gamepad(false, vibrate);
    if (ps2x.Button(PSB_PAD_UP)) {
      Serial.println("Up");
      Etat = 11;
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.println("Down");
      Etat = 12;
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.println("Left");
      Etat = 13;
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.println("Right");
      Etat = 14;
    }

    if (ps2x.Button(PSB_START)) {
      Serial.println("Start");
      Etat = 100;
    }
    if (ps2x.Button(PSB_SELECT)) {
      Serial.println("Select");
      Etat = 200;
    }
    if (ps2x.Button(PSB_R1)) {
      Serial.println("R1");
      Etat = 21;
    }
    if (ps2x.Button(PSB_L1)) {
      Serial.println("L1");
      Etat = 22;
    }
    if (ps2x.Button(PSB_R2)) {
      Serial.println("R2");
      Etat = 23;
    }
    if (ps2x.Button(PSB_L2)) {
      Serial.println("L2");
      Etat = 24;
    }
    if (ps2x.Button(PSB_R3)) {
      Serial.println("R3");
      Etat = 25;
    }
    if (ps2x.Button(PSB_L3)) {
      Serial.println("L3");
      Etat = 26;
    }
    if (ps2x.Button(PSB_BLUE)) {
      Serial.println("Croix");
      Etat = 1;
    }
    if (ps2x.Button(PSB_RED)) {
      Serial.println("Cercle");
      Etat = 2;
    }
    if (ps2x.Button(PSB_PINK)) {
      Serial.println("Carre");
      Etat = 3;
    }
    if (ps2x.Button(PSB_GREEN)) {
      Serial.println("Triangle");
      Etat = 4;
    }
  }
}
