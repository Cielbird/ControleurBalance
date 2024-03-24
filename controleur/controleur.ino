#include "ArduPID.h"
#include "LiquidCrystal.h"

// 
#define UNO // pour tester sur un UNO
//#define MEGA // pour le vrai prototype PCB avec le MEGA

//Déclaration E/S
#define PIN_BOUTON A0
#define PIN_AMP A2
#ifdef UNO
  #define PIN_POSITION A1
  #define PIN_PWM 3
#endif
#ifdef MEGA
  #define PIN_POSITION A9
  #define PIN_PWM 12
#endif

//constantes pour les boutons utilisés
#define BTN_NONE 0
#define BTN_RIGHT 1
#define BTN_UP 2
#define BTN_DOWN 3
#define BTN_LEFT 4
#define BTN_SELECT 5

// envoyer stabilité
// 1: tare
// 2: echelonnage + données A et B


// characters speciaux
byte FLECHES[8] = {
  B00100,
  B01010,
  B10001,
  B00000,
  B10001,
  B01010,
  B00100,
};
byte SMILEY[8] = {
  B00000,
  B01010,
  B01010,
  B00000,
  B10001,
  B10001,
  B01110,
};


//Setup écran LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

unsigned long lcdTimer;
unsigned long lcdCooldown = 500;
unsigned long ampReadingTimer;
unsigned long ampReadCooldown = 50;
unsigned long buttonTimer;
unsigned long buttonCooldown = 50;
unsigned long controllerTimer;
unsigned long controllerCooldown = 20;
// voir constantes BTN_NONE, BTN_RIGHT...
byte prevSelectedButton;
// modes haut-niveau
enum Mode {Pesage, Comptage, Tarage, Etalonnage, Precision};
const byte NUM_MODES = 5;
Mode mode;
// menu: on peut démarer un etalonnage, étape 1: mesure à vide, étape 2: mesure de 20g
enum SousModeEtalonnage {Menu, Etape1, Etape2};
SousModeEtalonnage sousModeEtalonnage = Menu;

//unsigned int weights[] = {1, 2, 5, 10, 20, 50};//faire un étalonnage de 0 9
//byte selectedCalib;
// nb de chiffres apres la virgule
byte precisionOpts[] = {0, 1, 2, 3};
// index
byte selectedPrecision = 0;
//   coin counting mode
String coinOpts[] = {"5c", "10c", "25c", "1$", "2$"};
double coinMasses[] = {3.95, 1.75, 4.4, 6.27, 6.92};
byte selectedCoin = 0;
byte numCoinTypes = sizeof(coinOpts)/sizeof(coinOpts[0]);

//déclaration boucle de régulation (Présentement en position)
ArduPID myController;
double output; // PID sortie 0~255
double tensionPos; // PID entrée 0~5
double setpoint = 1.445; //tension apres ampli du capteur à 16.4 mm
double mass;
double taredMass;
double tare;
// min et max dans les dernières secondes pour déterminer la stabilité
double stabilityPeriod = 3000; // en [ms]
double stabilityRange = 0.5; // the max differnce betweeen setpoint and the current tensionPos value that is considered stable.
unsigned long lastStableValTime;
bool isStable;

double ampV1;
double ampV2;
double calibConstA = 30;
double calibConstB = -80;

// lectures d'entrée et moyennage pour la sortie de l'ampli de courant
const int numAmpReadings = 256;
double ampReadings[numAmpReadings]; // lectures pour la sortie de l'ampli
int ampReadIndex = 0;                   // indexe de la lecture actuelle
double ampReadingsTotal = 0;               // total des lectures dans le tableau


/*
  Retourne un byte pour le bouton selectionné. Voir les constantes BTN_RIGHT, BTN_LEFT...
*/
byte getSelectedButton(){
  // val is from 0 to 1023
  int val = analogRead(0);
  int selected;
  if(val < 66){
    selected = BTN_RIGHT; // 0: right
  }else if(val < 220){  
    selected = BTN_UP; // 132: up
  }else if(val < 395){
    selected = BTN_DOWN; // 309: down
  }else if(val < 602){
    selected = BTN_LEFT; // 481: left
  }else if(val < 963){  
    selected = BTN_SELECT; // 722: select
  }else{
    selected = BTN_NONE; // 1023: nothing
  }
  return selected;
}

/*
  Passe à travers les modes haut-niveau à la droite
*/
void cycleModeRight(){
  mode = (Mode)(mode + 1);
  if(mode == NUM_MODES)
    mode = 0;
}

/*
  Passe à travers les modes haut-niveau à la gauche
*/
void cycleModeLeft(){
  if(mode == 0)
    mode = NUM_MODES;
  mode = (Mode)(mode - 1);
}

/*
  Gère la selection droite-gauche de mode.
*/
void handleInputMenuSelect(byte button) {
  switch(button){
    case BTN_RIGHT:
      cycleModeRight();
      break;
    case BTN_LEFT:
      cycleModeLeft();
      break;
  }
}

/*
  Gère toutes les entrées pour le mode tare
*/
void handleInputTarage(byte button){
  handleInputMenuSelect(button);
  switch(button){
    case BTN_SELECT:
      tareRoutine();
  }
}

/*
  Gère toutes les entrées pour le mode étalonnage
*/
void handleInputEtalonnage(byte button){
  switch(sousModeEtalonnage)
  {
    case Menu:
      handleInputMenuSelect(button);
      switch(button){
        case BTN_SELECT:
          sousModeEtalonnage = Etape1;
      }
      break;
    case Etape1:
      if(button == BTN_SELECT && true){
        ampV1 = readAmpVoltage();
        sousModeEtalonnage = Etape2;
      }
      break;
    case Etape2:
      if(button == BTN_SELECT && true){
        ampV2 = readAmpVoltage();
        calibConstA = 50.0 / (ampV2-ampV1);
        calibConstB = -ampV1 * calibConstA;
        sousModeEtalonnage = Menu;
      }
      break;    
  }
}

/*
  Gère toutes les entrées pour le mode precision
*/
void handleInputPrecision(byte button){
  handleInputMenuSelect(button);
  switch(button){
    case BTN_UP:
      if (selectedPrecision == sizeof(precisionOpts) - 1)
        selectedPrecision = 0;
      else
        selectedPrecision++;
      break;
    case BTN_DOWN:
      if (selectedPrecision == 0)
        selectedPrecision = sizeof(precisionOpts) - 1;
      else
        selectedPrecision--;
  }
}

/*
  Gère toutes les entrées pour le mode comptage de pieces
*/
void handleInputComptage(byte button){
  handleInputMenuSelect(button);
  switch(button){
    case BTN_UP:
      if (selectedCoin == numCoinTypes - 1)
        selectedCoin = 0;
      else
        selectedCoin++;
      break;
    case BTN_DOWN:
      if (selectedCoin == 0)
        selectedCoin = numCoinTypes - 1;
      else
        selectedCoin--;
  }
}

/*
  Gère toutes les entrées
*/
void handleInput(byte button){
  switch(mode){
    case Pesage:
      handleInputMenuSelect(button);
      break;
    case Tarage:
      handleInputTarage(button);
      break;
    case Etalonnage:
      handleInputEtalonnage(button);
      break;
    case Precision:
      handleInputPrecision(button);
      break;
    case Comptage:
      handleInputComptage(button);
      break;
  }
}

/*
  Reçoit et traite les données d'entrée, gère le debouncing.
*/
void updateInput(){
  if(millis() - buttonTimer > buttonCooldown){
    int s = getSelectedButton();
    if(prevSelectedButton != s)
    {
      if(s != BTN_NONE)
      {
        buttonTimer = millis();
        handleInput(s);
      }
      prevSelectedButton = s;
    }
  }
}

/*
  Tare la balance
*/
void tareRoutine(){
  tare = mass;
}

/*
  Imprime un titre de menu sur le LCD : "<>monTitre" en haut à gauche du LCD
*/
void lcdPrintTitle(String title){
  lcd.setCursor(0, 0);
  lcd.print("<>");
  lcd.print(title);
}

/*
  Imprime un smiley en haut à droite du LCD
*/
void lcdPrintStability(){
  lcd.setCursor(15, 0);
  if(isStable) lcd.write(byte(1)); // smiley
}

/*
  Imprime un OK en bas à gauche du LCD
*/
void lcdPrintOk(){
  lcd.setCursor(0, 1);
  lcd.print("OK");
}

/*
  Imprime un OK en bas à gauche du LCD si la balance est stable, ... sinon 
*/
void lcdPrintOkIfStable(){
  lcd.setCursor(0, 1);
  if(isStable)
    lcd.print("OK");
  else
    lcd.print("...");
}

/*
  Imprime la masse calculée en bas à droite du LCD
*/
void lcdPrintMass(){
  byte precision = precisionOpts[selectedPrecision];

  // convertir double en str
  byte decimalPosInLcd;
  if(precision==0) // no space for dot
    decimalPosInLcd = 15;
  else
    decimalPosInLcd = 14 - precision;
  char massStr[8]; // taille max de la zone qu'on accorde au poids
  dtostrf(taredMass, 7, precision, massStr); // Format the double to string with 7 total characters and 2 decimal places
  byte decimalPosInStr = 0;
  for(int i=0; massStr[i]!='.'&&massStr[i]!='\0'; i++)
  {
    decimalPosInStr++;
  }
  byte startPos = decimalPosInLcd - decimalPosInStr;
  lcd.setCursor(startPos, 1);
  lcd.print(massStr);
  lcd.setCursor(15, 1);
  lcd.print("g");
}

String getClosestCoin(double mass)
{
  int closestCoinType;
  double massDiff;
  double smallestMassDiff;
  for(int i = 0; i<numCoinTypes; i++)
  {
    massDiff = abs(coinMasses[i] - mass);
    if(i==0 || massDiff < smallestMassDiff)
    {
      closestCoinType = i;
      smallestMassDiff = massDiff;
    }
  }
  if(smallestMassDiff < 0.5){
    return coinOpts[closestCoinType];
  }
  return "...";
}

/*
  Met à jour l'LCD pour le mode pesage
*/
void updateLcdPesage(){
  lcdPrintTitle("Peser");
  lcdPrintStability();
  lcd.setCursor(0, 1);
  lcd.print(getClosestCoin(mass));
  lcdPrintMass();
}

/*
  Met à jour l'LCD pour le mode tarage
*/
void updateLcdTarage(){
  lcdPrintTitle("Tarer");
  lcdPrintOk();
  lcdPrintStability();
  lcdPrintMass();
}

/*
  Met à jour l'LCD pour le mode étalonnage
*/
void updateLcdEtalonnage(){
  switch (sousModeEtalonnage){
    case Menu:
      lcdPrintTitle("Etalonnage");
      lcdPrintOk();
      break;
    case Etape1:
      lcd.setCursor(0, 0);
      lcd.print("1)Aucun poids");
      lcdPrintOkIfStable();
      break;
    case Etape2:
      lcd.setCursor(0, 0);
      lcd.print("2)Avec 20g");
      lcdPrintOkIfStable();
      break;
  }
}

/*
  Met à jour l'LCD pour le mode précision
*/
void updateLcdPrecision(){
  lcdPrintTitle("Precision");
  lcd.setCursor(0, 1);
  lcd.write(byte(0)); // up down arrows
  lcd.setCursor(1, 1);
  lcd.print(precisionOpts[selectedPrecision]);
  lcd.print(" ch");
  lcdPrintMass();
}

/*
  Met à jour l'LCD pour le mode de comptage
*/
void updateLcdComptage(){
  lcdPrintTitle("Comptage");
  lcd.setCursor(0, 1);
  lcd.write(byte(0)); // up down arrows
  lcd.setCursor(1, 1);
  lcd.print(coinOpts[selectedCoin]);
  int num = mass / coinMasses[selectedCoin];
  lcd.setCursor(14, 1);
  lcd.print(num);
}

/*
  Met à jour l'LCD
*/
void updateLCD(){
  String lcdTopText;
  if(millis() - lcdTimer > lcdCooldown){
    lcdTimer = millis();
    lcd.clear();
    switch(mode){
      case Pesage:
        updateLcdPesage();
        break;
      case Tarage:
        updateLcdTarage();
        break;
      case Etalonnage:
        updateLcdEtalonnage();
        break;
      case Precision:
        updateLcdPrecision();
        break;
      case Comptage:
        updateLcdComptage();
    }
  }
}

/*
  Convertit l'entrée 10 bits analogue en tension
*/
double analogInToVoltage(int in){
  // convert arduino input to ampli output
  return in * (5.0 / 1023.0);
}

/*
  Evalue la stabilité du system, met à jour la variable isStable
*/
void updateStability()
{
  if (abs(tensionPos - setpoint) > stabilityRange)
  {
    lastStableValTime = 0;
    isStable = false;
  }
  else
  { // value in range of stability
    if (lastStableValTime == 0)
      lastStableValTime = millis();
    else if(millis() - lastStableValTime > stabilityPeriod)
      isStable = true;
  } 
}

/*
  Lit la valeur analogue du capteur, effectue le calcul du PID, et met à jour la sortie.
  Met la variable de stabilité isStable à jour.
*/
void updateController()
{
  if(millis() - controllerTimer > controllerCooldown)
  {
    controllerTimer = millis();
    int analogIn = analogRead(PIN_POSITION);
    tensionPos = analogInToVoltage(analogIn);
    myController.compute();
    analogWrite(PIN_PWM, output); //Output for Vamp_in

    // check stability
    updateStability();
    //Pour afficher les valeur de PID, et input, output, seulement enlever les commentaire.
    /*myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                                PRINT_OUTPUT   | // in the Serial plotter
                                                PRINT_SETPOINT |
                                                PRINT_BIAS     );*/
    //myController.debug(&Serial, "myController",PRINT_BIAS);
  }
}

/*
  Lit l'entrée de l'ampli, applique un moyennage, et retourne une tension moyennée.
*/
double readAmpVoltage(){
  int ampReading = analogRead(PIN_AMP);
  ampReadingsTotal = ampReadingsTotal - ampReadings[ampReadIndex];
  ampReadings[ampReadIndex] = ampReading;
  ampReadingsTotal = ampReadingsTotal + ampReadings[ampReadIndex];
  ampReadIndex++;
  // wrap around at the end
  if (ampReadIndex >= numAmpReadings) {
    ampReadIndex = 0;
  }
  // Calculate the average and convert to voltage
  return analogInToVoltage(ampReadingsTotal / numAmpReadings);  
}

/*
  Convertit une tension d'entrée de l'ampli en masse estimée avec les constantes calculées avec l'étalonnage. 
*/
void updateAmpCurrent(){
  if(millis() - ampReadingTimer > ampReadCooldown)
  {
    ampReadingTimer = millis();
    double ampVoltage = readAmpVoltage();
    // update current vars
    mass = calibConstA * ampVoltage + calibConstB;
    taredMass = mass - tare;
  }
}

/*
  Vérifie les données série entrantes, si elles existent, et traite les commandes.
*/
void receiveCom()
{
  if (Serial.available() > 0) { // Check if data is available to read
    char command = Serial.read(); // Read the incoming byte
    if (command == '1') {
      tareRoutine();
    } else if (command == '2') {
      calibConstA = Serial.parseFloat();
      calibConstB = Serial.parseFloat();
    }
    // Add more conditions for additional commands if needed
  }
}

/*
  Envoie les données au PC avec le format suivant : "masse : [masse],tension position : [tension pos]"
*/
void sendData()
{
  //Serial.println( "masse: " + String(taredCurrent) + "," + "tension position: " + String(tensionPos));
}

void setup()
{
  Serial.begin(115200);

  //Setup LCD screen
  lcd.begin(16, 2);
  lcd.createChar(0, FLECHES);
  lcd.createChar(1, SMILEY);

  //SETUP régulateur de position.
  double P = 0.1;
  double I = 0.5; 
  double D = 0.0;
  pinMode(PIN_POSITION, INPUT);
  myController.begin(&tensionPos, &output, &setpoint, P, I, D, P_ON_E, BACKWARD, 20);
  myController.setOutputLimits(0, 255);
  myController.setWindUpLimits(-500, 500); // Groth bounds for the integral term to prevent integral wind-up
  myController.start();

  //Set PWM speed to 37KHz
  //TCCR1B = TCCR1B & B11111000 | B00000010;
  pinMode(PIN_PWM, OUTPUT);
}

void loop()
{
  // read values from current input pin
  updateAmpCurrent();

  // update input and output of the PID controller
  updateController();

  // update the input read from LCD shield buttons
  updateInput();

  // print LCD menu
  updateLCD();
  
  // read incoming commands
  receiveCom();
  sendData();
}
