#include "ArduPID.h"
#include "LiquidCrystal.h"


//
//#define UNO  // pour tester sur un UNO
#define MEGA // pour le vrai prototype PCB avec le MEGA

//Déclaration E/S
#define PIN_BOUTON A0
#define PIN_AMP A2
#ifdef UNO
#define PIN_POSITION A1
#define PIN_PWM 3
#endif
//https://docs.arduino.cc/retired/hacking/hardware/PinMapping2560/
#ifdef MEGA
#define PIN_POSITION A9 // Analog in 9
#define PIN_PWM 12 // Digital 12
#endif

//constantes pour les boutons utilisés
#define BTN_NONE 0
#define BTN_RIGHT 1
#define BTN_UP 2
#define BTN_DOWN 3
#define BTN_LEFT 4
#define BTN_SELECT 5

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
const unsigned long LCD_INTERVAL = 500; // temps entre chaque mise à jour du LCD [ms]
unsigned long comTimer;
const unsigned long COM_INTERVAL = 100; // temps entre chaque envoie de données au PC [ms]
unsigned long ampReadingTimer;
const unsigned long AMP_READ_INTERVAL = 5; // temps entre chaque lecture de l'ampli [ms]
unsigned long buttonTimer;
const unsigned long BUTTON_INTERVAL = 50; // temps minimum entre chaque input différent [ms] 
unsigned long controllerTimer;
const unsigned long CONTROLLER_INTERVAL = 20; // temps de mise à jour du controleur [ms]
// voir constantes BTN_NONE, BTN_RIGHT...
byte prevSelectedButton;
// modes haut-niveau
enum Mode { Pesage,
            Comptage,
            Tarage,
            Etalonnage,
            Precision,
            Unitees,
            Moyennage };
const byte NUM_MODES = 7;
Mode mode;
// menu: on peut démarer un etalonnage, étape 1: mesure à vide, étape 2: mesure de 20g, étape 3: mesure de 50g (FAKE)
enum SousModeEtalonnage { Menu,
                          EtapeVide,
                          Etape1,
                          Etape2,
                          Etape10,
                          Etape20,
                          Etape50};
const byte NUM_CALIB_STEPS = 6;
SousModeEtalonnage sousModeEtalonnage = Menu;

const double CALIB_WEIGHTS[] = {0, 1, 2, 10, 20, 50};//faire un étalonnage de 0 9
const size_t NUM_CALIB_WEIGHTS = sizeof(CALIB_WEIGHTS)/sizeof(CALIB_WEIGHTS[0]);
double calibVoltages[NUM_CALIB_WEIGHTS];
//byte selectedCalib;
// nb de chiffres apres la virgule
const byte PRECISION_OPTS[] = { 0, 1, 2, 3 };
// index
byte selectedPrecision = 0;
//   coin counting mode
const String COIN_OPTS[] = { "5c", "10c", "25c", "1$", "2$" };
const double COIN_MASSES[] = { 3.95, 1.75, 4.4, 6.27, 6.92 };
byte selectedCoin = 0;
const size_t NUM_COIN_TYPES = sizeof(COIN_OPTS) / sizeof(COIN_OPTS[0]);
const double COIN_IDENT_RANGE = 0.5;

// units
String unitOpts[] = { "g", "oz" };
// ratio of the unit to gram
const double UNIT_CONVERSIONS[] = { 1, 0.035274, 0.01 };
byte selectedUnit = 0;
byte numUnits = sizeof(unitOpts) / sizeof(unitOpts[0]);

//déclaration boucle de régulation (Présentement en position)
ArduPID myController;
double output;            // PID sortie 0~255
double positionVoltage;        // PID entrée 0~5
double setpoint = 1.445;  //tension apres ampli du capteur à 16.4 mm
double ampVoltage;        // mis à jour à chaque AMP_READ_INTERVAL
double mass;
double taredMass;
double tare;
// min et max dans les dernières secondes pour déterminer la stabilité
const double stabilityPeriod = 3000;  // en [ms]
const double STABLE_RANGE = 0.04;    // the max differnce betweeen setpoint and the current positionVoltage value that is considered stable.
unsigned long lastStableValTime;
bool isStable;

double ampV1;
double ampV2;
double calibConstA = 67.08;
double calibConstB = -56.72;

// lectures d'entrée et moyennage pour la sortie de l'ampli de courant
const unsigned int MAX_NUM_AMP_READINGS = 256;
size_t numAmpReadings = 100;
unsigned int ampReadings[MAX_NUM_AMP_READINGS];  // lectures pour la sortie de l'ampli
size_t ampReadIndex = 0;                   // indexe de la lecture actuelle
unsigned long ampReadingsTotal = 0;            // total des lectures dans le tableau

/*
  Calcule les parametres d'une régression linéaire.
  Paramètres :
    x_values : Un tableau de valeurs doubles représentant les coordonnées x.
    y_values : Un tableau de valeurs doubles représentant les coordonnées y correspondantes.
    num_points : Un entier indiquant le nombre de points de données.
    slope : Une référence pour stocker la pente calculée.
    intercept : Une référence pour stocker l'ordonnée à l'origine calculée.
*/
void linearRegression(double x_values[], double y_values[], int num_points, double& slope, double& intercept) {
  double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x_squared = 0;

  for (int i = 0; i < num_points; i++) {
    sum_x += x_values[i];
    sum_y += y_values[i];
    sum_xy += x_values[i] * y_values[i];
    sum_x_squared += x_values[i] * x_values[i];
  }

  slope = (num_points * sum_xy - sum_x * sum_y) / (num_points * sum_x_squared - sum_x * sum_x);
  intercept = (sum_y - slope * sum_x) / num_points;
}

/*
  Retourne un byte pour le bouton selectionné. Voir les constantes BTN_RIGHT, BTN_LEFT...
*/
byte getSelectedButton() {
  // val is from 0 to 1023
  int val = analogRead(0);
  int selected;
  if (val < 66) {
    selected = BTN_RIGHT;  // 0: right
  } else if (val < 220) {
    selected = BTN_UP;  // 132: up
  } else if (val < 395) {
    selected = BTN_DOWN;  // 309: down
  } else if (val < 602) {
    selected = BTN_LEFT;  // 481: left
  } else if (val < 963) {
    selected = BTN_SELECT;  // 722: select
  } else {
    selected = BTN_NONE;  // 1023: nothing
  }
  return selected;
}

/*
  Passe à travers les modes haut-niveau à la droite
*/
void cycleModeRight() {
  mode = (Mode)(mode + 1);
  if (mode == NUM_MODES)
    mode = 0;
}

/*
  Passe à travers les modes haut-niveau à la gauche
*/
void cycleModeLeft() {
  if (mode == 0)
    mode = NUM_MODES;
  mode = (Mode)(mode - 1);
}

/*
  Gère la selection droite-gauche de mode.
*/
void handleInputMenuSelect(byte button) {
  switch (button) {
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
void handleInputTarage(byte button) {
  handleInputMenuSelect(button);
  switch (button) {
    case BTN_SELECT:
      tareRoutine();
  }
}

/*
  Gère toutes les entrées pour le mode étalonnage
*/
void handleInputEtalonnage(byte button) {
  if(sousModeEtalonnage == Menu)
  {
    handleInputMenuSelect(button);
    switch (button) {
      case BTN_SELECT:
        sousModeEtalonnage = EtapeVide;
    }
  }
  else
  {
    if (button == BTN_SELECT && isStable) {
      Serial.println(ampVoltage);
      calibVoltages[sousModeEtalonnage - 1] = ampVoltage;
      if(sousModeEtalonnage < NUM_CALIB_STEPS)
      {
        sousModeEtalonnage = sousModeEtalonnage + 1;
      }
      else
      {
        // calculate regression
        linearRegression(calibVoltages, CALIB_WEIGHTS, NUM_CALIB_WEIGHTS, calibConstA, calibConstB);
        sousModeEtalonnage = Menu;
        // return to main menu, reset tare
        tare = 0;
        mode = Pesage;
      }
    }
  }
}

/*
  Gère toutes les entrées pour le mode precision
*/
void handleInputPrecision(byte button) {
  handleInputMenuSelect(button);
  switch (button) {
    case BTN_UP:
      if (selectedPrecision == sizeof(PRECISION_OPTS) - 1)
        selectedPrecision = 0;
      else
        selectedPrecision++;
      break;
    case BTN_DOWN:
      if (selectedPrecision == 0)
        selectedPrecision = sizeof(PRECISION_OPTS) - 1;
      else
        selectedPrecision--;
  }
}

/*
  Gère toutes les entrées pour le mode comptage de pieces
*/
void handleInputComptage(byte button) {
  handleInputMenuSelect(button);
  switch (button) {
    case BTN_UP:
      if (selectedCoin == NUM_COIN_TYPES - 1)
        selectedCoin = 0;
      else
        selectedCoin++;
      break;
    case BTN_DOWN:
      if (selectedCoin == 0)
        selectedCoin = NUM_COIN_TYPES - 1;
      else
        selectedCoin--;
  }
}

/*
  Gère toutes les entrées pour le mode selection de unitées
*/
void handleInputUnitees(byte button) {
  handleInputMenuSelect(button);
  switch (button) {
    case BTN_UP:
      if (selectedUnit == numUnits - 1)
        selectedUnit = 0;
      else
        selectedUnit++;
      break;
    case BTN_DOWN:
      if (selectedUnit == 0)
        selectedUnit = numUnits - 1;
      else
        selectedUnit--;
  }
}

/*
  Efface les valeurs du tableau des lectures de l'ampli jusqu'à un index donnée, et met ampReadingsTotal à zero.
  À utiliser quand on change numAmpReadings
*/
void clearAveragingFields(int clearToIndex) {
  ampReadIndex = 0;
  for (int i = 0; i <= clearToIndex; i++) {
    ampReadings[i] = 0;
  }
  ampReadingsTotal = 0;
}

/*
  Gère toutes les entrées pour le mode selection de taille de moyennage
*/
void handleInputMoyennage(byte button) {
  handleInputMenuSelect(button);
  const int INTERVAL = 30;
  switch (button) {
    case BTN_UP:
      numAmpReadings += INTERVAL;
      if (numAmpReadings >= MAX_NUM_AMP_READINGS)
        numAmpReadings = MAX_NUM_AMP_READINGS;
      clearAveragingFields(numAmpReadings);
      break;
    case BTN_DOWN:
      numAmpReadings -= INTERVAL;
      if (numAmpReadings <= 1)
        numAmpReadings = 1;
      clearAveragingFields(numAmpReadings);
      break;
  }
}

/*
  Gère toutes les entrées
*/
void handleInput(byte button) {
  switch (mode) {
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
    case Unitees:
      handleInputUnitees(button);
      break;
    case Moyennage:
      handleInputMoyennage(button);
  }
}

/*
  Reçoit et traite les données d'entrée, gère le debouncing.
*/
void updateInput() {
  if (millis() - buttonTimer > BUTTON_INTERVAL) {
    int s = getSelectedButton();
    if (prevSelectedButton != s) {
      if (s != BTN_NONE) {
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
void tareRoutine() {
  tare = mass;
}

/*
  Imprime un titre de menu sur le LCD : "<>monTitre" en haut à gauche du LCD
*/
void lcdPrintTitle(String title) {
  lcd.setCursor(0, 0);
  lcd.print("<>");
  lcd.print(title);
}

/*
  Imprime un smiley en haut à droite du LCD
*/
void lcdPrintStability() {
  lcd.setCursor(15, 0);
  if (isStable) lcd.write(byte(1));  // smiley
}

/*
  Imprime un OK en bas à gauche du LCD
*/
void lcdPrintOk() {
  lcd.setCursor(0, 1);
  lcd.print("OK");
}

/*
  Imprime un OK en bas à gauche du LCD si la balance est stable, ... sinon 
*/
void lcdPrintOkIfStable() {
  lcd.setCursor(0, 1);
  if (isStable)
    lcd.print("OK");
  else
    lcd.print("...");
}

/*
  Imprime la masse calculée en bas à droite du LCD
*/
void lcdPrintMass() {
  byte precision = PRECISION_OPTS[selectedPrecision];

  // convertir double en str
  byte decimalPosInLcd;
  if (precision == 0)  // no space for dot
    decimalPosInLcd = 14;
  else
    decimalPosInLcd = 13 - precision;
  char massStr[8];  // taille max de la zone qu'on accorde au poids
  double convertedTaredMass = taredMass * UNIT_CONVERSIONS[selectedUnit];
  dtostrf(convertedTaredMass, 7, precision, massStr);  // Format the double to string with 7 total characters and 2 decimal places
  byte decimalPosInStr = 0;
  for (int i = 0; massStr[i] != '.' && massStr[i] != '\0'; i++) {
    decimalPosInStr++;
  }
  byte startPos = decimalPosInLcd - decimalPosInStr;
  lcd.setCursor(startPos, 1);
  lcd.print(massStr);
  lcd.setCursor(14, 1);
  lcd.print(unitOpts[selectedUnit]);
}

String getClosestCoin(double mass) {
  int closestCoinType;
  double massDiff;
  double smallestMassDiff;
  for (int i = 0; i < NUM_COIN_TYPES; i++) {
    massDiff = abs(COIN_MASSES[i] - mass);
    if (i == 0 || massDiff < smallestMassDiff) {
      closestCoinType = i;
      smallestMassDiff = massDiff;
    }
  }
  if (smallestMassDiff < COIN_IDENT_RANGE) {
    return COIN_OPTS[closestCoinType];
  }
  return "...";
}

/*
  Met à jour l'LCD pour le mode pesage
*/
void updateLcdPesage() {
  lcdPrintTitle("Peser");
  lcdPrintStability();
  lcd.setCursor(0, 1);
  lcd.print(getClosestCoin(mass));
  lcdPrintMass();
}

/*
  Met à jour l'LCD pour le mode tarage
*/
void updateLcdTarage() {
  lcdPrintTitle("Tarer");
  lcdPrintOk();
  lcdPrintStability();
  lcdPrintMass();
}

/*
  Met à jour l'LCD pour le mode étalonnage
*/
void updateLcdEtalonnage() {
  if(sousModeEtalonnage == Menu){
    lcdPrintTitle("Etalonnage");
    lcdPrintOk();
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print(sousModeEtalonnage);
    if(sousModeEtalonnage == EtapeVide){
      lcd.print(")Aucun poids");
    }
    else
    {
      lcd.print(")Avec ");
      lcd.print(CALIB_WEIGHTS[sousModeEtalonnage - 1]);
      lcd.print("g");
    }
    lcdPrintOkIfStable();
  }
}

/*
  Met à jour l'LCD pour le mode précision
*/
void updateLcdPrecision() {
  lcdPrintTitle("Precision");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));  // up down arrows
  lcd.setCursor(1, 1);
  lcd.print(PRECISION_OPTS[selectedPrecision]);
  lcd.print(" ch");
  lcdPrintMass();
}

/*
  Met à jour l'LCD pour le mode de comptage
*/
void updateLcdComptage() {
  lcdPrintTitle("Comptage");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));  // up down arrows
  lcd.setCursor(1, 1);
  lcd.print(COIN_OPTS[selectedCoin]);
  int num = mass / COIN_MASSES[selectedCoin];
  lcd.setCursor(14, 1);
  lcd.print(num);
}

/*
  Met à jour l'LCD pour le mode de selection d'unitées
*/
void updateLcdUnitees() {
  lcdPrintTitle("Unitees");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));  // up down arrows
  lcd.setCursor(1, 1);
  lcd.print(unitOpts[selectedUnit]);
  lcdPrintMass();
}

/*
  Met à jour l'LCD pour le mode de selection de moyennage
*/
void updateLcdMoyennage() {
  lcdPrintTitle("Moyennage");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));  // up down arrows
  lcd.setCursor(1, 1);
  lcd.print(numAmpReadings * AMP_READ_INTERVAL);
  lcd.print("ms");
  lcdPrintMass();
}

/*
  Met à jour l'LCD
*/
void updateLCD() {
  String lcdTopText;
  if (millis() - lcdTimer > LCD_INTERVAL) {
    lcdTimer = millis();
    lcd.clear();
    switch (mode) {
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
        break;
      case Unitees:
        updateLcdUnitees();
        break;
      case Moyennage:
        updateLcdMoyennage();
    }
  }
}

/*
  Convertit l'entrée 10 bits analogue en tension
*/
double analogInToVoltage(double in) {
  // convert arduino input to ampli output
  return in * (5.0 / 1023.0);
}

/*
  Evalue la stabilité du system, met à jour la variable isStable
*/
void updateStability() {
  if (abs(positionVoltage - setpoint) > STABLE_RANGE) {
    lastStableValTime = 0;
    isStable = false;
  } else {  // value in range of stability
    if (lastStableValTime == 0)
      lastStableValTime = millis();
    else if (millis() - lastStableValTime > stabilityPeriod)
      isStable = true;
  }
}

/*
  Lit la valeur analogue du capteur, effectue le calcul du PID, et met à jour la sortie.
  Met la variable de stabilité isStable à jour.
*/
void updateController() {
  if (millis() - controllerTimer > CONTROLLER_INTERVAL) {
    controllerTimer = millis();
    int analogIn = analogRead(PIN_POSITION);
    positionVoltage = analogInToVoltage(analogIn);
    myController.compute();
    analogWrite(PIN_PWM, output);  //Output for Vamp_in

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
  Lit l'entrée de l'ampli, applique un moyennage, et met la tension moyennée dans ampVoltage.
*/
void readAmpVoltage() {
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
  double avgReading = (double) (ampReadingsTotal) / (double) (numAmpReadings);
  ampVoltage = analogInToVoltage(avgReading);
}

/*
  Convertit une tension d'entrée de l'ampli en masse estimée avec les constantes calculées avec l'étalonnage. 
*/
void updateAmpCurrent() {
  if (millis() - ampReadingTimer > AMP_READ_INTERVAL) {
    ampReadingTimer = millis();
    readAmpVoltage();
    // update current vars
    mass = calibConstA * ampVoltage + calibConstB;
    taredMass = mass - tare;
  }
}

/*
  Vérifie les données série entrantes, si elles existent, et traite les commandes.
*/
void receiveCom() {
  if (Serial.available() > 0) {    // Check if data is available to read
    char command = Serial.read();  // Read the incoming byte
    switch (command){
      case 't':
        tareRoutine();
        break;
      case 'e':
        calibConstA = Serial.parseFloat();
        calibConstB = Serial.parseFloat();        
        break;
      case 'm':
        numAmpReadings = Serial.parseInt();
    }
    // Add more conditions for additional commands if needed
  }
}

/*
  Envoie les données au PC avec le format suivant : "masse : [masse],tension position : [tension pos]"
*/
void sendData() {
  if (millis() - comTimer > COM_INTERVAL) {
    comTimer = millis();
    const unsigned int precision = 5;
    Serial.println("v_amp:" + String(ampVoltage, precision) + "," + 
      "v_pos:" + String(positionVoltage, precision) + "," + 
      "taredMass:" + String(taredMass, precision) + "," + 
      "stable:" + String(isStable, precision));
  }
}

void setup() {
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
  myController.begin(&positionVoltage, &output, &setpoint, P, I, D, P_ON_E, BACKWARD, 20);
  myController.setOutputLimits(0, 255);
  myController.setWindUpLimits(-500, 500);  // Groth bounds for the integral term to prevent integral wind-up
  myController.start();

  //Set PWM speed to 37KHz
  //TCCR1A = (1 << WGM11) | (1 << COM1A1);
  //TCCR1B = (0 << WGM12)| (0 << WGM13) |(1 << CS11) | (0 << CS10);
  //ICR1 = 256;
  pinMode(PIN_PWM, OUTPUT);
}

void loop() {
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
