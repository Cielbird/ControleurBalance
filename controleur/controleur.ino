#include "ArduPID.h"
#include "LiquidCrystal.h"


//Déclaration E/S
#define PIN_BOUTON A0
#define PIN_POSITION A1
#define PIN_CURRENT A2
#define PIN_PWM 3

//constantes pour les boutons utilisés
#define BTN_NONE 0
#define BTN_RIGHT 1
#define BTN_UP 2
#define BTN_DOWN 3
#define BTN_LEFT 4
#define BTN_SELECT 5


//Setup écran LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);        // select the pins used on the LCD panel

unsigned long tepTimer ;
unsigned long buttonTimer;
unsigned long controllerTimer;
// 1:right, 2:up, 3:down, 4:left, 5:select, 0:nothing
byte prevSelectedButton;
// 0: pesée, 1: tarage, 2: étalonnage
enum Mode {Pesage, Tarage, Etalonnage};
// étape 1: mesure à vide, étape 2: mesure de 20g
enum SousModeEtalonnage {Etape1, Etape2};
Mode mode = Pesage;

unsigned int weights[] = {1, 2, 5, 10, 20, 50};//faire un étalonnage de 0 9
byte selectedCalib;

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

// input readings and running average for current output from the amplifier
const int numCurrentReadings = 256;
double currentReadings[numCurrentReadings]; // the readings from the analog input
int currentReadIndex = 0;                   // the index of the current reading
double currentInputTotal = 0;               // the running total
double latestMaxCurrent = 0;

//déclaration boucle de régulation (Présentement en position)
ArduPID myController;
double output; // PID output
double tensionPos; // PID input
double setpoint = 1.445; //16.4; dis en mm
bool isStable;
double current;
double taredCurrent;
double tare;
// min et max dans les dernières secondes pour déterminer la stabilité
double stabilityPeriod = 3000; // en [ms]
double stabilityRange = 0.5; // the max differnce betweeen setpoint and the current tensionPos value that is considered stable.
unsigned long lastStableValTime;

/**
Returns a byte for the currently selected button. 1:right, 2:up, 3:down, 4:left, 5:select, 0:nothing
**/
byte getSelectedButton(){
  int val = analogRead(0);                     // read the analog value for buttons
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

void cycleModeRight(){
  switch(mode){
    case Pesage:
      mode = Tarage;
      break;
    case Tarage:
      mode = Etalonnage;
      break;
    case Etalonnage:
      mode = Pesage;
      break;
  }
}

void cycleModeLeft(){
  switch(mode){
    case Pesage:
      mode = Etalonnage;
      break;
    case Tarage:
      mode = Pesage;
      break;
    case Etalonnage:
      mode = Tarage;
      break;
  }
}

void handleInput(byte button){
  switch(button){
    case 1: // right
      cycleModeRight();
      break;
    case 4: // left
      cycleModeLeft();
      break;
    case 2: // up
      selectedCalib = (selectedCalib+1)%6;
      break;
    case 3: // down
      if(selectedCalib==0)
        selectedCalib = 5;
      else
        selectedCalib = selectedCalib-1;
      break;
    case 5: // select
      switch(mode){
        case 1: //tarage
          tareRoutine();
          break;
        case 2: //étalonnage
          calibrate();
      }
  }
}

void calibrate(){
  bool calibrating = true;
  lcd.clear();
  // instrucitons étape 1
  lcd.setCursor(0, 0);
  lcd.print("1: Aucun poids");
  lcd.setCursor(0, 1);
  lcd.print("OK");
  while(!)
  //
  lcd.setCursor(0, 0);
  lcd.print("Mesure a vide en");
  lcd.setCursor(0, 1);
  lcd.print(" cours...");
  while(!isStable);

  delay(1000);
}

void tareRoutine(){
  bool taring = true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Tare");
  lcd.setCursor(0, 1);
  lcd.print("en cours...");
  tare = current;
  delay(100);
}

void updateLCD(){
  String lcdTopText;
  if(millis() - tepTimer > 500){
    tepTimer = millis();
    
    lcd.clear();
    lcd.setCursor(0, 0);
    switch(mode){
      case Pesage: //pesée
        lcdTopText = "< Peser >";
        break;
      case Tarage: //tarage
        lcdTopText = "< Tarer >";
        break;
      case Etalonnage: //étalonnage
        lcdTopText = "< Calibrer >";
    }
    lcd.print(lcdTopText);

    lcd.setCursor(15, 0);
    if(isStable) lcd.write(byte(1)); // smiley

    switch(mode){
      case Pesage: //pesée
        lcd.setCursor(11, 1);
        lcd.print(taredCurrent);
        lcd.setCursor(15, 1);
        lcd.print("g");
        break;
      case Tarage: //tarage
        lcd.setCursor(0, 1);
        lcd.print("OK");
        lcd.setCursor(11, 1);
        lcd.print(taredCurrent);
        lcd.setCursor(15, 1);
        lcd.print("g");
        break;
      case Etalonnage: //étalonnage
        lcd.setCursor(0, 1);
        lcd.print("OK");
        lcd.setCursor(12, 1);
        lcd.print(weights[selectedCalib]);
        lcd.print("g");
        lcd.write(byte(0)); // fleches up down
    }
  }
}

void updateInput(){
  if(millis() - buttonTimer > 50){
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

double capteurInputToVoltage(int in){
  // convert arduino input to ampli output
  return in * (5.0 / 1023.0);
}

void updateStability(double latestVal)
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

void updateController()
{
  if(millis() - controllerTimer > 20)
  {
    controllerTimer = millis();
    int analogIn = analogRead(PIN_POSITION);
    tensionPos = capteurInputToVoltage(analogIn);
    myController.compute();
    analogWrite(PIN_PWM, output); //Output for Vamp_in

    // check stability
    updateStability(tensionPos);
    //Pour afficher les valeur de PID, et input, output, seulement enlever les commentaire.
    /*myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                                PRINT_OUTPUT   | // in the Serial plotter
                                                PRINT_SETPOINT |
                                                PRINT_BIAS     );*/
    //myController.debug(&Serial, "myController",PRINT_BIAS);
  }
}


void readCurrent(){
  int currentReading = analogRead(PIN_CURRENT);
  currentInputTotal = currentInputTotal - currentReadings[currentReadIndex];
  currentReadings[currentReadIndex] = currentReading;
  currentInputTotal = currentInputTotal + currentReadings[currentReadIndex];
  currentReadIndex++;
  // wrap around at the end
  if (currentReadIndex >= numCurrentReadings) {
    currentReadIndex = 0;
  }
  // Calculate the average:
  double inputAverage = currentInputTotal / numCurrentReadings;

  // update current vars
  current = inputAverage * (5.0 / 1023.0) * 50.0 *50.0 /36.45;
  taredCurrent = current - tare;
}

void receiveCom()
{
  if (Serial.available() > 0) { // Check if data is available to read
    char command = Serial.read(); // Read the incoming byte
    if (command == '1') {
      // Perform action for command 1
      tareRoutine();
    } else if (command == '2') {
      // Perform action for command 2
      //Serial.println("Received command 2");
    }
    // Add more conditions for additional commands if needed
  }
}

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
  readCurrent();

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
