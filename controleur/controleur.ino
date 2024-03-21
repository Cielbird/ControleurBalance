#include "ArduPID.h"
#include "LiquidCrystal.h"


//Déclaration E/S
const int PIN_BOUTON = A0;
const int PIN_POSITION = A1;
const int PIN_CURRENT = A2;
const int PIN_PWM = 3;

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
byte mode;

unsigned int weights[] = {1, 2, 5, 10, 20, 50};//faire un étalonnage de 0 9
byte selectedCalib;

double tension = 0;

// characters speciaux
byte fleches[8] = {
  B00100,
  B01010,
  B10001,
  B00000,
  B10001,
  B01010,
  B00100,
};

// input readings and running average for current output from the amplifier
const int numCurrentReadings = 256;
double currentReadings[numCurrentReadings]; // the readings from the analog input
int currentReadIndex = 0;                   // the index of the current reading
double currentInputTotal = 0;               // the running total

//déclaration boucle de régulation (Présentement en position)
ArduPID myController;
double input;
double output;
double current;
double taredCurrent;
double tare = 0.0;
double setpoint = 1.445; //16.4; dis en mm

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

void handleInput(byte button){
  switch(button){
    case 1: // right
      mode = (mode+1)%3;
      break;
    case 4: // left
      // can't use modulo 255 is not multiple of 3
      if(mode==0)
        mode = 2;
      else
        mode = mode-1;
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
  lcd.setCursor(0, 0);
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("en cours...");
  delay(100);
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
      case 0: //pesée
        lcdTopText = "< Peser >";
        break;
      case 1: //tarage
        lcdTopText = "< Tarer >";
        break;
      case 2: //étalonnage
        lcdTopText = "< Calibrer >";
    }
    lcd.print(lcdTopText);

    switch(mode){
      case 0: //pesée
        lcd.setCursor(11, 1);
        lcd.print(taredCurrent);
        lcd.setCursor(15, 1);
        lcd.print("g");
        break;
      case 1: //tarage
        lcd.setCursor(0, 1);
        lcd.print("OK");
        lcd.setCursor(11, 1);
        lcd.print(taredCurrent);
        lcd.setCursor(15, 1);
        lcd.print("g");
        break;
      case 2: //étalonnage
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
    if(prevSelectedButton != s){
      if(s != BTN_NONE)
        buttonTimer = millis();
        handleInput(s);
      prevSelectedButton = s;
    }
  }
}

double capteurInputToVoltage(int in){
  // convert arduino input to ampli output
  return in * (5.0 / 1023.0);
}


void updateController()
{
  if(millis() - controllerTimer > 20){
    controllerTimer = millis();
    int analogIn = analogRead(PIN_POSITION);
    input = capteurInputToVoltage(analogIn);
    tension = input;
    myController.compute();
    analogWrite(PIN_PWM, output); //Output for Vamp_in
    //Pour afficher les valeur de PID, et input, output, seulement enlever les commentaire.
    /*myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                                PRINT_OUTPUT   | // in the Serial plotter
                                                PRINT_SETPOINT |
                                                PRINT_BIAS     );*/
    //myController.debug(&Serial, "myController",PRINT_BIAS);
  }
}


void readCurrent(){
  currentInputTotal = currentInputTotal - currentReadings[currentReadIndex];
  currentReadings[currentReadIndex] = analogRead(PIN_CURRENT);
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


void setup()
{
  Serial.begin(115200);  

  //Setup LCD screen
  lcd.begin(16, 2);
  lcd.createChar(0, fleches);

  //SETUP régulateur de position.
  double P = 0.1;
  double I = 0.5; 
  double D = 0.0;
  pinMode(PIN_POSITION, INPUT);
  myController.begin(&input, &output, &setpoint, P, I, D, P_ON_E, BACKWARD, 20);
  myController.setOutputLimits(0, 255);
  myController.setWindUpLimits(-500, 500); // Groth bounds for the integral term to prevent integral wind-up
  myController.start();

  //Set PWM speed to 37KHz
  //TCCR1B = TCCR1B & B11111000 | B00000010;
  pinMode(PIN_PWM, OUTPUT);
}

void sendData()
{
  Serial.println( "masse: " + String(taredCurrent) + "," + "tension position: " + String(tension));
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
