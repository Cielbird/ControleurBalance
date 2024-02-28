#include "ArduPID.h"
#include "LiquidCrystal.h"


//Déclaration E/S
const int PIN_BOUTON = A0;
const int PIN_POSITION = A1;
const int PIN_CURRENT = A2;
const int PIN_PWM = 10;


//initialisation de l'écran LCD
#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5


//Setup écran LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Setup menu et bouton LCD
int lcd_key = 0; 
int adc_key_in = 0;
unsigned long menuTriggeredTime = 0;
int currentScreen = -1;
const int numOfScreen = 2;
String menu[numOfScreen][2] ={
  {"Poid","Courrant = "},
  {"Calibration", "Mettre poids : "}
};
int parameter[numOfScreen];
bool updateScreen = true;
bool boutonAlreadyPressed = false;//Pour empecher les boucles lorsqu'une fois pressé.


//déclaration boucle de régulation (Présentement en position)
ArduPID myController;
double input;
double output;
double setpoint = 400;


//Les boutons vont varier la tension entrée A0. donc chaque bouton a un range spécifique à celui-ci.
int read_LCD_buttons(){ // read the buttons
  int adc_key_in = analogRead(PIN_BOUTON);
  if(boutonAlreadyPressed | adc_key_in > 1000)
  {return btnNONE;}
  else
  {
    if (adc_key_in < 50) return btnRIGHT;
    else if (adc_key_in < 250) return btnUP;
    else if (adc_key_in < 450) return btnDOWN;
    else if (adc_key_in < 650) return btnLEFT;
    else if (adc_key_in < 850) return btnSELECT;
    else return btnNONE;
  }
 }


//Liste des menu???
void navigationKey()
{
  if (lcd_key != btnNONE)
  {
    boutonAlreadyPressed = true;
    switch (lcd_key)
    {
      case (btnRIGHT):
        {}
        break;
      case (btnLEFT):
        {}
        break;
      case (btnUP):
        {}
        break;
      case (btnDOWN):
        {}
        break;
      case (btnSELECT):
        {}
        break;
    }
  }
}



void setup()
{
  Serial.begin(115200);  

  //Setup LCD screen
  lcd.begin(16, 2);

  //SETUP régulateur de position.
  double P= 0.0004;
  double I = 0.0005;
  double D = 0.0025;
  pinMode(PIN_POSITION, INPUT);
  myController.begin(&input, &output, &setpoint, P, I, D, P_ON_E, BACKWARD, 20);
  myController.setOutputLimits(-254, 254);
  myController.setWindUpLimits(-500, 500); // Groth bounds for the integral term to prevent integral wind-up
  myController.start();

  //Set PWM speed to 37KHz
  TCCR1B = TCCR1B & B11111000 | B00000010;
  pinMode(10, OUTPUT);
  pinMode()
}

double capteurVoltToDist(double tension)
{
  C_1 = 
  return pow(C_1 / (tension - C_3), 2/3) - C_2
}

void loop()
{
  //Lecture du bouton analogique, peut-être changer pour une interruption en mode falling-edge avec les registres.
  //lcd_key = read_LCD_buttons(); // Read the current button state
  //navigationKey();
  voltage = analogRead(PIN_POSITION) / 1000.0 * 5.0;
  input = capteurVoltToDist(voltage);

  myController.compute();
  //Pour afficher les valeur de PID, et input, output, seulement enlever les commentaire.
  /*myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     );*/
  
  //myController.debug(&Serial, "myController",PRINT_BIAS);

  //Cette section est utile pour trouver la position zero
  Serial.print("Valeur set = ");
  Serial.print(setpoint);
  Serial.print(" Valeur apercu = ");
  Serial.print(input);
  Serial.print(" Valeur Output = ");
  Serial.println(output);



  analogWrite(10, output+29); //Output for Vamp_in
  delay(20);
}
