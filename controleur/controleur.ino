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
  Serial.begin(9600);  

  //Setup LCD screen
  lcd.begin(16, 2);

  //SETUP régulateur de position.
  double P = 0.1;
  double I = 0.015;
  double D = 0.0;
  pinMode(PIN_POSITION, INPUT);
  myController.begin(&input, &output, &setpoint, P, I, D, P_ON_E, BACKWARD, 20);
  myController.setOutputLimits(-254, 254);
  myController.setWindUpLimits(-500, 500); // Groth bounds for the integral term to prevent integral wind-up
  myController.start();

  //Set PWM speed to 37KHz
  TCCR1B = TCCR1B & B11111000 | B00000010;
  pinMode(10, OUTPUT);
}

double capteurInputToDist(double tension)
{
  // convert arduino input to ampli output
  tension = input * (5.0 / 1023.0)
  // convert ampli output to input
  // voir tableau 7.5 du rapport
  tension /= 2.4; // K
  tension += 0.4; // V offset
  // convert capteur output to dist
  // voir graphique 7.6
  double C_1 = 5813.9;
  double C_2 = 198.24;
  double C_3 = 0.60750;
  double a = pow(C_1 / (tension - C_3), 2.0/3) - C_2;
  return sqrt(a);
}

void loop()
{
  //Lecture du bouton analogique, peut-être changer pour une interruption en mode falling-edge avec les registres.
  //lcd_key = read_LCD_buttons(); // Read the current button state
  //navigationKey();
  int in = analogRead(PIN_POSITION)
  input = capteurInputToDist(in);
  Serial.print("v: ");
  Serial.print(voltage);
  Serial.print("d pour 2: ");
  Serial.print(capteurInputToDist(512));

  myController.compute();
  //Pour afficher les valeur de PID, et input, output, seulement enlever les commentaire.
  /*myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     );*/
  
  //myController.debug(&Serial, "myController",PRINT_BIAS);

  //Cette section est utile pour trouver la position zero
  //Serial.print("Valeur set = ");
  //Serial.print(setpoint);
  //Serial.print(" Valeur apercu = ");
  //Serial.print(input);
  //Serial.print(" Valeur Output = ");
  //Serial.println(output);
  Serial.print('\n');
  analogWrite(10, output+29); //Output for Vamp_in
  delay(20);
}
