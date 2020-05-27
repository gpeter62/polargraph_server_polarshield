/**
*  Polargraph Server. - CORE
*  Written by Sandy Noble
*  Released under GNU License version 3.
*  http://www.polargraph.co.uk
*  https://github.com/euphy/polargraph_server_a1

Configuration.

This is one of the core files for the polargraph server program.  
It sets up the motor objects (AccelSteppers), and has default
values for the motor, sprocket and microstepping combinations used
by polargraphs so far.
*/

/* Motor setup if you are using serial stepper drivers 
(EasyDrivers, stepsticks, Pololu etc).

If you are wiring up yourself, just put the pin numbers in here.

Note that the pen lift servo usually lives on pin 9, so avoid 
that if you can. If you can't, then you know how to change it.
*/
#define MOTOR_A_ENABLE_PIN 24
#define MOTOR_A_STEP_PIN 28  //2
#define MOTOR_A_DIR_PIN 26   //3
  
#define MOTOR_B_ENABLE_PIN A2
#define MOTOR_B_STEP_PIN A5
#define MOTOR_B_DIR_PIN A4
AccelStepper motorA(1,MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN,MOTOR_A_ENABLE_PIN);     //GP első prm 1 volt
AccelStepper motorB(1,MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN,MOTOR_B_ENABLE_PIN); 


void configuration_motorSetup(){
  pinMode(MOTOR_A_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_A_ENABLE_PIN, HIGH);
  pinMode(MOTOR_B_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_B_ENABLE_PIN, HIGH);
  motorA.setEnablePin(MOTOR_A_ENABLE_PIN);
  motorA.setPinsInverted(false, false, true);   //  step,direction,enable
  motorB.setEnablePin(MOTOR_B_ENABLE_PIN);
  motorB.setPinsInverted(false, false, true); // this one turns the opposite direction to A, hence inverted.    t,f,t volt eredetileg!

}

void configuration_setup() {
  sd_initSD();
  lcd_initLCD();
  lcd_showSummary();
  delay(2000);
  
  // calibration pins
  pinMode(ENDSTOP_X_MIN, INPUT_PULLUP);
  pinMode(ENDSTOP_Y_MIN, INPUT_PULLUP);
  pinMode(ENDSTOP_X_MAX, INPUT_PULLUP);
  pinMode(ENDSTOP_Y_MAX, INPUT_PULLUP);
  
  lcd_displayFirstMenu();
//  releaseMotors();
  
}
