/**
*  Polargraph Server for ATMEGA1280+ 
*  Written by Sandy Noble
*  Released under GNU License version 3.
*  http://www.polargraph.co.uk
*  https://github.com/euphy/polargraph_server_polarshield

Modified by Peter Gautier
Usable without Polarshield with standard components
Arduino Mega
standard LCD 4x20
1 x Rotary Encoder
4 x pushbutton

Datalogger Shield with SD card. 
Modified for MEGA pinout!!!
UNO  ----> MEGA
A4,A5  --> Pin 20,21   (SCL,SDA)
11,12,13-->50,51,52 (SPI bus)
2 x PicoDrive Stepper driver modul
*/

// GP keyboard

//Keyboard pins and Variables
#define BtnEnter_Pin  27
#define BtnExit_Pin   25
#define BtnLeft_Pin   31
#define BtnRight_Pin  29

boolean BtnEnter,BtnExit,BtnLeft,BtnRight,RotaryKnob;
boolean btnEnter,btnExit,btnLeft,btnRight,rotaryKnob = LOW;

//----------- Rotary Encoder data ------------------------
static int pinA = 19; // 4. hardware interrupt pin is digital pin 19
static int pinB = 18; // 5. hardware interrupt pin is digital pin 18
static int pinC = 33; //rotary knob switch
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA 
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB 
volatile int encoderPos = 0; //this variable stores our current value of encoder position. 
volatile int oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed 
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

volatile int MenuEncoderPos = 0;
boolean MenuEncPossible = true;



volatile int currentMenu = 1;
volatile int currentLine = 1;      //menu line position inside the currentMenu
volatile int currentAbsLine = 1;   //menu line absolut position
volatile int pressedButton = 1;

#define BUTTON_PEN_UP_DOWN 11
#define BUTTON_INC_DEC_SPEED 12
#define BUTTON_PREV_NEXT_FILE 14
#define BUTTON_CANCEL_FILE 15
#define BUTTON_DRAW_THIS_FILE 16
#define BUTTON_DRAW_START_LINE 17
#define BUTTON_INC_DEC_ACCEL 18
#define BUTTON_SET_HOME 19
#define BUTTON_MOVE_INC_DEC_A 21
#define BUTTON_MOVE_INC_DEC_B 23
#define BUTTON_SET_MULTIPLIER 24
#define BUTTON_INC_DEC_PENSIZE 25
#define BUTTON_RESET 26
#define BUTTON_INC_DEC_PENSIZE_INC 27
#define BUTTON_POWER_ON_OFF 32
#define BUTTON_STOP_FILE 34
#define BUTTON_CALIBRATE 36
#define BUTTON_TOGGLE_ECHO 37
#define BUTTON_RESET_SD 38
#define BUTTON_RETURN_HOME 39
#define BUTTON_INC_DEC_PENLIFT_UP 40
#define BUTTON_INC_DEC_PENLIFT_DOWN 42
#define BUTTON_PENLIFT_TEST 43
#define BUTTON_PENLIFT_SAVE_TO_EEPROM 44
#define BUTTON_MACHINE_SPEC 45
#define BUTTON_GXOFFS 50
#define BUTTON_GYOFFS 51
#define BUTTON_GDIV 52
#define BUTTON_MACH_X_OFFS 53
#define BUTTON_MACH_Y_OFFS 54
#define BUTTON_FACTORY_RESET 55

#define mainMenuposX 0
#define mainMenuposY 2
#define lineMenuposX 0
#define lineMenuposY 3 
#define inputX 13
#define inputY  3
#define FileposX 1
#define FileposY 1


const int MENU_INITIAL = 1;
const int MENU_RUNNING = 2;
const int MENU_CHOOSE_FILE = 3;
const int MENU_MOVE_MOTORS = 4;
const int MENU_SETTINGS = 5;
const int MENU_GCODE_SETTINGS = 6;
volatile byte MaxLine[]     = {  0,6, 4, 6, 5, 7, 6};  
volatile byte StartLine[]   = {0,1,7,11,17,22,29,35};
volatile byte MenuLastPos[] = {1,1,1,1,1,1,1,1};

struct Menudef {
   char title[11];  //menu text
   byte function;    //function number  (below 10 -> submenu#)
   };

const struct Menudef Menu[] =
{ 
//------------- INITIAL MENU ------
  {"Main Menu ",0},
//-------------- MAIN MENU (1)-----------------
  {"Position  ",MENU_RUNNING}, 
  {"Draw File ",MENU_CHOOSE_FILE}, 
  {"MotorsMove",MENU_MOVE_MOTORS}, 
  {"Settings  ",MENU_SETTINGS},
  {"Set2-Gcode",MENU_GCODE_SETTINGS}, 
  {"About     ",BUTTON_MACHINE_SPEC},
//--------------- MENU_RUNNING (2)--------  
  {"Set HOME  ",BUTTON_SET_HOME},
  {"Pen Up/Dwn",BUTTON_PEN_UP_DOWN},
  {"ReturnHOME",BUTTON_RETURN_HOME},  
  {"ToggleEcho",BUTTON_TOGGLE_ECHO},
//----------- CHOOSE_FILE (3)---------
  {"ReloadCard",BUTTON_RESET_SD},
  {"ChooseFile",BUTTON_PREV_NEXT_FILE},
  {"CancelFile",BUTTON_CANCEL_FILE},
  {"Start Line",BUTTON_DRAW_START_LINE},
  {"DRAW File!",BUTTON_DRAW_THIS_FILE},
  {"STOP Draw!",BUTTON_STOP_FILE},
  //-------------- MOVE MOTORS (4)-------  
  {"ReturnHOME",BUTTON_RETURN_HOME},
  {"Move A Mot",BUTTON_MOVE_INC_DEC_A},
  {"Move B Mot",BUTTON_MOVE_INC_DEC_B},
  {"Set HOME  ",BUTTON_SET_HOME},
  {"Multiplier",BUTTON_SET_MULTIPLIER},
//  ----------- SETTINGS (5)----
  {"Plift Up  ",BUTTON_INC_DEC_PENLIFT_UP},
  {"Plift Down",BUTTON_INC_DEC_PENLIFT_DOWN},
  {"Plift Test",BUTTON_PENLIFT_TEST},
  {"Pl->EEPROM",BUTTON_PENLIFT_SAVE_TO_EEPROM},
  {"MotorSpeed",BUTTON_INC_DEC_SPEED},
  {"Accelerate",BUTTON_INC_DEC_ACCEL},
  {"SetPenSize",BUTTON_INC_DEC_PENSIZE},
   //-------- GCODE SETTINGS (6)-------
  {"GX Offset ",BUTTON_GXOFFS},
  {"GY Offset ",BUTTON_GYOFFS},
  {"GX Divide ",BUTTON_GDIV},
  {"Mach XOffs",BUTTON_MACH_X_OFFS},
  {"Mach YOffs",BUTTON_MACH_Y_OFFS},
  {"!FactReset",BUTTON_FACTORY_RESET},
};


void TestKeyboard()
{
  Serial.println("Testing Keyboard...");
  while (true)
    {
    GetKey();
    Serial.print(BtnEnter);  Serial.print(BtnExit); Serial.print(BtnLeft); Serial.print(BtnRight);
    Serial.print("/"); Serial.println(RotaryKnob);
    delay(500);
    if ((BtnEnter == 0) && (BtnExit == 0)) return;
    } 
}

void TestRotary()
{
  while (true)
  {
  Serial.println(encoderPos);
  delay(500);
  if (digitalRead(pinC) == 0) return;
  }
}


void InitKeyboard()
{
  Serial.println("Keyboard Init...");
  pinMode(BtnEnter_Pin,INPUT_PULLUP);
  pinMode(BtnExit_Pin,INPUT_PULLUP);
  pinMode(BtnLeft_Pin,INPUT_PULLUP);
  pinMode(BtnRight_Pin,INPUT_PULLUP);
  
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH
  pinMode(pinC, INPUT_PULLUP); // set pinC as an input, pulled HIGH

  attachInterrupt(4,PinA,RISING); // set an interrupt on PinA,
  attachInterrupt(5,PinB,RISING); // set an interrupt on PinB, 
}




boolean GetEnter()
{    return !digitalRead(BtnEnter_Pin); }

boolean GetExit()
{    return !digitalRead(BtnExit_Pin); }

boolean GetLeft()
{   return !digitalRead(BtnLeft_Pin);  }

boolean GetRight()
{    return !digitalRead(BtnRight_Pin); }

boolean GetRotary()
{    return !digitalRead(pinC); }

void GetKey()
{
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
  static boolean oldEnter,oldExit,oldLeft,oldRight; 
     
  btnEnter = !digitalRead(BtnEnter_Pin);
  btnExit = !digitalRead(BtnExit_Pin);
  btnLeft = !digitalRead(BtnLeft_Pin);
  btnRight = !digitalRead(BtnRight_Pin);

  if ((btnEnter != oldEnter) || (btnExit  != oldExit) || (btnLeft  != oldLeft) || (btnRight != oldRight))
        lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) 
    {
     if (btnEnter)   BtnEnter = btnEnter;
     if (btnExit)     BtnExit  = btnExit;
     if (btnLeft)     BtnLeft  = btnLeft;
     if (btnRight)   BtnRight = btnRight;
    }
  oldEnter = btnEnter;
  oldExit = btnExit;
  oldLeft = btnLeft;
  oldRight = btnRight;
}


void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    if (MenuEncPossible) MenuEncoderPos++;
    else                 encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    if (MenuEncPossible) MenuEncoderPos--;
    else                 encoderPos --;     //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}



void lcdGetValue(int fcode)
{
 if (fcode == BUTTON_MOVE_INC_DEC_A) lcd.print(motorA.currentPosition());
 else if (fcode == BUTTON_MOVE_INC_DEC_B) lcd.print(motorB.currentPosition());
 else if (fcode == BUTTON_INC_DEC_PENLIFT_UP) lcd.print(upPosition);
 else if (fcode == BUTTON_INC_DEC_PENLIFT_DOWN) lcd.print(downPosition);
 else if (fcode == BUTTON_INC_DEC_SPEED) lcd.print(currentMaxSpeed);
 else if (fcode == BUTTON_INC_DEC_ACCEL) lcd.print(currentAcceleration);
 else if (fcode == BUTTON_INC_DEC_PENSIZE) lcd.print(penWidth); 
 else if (fcode == BUTTON_GXOFFS) lcd.print(GXoffs); 
 else if (fcode == BUTTON_GYOFFS) lcd.print(GYoffs); 
 else if (fcode == BUTTON_GDIV) lcd.print(Gdiv);   
 else if (fcode == BUTTON_MACH_X_OFFS) lcd.print(MACH_X_offs);  
 else if (fcode == BUTTON_MACH_Y_OFFS) lcd.print(MACH_Y_offs);
 else if (fcode == BUTTON_DRAW_START_LINE) lcd.print(DrawStartLine);
 else if (fcode == BUTTON_SET_MULTIPLIER) lcd.print((int)stepMultiplier);  
 else lcd.print("       ");
}


void lcd_displayFirstMenu()
{
  lcd.clear();
  currentMenu = MENU_INITIAL;
  currentLine = 1;
  currentAbsLine = StartLine[currentMenu]+currentLine-1;
  lcd_updateDisplay(true);
}

void lcd_updateDisplay(boolean ForceRefresh)
{
static int old_currentMenu,old_currentLine;
static long oldcounter;  

  if (commandFileLineCount!=oldcounter) 
    {lcd.setCursor(14,1);
     oldcounter = commandFileLineCount;   
      if (commandFileLineCount == 0)  lcd.print("       ");
      else {lcd.print("C"); lcd.print(commandFileLineCount);}   //No. of Commands executed...
    }
    
if (!ForceRefresh && (old_currentMenu == currentMenu) && (old_currentLine == currentLine)) return;

  if ((millis()-lastOperationTime)>10000 || ForceRefresh)  {
      lcd.setCursor(0,0); lcd.print("Polargraph "); lcd.print(machineName); 
 //     if (echoingStoredCommands) lcd.print("e"); else lcd.print(" ");
  }
  
  lcd.setCursor(mainMenuposX+4,mainMenuposY);   
  lcd.print(Menu[currentMenu-1].title);
  Serial.print("Menu:");   Serial.print(currentMenu);
  Serial.print(" line:");  Serial.print(currentLine);
  Serial.print(" >>");     Serial.println(Menu[currentAbsLine].title);
  lcd.setCursor(mainMenuposX,mainMenuposY); lcd.print(currentMenu);lcd.print("."); lcd.print(currentLine);  //menu reference numbers
  ShowRunning();
  lcd.setCursor(lineMenuposX,lineMenuposY); lcd.print(" >                  "); 
  lcd.setCursor(lineMenuposX+2,lineMenuposY);
  if (Menu[currentAbsLine].function == BUTTON_PEN_UP_DOWN)
      {
        if (isPenUp) lcd.print(" Pen Down "); else lcd.print(" Pen Up   ");
      }
  else   
    lcd.print(Menu[currentAbsLine].title);

// print variables
  lcd.setCursor(inputX,inputY); lcdGetValue(Menu[currentAbsLine].function);
    
  lcd.setCursor(FileposX,FileposY); lcd.print("             ");  
  lcd.setCursor(FileposX,FileposY); lcd.print(commandFilename);  
  
  old_currentMenu = currentMenu; old_currentLine = currentLine;
}

void ShowRunning()
{
  lcd.setCursor(16,2); if (currentlyRunning) lcd.print("RUN "); else lcd.print("STOP");
}

void lcd_checkForInput()
{
  static long LastChecked;
  
  if ((millis()-LastChecked)<50) return;
  LastChecked = millis();
  MenuLastPos[currentMenu] = currentLine;   //Remember last line in current menu
  
  if (GetRotary()) {currentlyRunning = !currentlyRunning; while(GetRotary()); ShowRunning();}   //SUSPEND/RESUME PRINTING!
  if (MenuEncoderPos != 0) {currentLine = constrain(currentLine+MenuEncoderPos,1, MaxLine[currentMenu]); MenuEncoderPos = 0;}
  if (GetExit()) {
                currentMenu = 1; currentLine = MenuLastPos[currentMenu];
                currentAbsLine = StartLine[currentMenu]+currentLine-1;
                pressedButton = Menu[currentAbsLine].function;  
                while(GetExit()); 
                }  //vissza a főmenübe   
  else if (GetLeft())   {currentLine--; while(GetLeft()); }
  else if (GetRight())  {currentLine++; while(GetRight());}

  currentLine = constrain(currentLine,1, MaxLine[currentMenu]);   
  currentAbsLine = StartLine[currentMenu]+currentLine-1;
  pressedButton = Menu[currentAbsLine].function;
  
  if (GetEnter()) 
    {
      while(GetEnter());
    if (pressedButton >10)   //igazi parancs
         {lcd_processCommand();  lcd_updateDisplay(true);  }
    else     //choose a submenu
        {
         currentMenu = pressedButton;
         currentLine = MenuLastPos[currentMenu]; 
         currentAbsLine = StartLine[currentMenu]+currentLine-1;
         pressedButton = Menu[currentAbsLine].function;
         }
    }  
    lcd_updateDisplay(false);
}


int use_encoder(int curVal,int minVal,int maxVal,int incVal)
{
  int actVal,old_actVal;

  actVal = curVal;  old_actVal=-actVal;
  encoderPos = 0; 
  
  while (true)
  {
    delay(100);
    actVal += encoderPos * incVal;
    encoderPos = 0; 
    if (actVal > maxVal) {actVal = maxVal;}
    if (actVal<minVal)   {actVal = minVal;}
    
    if (actVal != old_actVal)
      {
      lcd.setCursor(inputX,inputY);
      lcd.print("<      ");
      lcd.setCursor(inputX+1,inputY);
      lcd.print(actVal); lcd.print(">");
      old_actVal = actVal;
      }
 
    if (GetExit())  {
      while(GetExit());  return curVal; }
    if (GetEnter()) {while(GetEnter()); return actVal; }
  }   
}

void use_encoderFile()
{
  int old_encoderPos=-9999;
  if (NoOfFiles == 0) return;
  
  encoderPos = commandFileNo;
  if (commandFileNo==0) encoderPos = 1;
  lcd.setCursor(FileposX-1,FileposY); lcd.print(">");
  while (true)
   {
    delay(100);
    if (encoderPos > NoOfFiles) encoderPos = NoOfFiles; 
    if (encoderPos<1) encoderPos = 0;

    if (old_encoderPos != encoderPos)
      {    
      lcd.setCursor(FileposX,FileposY); lcd.print("             ");  
      lcd.setCursor(FileposX,FileposY); 
      if (encoderPos>0) lcd.print(Txtfile[encoderPos-1]);
      old_encoderPos = encoderPos;
      }

    if (GetEnter()) {
                    if (encoderPos==0) {commandFilename = ""; commandFileNo = 0;}
                    else               {commandFilename = Txtfile[encoderPos-1];  commandFileNo = encoderPos;}
                    while(GetEnter()); break; 
                    }
                    
    if (GetExit())  {while(GetExit());  break; }                
   } //end while  
   lcd.setCursor(FileposX-1,FileposY); lcd.print(" ");   
}

void CMDMoveMotorA()
{
  encoderPos = 0; 
  while (true)
  {
    if (GetExit())  {while(GetExit()); break; }
    if (GetEnter()) {while(GetEnter()); break; }
     
     if (GetLeft()) 
        {motorA.move(0-100); }  //moveIncrement
     else if (GetRight()) 
        { motorA.move(100); }  //moveIncrement
      else 
        {motorA.move(encoderPos);  encoderPos = 0;} 
      while (motorA.distanceToGo() != 0)  motorA.run();  
      delay(50);
      lcd.setCursor(inputX,inputY); lcd.print("      ");
      lcd.setCursor(inputX,inputY);
      lcd.print("(");  lcd.print(motorA.currentPosition()); lcd.print(")");
  }    
}
void CMDMoveMotorB()
{
  encoderPos = 0; 
  while (true)
  {
    if (GetExit())  {while(GetExit()); break; }
    if (GetEnter()) {while(GetEnter()); break; }
     
     if (GetLeft()) 
        {motorB.move(0-100); }  //moveIncrement
     else if (GetRight()) 
        { motorB.move(100); }  //moveIncrement
      else 
        {motorB.move(encoderPos);  encoderPos = 0;} 
      while (motorB.distanceToGo() != 0)  motorB.run();  
      delay(50);
      lcd.setCursor(inputX,inputY); lcd.print("      ");
      lcd.setCursor(inputX,inputY);
      lcd.print("(");  lcd.print(motorB.currentPosition()); lcd.print(")");
  }    
}

void SetHomeMotors()
{
      motorA.setCurrentPosition(homeA*stepMultiplier);
      motorB.setCurrentPosition(homeB*stepMultiplier);
      engageMotors();
      reportPosition();
}      

void ReturnHomeMotors()
{
     changeLength((float)stepMultiplier*homeA,(float)stepMultiplier*homeB);
}

void PenLiftUpSetup()
{
  encoderPos = 0; 
  while (true)
  {
    if (GetExit())  {while(GetExit());   break; }
    if (GetEnter()) {while(GetEnter());  break; }
    if (upPosition < 300) {
        upPosition += encoderPos;
        if (isPenUp)
          penlift_movePen(upPosition-15, upPosition, penLiftSpeed);
        else
          penlift_movePen(downPosition, upPosition, penLiftSpeed);
        isPenUp = true;
      }
    if (upPosition > 0) {
        upPosition -= encoderPos;  encoderPos = 0;
        if (isPenUp)
          penlift_movePen(upPosition+10, upPosition, penLiftSpeed);
        else
          penlift_movePen(downPosition, upPosition, penLiftSpeed);
        isPenUp = true;
        }    
    lcd.setCursor(inputX,inputY); 
    lcd.print("(   )"); 
    lcd.setCursor(inputX+1,inputY); 
    lcd.print(upPosition);
    delay(200);
  }    
}

void PenLiftDownSetup()
{
  encoderPos = 0; 
  while (true)
  {
    if (GetExit())  {while(GetExit());   break; }
    if (GetEnter()) {while(GetEnter());  break; }
    if (downPosition < 300) {
        downPosition += encoderPos; encoderPos = 0;
        if (isPenUp)
          penlift_movePen(upPosition, downPosition, penLiftSpeed);
        else
          penlift_movePen(downPosition-10, downPosition, penLiftSpeed);
        isPenUp = false;
      }
    if (downPosition > 0) {
        downPosition -= encoderPos; encoderPos = 0;
        if (isPenUp)
          penlift_movePen(upPosition, downPosition, penLiftSpeed);
        else
          penlift_movePen(downPosition+15, downPosition, penLiftSpeed);
        isPenUp = false;
      }  
    lcd.setCursor(inputX,inputY); 
    lcd.print("(   )"); 
    lcd.setCursor(inputX+1,inputY); 
    lcd.print(downPosition);
    delay(200);
  }    
}

void lcd_processCommand()
{
  int oldstepmulti = 0;
  if (pressedButton ==0) return;
  MenuEncPossible = false;
  Serial.print("button:");
  Serial.println(pressedButton);
  switch (pressedButton)
  {
    case BUTTON_MACHINE_SPEC:
      lcd_displayMachineSpec();
      break;
    case BUTTON_POWER_ON_OFF:
      lcd_runStartScript();
//      else lcd_runEndScript();
      break;
    case BUTTON_RESET:
      break;
    case BUTTON_SET_HOME:      
      SetHomeMotors();
      break;
    case BUTTON_RETURN_HOME:      
      ReturnHomeMotors();
      break;  
    case BUTTON_PEN_UP_DOWN:
      inNoOfParams=0;
      if (isPenUp) penlift_penDown(); else penlift_penUp();
      break;
    case BUTTON_INC_DEC_SPEED:
      currentMaxSpeed = use_encoder(currentMaxSpeed,500,2500,speedChangeIncrement);
      exec_setMotorSpeed(currentMaxSpeed);
      EEPROM_writeAnything(EEPROM_MACHINE_MOTOR_SPEED, currentMaxSpeed);
      break;
    case BUTTON_INC_DEC_ACCEL:
      currentAcceleration = use_encoder(currentAcceleration,0,2500,accelChangeIncrement);
      exec_setMotorAcceleration(currentAcceleration);
      EEPROM_writeAnything(EEPROM_MACHINE_MOTOR_ACCEL, currentAcceleration);
      break;
    case BUTTON_SET_MULTIPLIER:
      oldstepmulti = stepMultiplier;
      stepMultiplier = (byte)use_encoder((int)stepMultiplier,1,16,1);
      if (stepMultiplier != oldstepmulti)
        { 
        EEPROM_writeAnything(EEPROM_MACHINE_STEP_MULTIPLIER, stepMultiplier);
        eeprom_loadMachineSpecFromEeprom();
        }
//      break;
    case BUTTON_INC_DEC_PENSIZE:
      penWidth = (float)use_encoder(penWidth*1000,penWidthIncrement*1000,25000,penWidthIncrement*1000) /1000;
      break;
    case BUTTON_TOGGLE_ECHO:
      echoingStoredCommands = !echoingStoredCommands;
      delay(500);
      break;
    case BUTTON_RESET_SD:
      sd_simpleInit();
      if (cardInit) 
           { LoadFilenameArray();  }
      else
          commandFilename = "";     
      break;  
    case BUTTON_DRAW_THIS_FILE:
      if (commandFilename != "None" && commandFilename != "" && commandFilename != "            ")
      {
        Serial.print("Drawing this file: ");
        Serial.println(commandFilename);
        currentlyDrawingFromFile = true;
        commandFileLineCount = 0;
        currentLine++;    //Set menu to STOP FILE!
        currentAbsLine = StartLine[currentMenu]+currentLine-1;
        pressedButton = Menu[currentAbsLine].function;
        lcd_updateDisplay(true);
        impl_exec_execFromStore(commandFilename);
        ReturnHomeMotors();
      } 
      break;
    case BUTTON_STOP_FILE:
      Serial.print("Cancelling drawing this file: ");
      Serial.println(commandFilename);
      currentlyDrawingFromFile = false;
      lcd_updateDisplay(true);
      break;  
    case BUTTON_PREV_NEXT_FILE:
      use_encoderFile();
      break;
    case BUTTON_CANCEL_FILE:
      // return to main menu
      commandFilename = "";
      commandFileNo = 0;
      commandFileLineCount = 0;
      break;
    case BUTTON_DRAW_START_LINE:
       DrawStartLine = use_encoder(DrawStartLine,0,32000,1);
       break;
    case BUTTON_MOVE_INC_DEC_A:
       CMDMoveMotorA();
       break;
     case BUTTON_MOVE_INC_DEC_B:
       CMDMoveMotorB();
       break;
    case BUTTON_CALIBRATE:
//      calibrate_doCalibration();
       break;
    case BUTTON_INC_DEC_PENLIFT_UP:
      PenLiftUpSetup();
      break;
    case BUTTON_INC_DEC_PENLIFT_DOWN:
      PenLiftDownSetup();
      break;
    case BUTTON_PENLIFT_TEST:
      penlift_testRange();
      break;  
    case BUTTON_PENLIFT_SAVE_TO_EEPROM:
      Serial.println("HJey");
      EEPROM_writeAnything(EEPROM_PENLIFT_DOWN, downPosition);
      EEPROM_writeAnything(EEPROM_PENLIFT_UP, upPosition);
      eeprom_loadPenLiftRange();
      lcd.setCursor(inputX,inputY); lcd.print("SAVED!");
      delay(1000);
      break;
    case BUTTON_GXOFFS:
      GXoffs = use_encoder((int)GXoffs,0,1000,1);
      EEPROM_writeAnything(EEPROM_GXOFFS, (float)GXoffs);
      break;   
    case BUTTON_GYOFFS:
      GYoffs = use_encoder((int)GYoffs,0,1000,1);
      EEPROM_writeAnything(EEPROM_GYOFFS, (float)GYoffs);
      break;   
    case BUTTON_GDIV:
      Gdiv = (float)use_encoder((int)(Gdiv*100),5,200,1) /100;
      EEPROM_writeAnything(EEPROM_GDIV,(float)Gdiv);
      break;   
     case BUTTON_MACH_X_OFFS:
      MACH_X_offs = use_encoder((int)MACH_X_offs,0,1000,1);
      EEPROM_writeAnything(EEPROM_MACH_X_OFFS, (float)MACH_X_offs);
      break;   
    case BUTTON_MACH_Y_OFFS:
      MACH_Y_offs = use_encoder((int)MACH_Y_offs,0,1000,1);
      EEPROM_writeAnything(EEPROM_MACH_Y_OFFS, (float)MACH_Y_offs);
      break;    
    case BUTTON_FACTORY_RESET:
      EEPROM_writeAnything(EEPROM_FACTORY_RESET,(byte)66);    //magical value to make a factory reset next startup
      software_Reset();
      break;
  }
    MenuEncPossible = true;
}

void lcd_runStartScript()
{
  impl_engageMotors();
  motorA.setCurrentPosition(startLengthStepsA);
  motorB.setCurrentPosition(startLengthStepsB);  
  Serial.println("finished start.");
}
void lcd_runEndScript()
{
  return;
  penlift_penUp();
  impl_releaseMotors();
  isCalibrated = false;
}

void lcd_setCurrentMenu(byte menu)
{
  currentMenu = menu;
}



/*
This intialises the LCD itself, and builds the map of the button corner coordinates.
*/
void lcd_initLCD()
{
  lcd.clear();
  lcd.print("Polargraph.");
  lcd.setCursor(0,1);
  lcd.print("Drawing with robots.");
  lcd.setCursor(0,2);
  lcd.print("version "); lcd.print(FIRMWARE_VERSION_NO);
  delay(500);
}

void lcd_showSummary()
{
  lcd.clear();
  lcd.setCursor(0,1);
      
  if (cardPresent) {
    lcd.print("SD card present");
    if (cardInit) 
    {
      lcd.print(", OK!");
      if (useAutoStartFromSD) {
        if (autoStartFileFound) {lcd.print("RUN "); lcd.print(autoStartFilename); }
        else {lcd.print("NO "); lcd.print(autoStartFilename); }
      } 
      else 
           lcd.print("Card loaded.");
    } //endif cardInit
    else
        {  lcd.print("Card init failed!");}
  }  //endif cardPresent
  else 
   {  lcd.print("No SD card present");}
      
}

 
void lcd_displayMachineSpec()
{
  int old_encoderPos = 1;
  encoderPos = 1;
  while(true)
  {
  lcd.clear();
  if (encoderPos == 1)
  {
  lcd.print("MachineName:"); lcd.print(machineName);
  lcd.setCursor(0,2); lcd.print("Use ROTARY or <>");
  }
  
  if (encoderPos == 2)
  {
  lcd.print("Machine  Width:"); lcd.print(machineWidth);
  lcd.setCursor(0,1);
  lcd.print("Machine Height:"); lcd.print(machineHeight);
  lcd.setCursor(0,2);
  lcd.print("MmPerRev:");  lcd.print(mmPerRev);
  lcd.setCursor(0,3);
  lcd.print("StepsPerRev:");  lcd.print(motorStepsPerRev);
  }
  
  if (encoderPos == 3)
  {  
  lcd.print("MmPerStep :");  lcd.print(mmPerStep);
  lcd.setCursor(0,1);
  lcd.print("StepsPerMM:"); lcd.print(stepsPerMM);
  lcd.setCursor(0,2);
  lcd.print("StepMulti :");  lcd.print((int)stepMultiplier);
  lcd.setCursor(0,3);
  lcd.print("HomeA"); lcd.print(homeA); lcd.print("B"); lcd.print(homeB);
  }
  
  if (encoderPos == 4)
  {
  lcd.print("Page  Width:");  lcd.print(pageWidth);
  lcd.setCursor(0,1);
  lcd.print("Page Height:"); lcd.print(pageHeight);
  lcd.setCursor(0,2);
  lcd.print("Autostart File:");  
  lcd.setCursor(5,3);
  lcd.print(autoStartFilename);
  }
  
  if (encoderPos == 5)
  {
  lcd.print("OffsX");   lcd.print(GXoffs);            lcd.print(" Y"); lcd.print(GYoffs);
  lcd.setCursor(0,1);   lcd.print("Gcode divider:");  lcd.print(Gdiv);
  lcd.setCursor(0,2);   lcd.print("MACH_X_offset:");  lcd.print(MACH_X_offs);
  lcd.setCursor(0,3);   lcd.print("MACH_Y_offset:");  lcd.print(MACH_Y_offs);
  }
 
  if (encoderPos == 6)
      lcd_showSummary();
      
  if (encoderPos == 7)
  {
  lcd.print("Original code:");    
  lcd.setCursor(0,1);
  lcd.print("www.polargraph.co.uk");
  lcd.setCursor(0,2);
  lcd.print("MOD by Peter Gautier");
  lcd.setCursor(0,3);
  lcd.print("        dec 23, 2017");
  }
  while (encoderPos == old_encoderPos)
     {    
     if (GetExit()) {while(GetExit()); lcd.clear(); lcd_updateDisplay(true); return;}
     if (GetLeft())  { encoderPos--;  while(GetLeft()); }
     if (GetRight()) { encoderPos++;  while(GetRight()); }
     }
  if (encoderPos>6) encoderPos = 7;
  if (encoderPos<1) encoderPos = 1;
  old_encoderPos = encoderPos;
  } //end while
}



void LoadFilenameArray()
{
  
  NoOfFiles = 0; commandFileNo = 0;   commandFilename = "";
  root = SD.open("/", FILE_READ);
  root.rewindDirectory();
  Serial.println("Loading directory to Txtfile[]");

 while(true) {
     File entry =  root.openNextFile();
     if (! entry || (NoOfFiles>=MaxNoOfFiles)) {
       // no more files
       Serial.println("...end of directory.");
       break;
     }
      if (!entry.isDirectory())  {
       // files have sizes, directories do not
       strncpy(Txtfile[NoOfFiles],entry.name(),13);
       NoOfFiles++;
       Serial.print(NoOfFiles); Serial.print(" ");
       Serial.print(entry.name());
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }  //end while
  if (NoOfFiles>0) commandFilename = Txtfile[0];
  Serial.print("Now command filename: ");
  Serial.println(commandFilename);
}  

/*
void lcd_echoLastCommandToDisplay(String com, String prefix)
{
//  if (currentMenu != MENU_INITIAL) return;
  lcd.setCursor(0,0);
//  lcd.print(prefix + com);
}
*/

void LCD_Motors()
{
  return;
  lcd.setCursor(0,1); lcd.print(cleanline);
  lcd.setCursor(0,1);  lcd.print("A:"); lcd.print(motorA.currentPosition());
  lcd.setCursor(8,1);   lcd.print("B:"); lcd.print(motorB.currentPosition()); 
//  lcd.setCursor(0,3);
//  lcd.print(cleanline);
//  lcd.setCursor(0,3);
//  lcd.print("X");   lcd.print(float((long)getCartesianX())*mmPerStep); lcd.print("mm");
//  lcd.print(" Y");  lcd.print(float((long)getCartesianY())*mmPerStep); lcd.print("mm");
}

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
asm volatile ("  jmp 0");  
}  
