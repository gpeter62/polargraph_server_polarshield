/**
*  Polargraph Server for ATMEGA1280+ based arduino boards.
*  Written by Sandy Noble
*  Released under GNU License version 3.
*  http://www.polargraph.co.uk
*  https://github.com/euphy/polargraph_server_polarshield
*  
* Modified by Peter Gautier 2017.12.19
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

Extended command set: G1,G21,G90
Offset and size% in case of external GCode source
*/

#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"

#define SERIAL_STEPPER_DRIVERS 


// 4.  Turn on some debugging code if you want horror
// =================================================
#define DEBUG
#define DEBUG_COMMS
//#define DEBUG_PENLIFT
//#define DEBUG_PIXEL

boolean debugComms = false;

/*  ===========================================================  
    These variables are common to all polargraph server builds
=========================================================== */    

// ==========================================================

const String FIRMWARE_VERSION_NO = "1.4.4";
const String MB_NAME = "TFTSHIELD";

//  EEPROM addresses
const int EEPROM_MACHINE_WIDTH = 0;
const int EEPROM_MACHINE_HEIGHT = 2;
const int EEPROM_MACHINE_NAME = 4;
const int EEPROM_MACHINE_MM_PER_REV = 14; // 4 bytes (float)
const int EEPROM_MACHINE_STEPS_PER_REV = 18;
const int EEPROM_MACHINE_STEP_MULTIPLIER = 20;

const int EEPROM_MACHINE_MOTOR_SPEED = 22; // 4 bytes float
const int EEPROM_MACHINE_MOTOR_ACCEL = 26; // 4 bytes float
const int EEPROM_MACHINE_PEN_WIDTH = 30; // 4 bytes float

const int EEPROM_MACHINE_HOME_A = 34; // 4 bytes
const int EEPROM_MACHINE_HOME_B = 38; // 4 bytes

const int EEPROM_PENLIFT_DOWN = 42; // 2 bytes
const int EEPROM_PENLIFT_UP = 44; // 2 bytes

const int  EEPROM_GXOFFS = 48;    // 4 bytes float
const int  EEPROM_GYOFFS = 52;    // 4 bytes float
const int  EEPROM_GDIV = 56;        // 4 bytes float
const int  EEPROM_MACH_X_OFFS = 60;  // 4 bytes float
const int  EEPROM_MACH_Y_OFFS = 64;  // 4 bytes float
const int  EEPROM_FACTORY_RESET = 65;  // 1 byte
const int  EEPROM_END = 70;

// Pen raising servo
Servo penHeight;
const int DEFAULT_DOWN_POSITION = 50;
const int DEFAULT_UP_POSITION = 80;
static int upPosition = DEFAULT_UP_POSITION; // defaults
static int downPosition = DEFAULT_DOWN_POSITION;
static int penLiftSpeed = 10; // ms between steps of moving motor
const byte PEN_HEIGHT_SERVO_PIN = A3; //UNL2003 driver uses pin 9

boolean isPenUp = false;

static int motorStepsPerRev = 200;  //200 lépés/360fok
static float mmPerRev = 48;         //24 fogú tárcsa * 2mm/fog
static byte stepMultiplier = 2;

static float translateX = 0.0;
static float translateY = 0.0;
static float scaleX = 1.0;
static float scaleY = 1.0;
static int rotateTransform = 0;

static int machineWidth = 936;
static int machineHeight = 950;
static int sqtest = 0;

// Machine specification defaults
const int defaultMachineWidth = 936;
const int defaultMachineHeight = 950;
const int defaultMmPerRev = 48;       //24 fogú tárcsa * 2mm/fog
const int defaultStepsPerRev = 200;   //200 lépés/360fok
const int defaultStepMultiplier = 2;

const float homeA = 2191.0; //1234
const float homeB = 2191.0;

static long startLengthStepsA = 1000;
static long startLengthStepsB = 1000;

String machineName = "";
const String DEFAULT_MACHINE_NAME = "GP180222";  //maximum 10char

static float currentMaxSpeed = 1500.0;
static float currentAcceleration = 1500.0;
static boolean usingAcceleration = true;

float mmPerStep = 0.0F;
float stepsPerMM = 0.0F;

long pageWidth = machineWidth * stepsPerMM;
long pageHeight = machineHeight * stepsPerMM;
long maxLength = 0;

//static char rowAxis = 'A';
const int INLENGTH = 50;
const char INTERMINATOR = 10;
const char SEMICOLON = ';';
const char cleanline[] = "                    ";

float penWidth = 0.4F; // line width in mm

boolean reportingPosition = true;
boolean acceleration = true;

extern AccelStepper motorA;
extern AccelStepper motorB;

volatile boolean currentlyRunning = true;

static char inCmd[10];
static char inParam1[14];
static char inParam2[14];
static char inParam3[14];
static char inParam4[14];

static byte inNoOfParams;

char lastCommand[INLENGTH+1];
boolean commandConfirmed = false;

int rebroadcastReadyInterval = 5000L;
long lastOperationTime = 0L;
long motorIdleTimeBeforePowerDown = 600000L;
boolean automaticPowerDown = true;

long lastInteractionTime = 0L;

static boolean lastWaveWasTop = true;
static boolean lastMotorBiasWasA = true;    //not used

//  Drawing direction
const static byte DIR_NE = 1;
const static byte DIR_SE = 2;
const static byte DIR_SW = 3;
const static byte DIR_NW = 4;

const static byte DIR_N = 5;
const static byte DIR_E = 6;
const static byte DIR_S = 7;
const static byte DIR_W = 8;
static int globalDrawDirection = DIR_NW;

const static byte DIR_MODE_AUTO = 1;
const static byte DIR_MODE_PRESET = 2;
const static byte DIR_MODE_RANDOM = 3;
static int globalDrawDirectionMode = DIR_MODE_AUTO;


const String READY_STR = "READY_200";
const String RESEND_STR = "RESEND";
const String DRAWING_STR = "DRAWING";
const static String OUT_CMD_CARTESIAN_STR = "CARTESIAN,";
const static String OUT_CMD_SYNC_STR = "SYNC,";

char MSG_E_STR[] = "MSG,E,";
char MSG_I_STR[] = "MSG,I,";
char MSG_D_STR[] = "MSG,D,";

static boolean pixelDebug = false;

static const byte ALONG_A_AXIS = 0;
static const byte ALONG_B_AXIS = 1;
static const byte SQUARE_SHAPE = 0;
static const byte SAW_SHAPE = 1;

const static char COMMA[] = ",";
const static char CMD_END[] = ",END";
const static String CMD_CHANGELENGTH = "C01";
const static String CMD_CHANGEPENWIDTH = "C02";
const static String CMD_CHANGEMOTORSPEED = "C03";
const static String CMD_CHANGEMOTORACCEL = "C04";
const static String CMD_DRAWPIXEL = "C05";
const static String CMD_DRAWSCRIBBLEPIXEL = "C06";
const static String CMD_CHANGEDRAWINGDIRECTION = "C08";
const static String CMD_SETPOSITION = "C09";
const static String CMD_TESTPATTERN = "C10";
const static String CMD_TESTPENWIDTHSQUARE = "C11";
const static String CMD_PENDOWN = "C13";
const static String CMD_PENUP = "C14";
const static String CMD_CHANGELENGTHDIRECT = "C17";
const static String CMD_SETMACHINESIZE = "C24";
const static String CMD_SETMACHINENAME = "C25";
const static String CMD_GETMACHINEDETAILS = "C26";
const static String CMD_RESETEEPROM = "C27";
const static String CMD_SETMACHINEMMPERREV = "C29";
const static String CMD_SETMACHINESTEPSPERREV = "C30";
const static String CMD_SETMOTORSPEED = "C31";
const static String CMD_SETMOTORACCEL = "C32";
const static String CMD_SETMACHINESTEPMULTIPLIER = "C37";
const static String CMD_SETPENLIFTRANGE = "C45";
const static String CMD_PIXELDIAGNOSTIC = "C46";
const static String CMD_SET_DEBUGCOMMS = "C47";


//GP kiegészítés
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);     //LCD pins from 4>9
const static String CMD_G1 = "G1";
const static String CMD_G21 = "G21";
const static String CMD_G90 = "G90";
const static String CMD_G99 = "G99";

//GP koordináta átszámolás a GCode generáló programból
float GXoffs = 190;    //X eltolás a GCode-ban középről a bal felső sarok lesz a 0 pont
float GYoffs = 0;    //Y eltolás
float Gdiv = 1.0;        //nagyítás  (A GCode referencia gép sokkal nagyobb
float MACH_X_offs = 280;  //a konkrét gépnél itt van a papír bal felső sarka, mm-ben
float MACH_Y_offs = 290;  //a konkrét gépnél itt van a papír bal felső sarka, mm-ben
long DrawStartLine = 0;

void setup() 
{
  machineName = DEFAULT_MACHINE_NAME;
  byte factory_reset = 0;
  Serial.begin(57600);           // set up Serial library at 57600 bps
  Serial.println("POLARGRAPH ON!");
  Serial.print(F("v"));
  Serial.println(FIRMWARE_VERSION_NO);
  Serial.print(F("Hardware: "));
  Serial.println(MB_NAME);
  Serial.print(F("Servo "));
  Serial.println(PEN_HEIGHT_SERVO_PIN);

   lcd.begin(20,4);

   mmPerStep = mmPerRev / multiplier(motorStepsPerRev);
   stepsPerMM = multiplier(motorStepsPerRev) / mmPerRev;
   
   EEPROM_readAnything(EEPROM_FACTORY_RESET, factory_reset);
   if (factory_reset==66)
         eeprom_save_factory_defaults();
         
  configuration_motorSetup();    
  eeprom_loadMachineSpecFromEeprom();
  configuration_setup();

  motorA.setMaxSpeed(currentMaxSpeed);
  motorA.setAcceleration(currentAcceleration);  
  motorB.setMaxSpeed(currentMaxSpeed);
  motorB.setAcceleration(currentAcceleration);

  motorA.setCurrentPosition(startLengthStepsA);
  motorB.setCurrentPosition(startLengthStepsB);

  for (int i = 0; i<INLENGTH; i++) {
    lastCommand[i] = 0;
  }    
  comms_ready();
  
  pinMode(PEN_HEIGHT_SERVO_PIN, OUTPUT);
  delay(500);
  penlift_penUp();
  SetHomeMotors();
  InitKeyboard();
//eeprom_resetEeprom();
//TestKeyboard();
//TestRotary();
  sd_autorunSD();
}

void loop()
{
  if (comms_waitForNextCommand(lastCommand)) 
  {
#ifdef DEBUG_COMMS    
    Serial.print(F("Last comm: "));
    Serial.print(lastCommand);
    Serial.println(F("..."));
#endif
    comms_parseAndExecuteCommand(lastCommand);
  }
}


const static String CMD_TESTPENWIDTHSCRIBBLE = "C12";
const static String CMD_DRAWSAWPIXEL = "C15,";
const static String CMD_DRAWCIRCLEPIXEL = "C16";
const static String CMD_SET_ROVE_AREA = "C21";
const static String CMD_DRAWDIRECTIONTEST = "C28";
const static String CMD_MODE_STORE_COMMANDS = "C33";
const static String CMD_MODE_EXEC_FROM_STORE = "C34";
const static String CMD_MODE_LIVE = "C35";
const static String CMD_RANDOM_DRAW = "C36";
const static String CMD_START_TEXT = "C38";
const static String CMD_DRAW_SPRITE = "C39";
const static String CMD_CHANGELENGTH_RELATIVE = "C40";
const static String CMD_SWIRLING = "C41";
const static String CMD_DRAW_RANDOM_SPRITE = "C42";
const static String CMD_DRAW_NORWEGIAN = "C43";
const static String CMD_DRAW_NORWEGIAN_OUTLINE = "C44";
const static String CMD_AUTO_CALIBRATE = "C48";

/*  End stop pin definitions  */
const int ENDSTOP_X_MAX = 40;
const int ENDSTOP_X_MIN = 41;
const int ENDSTOP_Y_MAX = 42;
const int ENDSTOP_Y_MIN = 43;

long ENDSTOP_X_MIN_POSITION = 130;
long ENDSTOP_Y_MIN_POSITION = 130;

long motorARestPoint = 0;
long motorBRestPoint = 0;

// size and location of rove area
long rove1x = 1000;
long rove1y = 1000;
long roveWidth = 4500;
long roveHeight = 6000;

boolean swirling = false;
String spritePrefix = "";
int textRowSize = 200;
int textCharSize = 180;

boolean useRoveArea = false;

int commandNo = 0;
int errorInjection = 0;

boolean storeCommands = false;
boolean drawFromStore = false;
String commandFilename = "";
long commandFileLineCount = 0;
#define MAXNOOFFILES 15
int NoOfFiles = 0;
int MaxNoOfFiles = MAXNOOFFILES;    //--> Set Txtfile[MaxNoOfFiles][] according this value
int commandFileNo = 0;
char Txtfile [MAXNOOFFILES+1][13]; 

// sd card stuff
const int chipSelect = 53;
boolean sdCardInit = false;

// set up variables using the SD utility library functions:
File root;
boolean cardPresent = false;
boolean cardInit = false;
boolean echoingStoredCommands = true;

// the file itself
File pbmFile;

// information we extract about the bitmap file
long pbmWidth, pbmHeight;
float pbmScaling = 1.0;
int pbmDepth, pbmImageoffset;
long pbmFileLength = 0;
float pbmAspectRatio = 1.0;

volatile int speedChangeIncrement = 100;           //100 volt
volatile int accelChangeIncrement = 100;           //100 volt
volatile float penWidthIncrement = 0.05;
volatile int moveIncrement = 400;               //400 volt

boolean currentlyDrawingFromFile = false;
String currentlyDrawingFilename = "";
boolean powerIsOn = false;
boolean isCalibrated = false;

boolean canCalibrate = false;

boolean useAutoStartFromSD = true;
String autoStartFilename = "AUTORUN.TXT";
boolean autoStartFileFound = false;

