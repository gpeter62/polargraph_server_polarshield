/**
*  Polargraph Server. - CORE
*  Written by Sandy Noble
*  Released under GNU License version 3.
*  http://www.polargraph.co.uk
*  https://github.com/euphy/polargraph_server_polarshield

Exec.

This is one of the core files for the polargraph server program.  
Purposes are getting a little more blurred here.  This file contains
the basic decision tree that branches based on command.

It has a set of the most general-purpose drawing commands, but only
methods that are directly called - none of the geometry or conversion
routines are here.

*/
/**  This method looks only for the basic command set
*/
boolean exec_executeBasicCommand(char* com) {
  boolean executed = true;
  //Serial.print("ExecuteBasic proc:"); Serial.println(com);
  
  commandFileLineCount++;
   if (memcmp(com,CMD_G1,2)==0)  {
          if (memcmp(inParam1,"Z0",2) == 0)  {delay(500); inNoOfParams=0; penlift_penUp();}
          else if (memcmp(inParam1,"Z1",2) == 0)  {delay(500); inNoOfParams=0; penlift_penDown();}
          else exec_G1();
        }  
  else if (memcmp(com,CMD_CHANGELENGTH,3)==0)
    exec_changeLength();
  else if (memcmp(com,CMD_CHANGELENGTHDIRECT,3)==0)
    exec_changeLengthDirect();
  else if (memcmp(com,CMD_CHANGEPENWIDTH,3)==0)
    exec_changePenWidth();
  else if (memcmp(com,CMD_SETMOTORSPEED,3)==0)
    exec_setMotorSpeed();
  else if (memcmp(com,CMD_SETMOTORACCEL,3)==0)
    exec_setMotorAcceleration();
  else if (memcmp(com,CMD_DRAWPIXEL,3)==0)
    pixel_drawSquarePixel();
  else if (memcmp(com,CMD_DRAWSCRIBBLEPIXEL,3)==0)
    pixel_drawScribblePixel();
  else if (memcmp(com,CMD_CHANGEDRAWINGDIRECTION,3)==0)
    exec_changeDrawingDirection();
  else if (memcmp(com,CMD_SETPOSITION,3)==0)
    exec_setPosition();
  else if (memcmp(com,CMD_TESTPENWIDTHSQUARE,3)==0)
    pixel_testPenWidth();
  else if (memcmp(com,CMD_PENDOWN,3)==0)
    penlift_penDown();
  else if (memcmp(com,CMD_PENUP,3)==0)
    penlift_penUp();
  else if (memcmp(com,CMD_SETMACHINESIZE,3)==0)
    exec_setMachineSizeFromCommand();
  else if (memcmp(com,CMD_SETMACHINENAME,3)==0)
    exec_setMachineNameFromCommand();
  else if (memcmp(com,CMD_SETMACHINEMMPERREV,3)==0)
    exec_setMachineMmPerRevFromCommand();
  else if (memcmp(com,CMD_SETMACHINESTEPSPERREV,3)==0)
    exec_setMachineStepsPerRevFromCommand();
  else if (memcmp(com,CMD_SETMACHINESTEPMULTIPLIER,3)==0)
    exec_setMachineStepMultiplierFromCommand();
  else if (memcmp(com,CMD_SETPENLIFTRANGE,3)==0)
    exec_setPenLiftRange();
  else if (memcmp(com,CMD_GETMACHINEDETAILS,3)==0)
    exec_reportMachineSpec();
  else if (memcmp(com,CMD_RESETEEPROM,3)==0)
    eeprom_resetEeprom();
// by GP    
  else if (memcmp(com,CMD_G21,3)==0)
    exec_G21();
  else if (memcmp(com,CMD_G90,3)==0)
    exec_G90();      
  else if (memcmp(com,CMD_G99,3)==0) 
    Serial.println(com);  
  else
    executed = false;
    
  LCD_Motors();  //by GP
  return executed;
}

void exec_changeDrawingDirection() 
{
  globalDrawDirectionMode = atoi(inParam1);
  globalDrawDirection = atoi(inParam2);
//  Serial.print(F("Changed draw direction mode to be "));
//  Serial.print(globalDrawDirectionMode);
//  Serial.print(F(" and direction is "));
//  Serial.println(globalDrawDirection);
}


void exec_reportMachineSpec()
{
  eeprom_dumpEeprom();
  Serial.print(F("PGNAME,"));
  Serial.print(machineName);
  Serial.println(CMD_END);
  
  Serial.print(F("PGSIZE,"));
  Serial.print(machineWidth);
  Serial.print(COMMA);
  Serial.print(machineHeight);
  Serial.println(CMD_END);

  Serial.print(F("PGMMPERREV,"));
  Serial.print(mmPerRev);
  Serial.println(CMD_END);

  Serial.print(F("PGSTEPSPERREV,"));
  Serial.print(motorStepsPerRev);
  Serial.println(CMD_END);
  
  Serial.print(F("PGSTEPMULTIPLIER,"));
  Serial.print(stepMultiplier);
  Serial.println(CMD_END);

  Serial.print(F("PGLIFT,"));
  Serial.print(downPosition);
  Serial.print(COMMA);
  Serial.print(upPosition);
  Serial.println(CMD_END);

  Serial.print(F("PGSPEED,"));
  Serial.print(currentMaxSpeed);
  Serial.print(COMMA);
  Serial.print(currentAcceleration);
  Serial.println(CMD_END);

}

void exec_setMachineSizeFromCommand()
{
  int width = atoi(inParam1);
  int height = atoi(inParam2);

  // load to get current settings
  int currentValue = width;
  EEPROM_readAnything(EEPROM_MACHINE_WIDTH, currentValue);  
  if (currentValue != width)
    if (width > 10)
    {
      EEPROM_writeAnything(EEPROM_MACHINE_WIDTH, width);
    }
  
  EEPROM_readAnything(EEPROM_MACHINE_HEIGHT, currentValue);
  if (currentValue != height)
    if (height > 10)
    {
      EEPROM_writeAnything(EEPROM_MACHINE_HEIGHT, height);
    }

  // reload 
  eeprom_loadMachineSize();
}


void exec_setMachineNameFromCommand()
{
  String name = inParam1;
  if (name != DEFAULT_MACHINE_NAME)
  {
    for (int i = 0; i < 8; i++)    {
      EEPROM.write(EEPROM_MACHINE_NAME+i, name[i]);
    }
  }
  eeprom_loadMachineSpecFromEeprom();
}

void exec_setMachineMmPerRevFromCommand()
{
  float newMmPerRev = atof(inParam1);
  eeprom_loadMachineSpecFromEeprom();
  if (mmPerRev != newMmPerRev) {
    EEPROM_writeAnything(EEPROM_MACHINE_MM_PER_REV, newMmPerRev);
    eeprom_loadMachineSpecFromEeprom();
  } 
}
void exec_setMachineStepsPerRevFromCommand()
{
  int newStepsPerRev = atoi(inParam1);
  eeprom_loadMachineSpecFromEeprom();
  if (motorStepsPerRev != newStepsPerRev) {
    EEPROM_writeAnything(EEPROM_MACHINE_STEPS_PER_REV, newStepsPerRev);
    eeprom_loadMachineSpecFromEeprom();
  }
}
void exec_setMachineStepMultiplierFromCommand()
{
  int newStepMultiplier = atoi(inParam1);
  eeprom_loadMachineSpecFromEeprom();
  if (stepMultiplier != newStepMultiplier) {
    EEPROM_writeAnything(EEPROM_MACHINE_STEP_MULTIPLIER, newStepMultiplier);
    eeprom_loadMachineSpecFromEeprom();
  }
}

void exec_setPenLiftRange()
{
  int down = atoi(inParam1);
  int up = atoi(inParam2);
  
  Serial.print(F("Down: "));
  Serial.println(down);
  Serial.print(F("Up: "));
  Serial.println(up);
  
  if (inNoOfParams == 3) 
  {
    // 4 params (C45,<downpos>,<uppos>,1,END) means save values to EEPROM
    EEPROM_writeAnything(EEPROM_PENLIFT_DOWN, down);
    EEPROM_writeAnything(EEPROM_PENLIFT_UP, up);
    eeprom_loadPenLiftRange();
  }
  else if (inNoOfParams == 2)
  {
    // 3 params (C45,<downpos>,<uppos>,END) means just do a range test
    penlift_movePen(up, down, penLiftSpeed);
    delay(200);
    penlift_movePen(down, up, penLiftSpeed);
    delay(200);
    penlift_movePen(up, down, penLiftSpeed);
    delay(200);
    penlift_movePen(down, up, penLiftSpeed);
    delay(200);
  }
}

/* Single parameter to set max speed, add a second parameter of "1" to make it persist.
*/
void exec_setMotorSpeed()
{
  exec_setMotorSpeed(atof(inParam1));
  if (inNoOfParams == 3 && atoi(inParam2) == 1)
    EEPROM_writeAnything(EEPROM_MACHINE_MOTOR_SPEED, currentMaxSpeed);
}

void exec_setMotorSpeed(float speed)
{
  currentMaxSpeed = speed;
  motorA.setMaxSpeed(currentMaxSpeed);
  motorB.setMaxSpeed(currentMaxSpeed);
  Serial.print(F("New max speed: "));
  Serial.println(currentMaxSpeed);
}

/* Single parameter to set acceleration, add a second parameter of "1" to make it persist.
*/
void exec_setMotorAcceleration()
{
  exec_setMotorAcceleration(atof(inParam1));
  if (inNoOfParams == 3 && atoi(inParam2) == 1)
    EEPROM_writeAnything(EEPROM_MACHINE_MOTOR_ACCEL, currentAcceleration);
}

void exec_setMotorAcceleration(float accel)
{
  currentAcceleration = accel;
  motorA.setAcceleration(currentAcceleration);
  motorB.setAcceleration(currentAcceleration);
  Serial.print(F("New acceleration: "));
  Serial.println(currentAcceleration);
}

void exec_changePenWidth()
{
  penWidth = atof(inParam1);
  Serial.print(MSG_I_STR);
  Serial.print(F("Changed Pen width to "));
  Serial.print(penWidth);
  Serial.print(F("mm"));
  Serial.println();
  msg_reportMinimumGridSizeForPen();
}

void exec_setPosition()
{
  long targetA = multiplier(atol(inParam1));
  long targetB = multiplier(atol(inParam2));

  motorA.setCurrentPosition(targetA);
  motorB.setCurrentPosition(targetB);
  
  engageMotors();
  
  reportPosition();
}

void exec_changeLengthRelative()
{
  long lenA = multiplier(atol(inParam1));
  long lenB = multiplier(atol(inParam2));
  
  changeLengthRelative(lenA, lenB);
}  

void exec_changeLength()
{
  float lenA = multiplier(atof(inParam1));
  float lenB = multiplier(atof(inParam2));
  
  changeLength(lenA, lenB);
}

void exec_changeLengthDirect()
{
  float endA = multiplier(atof(inParam1));
  float endB = multiplier(atof(inParam2));
  int maxSegmentLength = atoi(inParam3);

  float startA = motorA.currentPosition();
  float startB = motorB.currentPosition();

  if (endA < 20 || endB < 20 || endA > getMaxLength() || endB > getMaxLength())  {
    Serial.println("MSG,E,This point falls outside the area of this machine. Skipping it.");
    Serial.print(endA); Serial.print("/"); Serial.print(endB); Serial.print("  max:"); Serial.println(getMaxLength());
  }
  else  {
    exec_drawBetweenPoints(startA, startB, endA, endB, maxSegmentLength);
  }
}  

/**
This moves the gondola in a straight line between p1 and p2.  Both input coordinates are in 
the native coordinates system.  

The fidelity of the line is controlled by maxLength - this is the longest size a line segment is 
allowed to be.  1 is finest, slowest.  Use higher values for faster, wobblier.
*/
void exec_drawBetweenPoints(float p1a, float p1b, float p2a, float p2b, int maxSegmentLength)
{
//  Serial.print("From coords: ");
//  Serial.print(p1a);
//  Serial.print(",");
//  Serial.println(p1b);
//  Serial.print("To coords: ");
//  Serial.print(p2a);
//  Serial.print(",");
//  Serial.println(p2b);
  // ok, we're going to plot some dots between p1 and p2.  Using maths. I know! Brave new world etc.
  
  // First, convert these values to cartesian coordinates
  // We're going to figure out how many segments the line
  // needs chopping into.
  float c1x = getCartesianXFP(p1a, p1b);
  float c1y = getCartesianYFP(c1x, p1a);
  
  float c2x = getCartesianXFP(p2a, p2b);
  float c2y = getCartesianYFP(c2x, p2a);
  
 
  // test to see if it's on the page
  // AND ALSO TO see if the current position is on the page.
  // Remember, the native system can easily specify points that can't exist,
  // particularly up at the top.
  if (c2x > 20 
    && c2x<pageWidth-20 
    && c2y > 20 
    && c2y <pageHeight-20
    && c1x > 20 
    && c1x<pageWidth-20 
    && c1y > 20 
    && c1y <pageHeight-20 
    )
    {
    reportingPosition = false;
    float deltaX = c2x-c1x;    // distance each must move (signed)
    float deltaY = c2y-c1y;
    float totalDistance = sqrt(sq(deltaX) + sq(deltaY));

    long linesegs = 1;            // assume at least 1 line segment will get us there.
    if (abs(deltaX) > abs(deltaY))
    {
      // slope <=1 case    
      while ((abs(deltaX)/linesegs) > maxSegmentLength)
      {
        linesegs++;
      }
    }
    else
    {
      // slope >1 case
      while ((abs(deltaY)/linesegs) > maxSegmentLength)
      {
        linesegs++;
      }
    }
    
    // reduce delta to one line segments' worth.
    deltaX = deltaX/linesegs;
    deltaY = deltaY/linesegs;
  
    // render the line in N shorter segments
    long runSpeed = 0;

    usingAcceleration = false;
    while (linesegs > 0)  {
//      Serial.print("Line segment: " );
//      Serial.println(linesegs);
      // compute next new location
      c1x = c1x + deltaX;
      c1y = c1y + deltaY;
  
      // convert back to machine space
      float pA = getMachineA(c1x, c1y);
      float pB = getMachineB(c1x, c1y);
    
      // do the move
      runSpeed = desiredSpeed(linesegs, runSpeed, currentAcceleration*4);
      
//      Serial.print("Setting speed:");
//      Serial.println(runSpeed);
      
      motorA.setSpeed(runSpeed);
      motorB.setSpeed(runSpeed);
      changeLength(pA, pB);
  
      // one line less to do!
      linesegs--;
    }
    // reset back to "normal" operation
    reportingPosition = true;
    usingAcceleration = true;
    reportPosition();
  }
  else  {
    Serial.println("MSG,E,Line is not on the page. Skipping it.");
    Serial.print("From coords: ");  Serial.print(c1x);  Serial.print(",");  Serial.println(c1y);
    Serial.print("To coords: ");  Serial.print(c2x);  Serial.print(",");  Serial.println(c2y);
  }
}

/*
This is a method pinched from AccelStepper (older version).
*/
float desiredSpeed(long distanceTo, float currentSpeed, float acceleration)
{
    float requiredSpeed;

    if (distanceTo == 0)
  return 0.0f; // We're there

    // sqrSpeed is the signed square of currentSpeed.
    float sqrSpeed = sq(currentSpeed);
    if (currentSpeed < 0.0)
      sqrSpeed = -sqrSpeed;
      
    float twoa = 2.0f * acceleration; // 2ag
    
    // if v^^2/2as is the the left of target, we will arrive at 0 speed too far -ve, need to accelerate clockwise
    if ((sqrSpeed / twoa) < distanceTo)
    {
  // Accelerate clockwise
  // Need to accelerate in clockwise direction
  if (currentSpeed == 0.0f)
      requiredSpeed = sqrt(twoa);
  else
      requiredSpeed = currentSpeed + fabs(acceleration / currentSpeed);

  if (requiredSpeed > currentMaxSpeed)
      requiredSpeed = currentMaxSpeed;
    }
    else
    {
  // Decelerate clockwise, accelerate anticlockwise
  // Need to accelerate in clockwise direction
  if (currentSpeed == 0.0f)
      requiredSpeed = -sqrt(twoa);
  else
      requiredSpeed = currentSpeed - fabs(acceleration / currentSpeed);
  if (requiredSpeed < -currentMaxSpeed)
      requiredSpeed = -currentMaxSpeed;
    }
    
    //Serial.println(requiredSpeed);
    return requiredSpeed;
}


//GP extensions
void exec_G99() {Serial.println("Comment line...");}  //comment line


void exec_G21(){ Serial.println("G21...");}

void exec_G90(){Serial.println("G90...");}

void exec_G1() {   //G1 move relative to the last point

      float endA1 = (float)atof(inParam1);
      float endB1 = (float)atof(inParam2);
      float endA2,endB2;
      Serial.print("Line:"); Serial.print(commandFileLineCount);
      Serial.print("  Move: A");   Serial.print(endA1); Serial.print(" B:"); Serial.println(endB1);
      
      endA1 = (MACH_X_offs + (endA1 + GXoffs) * Gdiv) *  stepsPerMM;
      endB1 = (MACH_Y_offs + (endB1 + GYoffs) * Gdiv) *  stepsPerMM;
      Serial.print("ABSMove: A");   Serial.print(endA1); Serial.print(" B:"); Serial.println(endB1);
        
      endA2 = getMachineA(endA1,endB1) / stepMultiplier; 
      endB2 = getMachineB(endA1,endB1) / stepMultiplier;    //visszaszámolás a gép rendszerébe
//      lcd.print(endA2); lcd.print("/"); lcd.print(endB2); 
//      changeLength(endA2,endB2);
    
// call the drawing function with the new parameteres
      dtostrf(endA2, 6, 2, inParam1);  dtostrf(endB2, 6, 2, inParam2);  dtostrf(2, 6, 2, inParam3);
      exec_changeLengthDirect();
}
