/**
*  Polargraph Server. - CORE
*  Written by Sandy Noble
*  Released under GNU License version 3.
*  http://www.polargraph.co.uk
*  https://github.com/euphy/polargraph_server_polarshield

EEPROM.

This is one of the core files for the polargraph server program.  
Has a couple of little helper methods for reading and writing
ints and floats to EEPROM using the EEPROM library.

But mostly it contains the loadMachineSpecFromEeprom method, which is
used to retrieve the machines saved values when it restarts, or 
whenever a value is written to the EEPROM.

*/

void eeprom_resetEeprom(){
  for (int i = 0; i < (EEPROM_END +2); i++)  {
    EEPROM.write(i, 0);
  }
  eeprom_loadMachineSpecFromEeprom();
}

void eeprom_dumpEeprom(){
  for (int i = 0; i <40; i++)  {
    Serial.print(i);
    Serial.print(". ");
    Serial.println(EEPROM.read(i));
  }
}  

void eeprom_loadMachineSize(){
  EEPROM_readAnything(EEPROM_MACHINE_WIDTH, machineWidth);
  if (machineWidth < 1)  {
    machineWidth = defaultMachineWidth;
  }
  Serial.print(F("Loaded machine width:"));
  Serial.println(machineWidth);
  
  EEPROM_readAnything(EEPROM_MACHINE_HEIGHT, machineHeight);
  if (machineHeight < 1)  {
    machineHeight = defaultMachineHeight;
  }
  Serial.print(F("Loaded machine height:"));
  Serial.println(machineHeight);
}

void eeprom_loadSpoolSpec() {
  EEPROM_readAnything(EEPROM_MACHINE_MM_PER_REV, mmPerRev);
  if (isnan(mmPerRev)) {
    Serial.println("mmPerRev is nan, being corrected.");
    mmPerRev = defaultMmPerRev;
  }
  else if (mmPerRev < 1) {
    mmPerRev = defaultMmPerRev;
  }
  Serial.print(F("Loaded mm per rev:"));
  Serial.println(mmPerRev);

  EEPROM_readAnything(EEPROM_MACHINE_STEPS_PER_REV, motorStepsPerRev);
  if (isnan(motorStepsPerRev)) {
    Serial.println("motorStepsPerRev is nan, being corrected.");
    motorStepsPerRev = defaultStepsPerRev;
  }
  else if (motorStepsPerRev < 1) {
    motorStepsPerRev = defaultStepsPerRev;
  }
  Serial.print(F("Loaded motor steps per rev:"));
  Serial.println(motorStepsPerRev);
}  

void eeprom_loadPenLiftRange() {
  EEPROM_readAnything(EEPROM_PENLIFT_DOWN, downPosition);
  if (downPosition <= 0)  {
    downPosition = DEFAULT_DOWN_POSITION;
  }
  Serial.print(F("Loaded down pos:"));
  Serial.println(downPosition);

  EEPROM_readAnything(EEPROM_PENLIFT_UP, upPosition);
  if (upPosition <= 0)  {
    upPosition = DEFAULT_UP_POSITION;
  }
  Serial.print(F("Loaded up pos:"));
  Serial.println(upPosition);
}  

void eeprom_loadMachineName(){
  String name = "";
  for (int i = 0; i < 8; i++)  {
    char b = EEPROM.read(EEPROM_MACHINE_NAME+i);
    name = name + b;
  }
  
  if (name[0] == 0)  name = DEFAULT_MACHINE_NAME;
  maxLength = 0;
  machineName = name;
  Serial.print(F("Loaded machine name:"));
  Serial.println(machineName);
}

void eeprom_loadStepMultiplier() {
  EEPROM_readAnything(EEPROM_MACHINE_STEP_MULTIPLIER, stepMultiplier);
  if (stepMultiplier < 1)  {
    stepMultiplier = defaultStepMultiplier;
  }
  Serial.print(F("Loaded motor step multiplier:"));
  Serial.println(stepMultiplier);  
}  

void eeprom_loadSpeed() {
  // load speed, acceleration
  EEPROM_readAnything(EEPROM_MACHINE_MOTOR_SPEED, currentMaxSpeed);
  
  // not sure why this requires a cast to int for the comparision, but a 
  // if (currentMaxSpeed < 1.0) wasn't catching cases where 
  // currentMaxSpeed == 0.00, ODD.
  if (int(currentMaxSpeed) < 1) {
    currentMaxSpeed = 800.0;
  }
  Serial.print(F("Loaded motor maxspeed:"));
  Serial.println(currentMaxSpeed);
    
  EEPROM_readAnything(EEPROM_MACHINE_MOTOR_ACCEL, currentAcceleration);
  if (int(currentAcceleration) < 1) {
    currentAcceleration = 800.0;
  }
  Serial.print(F("Loaded motor acceleration:"));
  Serial.println(currentAcceleration);
}

void eeprom_GCode_params() {
  EEPROM_readAnything(EEPROM_GXOFFS, GXoffs);
  if (GXoffs<0) GXoffs = 190; 
  Serial.print(F("GCODE X offset:"));
  Serial.println(GXoffs);
  
  EEPROM_readAnything(EEPROM_GYOFFS, GYoffs);
  if (GYoffs<0) GYoffs = 0; 
  Serial.print(F("GCODE Y offset:"));
  Serial.println(GYoffs);
  
  EEPROM_readAnything(EEPROM_GDIV, Gdiv);
  if (Gdiv<=0) Gdiv = 1; 
  Serial.print(F("GCODE division scale:"));
  Serial.println(Gdiv);
  
  EEPROM_readAnything(EEPROM_MACH_X_OFFS,MACH_X_offs);
  if (MACH_X_offs<0) MACH_X_offs = 190.0; 
  Serial.print(F("GCODE MACHINE X offset:"));
  Serial.println(MACH_X_offs);
  
  EEPROM_readAnything(EEPROM_MACH_Y_OFFS,MACH_Y_offs);
  if (MACH_Y_offs <0) MACH_Y_offs = 290.0;
  Serial.print(F("GCODE MACHINE Y offset:"));
  Serial.println(MACH_Y_offs);
}


void eeprom_loadMachineSpecFromEeprom() {
  impl_loadMachineSpecFromEeprom();

  eeprom_loadMachineSize();
  eeprom_loadSpoolSpec();
  eeprom_loadStepMultiplier();
  eeprom_loadMachineName();
  eeprom_loadPenLiftRange();
  eeprom_loadSpeed();
  eeprom_GCode_params();

  Serial.print("Didn't load penWidth: ");
  Serial.println(penWidth);

  mmPerStep = mmPerRev / multiplier(motorStepsPerRev);
  stepsPerMM = multiplier(motorStepsPerRev) / mmPerRev;
  
  Serial.print(F("Recalc mmPerStep ("));
  Serial.print(mmPerStep);
  Serial.print(F("), stepsPerMM ("));
  Serial.print(stepsPerMM);
  Serial.print(F(")"));
  Serial.println();

  pageWidth = machineWidth * stepsPerMM;
  Serial.print(F("Recalc pageWidth in steps ("));
  Serial.print(pageWidth);
  Serial.print(F(")"));
  Serial.println();
  pageHeight = machineHeight * stepsPerMM;
  Serial.print(F("Recalc pageHeight in steps ("));
  Serial.print(pageHeight);
  Serial.print(F(")"));
  Serial.println();

  maxLength = 0;
}

void eeprom_save_factory_defaults() {
    EEPROM_writeAnything(EEPROM_MACHINE_WIDTH, defaultMachineWidth);
    EEPROM_writeAnything(EEPROM_MACHINE_HEIGHT, defaultMachineHeight);
    for (int i = 0; i < 8; i++)
         EEPROM.write(EEPROM_MACHINE_NAME+i, machineName[i]);
    EEPROM_writeAnything(EEPROM_MACHINE_MM_PER_REV, defaultMmPerRev);
    EEPROM_writeAnything(EEPROM_MACHINE_STEPS_PER_REV, defaultStepsPerRev);
    EEPROM_writeAnything(EEPROM_MACHINE_STEP_MULTIPLIER, defaultStepMultiplier);
    EEPROM_writeAnything(EEPROM_MACHINE_MOTOR_SPEED, currentMaxSpeed);
    EEPROM_writeAnything(EEPROM_MACHINE_MOTOR_ACCEL, currentAcceleration);
    EEPROM_writeAnything(EEPROM_MACHINE_PEN_WIDTH, penWidth);
    EEPROM_writeAnything(EEPROM_MACHINE_HOME_A, homeA);
    EEPROM_writeAnything(EEPROM_MACHINE_HOME_B, homeB);
    EEPROM_writeAnything(EEPROM_PENLIFT_DOWN, DEFAULT_DOWN_POSITION);
    EEPROM_writeAnything(EEPROM_PENLIFT_UP, DEFAULT_UP_POSITION);
    EEPROM_writeAnything(EEPROM_GXOFFS, GXoffs);
    EEPROM_writeAnything(EEPROM_GYOFFS, GYoffs);
    EEPROM_writeAnything(EEPROM_GDIV, Gdiv);
    EEPROM_writeAnything(EEPROM_MACH_X_OFFS, MACH_X_offs);
    EEPROM_writeAnything(EEPROM_MACH_Y_OFFS, MACH_Y_offs);
    EEPROM_writeAnything(EEPROM_FACTORY_RESET, (byte)0);
}
