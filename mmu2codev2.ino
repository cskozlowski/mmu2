
// CSK MMU2 Controller Version
//
//  Code developed by Chuck Kozlowski
//  September 19, 2018
//
//  Code was developed because I am impatiently waiting for my MMU2 to arrive so I thought
//  I would develop some code to operate the PRUSA MMU2 hardware
//
// This code uses 3 stepper motor controllers and 1 Pinda filament sensor
//
//  Work to be done:  Interface Control with the Einsy Board (MK3)
//                    Refine speed settings for each stepper motor
//                    Failure Recovery Modes - basically non-existent
//                    Uses the serial interface with a host computer at the moment - probably could do some smarter things
//                                                                                   like selection switches.
//                   10.2.18  Moved from breadboard to RAMPS 1.6 Board and remapped ALL addresses
//                            Discovered the filament idler motor needed to be set at a higher torque (more current)
//                            (this was affected filament load consistency)
//                   10.2.18  Major Disaster, lost my codebase on my PC (I am an idiot)
//                            Thank God for github so I could recover a week old version of my code
//                   10.1.18  Added filament sensor to the extruder head (helps reliability


//#include <SoftwareSerial.h>
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>


#define SERIAL1ENABLED    1
#define ENABLE LOW                // 8825 stepper motor enable is active low
#define DISABLE HIGH              // 8825 stepper motor disable is active high

#define MMU2_VERSION "3.02  10/04/18"

#define STEPSPERMM  144           // these are the number of steps required to travel 1 mm using the extruder motor

#define FW_VERSION 90             // config.h  (MM-control-01 firmware)
#define FW_BUILDNR 85             // config.h  (MM-control-01 firmware)

#define ORIGINALCODE 0            // code that is no longer needed for operational use
int command = 0;

#define MAXROLLERTRAVEL 125         // number of steps that the roller bearing stepper motor can travel

#define FULL_STEP  1
#define HALF_STEP  2
#define QUARTER_STEP 4
#define EIGTH_STEP 8
#define SIXTEENTH_STEP 16

#define STEPSIZE SIXTEENTH_STEP

#define STEPSPERREVOLUTION 200     // 200 steps per revolution  - 1.8 degree motors are being used

#define MMU2TOEXTRUDERSTEPS STEPSIZE*STEPSPERREVOLUTION*19   // for the 'T' command 

#define CW 0
#define CCW 1
#define INACTIVE 0
#define ACTIVE 1

#define TOOLSYNC 20                         // number of tool change (T) commands before a selector resync is necessary



#define PINHIGH 10                    // how long to hold stepper motor pin high in microseconds
#define PINLOW  10                    // how long to hold stepper motor pin low in microseconds

//#define BEARINGSTEPSIZE 24         // steps to each roller bearing
float bearingAbsPos[5] = {1, 24, 48, 72, 96}; // absolute position of roller bearing stepper motor
// changed position #2 to 372  (still tuning this little sucker


#define CSSTEPS 357                        //                                                 
int selectorAbsPos[5] = {0, CSSTEPS * 1, CSSTEPS * 2, CSSTEPS * 3, CSSTEPS * 4}; // absolute position of selector stepper motor


int trackToolChanges = 0;

int currentCSPosition = 0;         // color selector position
int currentPosition = 0;


int oldBearingPosition = 0;      // this tracks the roller bearing position (top motor on the MMU)
int filamentSelection = 0;       // keep track of filament selection (0,1,2,3,4))
int dummy[100];
char currentExtruder = '0';

int firstTimeFlag = 0;
int earlyCommands = 0;           // forcing communications with the mk3 at startup

int toolChangeCount = 0;

char receivedChar;
boolean newData = false;
int idlerStatus = INACTIVE;
int colorSelectorStatus = INACTIVE;


#define IDLERMOTORDELAY  450     // 2,000 useconds  (idler motor)
#define EXTRUDERMOTORDELAY 50     // 150 useconds    (controls filament feed speed to the printer)
#define COLORSELECTORMOTORDELAY 50 // 50 useconds

// modified code on 10.2.18 to accomodate RAMPS 1.6 board mapping
//
byte idlerDirPin = A7;
byte idlerStepPin = A6;
byte idlerEnablePin = A2;



byte extruderDirPin = 48;     //  pin 5 for extruder motor direction pin
byte extruderStepPin = 46;   //  pin 4 for extruder motor stepper motor pin
byte extruderEnablePin = A8;    //  pin 6 for extruder motor rst/sleep motor pin

byte colorSelectorDirPin = A1;    //color selector stepper motor (driven by trapezoidal screw)
byte colorSelectorStepPin = A0;
byte colorSelectorEnablePin = 38;



byte findaPin = A3;

byte filamentSwitch = 3;       // this switch was added on 10.1.18 to help with filament loading (X- signal on the RAMPS board)


//SoftwareSerial Serial1(10,11); // RX, TX (communicates with the MK3 controller board

int f0Min = 1000, f1Min = 1000, f2Min = 1000, f3Min = 1000, f4Min = 1000;
int f0Max, f1Max, f2Max, f3Max, f4Max = 0;
int f0Avg, f1Avg, f2Avg, f3Avg, f4Avg;
long f0Distance, f1Distance, f2Distance, f3Distance, f4Distance = 0;              // changed from int to long type 10.5.18
int f0ToolChange, f1ToolChange, f2ToolChange, f3ToolChange, f4ToolChange = 0;

unsigned long time0, time1, time2, time3, time4, time5;


void setup() {
  // static int findaStatus;

  int waitCount;


  Serial.begin(115200);  // startup the local serial interface
  while (!Serial) {
    ; // wait for serial port to connect. needed for native USB port only
    Serial.println("waiting for serial port");
  }

  Serial.println(MMU2_VERSION);

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // THIS DELAY IS CRITICAL DURING POWER UP/RESET TO PROPERLY SYNC WITH THE MK3 CONTROLLER BOARD
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  delay(4000);                    // this is key to syncing to the MK3 controller - currently 4 seconds




  Serial1.begin(115200);         // startup the mk3 serial
  // Serial1.begin(115200;              // ATMEGA hardware serial interface

  //Serial.println("started the mk3 serial interface");
  delay(100);


  Serial.println("Sending START command to mk3 controller board");
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // THIS NEXT COMMAND IS CRITICAL ... IT TELLS THE MK3 controller that an MMU is present
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Serial1.print("start\n");                 // attempt to tell the mk3 that the mmu is present

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //  check the serial interface to see if it is active
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  waitCount = 0;
  while (!Serial1.available()) {

    //delay(100);
    Serial.println("Waiting for message from mk3");
    delay(1000);
    ++waitCount;
    if (waitCount >= 7) {
      Serial.println("7 seconds have passed, aborting wait for mk3 to respond");
      goto continue_processing;
    }
  }
  Serial.println("inbound message from mk3");

continue_processing:



  pinMode(idlerDirPin, OUTPUT);
  pinMode(idlerStepPin, OUTPUT);

  pinMode(findaPin, INPUT);                        // pinda Filament sensor
  pinMode(filamentSwitch, INPUT);

  pinMode(idlerEnablePin, OUTPUT);
  // pinMode(bearingRstPin, OUTPUT);

  pinMode(extruderEnablePin, OUTPUT);
  pinMode(extruderDirPin, OUTPUT);
  pinMode(extruderStepPin, OUTPUT);

  pinMode(colorSelectorEnablePin, OUTPUT);
  pinMode(colorSelectorDirPin, OUTPUT);
  pinMode(colorSelectorStepPin, OUTPUT);

  Serial.println("finished setting up input and output pins");




  digitalWrite(idlerEnablePin, ENABLE);           // enable the roller bearing motor (motor #1)
  // digitalWrite(bearingRstPin, HIGH);

  digitalWrite(extruderEnablePin, ENABLE);        //  enable the extruder motor  (motor #2)
  digitalWrite(colorSelectorEnablePin, ENABLE);  // enable the color selector motor  (motor #3)





  // moved these inits to the loop() section since the mk3 serial interface needs to be handled
  //

#ifdef NOTDEF
  if (isFilamentLoaded()) {               // check to see if filament in the bowden tube (between the mmu2 and mk3
    Serial.println("Filament was in the bowden tube at startup, unloading filament automatically");
    unloadFilamentToFinda();            //
  }
#endif

  Serial.println("Syncing the Idler Selector Assembly");             // do this before moving the selector motor
  initIdlerPosition();    // reset the roller bearing position


  Serial.println("Syncing the Filament Selector Assembly");
  if (!isFilamentLoaded()) {
       initColorSelectorPosition();   // reset the color selector if there is NO filament present
  } else {
       Serial.println("Unable to clear the Color Selector, please remove filament");
  }   

  Serial.println("Inialialization Complete, let's multicolor print ....");



}  // end of init() routine


// infinite loop - core of the program

void loop() {
  int i;
  char rcvChar;
  int pindaStatus;
  char c1, c2, c3;
  String kbString;
  int fstatus;


  // Serial.println("looping");
  delay(100);                       // wait for 100 milliseconds
  checkSerialInterface();           // check the serial interface for input commands from the mk3
#ifdef NOTDEF
  while (1) {
    fstatus = digitalRead(filamentSwitch);
    Serial.print("Filament Status: ");
    Serial.println(fstatus);
    delay(1000);
  }
#endif

  // Serial.println("Enter Filament Selection (1-5),Disengage Roller (D), Load Filament (L), Unload Filament (U), Test Color Extruder(T)");
  //Serial.print("FINDA status: ");
  //fstatus = digitalRead(findaPin);
  //Serial.println(fstatus);


  // check for keyboard input

  if (Serial.available()) {
    Serial.print("Key was hit ");
    //c1 = Serial.read();
    //c2 = Serial.read();
    //c3 = Serial.read();

    kbString = Serial.readString();
    // Serial.print(c1); Serial.print(" "); Serial.println("c2");
    Serial.print(kbString);

    if (kbString[0] == 'C') {
      //if (c1 == 'C') {
      Serial.println("Processing 'C' Command");
      filamentLoadWithBondTechGear();

      //filamentLoadToExtruder();
    }
    if (kbString[0] == 'T') {
      //if (c1 == 'T') {
      Serial.println("Processing 'T' Command");
      toolChange(kbString[1]);                 // invoke the tool change command
      //toolChange(c2);
      // processKeyboardInput();
    }
    if (kbString[0] == 'U') {
      Serial.println("Processing 'U' Command");

      unParkIdler();                    // turn on the idler motor
      unloadFilamentToFinda();          //unload the filament
      parkIdler();                      // turn off the idler motor
    }
  }



}   // end of infinite loop



// need to check the PINDA status

void checkSerialInterface() {
  unsigned char c1, c2, c3;
  int i;
  int cnt;
  char c;
  String inputLine;
  int counter = 0;
  int findaStatus;
  int index;
  long steps;


  // Serial.println("Waiting for communication with mk3");

  // while (earlyCommands == 0) {
  // Serial.println("waiting for response from mk3");
  index = 0;
  if ((cnt = Serial1.available()) > 0) {

    //Serial.print("chars received: ");
    //Serial.println(cnt);

    inputLine = Serial1.readString();      // fetch the command from the mmu2 serial input interface

    if (inputLine[0] != 'P') {
      Serial.print("MMU Command: ");
      Serial.println(inputLine);
    }
process_more_commands:  // parse the inbound command
    c1 = inputLine[index++];                      // fetch single characer from the input line
    c2 = inputLine[index++];                      // fetch 2nd character from the input line
    c3 = inputLine[index++];                      // carriage return


    // process commands coming from the mk3 controller
    //***********************************************************************************
    // Commands still to be implemented:  X0 (MMU Reset), F0 (Filament type select),
    // E0->E4 (Eject Filament), R0 (recover from eject)
    //***********************************************************************************
    switch (c1) {
      case 'T':
        // request for idler and selector based on filament number
        time4 = millis();           // grab the current time

        if ((c2 >= '0')  && (c2 <= '4')) {
          toolChange(c2);

        } else {
          Serial.println("T: Invalid filament Selection");
        }

        // delay(200);                      //removed this 200msec delay on 10.5.18
        Serial1.print("ok\n");              // send command acknowledge back to mk3 controller
        time5 = millis();          // grab the current time
        break;
      case 'C':
        // move filament from selector ALL the way to printhead
#ifdef NOTDEF
        Serial.println("C: Moving filament to Bondtech gears");
#endif
        // filamentLoadToExtruder();
        filamentLoadWithBondTechGear();
        // delay(200);
        Serial1.print("ok\n");
        break;

      case 'U':
        // request for filament unload
        Serial.println("U: Filament Unload Selected");
        if (idlerStatus == INACTIVE)
          unParkIdler();
        if ((c2 >= '0') && (c2 <= '4')) {

          unloadFilamentToFinda();
          parkIdler();
          Serial.println("U: Sending Filament Unload Acknowledge to MK3");
          delay(200);
          Serial1.print("ok\n");

        } else {
          Serial.println("U: Invalid filament Unload Requested");
          delay(200);
          Serial1.print("ok\n");
        }
        break;
      case 'L':
        // request for filament load
        Serial.println("L: Filament Load Selected");
        if (idlerStatus == INACTIVE)
          unParkIdler();

        if (colorSelectorStatus == INACTIVE)
          activateColorSelector();         // turn on the color selector motor

        if ((c2 >= '0') && (c2 <= '4')) {

          Serial.println("L: Moving the bearing idler");
          idlerSelector(c2);   // move the filament selector stepper motor to the right spot
          Serial.println("L: Moving the color selector");
          colorSelector(c2);     // move the color Selector stepper Motor to the right spot
          Serial.println("L: Loading the Filament");
          // loadFilament(CCW);
          loadFilamentToFinda();
          parkIdler();             // turn off the idler roller

          Serial.println("L: Sending Filament Load Acknowledge to MK3");

          delay(200);

          Serial1.print("ok\n");



        } else {
          Serial.println("Error: Invalid Filament Number Selected");
        }
        break;

      case 'S':
        // request for firmware version
        // Serial.println("S Command received from MK3");
        // this is a serious hack since the serial interface is flaky at this point in time
#ifdef NOTDEF
        if (command == 1) {
          Serial.println("S: Processing S2");
          Serial1.print(FW_BUILDNR);
          Serial1.print("ok\n");

          command++;

        }
        if (command == 0) {
          Serial.println("S: Processing S1");
          Serial1.print(FW_VERSION);
          Serial1.print("ok\n");

          command++;
        }
#endif

        switch (c2) {
          case '0':
            Serial.println("S: Sending back OK to MK3");
            Serial1.print("ok\n");
            break;
          case '1':
            Serial.println("S: FW Version Request");
            Serial1.print(FW_VERSION);
            Serial1.print("ok\n");
            break;
          case '2':
            Serial.println("S: Build Number Request");
            Serial.println("Initial Communication with MK3 Controller: Successful");
            Serial1.print(FW_BUILDNR);
            Serial1.print("ok\n");
            break;
          default:
            Serial.println("S: Unable to process S Command");
            break;
        }  // switch(c2) check
        break;
      case 'P':

        // check FINDA status
        // Serial.println("Check FINDA Status Request");
        findaStatus = digitalRead(findaPin);
        if (findaStatus == 0) {
          // Serial.println("P: FINDA INACTIVE");
          Serial1.print("0");
        }
        else {
          // Serial.println("P: FINDA ACTIVE");
          Serial1.print("1");
        }
        Serial1.print("ok\n");

        break;
      case 'F':                                         // 'F' command is acknowledged but no processing goes on at the moment
        // will be useful for flexible material down the road
        Serial.println("Filament Type Selected: ");
        Serial.println(c2);
        Serial1.print("ok\n");                        // send back OK to the mk3
        break;
      default:
        Serial.print("ERROR: unrecognized command from the MK3 controller");
        Serial1.print("ok\n");


    }  // end of switch statement
    if (cnt != 3) {
      Serial.print("Index: ");
      Serial.print(index);
      Serial.print(" cnt: ");
      Serial.println(cnt);
    }
  }  // end of cnt > 0 check

  if (index < cnt) {
    Serial.println("More commands in the buffer");

    goto process_more_commands;
  }
  // }  // check for early commands
}


void colorSelector(char selection) {

  int findaStatus;

  // this error check was added on 10.4.18

  if ((selection < '0') || (selection > '4')) {
    Serial.println("colorSelector():  Error, invalid filament selection");
    return;
  }
  
  // Serial.println("Entering colorSelector() routine");

loop:
  findaStatus = digitalRead(findaPin);    // check the pinda status ( DO NOT MOVE THE COLOR SELECTOR if filament is present)
  if (findaStatus == 1) {
    Serial.println("colorSelector(): Serious Problem, filament is present between the MMU2 and Extruder:  UNLOAD FILAMENT !!!");
    Serial.println("Clear the problem and then hit any key");

    while (!Serial.available()) {
      //  wait until key is entered to proceed
    }
    Serial.readString();  // clear the keyboard buffer
    goto loop;
  }

  digitalWrite(colorSelectorEnablePin, ENABLE );    // turn on the color selector motor
  delay(1);                                        // wait for 1 milliseconds          added on 10.4.18

  switch (selection) {
    case '0':                                       // position '0' is always just a move to the left
      // added the '+1' on 10.4.18
      csTurnAmount(currentPosition, CCW);         // the '+1' is an attempt to move the selector ALL the way left (puts the selector into known position)
      currentPosition = selectorAbsPos[0];
      break;
    case '1':
      if (currentPosition <= selectorAbsPos[1]) {
        csTurnAmount((selectorAbsPos[1] - currentPosition), CW);
      } else {
        csTurnAmount((currentPosition - selectorAbsPos[1]), CCW);
      }
      currentPosition = selectorAbsPos[1];
      break;
    case '2':
      if (currentPosition <= selectorAbsPos[2]) {
        csTurnAmount((selectorAbsPos[2] - currentPosition), CW);
      } else {
        csTurnAmount((currentPosition - selectorAbsPos[2]), CCW);

      }
      currentPosition = selectorAbsPos[2];
      break;
    case '3':
      if (currentPosition <= selectorAbsPos[3]) {
        csTurnAmount((selectorAbsPos[3] - currentPosition), CW);
      } else {
        csTurnAmount((currentPosition - selectorAbsPos[3]), CCW);

      }
      currentPosition = selectorAbsPos[3];
      break;
    case '4':
      if (currentPosition <= selectorAbsPos[4]) {
        csTurnAmount((selectorAbsPos[4] - currentPosition), CW);
      } else {
        csTurnAmount((currentPosition - selectorAbsPos[4]), CCW);

      }
      currentPosition = selectorAbsPos[4];
      break;

  }
  digitalWrite(colorSelectorEnablePin, DISABLE);    // turn off the color selector motor


}  // end of colorSelector routine()


// this is the motor with the lead screw (final stage of the MMU2 unit)

void csTurnAmount(int steps, int direction) {
  int i;

  if (direction == CW)
    digitalWrite(colorSelectorDirPin, LOW);      // set the direction for the Color Extruder Stepper Motor
  else
    digitalWrite(colorSelectorDirPin, HIGH);
   delayMicroseconds(500);                     // wait 500 microseconds  (10.5.18)

  for (i = 0; i < (steps * STEPSIZE); ++i) {
    digitalWrite(colorSelectorStepPin, HIGH);
    delayMicroseconds(PINHIGH);               // delay for 10 useconds
    digitalWrite(colorSelectorStepPin, LOW);
    delayMicroseconds(PINLOW);               // delay for 10 useconds
    delayMicroseconds(COLORSELECTORMOTORDELAY);         // wait for 400 useconds

  }
}





// test code snippet for moving a stepper motor
//  (not used operationally)
void completeRevolution() {
  int i, delayValue;

  for (i = 0; i < STEPSPERREVOLUTION * STEPSIZE; i++) {
    digitalWrite(idlerStepPin, HIGH);
    delayMicroseconds(PINHIGH);               // delay for 10 useconds
    digitalWrite(idlerStepPin, LOW);
    delayMicroseconds(PINLOW);               // delay for 10 useconds

    delayMicroseconds(IDLERMOTORDELAY);
    //delayValue = 64/stepSize;
    //delay(delayValue);           // wait for 30 milliseconds

  }
}

//
// turn any of the three sepper motors
//
void bearingTurnAmount(int steps, int dir) {
  int i;
  int delayValue;


#ifdef DEBUG
  Serial.println("moving the idler ...");
#endif
  digitalWrite(idlerEnablePin, ENABLE);   // turn on motor
  delay(1);

  // digitalWrite(ledPin, HIGH);

  digitalWrite(idlerDirPin, dir);
  delay(1);

  for (i = 0; i < steps * STEPSIZE; i++) {
    digitalWrite(idlerStepPin, HIGH);
    delayMicroseconds(PINHIGH);               // delay for 10 useconds
    digitalWrite(idlerStepPin, LOW);
    delayMicroseconds(PINLOW);               // delay for 10 useconds

    delayMicroseconds(IDLERMOTORDELAY);

  }
#ifdef NOTDEF
  Serial.println("finished moving the idler ...");
#endif

}

// turns on the extruder motor
void loadFilamentToFinda() {
  int i;
  int findaStatus;
  unsigned int steps;
  unsigned long startTime, currentTime;


  digitalWrite(extruderDirPin, CCW);  // set the direction of the MMU2 extruder motor

  startTime = millis();

loop:
  currentTime = millis();
  if ((currentTime - startTime) > 10000) {         // 15 seconds worth of trying to unload the filament
    Serial.println("loadFilamentToFinda() ERROR: Timeout occurred, Filament is not loading to FINDA sensor");
    Serial.println("loadFilamentToFinda()        Fix the issue and then hit any key to continue");
    parkIdler();                               // disengge the idler
    while (!Serial.available()) {  // wait for keyboard entry


    }
    unParkIdler();          // re-engage the idler
    startTime = millis();   // reset the start time clock
    Serial.readString();   // clear the serial buffer
  }
  feedFilament(1);        // 1 step and then check the pinda status

  findaStatus = digitalRead(findaPin);
  if (findaStatus == 0)              // keep feeding the filament until the pinda sensor triggers
    goto loop;

#ifdef NOTDEF
  Serial.println("Pinda Sensor Triggered during Filament Load");
#endif
  //
  // for a filament load ... need to get the filament out of the selector head !!!
  //
  digitalWrite(extruderDirPin, CW);   // back the filament away from the selector
  steps = 200 * STEPSIZE + 50;
  feedFilament(steps);
#ifdef NOTDEF
  Serial.println("Loading Filament Complete ...");
#endif

  // digitalWrite(ledPin, LOW);     // turn off LED
}

// unload Filament using the FINDA sensor
// turns on the extruder motor
void unloadFilamentToFinda() {
  int i;
  int findaStatus;
  unsigned int steps;
  unsigned long startTime, currentTime;


  if (!isFilamentLoaded()) {               // if the filament is already unloaded, do nothing

    Serial.println("unloadFilamentToFinda():  filament already unloaded");
    return;
  }

  digitalWrite(extruderDirPin, CW);  // set the direction of the MMU2 extruder motor

  startTime = millis();

loop:

  currentTime = millis();
  if ((currentTime - startTime) > 15000) {         // 15 seconds worth of trying to unload the filament
    Serial.println("unloadFilamenttoFinda() ERROR: Timeout occurred, Filament is not unloading");
    Serial.println("unloadFilamenttoFinda(): Unload the filament manually, hit any key when done");
    while (!Serial.available()) {
      // wait until key is hit

    }
    Serial.readString();     // clear the serial input
    startTime = millis();   // reset the start time
  }

  feedFilament(STEPSPERMM);        // 1mm and then check the pinda status


  if (isFilamentLoaded()) {
    goto loop;
  }
  //      findaStatus = digitalRead(findaPin);

  //      if (findaStatus == 1)              // keep feeding the filament until the pinda sensor triggers

  //          goto loop;

#ifdef NOTDEF
  Serial.println("unloadFilamenttoFinda(): Pinda Sensor Triggered during Filament unload");
#endif
  //
  // for a filament unload ... need to get the filament out of the selector head !!!
  //
  digitalWrite(extruderDirPin, CW);   // back the filament away from the selector
  steps = 200 * STEPSIZE + 50;
  feedFilament(steps);    //

#ifdef NOTDEF
  Serial.println("unloadFilamentToFinda(): Unloading Filament Complete ...");
#endif

  // digitalWrite(ledPin, LOW);     // turn off LED
}


void loadFilament(int direction) {
  int i;
  int findaStatus;
  unsigned int steps;


  // digitalWrite(ledPin, HIGH);          // turn on LED to indicate extruder motor is running
  digitalWrite(extruderDirPin, direction);  // set the direction of the MMU2 extruder motor


  switch (direction) {
    case CCW:                     // load filament
loop:
      feedFilament(1);        // 1 step and then check the pinda status

      findaStatus = digitalRead(findaPin);
      if (findaStatus == 0)              // keep feeding the filament until the pinda sensor triggers
        goto loop;
      Serial.println("Pinda Sensor Triggered");
      // now feed the filament ALL the way to the printer extruder assembly

      steps = 17 * 200 * STEPSIZE;

      Serial.print("steps: ");
      Serial.println(steps);
      feedFilament(steps);    // 17 complete revolutions
      Serial.println("Loading Filament Complete ...");
      break;

    case CW:                      // unload filament
loop1:
      feedFilament(1);            // 1 step and then check the pinda status
      findaStatus = digitalRead(findaPin);
      if (findaStatus == 1)        // wait for the filament to unload past the pinda sensor
        goto loop1;
      Serial.println("Pinda Sensor Triggered, unloading filament complete");
      // the +40 is a fudge factor to get the filament out of the way of the cutter
      feedFilament(200 * STEPSIZE + 40);  // turn for 1 more complete turn to get past the cutter head (21.2mm per revolution)

      break;
    default:
      Serial.println("loadFilament:  I shouldn't be here !!!!");


  }

  // digitalWrite(ledPin, LOW);     // turn off LED
}

//
// this routine feeds filament for 1 complete revolution - this represents 21.2mm of filament
//
void feedFilament(unsigned int steps) {

  int i;

#ifdef NOTDEF
  if (steps > 1) {
    Serial.print("Steps: ");
    Serial.println(steps);
  }
#endif
  // for (i = 0; i < 200*STEPSIZE; i++) {
  for (i = 0; i <= steps; i++) {
    digitalWrite(extruderStepPin, HIGH);
    delayMicroseconds(PINHIGH);               // delay for 10 useconds
    digitalWrite(extruderStepPin, LOW);
    delayMicroseconds(PINLOW);               // delay for 10 useconds
    // delayValue = 32/stepSize;
    //delayValue = 1;
    delayMicroseconds(EXTRUDERMOTORDELAY);         // wait for 400 useconds
    //delay(delayValue);           // wait for 30 milliseconds

  }
}


void recoverfilamentSelector() {

}

// this routine drives the 5 position bearings (aka idler) on the top of the MMU2 carriage
//
void idlerSelector(char filament) {
  int steps;
  int newBearingPosition;
  int newSetting;

#ifdef DEBUG
  Serial.print("idlerSelector(): Filament Selected: ");
  Serial.println(filament);
#endif

  if ((filament < '0') || (filament > '4')) {
    Serial.println("idlerSelector() ERROR, invalid filament selection");
    Serial.print("idlerSelector() filament: ");
    Serial.println(filament);
    return;
  }
  // move the selector back to it's origin state

#ifdef DEBUG
  Serial.print("Old Idler Roller Bearing Position:");
  Serial.println(oldBearingPosition);
  Serial.println("Moving filament selector");
#endif

  switch (filament) {
    case '0':
      newBearingPosition = bearingAbsPos[0];                         // idler set to 1st position
      filamentSelection = 0;
      currentExtruder = '0';
      break;
    case '1':
      newBearingPosition = bearingAbsPos[1];
      filamentSelection = 1;
      currentExtruder = '1';
      break;
    case '2':
      newBearingPosition = bearingAbsPos[2];
      filamentSelection = 2;
      currentExtruder = '2';
      break;
    case '3':
      newBearingPosition = bearingAbsPos[3];
      filamentSelection = 3;
      currentExtruder = '3';
      break;
    case '4':
      newBearingPosition = bearingAbsPos[4];
      filamentSelection = 4;
      currentExtruder = '4';
      break;
    default:
      Serial.println("idlerSelector(): ERROR, Invalid Idler Bearing Position");
      break;
  }

  // turnAmount(newFilamentPosition,CCW);                        // new method


  newSetting = newBearingPosition - oldBearingPosition;
#ifdef DEBUG
  Serial.print("Old Bearing Position: ");
  Serial.println(oldBearingPosition);
  Serial.print("New Bearing Position: ");
  Serial.println(newBearingPosition);

  Serial.print("New Setting: ");
  Serial.println(newSetting);
#endif

  if (newSetting < 0) {
    bearingTurnAmount(-newSetting, CW);                     // turn idler to appropriate position
  } else {
    bearingTurnAmount(newSetting, CCW);                     // turn idler to appropriate position
  }
  oldBearingPosition = newBearingPosition;

}


// perform this function only at power up/reset
//
void initIdlerPosition() {

#ifdef NOTDEF
  Serial.println("initIdlerPosition(): resetting the Idler Roller Bearing position");
#endif

  digitalWrite(idlerEnablePin, ENABLE);   // turn on the roller bearing motor
  delay(1);
  oldBearingPosition = 125;                // points to position #1
  bearingTurnAmount(MAXROLLERTRAVEL, CW);
  bearingTurnAmount(MAXROLLERTRAVEL, CCW);                // move the bearings out of the way
  digitalWrite(idlerEnablePin, DISABLE);   // turn off the idler roller bearing motor

  filamentSelection = 0;       // keep track of filament selection (0,1,2,3,4))
  currentExtruder = '0';


}

// perform this function only at power up/reset
//
void initColorSelectorPosition() {
#ifdef NOTDEF
  Serial.println("Syncing the Color Selector Assembly");
#endif

  digitalWrite(colorSelectorEnablePin, ENABLE);   // turn on the stepper motor
  csTurnAmount(1900, CW);             // move to the right
  csTurnAmount(1920, CCW);        // move all the way to the left

  digitalWrite(colorSelectorEnablePin, DISABLE);   // turn off the stepper motor

}

// this just energizes the roller bearing extruder motor
//
void activateRollers() {

  digitalWrite(idlerEnablePin, ENABLE);   // turn on the roller bearing stepper motor

  // turnAmount(120, CW);   // move the rollers to filament position #1
  // oldBearingPosition = 45;  // filament position #1

  // oldBearingPosition = MAXROLLERTRAVEL;   // not sure about this CSK

  idlerStatus = ACTIVE;
}

// move the filament Roller pulleys away from the filament

void parkIdler() {
  int newSetting;

  digitalWrite(idlerEnablePin, ENABLE);
  delay(1);
  oldBearingPosition = bearingAbsPos[filamentSelection];          // fetch the bearing position based on the filament state

#ifdef DEBUG
  Serial.print("parkIdler() oldBearingPosition: ");
  Serial.print(oldBearingPosition);
  Serial.print("   filamentSelection: ");
  Serial.println(filamentSelection);
#endif
  newSetting = MAXROLLERTRAVEL - oldBearingPosition;

#ifdef DEBUG
  Serial.print("parkIdler() DeactiveRoller newSetting: ");
  Serial.println(newSetting);
#endif

  bearingTurnAmount(newSetting, CCW);     // move the bearing roller out of the way
  oldBearingPosition = MAXROLLERTRAVEL;   // record the current roller status  (CSK)

  idlerStatus = INACTIVE;
  digitalWrite(idlerEnablePin, DISABLE);    // turn off the roller bearing stepper motor  (nice to do, cuts down on CURRENT utilization)

}


// turn on the idler bearing rollers

void unParkIdler() {
  int rollerSetting;

  digitalWrite(idlerEnablePin, ENABLE);   // turn on the roller bearing motor
  delay(1);                              // wait for 10 useconds

  //Serial.println("Activating the Idler Rollers");

  rollerSetting = MAXROLLERTRAVEL - bearingAbsPos[filamentSelection];

  //Serial.print("unParkIdler() Idler Setting: ");
  //Serial.println(rollerSetting);

  bearingTurnAmount(rollerSetting, CW);    // restore the old position
  idlerStatus = ACTIVE;                   // mark the idler as active


}

void deActivateColorSelector() {
  int newSetting;

  digitalWrite(colorSelectorEnablePin, DISABLE);    // turn off the color selector stepper motor  (nice to do, cuts down on CURRENT utilization)
  colorSelectorStatus = INACTIVE;
}

void activateColorSelector() {
  digitalWrite(colorSelectorEnablePin, ENABLE);
  colorSelectorStatus = ACTIVE;
}




void recvOneChar() {
  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    newData = true;
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedChar);
    newData = false;
  }
}

#ifdef ORIGINALCODE

void processKeyboardInput() {


  while (newData == false) {
    recvOneChar();
  }

  showNewData();      // character received

  Serial.print("Filament Selected: ");
  Serial.println(receivedChar);

  switch (receivedChar) {
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      if (idlerStatus == INACTIVE)
        activateRollers();

      if (colorSelectorStatus == INACTIVE)
        activateColorSelector();         // turn on the color selector motor


      idlerSelector(receivedChar);   // move the filament selector stepper motor to the right spot
      colorSelector(receivedChar);     // move the color Selector stepper Motor to the right spot

      break;
    case 'd':                             // de-active the bearing roller stepper motor and color selector stepper motor
    case 'D':
      parkIdler();
      deActivateColorSelector();
      break;
    case 'l':                            // start the load process for the filament
    case 'L':
      // unParkIdler();
      if (idlerStatus == INACTIVE)
        unParkIdler();
      loadFilament(CCW);
      parkIdler();          // move the bearing rollers out of the way after a load is complete
      break;
    case 'u':                           // unload the filament from the MMU2 device
    case 'U':
      unParkIdler();           // working on this command
      loadFilament(CW);
      parkIdler();         // after the unload of the filament, move the bearing rollers out of the way
      break;
    case 't':
    case 'T':
      csTurnAmount(200, CW);
      delay(1000);
      csTurnAmount(200, CCW);
      break;
    default:
      Serial.println("Invalid Serial Output Selection");
  } // end of switch statement
}
#endif

void filamentLoadToExtruder() {
  float fsteps;
  unsigned int steps;
  int findaStatus;
  int flag;
  int filamentDistance;
  int fStatus;
  int startTime, currentTime;


  if ((currentExtruder < '0')  || (currentExtruder > '4')) {
    Serial.println("filamentLoadToExtruder(): fixing current extruder variable");
    currentExtruder = '0';
  }
#ifdef DEBUG
  Serial.println("Attempting to move Filament to Print Head Extruder Bondtech Gears");
  //unParkIdler();
  Serial.print("filamentLoadToExtruder():  currentExtruder: ");
  Serial.println(currentExtruder);
#endif

  // idlerSelector(currentExtruder);        // active the idler before the filament load

  deActivateColorSelector();


  digitalWrite(extruderDirPin, CCW);
  startTime = millis();
loop:
  feedFilament(1);        // 1 step and then check the pinda status

  findaStatus = digitalRead(findaPin);
  currentTime = millis();
  // added this timeout feature on 10.4.18
  if ((currentTime - startTime) > 2000) {
    Serial.println("filamentLoadToExtruder():  Error,  filament is not loading properly");
    Serial.println("filamentLoadToExtruder():          fix the issue and then hit any key");
    while (!Serial.available()) {}
    Serial.readString();   // flush the keyboard buffer
    startTime = millis();
  }
  if (findaStatus == 0)              // keep feeding the filament until the pinda sensor triggers
    goto loop;

  //Serial.println("filamentLoadToExtruder(): Pinda Sensor Triggered during Filament Load");
  // now loading from the FINDA sensor all the way down to the NEW filament sensor

  feedFilament(STEPSPERMM * 350);       // go 350 mm then look for the 2nd filament sensor
  filamentDistance = 350;
  //delay(15000);                         //wait 15 seconds
  //feedFilament(STEPSPERMM*100);         //go 100 more mm
  //delay(15000);
  //goto skipeverything;

  startTime = millis();
  flag = 0;
  //filamentDistance = 0;

  // wait until the filament sensor on the mk3 extruder triggers
  while (flag == 0) {

    currentTime = millis();
    if ((currentTime - startTime) > 8000) { // only wait for 8 seconds
      Serial.println("ERROR: filament load issue between FINDA and MK3 Switch, fix issue and hit any key");
      // probably should wait for keyboard hit at this point
      parkIdler();                  // disengage the idler to fix the filament issue
      while (!Serial.available()) {

        // wait for key to be hit
      }
      unParkIdler();                // re-engage the idler after keyboard entry

      // need to clear the keyboard buffer
      Serial.readString();          // clear the keyboard buffer

      startTime = millis();         // reset the start Time

    }

    feedFilament(STEPSPERMM);        // step forward 1 mm
    filamentDistance++;
    fStatus = digitalRead(filamentSwitch);             // read the filament switch on the mk3 extruder
    if (fStatus == 0) {
      // Serial.println("filament switch triggered");
      flag = 1;
      Serial.print("Filament distance traveled (mm): ");
      Serial.println(filamentDistance);

      switch (filamentSelection) {
        case 0:
          if (filamentDistance < f0Min) {
            f0Min = filamentDistance;
          }
          if (filamentDistance > f0Max) {
            f0Max = filamentDistance;
          }
          f0Distance += filamentDistance;
          f0ToolChange++;
          f0Avg = f0Distance / f0ToolChange;
          break;
        case 1:
          if (filamentDistance < f1Min) {
            f1Min = filamentDistance;
          }
          if (filamentDistance > f1Max) {
            f1Max = filamentDistance;
          }
          f1Distance += filamentDistance;
          f1ToolChange++;
          f1Avg = f1Distance / f1ToolChange;
          break;

        case 2:
          if (filamentDistance < f2Min) {
            f2Min = filamentDistance;
          }
          if (filamentDistance > f2Max) {
            f2Max = filamentDistance;
          }
          f2Distance += filamentDistance;
          f2ToolChange++;
          f2Avg = f2Distance / f2ToolChange;
          break;
        case 3:
          if (filamentDistance < f3Min) {
            f3Min = filamentDistance;
          }
          if (filamentDistance > f3Max) {
            f3Max = filamentDistance;
          }
          f3Distance += filamentDistance;
          f3ToolChange++;
          f3Avg = f3Distance / f3ToolChange;
          break;
        case 4:
          if (filamentDistance < f4Min) {
            f4Min = filamentDistance;
          }
          if (filamentDistance > f4Max) {
            f4Max = filamentDistance;
          }
          f4Distance += filamentDistance;
          f4ToolChange++;
          f4Avg = f4Distance / f4ToolChange;
          break;
        default:
          Serial.println("Error, Invalid Filament Selection");

      }
      // printFilamentStats();

    }
  }
  // feed filament an additional 32 mm to hit the middle of the bondtech gear
  // go an additional 32mm (increased to 32mm on 10.4.18)
  feedFilament(STEPSPERMM * 32);



  //#############################################################################################################################
  //# NEWEXPERIMENT:  removed the parkIdler() command on 10.5.18 to improve timing between 'T' command followng by 'C' command
  //#############################################################################################################################
  // parkIdler();              // park the IDLER (bearing) motor

  //delay(200);             // removed on 10.5.18
  //Serial1.print("ok\n");    // send back acknowledge to the mk3 controller (removed on 10.5.18)

}

void printFilamentStats() {
  Serial.print("F0 Min: ");
  Serial.print(f0Min);
  Serial.print("  F0 Max: ");
  Serial.print(f0Max);
  Serial.print("  F0 Avg: ");
  Serial.print(f0Avg);
  Serial.print("  F0 Length: ");
  Serial.print(f0Distance);
  Serial.print("  F0 count: ");
  Serial.println(f0ToolChange);

  Serial.print("F1 Min: ");
  Serial.print(f1Min);
  Serial.print("  F1 Max: ");
  Serial.print(f1Max);
  Serial.print("  F1 Avg: ");
  Serial.print(f1Avg);
  Serial.print("  F1 Length: ");
  Serial.print(f1Distance);
  Serial.print("  F1 count: ");
  Serial.println(f1ToolChange);

  Serial.print("F2 Min: ");
  Serial.print(f2Min);
  Serial.print("  F2 Max: ");
  Serial.print(f2Max);
  Serial.print("  F2 Avg: ");
  Serial.print(f2Avg);
  Serial.print("  F2 Length: ");
  Serial.print(f2Distance);
  Serial.print("  F2 count: ");
  Serial.println(f2ToolChange);

  Serial.print("F3 Min: ");
  Serial.print(f3Min);
  Serial.print("  F3 Max: ");
  Serial.print(f3Max);
  Serial.print("  F3 Avg: ");
  Serial.print(f3Avg);
  Serial.print("  F3 Length: ");
  Serial.print(f3Distance);
  Serial.print("  F3 count: ");
  Serial.println(f3ToolChange);

  Serial.print("F4 Min: ");
  Serial.print(f4Min);
  Serial.print("  F4 Max: ");
  Serial.print(f4Max);
  Serial.print("  F4 Avg: ");
  Serial.print(f4Avg);
  Serial.print("  F4 Length: ");
  Serial.print(f4Distance);
  Serial.print("  F4 count: ");
  Serial.println(f4ToolChange);

}

int isFilamentLoaded() {
  int findaStatus;

  findaStatus = digitalRead(findaPin);
  return (findaStatus);
}

//
// (T) Tool Change Command - this command is the core command used my the mk3 to drive the mmu2 filament selection
//
void toolChange( char selection) {
  int newExtruder;

  ++toolChangeCount;                             // count the number of tool changes
  ++trackToolChanges;

  Serial.print("Tool Change Count: ");
  Serial.println(toolChangeCount);


  newExtruder = selection - 0x30;                // convert ASCII to a number (0-4)

  if (newExtruder == filamentSelection) {  // already at the correct filament selection
#ifdef NOTDEF
    Serial.println("toolChange: Already at correction position !!!");
    if (toolChangeCount == 1) {         // only do this at the very beginning (1st time code)
      Serial.println("toolChange: 1st time processing only");
      idlerSelector(selection);
      colorSelector(selection);
      filamentLoadToExtruder();
    }
#endif

    if (!isFilamentLoaded() ) {            // no filament loaded
#ifdef NOTDEF
      Serial.println("toolChange: filament not currently loaded, loading ...");
#endif
      idlerSelector(selection);   // move the filament selector stepper motor to the right spot
      colorSelector(selection);     // move the color Selector stepper Motor to the right spot
      filamentLoadToExtruder();
      //loadFilamentToFinda();
    } else {
      Serial.println("toolChange:  filament already loaded to mk3 extruder");
    }

    //                               else {                           // added on 9.24.18 to
    //                                     Serial.println("Filament already loaded, unloading the filament");
    //                                     idlerSelector(selection);
    //                                     unloadFilamentToFinda();
    //                               }
     
  }  else {                                 // different filament position

    if (isFilamentLoaded()) {
      //**************************************************************
      // added on 10.5.18 to get the idler into the correct state
      // idlerSelector(currentExtruder);
      //**************************************************************
      Serial.println("Unloading filament");

      idlerSelector(currentExtruder);    // point to the current extruder
      unloadFilamentToFinda();          // have to unload the filament first
    }




    if (trackToolChanges > TOOLSYNC) {             // currently set to 5,  can be greater than 30 after debug
      Serial.println("Synchronizing the Filament Selector Head");
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // NEEDS TO BE MORE ELEGANT, MOVE TO THE RIGHT IS WAY TOO LARGE
      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      initColorSelectorPosition();              // reset the color selector
      activateColorSelector();                  // turn the color selector motor back on
      currentPosition = 0;                   // reset the color selector

      // colorSelector('0');                       // move selector head to position 0

      trackToolChanges = 0;

    }
#ifdef DEBUG
    Serial.println("Selecting the proper Idler Location");
#endif   
    idlerSelector(selection);
#ifdef DEBUG
    Serial.println("Selecting the proper Selector Location");
#endif
    colorSelector(selection);
#ifdef DEBUG
    Serial.println("Loading Filament: loading the new filament to the mk3");
#endif
    
    filamentLoadToExtruder();                // moves the idler and loads the filament


    filamentSelection = newExtruder;
    currentExtruder = selection;
    
  }
  
  // filamentLoadToExtruder();              // load the filament ALL the way to the bondtech gear
  parkIdler();                            // move the idler away 
  

}  // end of ToolChange processing


// part of the 'C' command,  does the last little bit to load into the past the extruder gear
void filamentLoadWithBondTechGear() {
  int findaStatus;
  long steps;
  int i;

  int stepCount;


  time3 = millis();



  findaStatus = digitalRead(findaPin);

  if (findaStatus == 0) {
    Serial.println("filamentLoadWithBondTechGear()  Error, filament sensor thinks there is no filament");
    return;
  }

  if ((currentExtruder < '0')  || (currentExtruder > '4')) {
    Serial.println("filamentLoadWithBondTechGear(): fixing current extruder variable");
    currentExtruder = '0';
  }


  idlerSelector(currentExtruder);        // this was not done properly before
  



#ifdef NOTDEF
  Serial.println("C:  320+450 steps");
#endif


  stepCount = 0;
  time0 = millis();
  //*******************************************************************************************
  // feed the filament from the MMU2 into the bondtech gear for 2 seconds at 10 mm/sec
  // STEPPERMM : 144, 1: duration in seconds,  21: feed rate (in mm/sec)
  // delay: 674 (for 10 mm/sec)
  // delay: 350 (for 21 mm/sec)
  //*******************************************************************************************
  for (i = 0; i < (STEPSPERMM * 1 * 21); i++) {

    digitalWrite(extruderStepPin, HIGH);  // step the extruder stepper in the MMU2 unit
    delayMicroseconds(PINHIGH);
    digitalWrite(extruderStepPin, LOW);
    delayMicroseconds(350);             // this was calculated in order to arrive at a 10mm/sec feed rate
    ++stepCount;
  }
  time1 = millis();


  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // copied from the MM-control-01/blob/master/motion.cpp routine
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //***********************************************************************************************************
  //* THIS CODE WORKS BUT IT LEADS TO SOME GRINDING AT THE MMU2 WHILE THE BONDTECH GEAR IS LOADING THE FILAMENT
  //***********************************************************************************************************
#ifdef NOTDEF
  for (i = 0; i <= 320; i++) {
    digitalWrite(extruderStepPin, HIGH);
    delayMicroseconds(PINHIGH);               // delay for 10 useconds
    digitalWrite(extruderStepPin, LOW);
    //delayMicroseconds(2600);             // originally 2600
    delayMicroseconds(800);              // speed up by a factor of 3

  }
  for (i = 0; i <= 450; i++) {
    digitalWrite(extruderStepPin, HIGH);
    delayMicroseconds(PINHIGH);               // delay for 10 useconds
    digitalWrite(extruderStepPin, LOW);
    // delayMicroseconds(2200);            // originally 2200
    delayMicroseconds(800);             // speed up by a factor of 3
  }
#endif

#ifdef DEBUG
  Serial.println("C Command: parking the idler");
#endif
  parkIdler();       // turn OFF the idler rollers when filament is loaded

  time2 = millis();

  printFilamentStats();   // print current Filament Stats

  Serial.print("Time 'T' Command Received: ");
  Serial.println(time4);
  Serial.print("Time 'T' Command Completed: ");
  Serial.println(time5);

  Serial.print("Time 'C' Command Received: ");
  Serial.println(time3);
  Serial.print("Time at MMU2 filament load start: ");
  Serial.println(time0);
  Serial.print("Time at MMU2 filament load end: ");
  Serial.println(time1);
  Serial.print("Time at Parking the Idler Complete: ");
  Serial.println(time2);
  Serial.print("Number of commanded steps to the Extruder: ");
  Serial.println(stepCount);


#ifdef DEBUG
  Serial.println("filamentLoadToExtruder(): Loading Filament to Print Head Complete");
#endif
  //####################################################################################################################
  //# this acknowledge back to the MK3 Controller is done by the calling program
  //####################################################################################################################
  // delay(200);
  // Serial1.print("ok\n");    // send back acknowledge to the mk3 controller

} // end of filamentLoad with BondTechGear() routine
