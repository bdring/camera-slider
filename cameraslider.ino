
#include <TimerOne.h>      // used for interrupt
#include <avr/eeprom.h>    // used to save settings to non volitile memory

/*
================ MakerSlide Camera Slider Controller ===============

This program provides a simple method to control a stepper motor for 
setting up and running a camera up and down a track.  This outputs 
step and direction signals to control a driver such as the cheap and
simple pololu A4988 drivers.

This is basically a 1 axis CNC controller, but instead of G-Code it
uses a simpler set of codes to enter the motion program.  It also
allows you set the acceleration of each move.

License:

Creative Commons 3.0 Attribution - Share-Alike.

This means you can use and modify it however you want for free, but
any changes must be share with this same license and you must attribute the 
origial design to me.

Bart Dring
www.buildlog.net
12/2011



*/

// stepper driver pins
//
// CHANGE THESE TO YOUR ARDUINO SETUP
//

#define BOARD_BUILDLOG 1
#define BOARD_ITEAD 2

#define STEPPER_SHIELD_TYPE BOARD_BUILDLOG

#if STEPPER_SHIELD_TYPE == BOARD_BUILDLOG
  #define PIN_X_STEP 2         // pin connected to step signal of stepper board
  #define PIN_X_DIR  3         // pin connected to direction signal of stepper board
  #define PIN_DRIVER_DISABLE 8 // high disables drivers
  #define DRIVER_DISABLE true  // Set to true if output high disables your driver..false if low disables it.
#endif 

#if STEPPER_SHIELD_TYPE == BOARD_ITEAD
  #define PIN_X_STEP 2         // D2 pin connected to step signal of stepper board
  #define PIN_X_DIR  3         // D3 pin connected to direction signal of stepper board
  #define PIN_DRIVER_DISABLE A2 // high disables drivers
  #define DRIVER_DISABLE false  // Set to true if output high disables your driver..false if low disables it.
#endif


#define PIN_START_SWITCH 11  // this is the switch used to run the sotred program (high = run)
#define DRIVER_DISABLE true  // Set to true if output high disables your driver..false if low disables it.
#define STEP_PULSE_ON true   // Set to true if your stepper driver want (usually true unless certain opto isolators are used) 
// Note: If your motor direction is different from what you want swap the wires on one pole of you motor, or modify the code

#define ENGINE_RATE 40000                     // Hz of Timer1 interrupt
#define ENGINE_RATE_SQUARED 1600000000.00     // This needs to be written as a float for the code to compile correctly
#define SPEED_OFFSET 16777216                 // 2^24 The is the offset of the accumulator 2^24 units = 1 step

// these defaults are only used until your values are saved in EEPROM
#define MAX_SPEED 50000
#define MAX_ACCEL 50000
#define DEFAULT_MAX_SPEED 1000
#define DEFAULT_MAX_ACCEL 1000

// these are defined to make code easier to read and can be applied to the current position directly during moves
#define DIRECTION_FORWARD 1   
#define DIRECTION_REVERSE -1 

// serial communications stuff
#define RX_BUFF_SIZE    30 
#define CARRIAGE_RETURN 13
#define DEFAULT_RATE 300      // conservatively slow
#define INPUT_BUFFER_SIZE 50
#define BAUD_RATE 9600

#define MOVE_ARRAY_MAX 24     // this sets the number of program lines.  It can be increased, but will affect RAM and EEPROM usage

// EEPROM locations of saved parameters (4 bytes each)
#define EEPROM_LOC_RESOLUTION   0  // steps millimeter
#define EEPROM_LOC_MAX_SPEED   EEPROM_LOC_RESOLUTION + 4
#define EEPROM_LOC_MAX_ACCEL   EEPROM_LOC_MAX_SPEED + 4
#define EEPROM_LOC_LOWER_LIMIT EEPROM_LOC_MAX_ACCEL + 4
#define EEPROM_LOC_UPPER_LIMIT EEPROM_LOC_LOWER_LIMIT + 4

// EEPROM location of move program
#define EEPROM_LOC_MOVE_PROG   40  // gives room for growth of basic parameters 

long currentLocation;        // store current location in steps so we can use absolute moves
int  motionDirection;        // what direction do we move 1 = fwd -1 = rev
boolean bEnableMotion;       // this turns on/off the 25kHz Pulse Engine
unsigned long accumulator;   // this is what we use to count up to the next step
unsigned long accel;         // accel rate in accum units per engine tick
signed long targetLocation;  // target position in steps

float moveAccelSteps;             // accel rate in steps / sec /sec (of current move)
unsigned long moveMaxSpeedSteps;  // max speed in steps /sec (of current move)
unsigned long maxSpeed;           // max speed in accum units per tick (of current move)
long decelLocation;               // pre-calculated point we start to decelerated
short bDecel;                     // in decel mode
long currentSpeed;                // current speed in accum units per engine tick
long dwellTime;                   // dwell length in millisocond we count up to this number  
short bMoveProgram;               // this is true while we are executing a move program
long accelDist;

int programIndex;    // The current line of the motion program

String inputString = "";         // a string to hold incoming data (from user)
boolean stringComplete = false;  // whether the string is complete (has a carriage return)

// data structure for the move program lines
typedef struct {
  long destination;
  long rate;
  long accel;
} moveCommand;

// data structure to represent the entire system including saved program
// A single structure makes the EEPROM read/save so freaking easy
typedef struct {
  long maxSpeed;
  long maxAccel;
  moveCommand moveProgram[MOVE_ARRAY_MAX];
} cameraSlider;

cameraSlider myCamSlider;

void setup()
{
  bEnableMotion = false;
  dwellTime = 0;
  currentLocation = 0;   
  programIndex = MOVE_ARRAY_MAX;
  bMoveProgram = false;
  
  eeprom_read_block((void*)&myCamSlider, (void*)0, sizeof(myCamSlider));  // read any saved values
  
  // cleanup in case EEPROM is fresh or erased
  if (myCamSlider.maxSpeed < 1  || myCamSlider.maxSpeed > MAX_SPEED)
    myCamSlider.maxSpeed = DEFAULT_MAX_SPEED;
    
  if (myCamSlider.maxAccel < 1 || myCamSlider.maxAccel > MAX_ACCEL)
    myCamSlider.maxAccel = DEFAULT_MAX_ACCEL;
  
  if (myCamSlider.moveProgram[0].destination < 0)
     clearProgram();
  // end EEPROM read cleanup   
     
  
  // setup output pins
  pinMode(PIN_X_STEP, OUTPUT);
  pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_DRIVER_DISABLE,OUTPUT);
  pinMode(PIN_START_SWITCH, INPUT);
  
  digitalWrite(PIN_DRIVER_DISABLE, DRIVER_DISABLE); // start with motors disabled so you can push the carriage
  
  Timer1.initialize(1000000 / ENGINE_RATE);         // initialize timer1 and set period
  Timer1.attachInterrupt(timerInterruptCallback);  // attaches timerInterruptCallback() as a timer overflow interrupt
  
  // start up serial communications
  inputString.reserve(INPUT_BUFFER_SIZE);
  Serial.begin(BAUD_RATE);
  
  // default values in case moves are missing parameters 
  moveMaxSpeedSteps = myCamSlider.maxSpeed;
  moveAccelSteps = myCamSlider.maxAccel;
           
  // hello user...
  showMenu();
  prompt();

}


// The standard arduino loop only handles user input.  All motion is handled in the timer interrupt
// You can enter commands while the motion is running
void loop()
{ 
  // stringComplete means the users has sent a carriage return
  if (stringComplete) {
    stringComplete = false; // prevent this loop from entering again while we process the command
    inputString.toUpperCase();  // Allows lowercase to also be recognized
    processCommand();
  }
  
  // look for start switch
  
  
  
    if (digitalRead(PIN_START_SWITCH))
    {
      // debounce      
      while (digitalRead(PIN_START_SWITCH))
      {
      }
      delay(100);
      
      startMoveProgram();
    }
}

 
/* ============== Timer Interrupt ===========================

   This is the pulsing engine.
   It uses "accumulator" to count up until it is time to take a step
   The amount "accumulator" increases at each interrupt is due to the current speed.
   Current speed changes due to the acceleration.
   Acceleration stopss when we reach full speed
   At some point the motion gets to a deceleration point and then accel subtracts from speed.
   
   Each step turns the pulse on.  The next time through the interrupt the pulse turns off.  This yields a 
   rather long pulse, but does not require the use of delay() or another timer.
   
   The motor should never reach the speed where there is a pulse at each interrupt so there will
   always be some amount off pulse off time.
   
*/

void timerInterruptCallback()
{
  digitalWrite(PIN_X_STEP, ! STEP_PULSE_ON); // turn off step pulse pin
  
  // if we are in a dwell rather than a move do this and return
  if (dwellTime > 0)
  {
     dwellTime--;
     if (dwellTime == 0)
     {
        doMotion();  
     }  
     return;
  }
  
  // we only do anything if motion is enabled and we are not at the target position
   if (bEnableMotion && (currentLocation != targetLocation) )
   {
      
      if (currentLocation == decelLocation)
         bDecel = true;
      
      if (!bDecel)
      {
         // if we are not at max speed continue to accel
         if (currentSpeed < maxSpeed)
         {
            currentSpeed += accel;
         }
      }
      else // decel
      {
         currentSpeed -= accel;
         if (currentSpeed <= 0) // never let it get to zero otherwise we could get stuck 
         {
            currentSpeed = accel; 
         }      
      }
      
      accumulator += currentSpeed;
   
      //see if we have accumulated enough for a step step
      if (accumulator > SPEED_OFFSET)
      {
         currentLocation += motionDirection;          
         digitalWrite(PIN_X_STEP, STEP_PULSE_ON); //  turn on step pin        
         accumulator -= SPEED_OFFSET;  // subtract the offset this keeps whatever we have over SPEED_OFFSET 
      }
   }
   else
   {
      if (bEnableMotion)
      {
        bEnableMotion = false;
        if (bMoveProgram)     
          doMotion(); // look for next program sequence
        else
          prompt();
      }
   }
}
 




/* ============ doMotion ==========================================

     
   The speed is going to follow a profile like this
   
                      D
                      E
                      C
                      E
                      L
                      
  S          **********    <--- MAX SPEED
  P        *            *
  E      *                *
  E    *                    *
  D  *                        *
       ------  TIME ----------
   
  There are a lot of complex and CPU consuming calculations to follow this profile,
  so we will do them once for each move before we enable the pulsing engine.  
  These are done in this function.
  
  The calculations are done over many lines to make it more readable, control
  how the compiler handles it and make sure no variables overrun. 
  
*/
void doMotion()
{
  
  float tempAccel;
  float tempMaxSpeed;
  float t;
  long distToTarget;
  
  digitalWrite(PIN_DRIVER_DISABLE, !DRIVER_DISABLE); // make sure motors are enabled
  
  if (bMoveProgram)
  {
    
    Serial.print("Program index:");
    Serial.println(programIndex);
   
    if ( (myCamSlider.moveProgram[programIndex].destination == 0 && myCamSlider.moveProgram[programIndex].rate == 0) || programIndex == MOVE_ARRAY_MAX)     
    {
      Serial.print("End Of Program");
      bMoveProgram = false;
      prompt();
      return;
    }
    
    
    // see if this is a dwell
    if (myCamSlider.moveProgram[programIndex].rate == 0)
    {
      dwellTime = myCamSlider.moveProgram[programIndex].destination * (ENGINE_RATE / 1000);      
      programIndex++;
      return;
    }
    
    moveMaxSpeedSteps = myCamSlider.moveProgram[programIndex].rate;
    targetLocation = myCamSlider.moveProgram[programIndex].destination;   
    moveAccelSteps = myCamSlider.moveProgram[programIndex].rate;
    
    programIndex++;
    
  }  // ============= end if move program
  
  
  // check values against maximums
  if (moveMaxSpeedSteps > myCamSlider.maxSpeed) //systemMaxSpeed)
  {
    Serial.println("Reduced speed to max");
    moveMaxSpeedSteps = myCamSlider.maxSpeed;
  }
  
  if (moveAccelSteps > myCamSlider.maxAccel)
  {
    Serial.println("Reduced accel to max");
    moveAccelSteps = myCamSlider.maxAccel;
  } 
  
  distToTarget = abs(targetLocation - currentLocation); // figure out the distance we will travel on this move
  
  tempAccel = (float)moveAccelSteps;// * stepsInch; // in steps per sec   
  tempAccel = tempAccel / (ENGINE_RATE_SQUARED);  
  tempAccel = tempAccel * SPEED_OFFSET; 
  accel = (long)tempAccel;
    
  tempMaxSpeed = moveMaxSpeedSteps;
  tempMaxSpeed = (tempMaxSpeed / ENGINE_RATE) * SPEED_OFFSET;   
  maxSpeed = (long)tempMaxSpeed;
   
  // determine Accel distance
  //     d = 1/2 a t^2
  t = (float)moveMaxSpeedSteps / (float)moveAccelSteps;
  accelDist = (moveAccelSteps / 2) * t * t;
  
  // determine the direction
   if (targetLocation > currentLocation)
   {
      motionDirection = DIRECTION_FORWARD;
      digitalWrite(PIN_X_DIR, true);
      
     if (accelDist >= distToTarget / 2)
         decelLocation = targetLocation - distToTarget / 2;
      else
         decelLocation = targetLocation - accelDist;
      
   }
   else
   {
      motionDirection = DIRECTION_REVERSE;
      digitalWrite(PIN_X_DIR, false);
      
      if (accelDist >= distToTarget / 2)
         decelLocation = targetLocation + distToTarget / 2;
      else
         decelLocation = targetLocation + accelDist;
      
   }
   
   bDecel = false;  // this is true when in the deceleration zone
    
   currentSpeed = 0; // initialize this 
   bEnableMotion = true; // after all calculations are done, set this to true to enable the pulse engine
}


// this fires whenever there are characters in the buffer
void serialEvent()
{
    while (Serial.available()) 
    {
      // get the new byte:
      char inChar = (char)Serial.read(); 
      Serial.print(inChar);  // echo it to the screen
      
      inputString += inChar;  // add it to the inputString:
            
      if (inChar == '\n')
        stringComplete = true;  // if the incoming character is a newline, set a flag
  

    } // while
}

void processCommand()
{
      long lineNumber;
      long destination;
      long rate;
      long accel;
      
      // switch on the first character
      switch (inputString[0])
      {
        case '0':
          Serial.print("Zero Set");
          currentLocation = 0;
          prompt();
        break;
        
        case 'A':
          
          if (getStringItem(2) > 0)
          {
             myCamSlider.maxAccel = getStringItem(2);
             Serial.print("Max accel set");
          }
          else
          {
            Serial.print("Bad accel value");
          }
          prompt();
        break;
        
        case 'R':
          
          if (getStringItem(2) > 0)
          {
             myCamSlider.maxSpeed = getStringItem(2);
             Serial.print("Max speed set");
          }
          else
          {
            Serial.print("Bad speed value");
          }
          prompt();
        break;
        
        case 'S':
          Serial.println("Stopped");
         
         
          /* slow stop
          targetLocation += (accelDist * motionDirection);
          bDecel = true;
          */
            
          targetLocation = currentLocation;  // just tell the pulse engine we are there
          programIndex = MOVE_ARRAY_MAX;    // this will terminate any running move program
          prompt();
        break;
       
        case 'D':
          Serial.print("Motor Disabled");
          digitalWrite(PIN_DRIVER_DISABLE,true);
          prompt();
        break;
        
        case 'E':
          Serial.print("Motor Enabled");
          digitalWrite(PIN_DRIVER_DISABLE,false);
          prompt();
        break;
          
        case 'G':  // start the move program
          startMoveProgram();
        break;
        
        case 'H':
          Serial.println("Returning to 0");
          
          if (getStringItem(2) != 0)
           {
             moveMaxSpeedSteps = getStringItem(3);
           }
           
           if (getStringItem(3) != 0)
           {
             moveAccelSteps = getStringItem(4);
           }
           
          targetLocation = 0;
          doMotion(); 
        break;
        
        case 'I':  // current information
          Serial.print("Current location:");
          Serial.print(currentLocation);
          Serial.print("\nMax Speed:");
          Serial.print(myCamSlider.maxSpeed);
          Serial.print("\nMax Accel:");
          Serial.println(myCamSlider.maxAccel);
          
          prompt();
              
        break;
        
        case 'J': // jog
          targetLocation = getStringItem(2);
          
         // no target location means stop jog
         if (targetLocation == 0 && targetLocation != currentLocation)
         {
            targetLocation += (accelDist * motionDirection);
            bDecel = true;
            prompt();
         }
         else
         {
           
         //set target location to a huge number in the jog direction
          if (targetLocation > 0 )
            targetLocation = 1000000;
          else
            targetLocation = -1000000;
          
          
           if (getStringItem(3) != 0)
           {
             moveMaxSpeedSteps = getStringItem(3);
           }
           
           
           // it will use the current accel
           
           prompt();
           doMotion();  
         }
            
        break;
        
        case 'P':
          displayProgram();
          prompt();
        break;
        
        case 'C':  // clear program
          Serial.print("Program cleared");
          clearProgram();
          prompt();
        break;
          
        case 'V':
          Serial.print("Saving...");
          eeprom_write_block((const void*)&myCamSlider, (void*)0, sizeof(myCamSlider));
          prompt();
        break;
        
        case 'L':  // edit the line of a program
          // add move command
          
          lineNumber = getStringItem(2);
          
          if (lineNumber > MOVE_ARRAY_MAX)
          {
            Serial.print("Error: Line number too large");
            break;
          }
          
          destination = getStringItem(3);
          rate = getStringItem(4);          
          accel = getStringItem(5);
          
          Serial.print("Edited line #");
          Serial.print(lineNumber);
          
          Serial.print(" Destination:");
          Serial.print(destination);
          
          
          Serial.print(" Rate:");
          Serial.print(rate);
          
          
          Serial.print(" Accel:");
          Serial.println(accel);
         
          myCamSlider.moveProgram[lineNumber].destination = destination;
          myCamSlider.moveProgram[lineNumber].rate = rate;
          myCamSlider.moveProgram[lineNumber].accel = accel;
          
          displayProgram();
          
        break;
        
        case 'M': // move to
           
           targetLocation = getStringItem(2);
                                
           if (getStringItem(3) != 0)
           {
             moveMaxSpeedSteps = getStringItem(3);
           }
           
           if (getStringItem(4) != 0)
           {
             moveAccelSteps = getStringItem(4);
           }
         
           doMotion(); 
        break;
        
        case '?':
           showMenu();
           prompt();
        break;
        
        default:
          Serial.println("Unknown command");
          prompt();
        break;
        
        
      } // switch
      
      // done processing command
      //prompt();
      inputString = "";  // clear the string
}



/*  =================== getStringItem ====================

The format of the strings from the user look like this.

L 1111 2222 3X3 444

The format is: A single letter followed by a space delimited set of numbers.

This will return any of the numbers with a base of 1, so...

getStringItem(2) would return 1111 using the example string above.

It will return 0 for all errors or missing strings

getStringItem(4) & getStringItem(7) would both return 0

*/
long getStringItem(int itemNumber)
{
  
  String testString = String(inputString);
  char buf[30];
  
  int index = 0;
  int startIndex = 0;
  int endIndex = 0;
  int delimiterCount = 0;
  
  if (itemNumber == 1) // the first item always starts at position 0
  {
      endIndex = testString.indexOf(' ');
       
      testString.substring(0,endIndex).toCharArray(buf,endIndex);
      long i = atol(buf) ;
      return i;
  }
  else
  {
    while (index < testString.length())
    {
        if (testString.charAt(index) == ' ')
        {
           delimiterCount++;
           if (delimiterCount == (itemNumber - 1))
           {
             startIndex = index + 1;
           }
           else if (delimiterCount == itemNumber)
           {
             endIndex = index;
             
             testString.substring(startIndex,endIndex).toCharArray(buf,7);             
             
             return atol(buf);
           }
        }
        index++;
    }
    // we got to the end
    endIndex = testString.length();
    testString.substring(startIndex).toCharArray(buf,7);
    return atol(buf);
  }
}

/*

Each command is string of parameters separated by a single space.
The first parameter is the single character command.
The rest of the parameters are numeric.
Many of the parameters are optional.  The Move command, for example requires the destination, but
will use default speed and acceleration values if they are not provided.

The command will be acted upon receipt of the carriage return.

A move program is entered by editing one line of the move prgram at a time.
The first line is number 0
"L 0 2000 3000 1500" would make the first move line be move to 2000 with speed 3000 and accelration of 1500.
You can use up to MOVE_ARRAY_MAX lines.
To end a move program add a line with 0 for destination and speed
"L 6 0 0" This would end the program and only run lines 0 through 5.
If you want to pause or dwell for a while enter a line with zero for speed and put the duration where the destination goes.
"L 3 2000 0"  This would pause for 2000 milliseconds.

*/
void showMenu()
{
    Serial.print("\n\nMakerSlide: Camera Slider Control Program 2011-2013 GPL V2");
    Serial.print("\n0 = Set Current Location as 0");
    Serial.print("\nS = Stop now!");
    Serial.print("\nD = Disable Motor");
    Serial.print("\nE = Enable Motor");
    Serial.print("\nH = Home (Move to 0)");
    Serial.print("\nM = Move to ..(M Dest Speed Accel)");
    Serial.print("\nJ = Jog until stopped ('J 1' for positive motion, 'J -1' for negative, 'J' to stop )");
    Serial.print("\nI = Info (current parameters)");
    Serial.print("\nG = Go (start program)");
    Serial.print("\nP = Show Program");
    Serial.print("\nC = Clear Program");
    Serial.print("\nL = Edit Line.  Format: L Line# Dest Speed Accel)  Ex: L 0 2000 3000 1500");
    Serial.print("\n      Use 0 for destination and speed to indicate end of program");
    Serial.print("\n      Use 0 for speed to indicate a pause.  Dest is pause in milliseconds");
    Serial.print("\nR = Set Max Speed");
    Serial.print("\nA = Set Max Accel");
    Serial.print("\nV = SaVe to EEPROM"); 
    Serial.print("\n? = Redisplay this menu\n");
    
}


/*  =================== displayMoveProgram =====================

This prints the shows current move menu to the user vai serial port

*/
void displayProgram()
{
  int i;
  for (i=0; i<MOVE_ARRAY_MAX; i++)
  {
    if (myCamSlider.moveProgram[i].destination == 0 && myCamSlider.moveProgram[i].rate == 0)
    {
      if (i == 0)
        Serial.print("No program lines");
      
      return;
    }
    Serial.print("Line:");
    Serial.print(i);
    if (myCamSlider.moveProgram[i].rate == 0)
    {
      Serial.print(" Dwell:");
      Serial.println(myCamSlider.moveProgram[i].destination);
    }
    else
    {
       Serial.print(" Destination:");
       Serial.print(myCamSlider.moveProgram[i].destination);
       Serial.print(" Rate:");
       Serial.print(myCamSlider.moveProgram[i].rate);
       Serial.print(" Accel:");
      Serial.println(myCamSlider.moveProgram[i].accel);
    }
  }
}

void clearProgram()
{
  for (int i=0; i < MOVE_ARRAY_MAX; i++)
          {
            myCamSlider.moveProgram[i].destination = 0;
            myCamSlider.moveProgram[i].rate = 0;
            myCamSlider.moveProgram[i].accel = 0;
          }
}

// 
void startMoveProgram()
{
  Serial.println("Starting move program");          
  programIndex = 0;
  bMoveProgram = true;
  doMotion(); 
}

// prompt shows the last command is done.
// you can enter commends before the prompt comes ... like stop
void prompt()
{
  Serial.print("\nMakerSlide>");
}
