/*
Toolerator 3000
Tool changer for EmcoTurn 120
Early style turret, 8 tool position
Uses LMD18245 motor driver
Revised 8/24/15--was not indexing correctly.

Requires the elapsedMillis library

pins:
A0:  Forward/stop button switch
A1:  next position button switch
A2-A7:  not used

D0:  (tx)
D1:  (rx)
D2-D5:  M1-M4 of LMD18245
D6:  LMD18245 Comparator output (low if current exceeds limit)
D7:  LMD18245 Direction (low/Fwd: 1=src 2=sink; high/Rev: 1=sink, 2=src)
D8:  LMD18245 Brake (active high) (not used)
D9-D12:  S1-S4 (inverted) from encoder (see below)
D13:  not used

Misc:
M1-M4 programs maximum current.  3A * M/16 = current limit

position to inputs D9-D12
   D12-D9
P  D12-D9  Val
1  1000 0x08
2  1100 0x0c
3  0100 0x04
4  0110 0x06
5  0010 0x02
6  0011 0x03
7  0001 0x01
8  1001 0x09

(D9-D12 are inverted from encoder PCB outputs S1-S4)


What happens:
The encoder values above are correct for the actual position.  To go to a position,
go until the encoder reports the correct position, continue going forward
for a brief period to set the pawl solidly into the new position,
then go backwards to lock the pawl.

At roughly 4.26 sec/rev, this is 0.53 sec/position.  So go past the desired position for
about 25% more, or 130 msec.  This is enough time for the pawl to get into the new position.

Commands:

pushbutton switch 1 (corner of board), toggle forward/off
pushbutton switch 2, move to next position

Ascii commands:
V:  return software version / help
Q:  return current encoder reading ("1"..."8" ) and individual bits S1-S4
    note--might not correspond to actual position??
F:  motor forward until S
S:  stop motor
R:  reverse motor until S
1...8:  go to position, report position when done or X if error


*/

#include <elapsedMillis.h>

#define VERSION_STRING "Toolerator 3000 version 1.4 Cursing Cat"

// motor currents:  full=15=3A, reverse hold=4=0.8A, reverse=15=3A
#define MOTOR_FORWARD_CURRENT 15
#define MOTOR_REVERSE_CURRENT 13
#define MOTOR_REVERSE_HOLD_CURRENT 4
#define MOTOR_OFF_CURRENT 0

#define PIN_MOTOR_DIR 7
#define PIN_MOTOR_BRAKE 8
#define PIN_M1 2
#define PIN_M2 3
#define PIN_M3 4
#define PIN_M4 5
#define PIN_OVER_CURRENT 6
#define PIN_SW2 A0
#define PIN_SW1 A1
#define PIN_S1 9
#define PIN_S2 10
#define PIN_S3 11
#define PIN_S4 12

int currentPosition = 0; // unknown
int targetPosition = 0;
int targetPositionPlusOne; // next position past target
int seenTargetPosition;
enum states { initState, waitForInputState, moveForwardToPositionState, moveReverseToLockState, queryState, versionState,
  motorForwardState, motorReverseState, motorStopState, moveForwardState, keepMovingForwardState} state;
enum motorDirections {forward, reverse} motorDirection;
int motorCurrent=0;

// timing
elapsedMillis timer; // unsigned int
elapsedMillis switchDeadTime;

// once switch active, don't re-activate until deadtime passes
#define DEADTIME 300

// timeout in milleseconds
#define FORWARD_TIMEOUT 7000
#define REVERSE_TIMEOUT 2000
#define IGNORE_OVER_CURRENT_TIMEOUT 500
#define FORWARD_PAWL_TIME 200

// buttons
int switchState=0;

// map inputs on D9-D12 to encoder position 1...8
const byte encoderToPosition[]={0,7,5,6,3,0,4,0,1,8,0,0,2,0,0,0};


// set up motorCurrent and motorDirection before call
void setMotor()
{
  // set direction
  if( motorDirection==forward)
     digitalWrite(PIN_MOTOR_DIR,0);
  else
     digitalWrite(PIN_MOTOR_DIR,1);
 
  // set current
  digitalWrite(PIN_M1, motorCurrent & 0x01);
  digitalWrite(PIN_M2, motorCurrent & 0x02);
  digitalWrite(PIN_M3, motorCurrent & 0x04);
  digitalWrite(PIN_M4, motorCurrent & 0x08);
  return;
}  // setMotor()


// returns positon 1..8
int readPosition()
{
  int encoder=0;
  encoder = digitalRead(PIN_S1);
  encoder += digitalRead(PIN_S2)<<1;
  encoder += digitalRead(PIN_S3)<<2;
  encoder += digitalRead(PIN_S4)<<3;
  return(encoderToPosition[encoder]);
}  // readPosition()


void setup() 
{
  // init I/O pins
  pinMode(PIN_MOTOR_DIR,OUTPUT);
  pinMode(PIN_MOTOR_BRAKE,OUTPUT);
  pinMode(PIN_M1,OUTPUT);
  pinMode(PIN_M2,OUTPUT);
  pinMode(PIN_M3,OUTPUT);
  pinMode(PIN_M4,OUTPUT);
  pinMode(PIN_SW1,INPUT_PULLUP);
  pinMode(PIN_SW2,INPUT_PULLUP);
  pinMode(PIN_S1,INPUT_PULLUP);
  pinMode(PIN_S2,INPUT_PULLUP);
  pinMode(PIN_S3,INPUT_PULLUP);
  pinMode(PIN_S4,INPUT_PULLUP);
  pinMode(PIN_OVER_CURRENT,INPUT_PULLUP);
  Serial.begin(9600); // 9600 baud (via USB cable)
  state=initState;
  switchDeadTime=0;
}  // setup()


void loop() 
{
  byte b;
  int encoder;

  
  switch(state)
  {
    case initState:  // initial state
      // brake off (brake not used elsewhere)
      digitalWrite(PIN_MOTOR_BRAKE,0);
      // set motor to reverse, current to hold
      motorCurrent=MOTOR_REVERSE_HOLD_CURRENT;
      motorDirection=reverse;
      setMotor();      
      // print "T" for Toolerator 3000--ready message
      Serial.write('T');
      state=waitForInputState;
      break;
      
    // vqfsr1-8
    case waitForInputState:
      if (Serial.available() > 0)
      {
        b=Serial.read(); // reads 1 byte
        switch(b)
        {
          case 'Q':
            state=queryState;
            break;
          case 'V':
            state=versionState;
            break;
          case 'F':
            state=motorForwardState;
            break;
          case 'R':
            state=motorReverseState;
            break;
          case 'S':
            state=motorStopState;
            break;
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
            targetPosition=b-'0';
            if( currentPosition==targetPosition)
            {
              // no need to move--report success
              state=waitForInputState;
              Serial.write('0'+currentPosition);
            }
            else
            {
              state=moveForwardToPositionState;
              seenTargetPosition=0; // have not seen target position yet
              timer=0; // reset timeout timer
            }
            break;
          default:
            // unknown command
            Serial.write('?');
            // keep state
            break;
        } // switch(b)
      }  // if char available
      
      // now check switches
      // if switch 1, then toggle forward/off
      // if switch 2, move to next position
      if( (digitalRead(PIN_SW1)==LOW) && (switchDeadTime >= DEADTIME) )
      {
        switchState = !switchState;
        if( switchState ) // active?
        {
          state=motorForwardState;
        }
        else
          state=motorStopState;
        switchDeadTime=0; // reset dead time
      }
      
      if( (digitalRead(PIN_SW2)==LOW) && (switchDeadTime >= DEADTIME) )
      {
        state=moveForwardToPositionState;
        seenTargetPosition=0; // have not seen target position yet
        targetPosition= targetPosition+1;
        if( targetPosition > 8)
          targetPosition=1;
        timer=0; // reset timer for timeout
        switchDeadTime=0; // reset dead time
      }
      // note--no need to watch timeout in this state  
      break;

    case queryState:
      // what is current position from encoder?
      encoder=0;
      Serial.write( '0'+ readPosition() );
      encoder = digitalRead(PIN_S1);
      encoder += digitalRead(PIN_S2)<<1;
      encoder += digitalRead(PIN_S3)<<2;
      encoder += digitalRead(PIN_S4)<<3;
      Serial.print(" ");
      Serial.println(encoder,BIN);
      state=waitForInputState;
      break;
    
    case versionState:
      Serial.println(VERSION_STRING);
      state=waitForInputState;
      break;
    
    case motorForwardState:
      // set motor to forward, current to forward
      motorCurrent=MOTOR_FORWARD_CURRENT;
      motorDirection=forward;
      setMotor();      
      state=waitForInputState;
      break;
      
    case motorReverseState:
      // set motor to reverse, current to reverse
      motorCurrent=MOTOR_REVERSE_CURRENT;
      motorDirection=reverse;
      setMotor();      
      state=waitForInputState;
      break;
      
    case motorStopState:
      // set motor current off
      motorCurrent=MOTOR_OFF_CURRENT;
      motorDirection=reverse; // could be forward...
      setMotor();      
      state=waitForInputState;
      break;

    case moveForwardToPositionState:
      // keep moving forward direction
      motorCurrent=MOTOR_FORWARD_CURRENT;
      motorDirection=forward;
      setMotor();
      // have we seen the target position yet?
      if( readPosition() == targetPosition)
      {
        state=keepMovingForwardState;
        timer=0; // reset timer for going forward
      }
      if( timer >= FORWARD_TIMEOUT)
      {
        // timed out--took too long to get there
        Serial.write('X'); //error, timeout
        state=motorStopState; // or, could hang out in a dead state...
      }
      else // just waiting until we get there
      {
        // nothing
      }
      break;

    // keep going forward far enough to let pawl drop into new position
    // (the encoder can signal the target position before the pawl
    //  is ready to drop into place)
    case keepMovingForwardState:
      // long enough time?
      if( timer >= FORWARD_PAWL_TIME )
      {
        state=moveReverseToLockState;
        currentPosition=targetPosition; // since this may go away with reversing
        // it actually doesn't go away...
        timer=0; // reset timeout timer for reversing
      }
      break;

    case moveReverseToLockState:
      // keep moving reverse direction
      motorCurrent=MOTOR_REVERSE_CURRENT;
      motorDirection=reverse;
      setMotor();      
      // timed out?
      if( timer >= REVERSE_TIMEOUT )
      {
        // timed out--took too long to get there
        Serial.write('X'); //error, timeout
        state=motorStopState; // or, could hang out in a dead state...
      }
      // now check if motor is over current after we've given chance for motor current to
      // drop from an initial surge
      else if( (timer >= IGNORE_OVER_CURRENT_TIMEOUT) && (digitalRead(PIN_OVER_CURRENT)==LOW))
      {  // we've backed onto the lock, so engage holding current
        // set motor to reverse, current to hold
        motorCurrent=MOTOR_REVERSE_HOLD_CURRENT;
        motorDirection=reverse;
        setMotor();
        state=waitForInputState;
        // report success
        Serial.write( '0'+ currentPosition );        
      }
      else
      {
        // do nothing--keep reversing
      }      
      break;
      
    default: // unknown state
      Serial.write('X'); //error, timeout
      state=motorStopState; // or, could hang out in a dead state...
      break;
  } // switch state
  
} // loop()

// end of file
