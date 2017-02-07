#include <avr/io.h>
#include <avr/interrupt.h>

#include "usi_i2c_slave.h" //following: http://www.instructables.com/id/ATTiny-USI-I2C-The-detailed-in-depth-and-infor/?ALLSTEPS

int smDirectionPin = 3; //Direction pin
int smStepPin = 2; //Stepper pin

// Pin 3  & 4 connected to Limit switch out
#define LMT_PIN_UP 0
#define LMT_PIN_DOWN 1
#define PINA_LMT_UP (PINA & 0b00000001) 
#define PINA_LMT_DOWN (PINA & 0b00000010) 

//TODO: check direction
#define DIRECTION_UP 1
#define DIRECTION_DOWN 0

int lastDirection;

volatile bool swUp, swDown,pinChanged;

bool homed = false;

long motorPosition; //current position of the motor, only usable if homed. homing sets this to 0 if successful.
float moveSpeed;


//Define a reference to the I2C slave register bank pointer array
extern char* USI_Slave_register_buffer[];
unsigned int targetPosition = 0;

void setup() {
  //setup pins
  pinMode(smDirectionPin, OUTPUT);
  pinMode(smStepPin, OUTPUT);

  pinMode(LMT_PIN_UP, INPUT); //TODO: test with internal pullups, to simplify board
  pinMode(LMT_PIN_DOWN, INPUT);

  swUp = false;
  swDown = false;


  //Any logical change on INT0 generates an interrupt request.
  MCUCR = MCUCR & (0b11111100);
  MCUCR += 0b00000001;

  GIMSK |= (1 << PCIE0);
  PCMSK0 |= 0b00000011; //enable pin change interrupts on PA0 and PA1 (pins 13 and 12)

//  SREG |= 0b10000000; //Set SREG bit I to 1  
  sei();     // enable interrupts

  digitalWrite(smDirectionPin, DIRECTION_DOWN);
  lastDirection = DIRECTION_DOWN;


  //Assign the target value low byte to I2C internal address 0x00
  //Assign the target value high byte to I2C internal address 0x01
  USI_Slave_register_buffer[0] = (unsigned char*)&targetPosition;
  USI_Slave_register_buffer[1] = (unsigned char*)(&targetPosition)+1;

  USI_I2C_Init(0x40);
   
  seekHome();
}

//for I2C receive
void receiveEvent(int howMany) {
//  while (1 < Wire.available()) { // loop through all but the last
//    char c = Wire.read(); // receive byte as a character
//    Serial.print(c);         // print the character
//  }
//  int x = Wire.read();    // receive byte as an integer
//  Serial.println(x);         // print the integer
}

//Interrupt Service Routine
ISR(PCINT0_vect)
{
  pinChanged=true;
  swUp = (PINA_LMT_UP)!=0;
  swDown = (PINA_LMT_DOWN)!=0;
}

/**
 * This function will step the motor in the given direction, 
 * up to the given number of steps. Interrupted if any of the limit switches CHANGE state.
 * 
 * returns: number of steps actually taken.
 */

int stepMotor(int dir, int steps, float spd) {
  int stepsTaken = 0;
  
  if (lastDirection!=dir) {
    digitalWrite(smDirectionPin, dir); //todo : switch to fast (direct port manipulation)
    lastDirection=dir;
  }
  
  spd = 1 / spd * 70; //Calculating speed
  steps = abs(steps); //Stores the absolute value of the content in 'steps' back into the 'steps' variable

  /*Steppin'*/
  pinChanged = false;
  for (int i = 0; (i < steps) && !pinChanged ; i++) {
    digitalWrite(smStepPin, HIGH);
    delayMicroseconds(spd);
    digitalWrite(smStepPin, LOW);
    delayMicroseconds(spd);
    stepsTaken++;
  }

  return stepsTaken;
}

#define MAX_HOMING_STEPS 10000 
#define MAX_UP_HOMING_BACKOFF 100000
#define HOMING_SPEED 0.2
#define MAX_BACKOFF_STEPS 10000
#define BACKOFF_SPEED 0.05
#define EXTRA_BACKOFF_STEPS 80

void seekHome() {
  int stepsTaken;

/*** VVVV DEBUG ***/
  //check if we are hitting the up button...
  while (swUp && stepsTaken<MAX_UP_HOMING_BACKOFF ) {
    stepsTaken += stepMotor(DIRECTION_DOWN,MAX_UP_HOMING_BACKOFF-stepsTaken,BACKOFF_SPEED);
    delay(5);
  }
  delay(15);
  if (stepsTaken>=MAX_UP_HOMING_BACKOFF || swUp) {
    //we used more steps than expected, or the switch is not set as expected
    while(1); // HALT!
  }
/**** ^^^^ DEBUG ****/

  if (!swDown) {
    stepsTaken = stepMotor(DIRECTION_DOWN,MAX_HOMING_STEPS,HOMING_SPEED);
  }
  delay(15);
  if (stepsTaken>=MAX_HOMING_STEPS || !swDown) {
    //we used more steps than expected, or the switch is not set as expected
    while(1); // HALT!
  }

  stepsTaken = stepMotor(DIRECTION_UP,MAX_BACKOFF_STEPS,BACKOFF_SPEED);
  delay(15);

  if (stepsTaken>=MAX_BACKOFF_STEPS || swDown) {
    while(1); //HALT!
  }

  stepsTaken = stepMotor(DIRECTION_UP,EXTRA_BACKOFF_STEPS,BACKOFF_SPEED);
 
  motorPosition = 0;
  homed = true;
}

void runToPosition() {
  long steps = targetPosition - motorPosition;
  int dir = (steps<0) ? DIRECTION_DOWN : DIRECTION_UP;
  int stepsTaken = stepMotor(dir, abs(steps), moveSpeed);
  motorPosition = motorPosition + stepsTaken * ((dir==DIRECTION_DOWN)?-1:1);
}

void loop() {  
  //TODO: Notice, right now, run to position is blocking. Meaning, you cannot change direction mid motion... and only the 'latest' target will be used each time... no queue
  runToPosition();
}


