#include <avr/io.h>
#include <avr/interrupt.h>

//using https://github.com/eriksl/usitwislave

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

long motorPosition; //current position of the motor, only usable if homed. homing sets this to 0 if successful.
long targetPosition;
float moveSpeed;

int lastDirection;

volatile bool swUp, swDown,pinChanged;

bool homed = false;


static void twi_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                uint8_t *output_buffer_length, uint8_t *output_buffer)
{
    if(input_buffer_length > 0){ //we're getting an instruction from master
        if(*input_buffer == 1){
            //do something
        }
        if(*input_buffer == 2){
            //do something else
        }

    }else{// we're getting a write request from master
        *output_buffer_length = 8; // set 8 bit buffer size
        *output_buffer = 0x03; // send the device ID
    }
}

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

  usi_twi_slave(0x03, 1, *twi_callback, 0);

  seekHome();
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
  targetPosition = 5000;
  moveSpeed = 0.5;
  delay(3000);
  while (1) {
    runToPosition(); 
  }
}


