#include "input.hpp"
#include "mbed.h"
#include "definitions.hpp"
#include <Queue.h>
#include <Thread.h>
#include <Mail.h>
#include "motorControl.hpp"
#include "SHA256.h"
#include <inttypes.h>
#include <stdio.h>

void motorControlLoop();
void motorOut(int8_t driveState, int32_t pwmPeriod);
void motorISR();
void motorEncoderISR();
inline int8_t readRotorState();
void motorOut(int8_t driveState, int32_t pwmPeriod);
void motorCommutateOpenLoop(int direction);
void setMusicFrequency(int period_us);
void setBridgePWMFrequency(int period_us);
void serialISR();
void pollSerialQueue();
void pollOutMailBox();
void putOutput(uint64_t data, bool isHashCount);

bool integralLatch = 0;
// Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
// Drive state to output table
const int8_t driveTable[] = {0x12, 0x18, 0x09, 0x21, 0x24, 0x06, 0x00, 0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are
// not valid
const int8_t stateMap[] = {0x07, 0x05, 0x03, 0x04, 0x01, 0x00, 0x02, 0x07};
// const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07};
// //Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin
int8_t lead = 2; // 2 for forwards, -2 for backwards

// Status LED
DigitalOut led1(LED1);

// Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

InterruptIn encoderPinA(CHApin);
DigitalIn encoderPinB(CHBpin);

// Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);


//DigitalOut MYPIN(D13);


// Declrations
volatile bool speedFlag = 0;
volatile int8_t isrCounter = 0;
int8_t orState = 0; // Rotot offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;

int32_t highPrecisionCounter = 0; //32
int32_t oldHighPrecisionCounter = 0; //32
int32_t motorPosition; //64
float currentSpeed = 0; // Current speed in RPS

volatile int64_t deltaHighPrecisionCounter = 0;

Ticker controlLoopTicker;

int32_t pwmDutyCycle = 0;
float pwmTimeOn = 0;

MotorParams mp;

// Speed PID Declaration Parameters
float speedSetpoint = 0; // This is now always controlled by position loop
float lastError = 0; // used for I term in PID
float errorA = 0;
float accError = 0;

// Position PID Declaration Parameters
// int32_t posSetpoint = -500000;
int32_t pos_lastError = 0; // used for I term in PID
int32_t pos_errorA = 0;
int32_t pos_accError = 0;

int32_t MusicPeriod = 225; // Range in our octave is 127 - 225us

RawSerial rawSerial(SERIAL_TX, SERIAL_RX);
rtos::Queue<void, MAX_CHARS> serialQ;
InputBuf ibuf;
InputDecoder inputDecoder(&ibuf, &mp, &rawSerial);
rtos::Mail<Output, OUT_MAIL_SIZE> outMailBox;

uint8_t sequence[]= {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
                    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
                    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
                    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
                    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
                    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint64_t* key = (uint64_t*) ((int) sequence + 48);
uint64_t* nonce = (uint64_t*) ((int) sequence + 56);
uint8_t hash[32];

rtos::Thread inputThread;
rtos::Thread outputThread;

// TODO: Startup lead is compensated for our setup. Do this generally for any
// motor?
//Serial pc(SERIAL_TX, SERIAL_RX);
/*************************************************************Main****************************************************/
int main() {
  ibuf.charCount = 0;
  mp.speedCap = 0;
  mp.position = 0;
  mp.posUpdated = 0;
  mp.key = *key;
  // Attach the control loop to an timer interrupt
  controlLoopTicker.attach_us(&motorControlLoop, 100000);
init_rrs(&rawSerial);                                                                                                                                                     
  // Set Bridge PWM Frequency
  setBridgePWMFrequency(100);

  rawSerial.attach(&serialISR);
  // Initialise the serial port
  
  rawSerial.puts("Hello\n\r");

  // Run the motor synchronisation
  orState = motorHome();
  //pc.printf("Rotor origin: %x\n\r", orState);
  // orState is subtracted from future rotor state inputs to align rotor and
  // motor states
  inputThread.start(pollSerialQueue);
  outputThread.start(pollOutMailBox);
  // Attach the hall effect + encoder sensors to interrupts
  I1.rise(&motorISR);
  I2.rise(&motorISR);
  I3.rise(&motorISR);
  I1.fall(&motorISR);
  I2.fall(&motorISR);
  I3.fall(&motorISR);
  encoderPinA.rise(&motorEncoderISR);

  // Commutate the motor in openloop using only current rotor + disk postion and
  // offset. This will keep running until a few encoder edges are detected
  //motorCommutateOpenLoop(mp.position > 0 ? 1 : 0); // 0 for minus, 1 for plus

  // Poll the rotor state and set the motor outputs accordingly to spin the
  // motor
  /*
  while (1) {
    static char buf[50];
    sprintf(buf, "%d,%0.2f,%0.2f,%d\n\r", highPrecisionCounter, currentSpeed, mp.speedCap, mp.position);
    rawSerial.puts(buf);
    wait(0.5);
  }
  */
  SHA256 sha;
  Timer hashCountTimer;
  hashCountTimer.start();
  int currentHashCount = 0;
  while(true) {
    mp.keyLock.lock();
    uint64_t currentKey = mp.key;
    mp.keyLock.unlock();
    if(currentKey != *key) {
        *key = currentKey;
        *nonce = 0;
    }
    sha.computeHash(hash, sequence, 64);
    if(hash[0] == 0 && hash[1] == 0) {
        putOutput(*nonce, false);
    }
    currentHashCount++;
    if(hashCountTimer > 1.0f) {
        putOutput(currentHashCount, true);
        hashCountTimer.reset();
        currentHashCount = 0;
    }
    (*nonce)++;
   
 }

}

// Convert photointerrupter inputs to a rotor state
//***********************************************************************************************************************************************************
inline int8_t readRotorState() { return stateMap[I1 + 2 * I2 + 4 * I3]; }

//************************************************************************MOTOR
// CONTROL***********************************************************************************
void motorControlLoop() {
  if(mp.posUpdated) {
    mp.posUpdated = false;
    mp.speedCap *= mp.position * mp.speedCap < 0 ? -1 : 1;
    mp.position += highPrecisionCounter;
  }
  deltaHighPrecisionCounter = highPrecisionCounter - oldHighPrecisionCounter;
  oldHighPrecisionCounter = highPrecisionCounter;

  /*POSITION CONTROL*/
  pos_errorA = mp.position - highPrecisionCounter; // motorPosition;
  pos_accError = pos_accError + pos_errorA;        // Integral accumulator

  // Anti Windup for Position loop - However we are not really using it
  if (pos_accError > 100) {
    pos_accError = 100;
  } else if (pos_accError < -100) {
    pos_accError = -100;
  }

  int32_t positonLoopOutput =
      ((POSITION_KP * pos_errorA) + POSITION_KD * (pos_errorA - pos_lastError) +
       POSITION_KI * pos_accError) /
      1000;
  pos_lastError = pos_errorA; // Save current error to calcuate derivative term
                              // next iteration

  // Logic to determine input of speed loop, ie cascaded position loop -> speed
  // loop or just speed loop

  // Cascade mode - speed loop is driver by position loop. Speed loop can be
  // simple firt order with KD KI =0
  speedSetpoint = positonLoopOutput;
  
  //int speedSign = pos_errorA > 0 ? 1 : -1; 

  // Limit speed here
  (abs(speedSetpoint) > abs(mp.speedCap)) ? (speedSetpoint = mp.speedCap)
                                          : (speedSetpoint = speedSetpoint);


  /*SPEED control*/
  currentSpeed = ((float)deltaHighPrecisionCounter / COUNT_TO_RPS);
  errorA = speedSetpoint - currentSpeed;
  accError = accError + errorA; // Integral accumulator
  
      if(accError > 100){     
        accError = 100;
    }
    else if(accError < -100){
        accError = -100;
        }


  pwmTimeOn = ((SPEED_KP * errorA) + SPEED_KD * ((errorA - lastError)) +
               SPEED_KI * accError) /
              1000; // Output of speed loop

  // Output limiter and logic to change direction
  if (pwmTimeOn < 0) {
    pwmTimeOn = -1 * pwmTimeOn;
    lead = -1;
  } else {
    lead = 2;
  } // TODO: effective lead is compensated for bad startup disk
  if (pwmTimeOn > 100) {
    pwmTimeOn = 100;
  }
  pwmDutyCycle = (int32_t)pwmTimeOn;

  lastError = errorA;
}

// Set a given drive state

//*****************************************************************************MOTOR
// OUTPUT******************************************************************************
void motorOut(int8_t driveState, int32_t pwmPeriod) {

  // Lookup the output byte from the drive state.
  int8_t driveOut = driveTable[driveState & 0x07];

  // Vary duty cycle of the top FET
  // topFET.write((float)pwmPeriod/100);

  // Turn off first
  if (~driveOut & 0x01)
    L1L.pulsewidth_us(0); // turn off low side switch
  if (~driveOut & 0x02)
    L1H = 1; // 0000 0010
  if (~driveOut & 0x04)
    L2L.pulsewidth_us(0); // turn off low side switch
  if (~driveOut & 0x08)
    L2H = 1; // 0000 1000
  if (~driveOut & 0x10)
    L3L.pulsewidth_us(0); // turn off low side switch
  if (~driveOut & 0x20)
    L3H = 1; // 0010 0000

  // Then turn on
  if (driveOut & 0x01)
    L1H = 0; // 0000 0001
  if (driveOut & 0x02)
    L1L.pulsewidth_us(pwmPeriod); // turn on low side switch
  if (driveOut & 0x04)
    L2H = 0; // 0000 0100
  if (driveOut & 0x08)
    L2L.pulsewidth_us(pwmPeriod); // turn on low side switch
  if (driveOut & 0x10)
    L3H = 0; // 0001 0000
  if (driveOut & 0x20)
    L3L.pulsewidth_us(pwmPeriod); // turn on low side switch
}

// Basic synchronisation routine
//***************************************************************MOTOR
// HOME********************************************************************************************

int8_t motorHome() {
  // Put the motor in drive state 0 and wait for it to stabilise
  motorOut(0, 100);
  wait(2.0);
  // Get the rotor state
  return readRotorState();
}

//*************************************************************ISR**********************************************************************************************
void motorISR() {
  static int8_t oldRotorState;
  intState = readRotorState();
  motorOut((intState - orState + lead + 6) % 6, pwmDutyCycle);
  oldRotorState = intState;
}

//*************************************************************ISR**********************************************************************************************
void motorEncoderISR() {
  if (encoderPinB) {
    highPrecisionCounter--;
  } else {
    highPrecisionCounter++;
  }
}

void motorCommutateOpenLoop(int direction) {
  if(!(currentSpeed <= 0.000001 && currentSpeed >= -0.000001)) {
    return;
  }
  motorHome();
  //while ((abs(highPrecisionCounter) <
  //        100)) { // Loop until you get some ticks, direction not so important
                  // as commutate cycle forces correct direction
    int8_t State = readRotorState(); // Read where the rotor is currently
    for (int i = 0; i < 5;
         i++) { // Run a seqeunce from the current state to the next 5
      (direction == 1)
          ? motorOut(((State + i) - orState) % 6, 100)
          : motorOut(((State - i - 1) - orState) % 6,
                     100); // Exact rotation direction that important as after
                           // fist edge, closed loop control takes over.
      wait(0.5);           // Delay for some suitable startup speed
    }
  //}
}



void setBridgePWMFrequency(int period_us) {

  // Set PWM Frequency to 10KHz
  L1L.period_us(period_us);
  L2L.period_us(period_us);
  L3L.period_us(period_us);
  ;
}

void pollSerialQueue() {
  
  bool isOverflow = false;
  while (true) {
   // MYPIN = 1; //REMOVE
    osEvent serialEvent = serialQ.get();
    if (serialEvent.status != osEventMessage) {
      return;
    }
    char c = (char)((uint8_t)serialEvent.value.p);
    //rawSerial.puts("Got char\n");
    if (ibuf.charCount >= 18) {
      isOverflow = true;
    }
    if (c == '\r') {
      if (!isOverflow) {
        ibuf.commandChars[ibuf.charCount++] = '\0';
        //rawSerial.puts("Decoding command: ");
        //rawSerial.puts(ibuf.commandChars);
        //rawSerial.puts("\n\r");
        //pc.printf("Decoding command: %s\n", ibuf.commandChars);
        inputDecoder.decodeCommand();
        ibuf.charCount = 0;
      } else {
        ibuf.charCount = 0;
        isOverflow = false;
      } 
    } else {
      if (!isOverflow) {
        ibuf.commandChars[ibuf.charCount++] = c;
      }
    }
    // MYPIN = 0; ////REMOVE
  }
 
}

void serialISR() {
  //rawSerial.puts("lol\n");
  char c = (char)rawSerial.getc();
  serialQ.put((void *)c);
}

void pollOutMailBox() {
   
    while(true) {
       // MYPIN=1; //REMOVE
        osEvent outEvt = outMailBox.get();
        if(outEvt.status == osEventMail) {
            Output *out = (Output *) outEvt.value.p;
            char outStr[24];
            if(out->isHashCount) {
                rawSerial.puts("Hash rate: ");   
                sprintf(outStr, "%d", (int) out->data);
                rawSerial.puts(outStr);
                rawSerial.puts(" hases/s\n\r");
            } else {
                rawSerial.puts("Nonce: ");
                sprintf(outStr, "%016llx\n\r", out->data);
                rawSerial.puts(outStr);
            }
            outMailBox.free(out);
        }
       // MYPIN = 0;//REMOVE
    }
}

void putOutput(uint64_t data, bool isHashCount) {
    Output *out = outMailBox.alloc();
    out->data = data;
    out->isHashCount = isHashCount;
    outMailBox.put(out);
}
