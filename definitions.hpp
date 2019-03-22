#ifndef DEFINITIONS_INCLUDED
#define DEFINITIONS_INCLUDED

// Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

// Incremental encoder input pins
#define CHApin D12
#define CHBpin D11

// Motor Drive output pins   //Mask in output byte
#define L1Lpin D1  // 0x01
#define L1Hpin A3  // 0x02
#define L2Lpin D0  // 0x04
#define L2Hpin A6  // 0x08
#define L3Lpin D10 // 0x10
#define L3Hpin D2  // 0x20

#define PWMpin D9

// Motor current sense
#define MCSPpin A1
#define MCSNpin A0

// Our definations

#define COUNT_TO_RPS 27.8f // 278 ticks per revolution

// Control Loop Tuning
#define SPEED_KP 4000
#define SPEED_KD 0
#define SPEED_KI 15

#define POSITION_KP 35
#define POSITION_KD 95
#define POSITION_KI 0


#define MAX_CHARS 24

#define OUT_MAIL_SIZE 16

#endif