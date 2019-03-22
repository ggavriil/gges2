#include "definitions.hpp"
#include "mbed.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <map>
#include <algorithm>
#include <vector>
#include "string.h"
#include "input.hpp"
#include "motorControl.hpp"
#include "RtosTimer.h"


RawSerial *rrs = NULL;
static Thread musicThread; 
vector<int> eventIds;

int directionRotation = 1;

void init_rrs(RawSerial *rs)  {
   rrs = rs;                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                            
}

                      // B         A#/B^     A         G#/A^
int note_freqs[] = {127, 134, 142, 150,
                      // G         F#/G^     F         E        
                      159, 169, 179, 190, 
                      // D#/#^     D         C#/D^     C
                      239, 213, 225, 239};


int note_to_idx(char note) {
    switch(note) {
        case 'B': return 0;
        case 'A': return 2;
        case 'G': return 4;
        case 'F': return 6;
        case 'E': return 7;
        case 'D': return 9;
        case 'C': return 11;
    }
    return 0;
}

// High Side FET for the whole inverter bridge
PwmOut topFET(PWMpin);

#define VERY_LOW_PERIOD 100


void setMusicFrequency(int period_us, EventQueue *eq, int totalPer) {
  topFET.period_us((int) period_us);// 225, 127
  topFET.write(1);
  if(totalPer > 0) {
    int eid = eq->call_every(totalPer * 1000, setMusicFrequency, period_us, eq, 0);
    eventIds.push_back(eid);
  }
}

void InputDecoder::printStr(float f) {
    static char buf[16];
    sprintf(buf, "%f", f);
    _rs->puts(buf);
}

void InputDecoder::decodeRotation() {
  float num;
  //_rs->puts("HEY THERE\n\r");
  int res = sscanf(_inputBuf->commandChars + 1, "%f", &num);
  if(res == 1) {
    if(num != 0){
    directionRotation = num > 0 ?  1 : -1; //ADIL ADDED
    _motorParams->speedCap = abs(_motorParams->speedCap) * directionRotation;//ADIL ADDED
    _motorParams->position = (int) (num * COUNT_TO_RPS * 10);

  } 
  else{
      _motorParams->speedCap = abs(_motorParams->speedCap) * directionRotation;//ADIL ADDED
      _motorParams->position = (int) (999999 *directionRotation* COUNT_TO_RPS * 10);
      }
    _motorParams->posUpdated = true;
    motorCommutateOpenLoop(num > 0 ? 1 : 0);
   // printStr(num);  
      
  }
}

void InputDecoder::decodeVelocity() {
  if (_inputBuf->commandChars[1] == '0' && _inputBuf->commandChars[2] == '\0') {
    //TODO: REMOVE SOFTWARE LIMIT
  }
  float num;
  int res = sscanf(_inputBuf->commandChars + 1, "%f", &num);
  
  if (res == 1) {
    if(num !=0){
    _motorParams->speedCap = num*directionRotation;
    }
    else{
         _motorParams->speedCap = 9999*directionRotation;
        
        }
    
    
    //printStr(num);
  }
}

void InputDecoder::decodeKey() { 
    _motorParams->keyLock.lock();
    _motorParams->key = *((uint64_t *) (_inputBuf->commandChars + 1));
    _motorParams->keyLock.unlock();
}

void InputDecoder::decodeMelody() {
  char note;
  bool isFlatOrSharp;
  int flatOrSharp = 0;
  int duration;
  size_t curPtr = 1;
  std::vector<Note> notes;
  static std::vector<rtos::RtosTimer> timers;
  while (_inputBuf->commandChars[curPtr]) {
    note = _inputBuf->commandChars[curPtr++];
    isFlatOrSharp = _inputBuf->commandChars[curPtr] == '#' ||
                    _inputBuf->commandChars[curPtr] == '^';
    flatOrSharp =
        isFlatOrSharp ? (_inputBuf->commandChars[curPtr] == '#' ? 1 : -1) : 0;
    curPtr = curPtr + (isFlatOrSharp ? 1 : 0);
    duration = _inputBuf->commandChars[curPtr++] - '0';
    struct Note noteDur;
    noteDur.frequency = note_freqs[std::max(note_to_idx(note) + flatOrSharp, 0)];
    noteDur.duration = duration;
    notes.push_back(noteDur);
    //_rs->putc(note);
    //_rs->putc(' ');
   // _rs->putc(noteDur.duration + '0');
   // _rs->puts("\n\r");                                                                                                                                                                                                                                        
  }
  int totalTime = 0;
  for(size_t i = 0; i < notes.size(); i++) {
    totalTime += notes[i].duration;
  }
  int cumulativeTime = 0;
  for(size_t i = 0; i < eventIds.size(); i++) {
    _eq.cancel(eventIds[i]);
  }
  eventIds.clear();
  int eid;
  for(size_t i = 0; i < notes.size() - 1; i++) {
    cumulativeTime += notes[i].duration;
    int period = notes[i + 1].frequency;
    eid = _eq.call_in(cumulativeTime * 1000, setMusicFrequency, period, &_eq, totalTime);
    eventIds.push_back(eid);
  }
  eid = _eq.call_every(totalTime  * 1000, setMusicFrequency, notes[0].frequency, (EventQueue *)NULL, 0);
  eventIds.push_back(eid);
  setMusicFrequency(notes[0].frequency, (EventQueue *)NULL, 0);
  //eq.dispatch((cumulativeTime + notes[notes.size() - 1].duration)  * 1000);
}

void InputDecoder::decodeCommand() {
  //_rs->puts("IN DECODE COMMAND ");
  //_rs->putc(_inputBuf->commandChars[0]);
  //_rs->puts("\n\r");
  switch (_inputBuf->commandChars[0]) {
  case 'R':
    decodeRotation();
    break;
  case 'V':
    decodeVelocity();
    break;
  case 'K':
    decodeKey();
    break;
  case 'T':
    decodeMelody();
  }
}


InputDecoder::InputDecoder(InputBuf *ib, MotorParams *mp, RawSerial *rs) 
    : _inputBuf(ib), _motorParams(mp), _rs(rs) {
    setMusicFrequency(100, (EventQueue *) NULL, 0);     
    musicThread.start(callback(&_eq, &EventQueue::dispatch_forever));                                                                                     
}
