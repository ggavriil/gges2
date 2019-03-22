#ifndef INPUT_INCLUDED
#define INPUT_INCLUDED

#include <stdint.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include "definitions.hpp"
#include "mbed.h"
#include "Mutex.h"


struct Note {
  int frequency;
  int duration;
};

struct Output {
    bool isHashCount;
    uint64_t data;
};

struct MotorParams {
  volatile float speedCap;
  volatile int32_t position;
  volatile bool posUpdated;
  std::vector<Note> notes; //TODO[gg]: Probably Remove
  rtos::Mutex keyLock;
  uint64_t key;
};

struct InputBuf {
  char commandChars[MAX_CHARS];
  size_t charCount;
};


class InputDecoder {
  private:
  struct InputBuf *_inputBuf;
  struct MotorParams *_motorParams;
  RawSerial *_rs;
  EventQueue _eq;
  std::map<char, int> _note_to_idx;
  
  void decodeRotation();
  void decodeVelocity();
  void decodeKey();
  void decodeMelody();
  void printStr(float f);
  
  public:
  void decodeCommand();
  InputDecoder(InputBuf *ib, MotorParams *mp, RawSerial *rs);
};

void init_rrs(RawSerial *rs);

#endif
