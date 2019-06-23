#ifndef Morse_h
#define Morse_h

#include "Arduino.h"

class Vector {
  public:
    Vector(int x, int y, int z);
    int add();
    int mult();
};

#endif
