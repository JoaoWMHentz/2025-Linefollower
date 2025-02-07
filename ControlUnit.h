#ifndef __h
#define _CONTROLUNIT_h

#include "arduino.h"
#include "Definitions.h"

class ControlUnit {
public:
	float KP;
	float KI;
	float KD;

	int calcPid(int error){
    int P = error;
    Integral += (error * DELTA_TIME);

    if (Integral > I_WINDUP_LIMIT) {
      Integral = I_WINDUP_LIMIT;
    }
    else if (Integral < -I_WINDUP_LIMIT) {
      Integral = -I_WINDUP_LIMIT;
    }
    int D = error - PreviosError;
    PreviosError = error;
    return  (int)(KP * P) + (KI * Integral) + (KD * D);
  }

  
private:
	int PreviosError;
	double Integral;
};

#endif