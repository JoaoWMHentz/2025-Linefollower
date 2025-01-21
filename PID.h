#ifndef _PID_h
#define _PID_h

#include "arduino.h"
#include "Definitions.h"

class PID {
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
    int output = (int)(KP * P) + (KI * Integral) + (KD * D);
    return map(output, MAP_MAX_VALUE, -MAP_MAX_VALUE, MAP_MIN_VALUE, -MAP_MIN_VALUE);
  }
private:
	int PreviosError;
	double Integral;
};

#endif