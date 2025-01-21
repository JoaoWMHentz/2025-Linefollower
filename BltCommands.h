#ifndef _BLTCOMMANDS_h
#define _BLTCOMMANDS_h
#include <BluetoothSerial.h>
#include <Arduino.h>

class BltCommands
{
public:
	BluetoothSerial SerialBlt;
	bool robotRun;
	float Kp;
	float Kd;
	float Ki;
	float PWM;
	float PotSuc;
	void begin();
	bool getRobotRun();
	void robotBLT();
	void splitString(const String& input, char separator, String* outputArray, int outputArraySize);
	void bltCommands(String bltData);
	void printValues();
private:

};

#endif

