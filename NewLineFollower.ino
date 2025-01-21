#include "PID.h"
#include "Locomotion.h"
#include "Definitions.h"
#include "JsumoSensor.h"
#include "BltCommands.h"
#include <BluetoothSerial.h>


Locomotion locomotion = Locomotion();
JsumoSensor sensor = JsumoSensor();
PID pid = PID();
BltCommands blt = BltCommands();
BluetoothSerial SerialBt;
TaskHandle_t BLTReadHandle;
long timePID = 0;

void setup() {
	locomotion.setupSuc();
	Serial.begin(115200);
	locomotion.begin();
  locomotion.setupEncoder();
  locomotion.setupLed();
  sensor.begin();
  sensor.calibrateAllSensors();
  locomotion.setupEncoder();
  printWhiteThresholds(); ;
}

void printWhiteThresholds() {
    Serial.println("White Thresholds of Sensors:");
    for (uint8_t i = 0; i < S_QTD; i++) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(sensor.sensors[i].whiteThreshold);
    }
  }

void loop() {
    locomotion.ledControl(0, 0, 0);
    locomotion.brake();
    // Lê os valores dos contadores
    locomotion.setPotSuc(0);
    for (uint8_t i = 0; i < S_QTD; i++) {
      Serial.print(sensor.isWhite(i));
      Serial.print(" ");
    }
    Serial.println();
    // Exibe os valores no Serial Monitor
    delay(100);

}