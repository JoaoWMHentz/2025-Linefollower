#include "ControlUnit.h"
#include "LocomotionUnit.h"
#include "Definitions.h"
#include "SensorUnit.h"
#include "PeripheralUnit.h"
#include <BluetoothSerial.h>

LocomotionUnit locomotion = LocomotionUnit();
SensorUnit sensor = SensorUnit();
ControlUnit pid = ControlUnit();
PeripheralUnit blt = PeripheralUnit();
TaskHandle_t BLTReadHandle;
long timePID = 0;
bool robotRunning = false;
bool lastButtonState = HIGH;
unsigned long runTime = 0;
unsigned long lastSensorTriggerTime = 0;
int cruzamento_count = 0;
long delayToStop = 0;

bool initialValue = 1;

void setupRobotTask() {

  xTaskCreatePinnedToCore(
    robotSecundaryTask,
    "BLTRead",
    10000,
    NULL,
    1,
    &BLTReadHandle,
    1);

  delay(200);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  locomotion.setupLed();
  sensor.begin();
  pinMode(BUTTON_PIN, INPUT);
  locomotion.ledControl(0, 1, 0);
  sensor.calibrateAllSensors();
  locomotion.ledControl(0, 0, 0);
  printWhiteThresholds();
  locomotion.begin();
  locomotion.setupSuc();
  locomotion.setupEncoder();
  blt.begin();

  blt.initializeSPIFFS();
  setupRobotTask();
}

void printWhiteThresholds() {
  Serial.println("White Thresholds of Sensors:");
  for (uint8_t i = 0; i < S_QTD; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensor.sensors[i].whiteThreshold);
  }

  Serial.println("Black Thresholds of Sensors:");
  for (uint8_t i = 0; i < S_QTD; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensor.sensors[i].whiteThreshold);
  }
}

void loop() {
  //debugLoop();
  robotLoop();
}

void debugEncoder() {
  Serial.print(locomotion.readLeftEncoder());
  Serial.print(":");
  Serial.println(locomotion.readRightEncoder());
  delay(100);
}

void debugLoop() {
  for (uint8_t i = 0; i < S_QTD; i++) {
    Serial.print(sensor.readSensor(i));
    Serial.print(" ");
  }
  Serial.print(sensor.calculateOffset());
  Serial.println();
  delay(20);
}

void robotLoop() {
  robotRunning = blt.getRobotRun();

  if (robotRunning) {
    locomotion.setPotSuc(blt.PotSuc);
    pid.KD = blt.Kd;
    pid.KP = blt.Kp;
    pid.KI = blt.Ki;
    int output = pid.calcPid(sensor.calculateOffset());
    if (output > MAX_PWM) output = MAX_PWM;
    if (output < -MAX_PWM) output = -MAX_PWM;
    locomotion.motorControl(blt.PWM + -output, blt.PWM + output);
  } else {
    locomotion.brake();
    locomotion.setPotSuc(0);
    cruzamento_count = 0;
    delayToStop = 0;
  }
}

void robotSecundaryTask(void* pvParameters) {
  for (;;) {
    if ((millis() - runTime) >= 10 && robotRunning) {
      runTime = millis();
      if (initialValue) {
        locomotion.resetEncoders();
        initialValue = 0;
      }
      int leftEncoder = locomotion.readLeftEncoder();
      int rightEncoder = locomotion.readRightEncoder();
      blt.recordRobotEncoder(leftEncoder, rightEncoder);

      if(sensor.readSensorLeft() && sensor.isInMidle) {
        locomotion.ledControl(1, 0, 0);
        lastSensorTriggerTime = millis();
      }

      if(sensor.readSensorRigth() && sensor.isInMidle) {
        locomotion.ledControl(0, 1, 0);
        lastSensorTriggerTime = millis();
        cruzamento_count ++;

      }
      

      if(cruzamento_count >= NUM_CRUZAMENTOS){
        if(delayToStop == 0){
          delayToStop = millis();
        }
        if((millis() - delayToStop) > 150){
          blt.robotStop();
        }
      }

      if (millis() - lastSensorTriggerTime >= 200 && robotRunning) {
        locomotion.ledControl(0, 0, 1);
      }
    }
  }
}
