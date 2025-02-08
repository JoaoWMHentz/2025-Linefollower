#include "ControlUnit.h"
#include "LocomotionUnit.h"
#include "Definitions.h"
#include "SensorUnit.h"
#include "PeripheralUnit.h"
#include <BluetoothSerial.h>

#define BUTTON_PIN 4

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
  locomotion.ledControl(0, 1, 0);
  sensor.begin();
  sensor.calibrateAllSensors();
  locomotion.ledControl(0, 0, 0);
  locomotion.begin();
  locomotion.setupSuc();
  locomotion.setupEncoder();
  blt.begin();

  blt.initializeSPIFFS();
  setupRobotTask();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
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
  //debugEncoder();
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
    locomotion.motorControl(0, 0);
    locomotion.setPotSuc(0);
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


      sensor.readSensorLeft();
      sensor.readSensorRigth();

      if (sensor.senLeft.alreadyRead && sensor.senRigth.alreadyRead && (runTime - ((sensor.senLeft.readTime + sensor.senRigth.readTime)/2)) <= 50) {
        locomotion.ledControl(1, 0, 0);
        lastSensorTriggerTime = millis();
      } else if(sensor.senLeft.alreadyRead && (runTime - sensor.senLeft.readTime) <= 50) {
          locomotion.ledControl(1, 1, 0);
          lastSensorTriggerTime = millis();
      } else if(sensor.senRigth.alreadyRead && (runTime - sensor.senRigth.readTime) <= 50) {
        locomotion.ledControl(0, 1, 0);
        lastSensorTriggerTime = millis();
      }

      if (millis() - lastSensorTriggerTime >= 100) {
        locomotion.ledControl(0, 0, 1);
      }
    }
  }
}
