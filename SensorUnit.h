#ifndef SENSORUNIT.H
#define SENSORUNIT .H

#include "Definitions.h"
#include <Arduino.h>

struct SensorCalibration {
  uint16_t whiteThreshold;  // Limite para cor branca
  uint16_t blackThreshold;  // Limite para cor preta
};

struct sensorSt{
  int pin;
  int readTime;
  bool alreadyRead;
};

//uint8_t sensorSequence[S_QTD] = {0,2,1,3,4,6,5,7,8,10,9,11,12,14,13,15};


class SensorUnit {
public:
  SensorCalibration sensors[S_QTD];
  struct sensorSt senLeft;
  struct sensorSt senRigth;

  void begin() {
    pinMode(S_OUT, INPUT);
    pinMode(S_PINS[0], OUTPUT);
    pinMode(S_PINS[1], OUTPUT);
    pinMode(S_PINS[2], OUTPUT);
    pinMode(S_PINS[3], OUTPUT);
    senLeft.pin = LEFT_SENSOR;
    senRigth.pin = RIGHT_SENSOR;
    senLeft.alreadyRead = false;
    senRigth.alreadyRead = false;
  }

  uint16_t readSensor(uint8_t sensor) {
    digitalWrite(S_PINS[0], (int)bitRead(sensor, 0));
    digitalWrite(S_PINS[1], (int)bitRead(sensor, 1));
    digitalWrite(S_PINS[2], (int)bitRead(sensor, 2));
    digitalWrite(S_PINS[3], (int)bitRead(sensor, 3));
    int read = analogRead(S_OUT);
    read = read == 0 ? 0 : (read / 10);
    return read == 0 ? 0 : 10000 / read;
  }

  void calibrateAllSensors() {
    uint16_t sensorReads[S_QTD] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (uint8_t i = 0; i < CALIB_RUNS; i++) {
      for (uint8_t j = 0; j < S_QTD; j++) {
        sensorReads[j] += readSensor(j);
      }
      delay(200);
    }
    for (uint8_t k = 0; k < S_QTD; k++) {
      sensorReads[k] = sensorReads[k] == 0 ? 0 : sensorReads[k] / CALIB_RUNS;
      sensors[k].whiteThreshold = sensorReads[k] + (sensorReads[k] * 8);
    }
  }

  int calculateOffset() {
    int16_t weightedSum = 0;
    uint8_t sumReadings = 0;
    bool isInMidle;
    bool readSensor0 = 0;
    bool readSensor15 = 0;
    for (uint8_t i = 0; i < S_QTD; i++) {
      uint16_t read = readSensor(i);
      if (read >= sensors[i].whiteThreshold) {
        weightedSum += read * S_WEIGHT[i];
        sumReadings++;
        readSensor0 = (i == 0);
        readSensor15 = (i == 15);
      }
    }
    if (sumReadings <= 1 && (readSensor0 || readSensor15)) {
      weightedSum = readSensor0 ? 800 * S_WEIGHT[0] : 800 * S_WEIGHT[15];
    }
    weightedSum = weightedSum != 0 ? (weightedSum / sumReadings) / 10 : lastWeightedSum;
    lastWeightedSum = weightedSum;
    return weightedSum;
  }

  bool isWhite(int sensorIndex) {
    return readSensor(sensorIndex) <= sensors[sensorIndex].whiteThreshold;
  }

  bool verifyTimer(int time, int timeTo) {
    return getTime(time) >= timeTo;
  }

  int getTime(int time) {
    return millis() - time;
  }

  bool readSensorLeft() {
    if (analogRead(senLeft.pin) < 3000) {
      if (verifyTimer(senLeft.readTime, 75) && senLeft.alreadyRead == false) {
        senLeft.readTime = (int)millis();
        senLeft.alreadyRead = true;
        return true;
      } else {
        return false;
      }
    } else if (analogRead(senLeft.pin) > 3500) {
      senLeft.alreadyRead = false;
      return false;
    }
    return false;
  }

  bool readSensorRigth() {
    if (analogRead(senRigth.pin) < 3000) {
      if (verifyTimer(senRigth.readTime, 75) && senRigth.alreadyRead == false) {
        senRigth.readTime = (int)millis();
        senRigth.alreadyRead = true;
        return true;
      } else {
        return false;
      }
    } else if (analogRead(senRigth.pin) > 3500) {
      senRigth.alreadyRead = false;
      return false;
    }
    return false;
  }

private:
  int16_t lastWeightedSum = 0;
};

#endif