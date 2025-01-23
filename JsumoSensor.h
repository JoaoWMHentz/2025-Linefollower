#ifndef JSUMOSENSOR.H
#define JSUMOSENSOR .H

#include "JsumoSensor.h"
#include "Definitions.h"
#include <Arduino.h>

struct SensorCalibration {
  uint16_t whiteThreshold;  // Limite para cor branca
  uint16_t blackThreshold;  // Limite para cor preta
};

uint8_t sensorSequence[S_QTD] = {0,2,1,3,4,6,5,7,8,10,9,11,12,14,13,15};

class JsumoSensor {
public:
  SensorCalibration sensors[S_QTD];

  void begin() {
    pinMode(S_OUT, INPUT);
    pinMode(S_PINS[0], OUTPUT);
    pinMode(S_PINS[1], OUTPUT);
    pinMode(S_PINS[2], OUTPUT);
    pinMode(S_PINS[3], OUTPUT);
  }

  uint16_t readSensor(uint8_t sensor) {
    for (uint8_t i = 0; i < S_BIT; i++) {
      digitalWrite(S_PINS[i], (int)bitRead(sensorSequence[sensor], i));
    }
    return analogRead(S_OUT)/10;
  }

  void calibrateAllSensors() {
    uint16_t sensorReads[S_QTD] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    for (uint8_t i = 0; i < CALIB_RUNS; i++) {
      for (uint8_t j = 0; j < S_QTD; j++) {
        sensorReads[j] += readSensor(j);
      }
      delay(200);
    }
    for (uint8_t k = 0; k < S_QTD; k++) {
      sensorReads[k] = sensorReads[k] / CALIB_RUNS;
      sensors[k].whiteThreshold = sensorReads[k] + (sensorReads[k] * 0.75);
    }
  }

  int calculateOffset() {
    int16_t weightedSum = 0;
    uint8_t sumReadings = 0;
    bool isInMidle;
    for (uint8_t i = 0; i < S_QTD; i++) {
      uint16_t read = readSensor(i);
      bool notRead = isInMidle && (i >= 14 || i <= 1);
      if (read <= sensors[i].whiteThreshold && !notRead) {
        weightedSum += read * S_WEIGHT[i];
        sumReadings++;
        isInMidle = (i >= 6 || i <= 9);
      }
    }
    return weightedSum != 0 ? weightedSum / sumReadings : 0;
  }

  bool isWhite(int sensorIndex) {
    return readSensor(sensorIndex) <= sensors[sensorIndex].whiteThreshold;
  }

private:
};

#endif