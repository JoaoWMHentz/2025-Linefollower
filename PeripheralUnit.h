#ifndef _PERIPHERALUNIT_h
#define _PERIPHERALUNIT_h
#include <BluetoothSerial.h>
#include <Arduino.h>
#include "Definitions.h"
#include <SPIFFS.h>

class PeripheralUnit {
public:
  BluetoothSerial SerialBlt;
  bool robotRun;
  float Kp, Kd, Ki, PWM, PotSuc;
  int encoderLeftCounts[10000];
  int encoderRigthCounts[10000];
  int encoderRightIndex = 0;
  int encoderLeftIndex = 0;
  
  static PeripheralUnit* instance;  

  PeripheralUnit() {
    instance = this;  
  }

  void begin() {
    SerialBlt.begin("Follower");
    SerialBlt.register_callback(btCallback);  
    Kp = 0;
    Kd = 0;
    Ki = 0;
    robotRun = 0;
    PWM = 0;
    memset(encoderLeftCounts, 0, sizeof(encoderLeftCounts));
    memset(encoderRigthCounts, 0, sizeof(encoderRigthCounts));
  }

  static void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    if (event == ESP_SPP_DATA_IND_EVT && instance) {
      instance->processBluetoothData();
    }
  }

  void processBluetoothData() {
    String receivedString;
    char receivedData;

    while (SerialBlt.available()) {
      receivedData = SerialBlt.read();
      if (receivedData == '\n') break;
      receivedString += receivedData;
    }

    if (receivedString.indexOf(";") >= 0) {
      String CommandsArray[QTDEPARAM];
      splitString(receivedString, ';', CommandsArray, QTDEPARAM);
      for (int i = 0; i < QTDEPARAM; i++) {
        bltCommands(CommandsArray[i]);
      }
    } else {
      bltCommands(receivedString);
    }
  }

  void splitString(const String& input, char separator, String* outputArray, int outputArraySize) {
    int arrayIndex = 0;
    int startIndex = 0;
    int endIndex;

    for (int i = 0; i < input.length(); i++) {
      if (input[i] == separator || arrayIndex >= outputArraySize - 1) {
        endIndex = i;
        outputArray[arrayIndex++] = input.substring(startIndex, endIndex);
        startIndex = i + 1;
      }
    }
    outputArray[arrayIndex] = input.substring(startIndex);
  }

  bool getRobotRun() {
    return robotRun;
  }

  void robotStop() {
      bool robotRunState = robotRun;
      robotRun = false;
      ledControl(0, 0, 0);
      if(robotRunState){
        recordSPIFFSencoderValue();
      }
      SerialBlt.printf("Robo parado %i", robotRun);
  }

  void ledControl(bool R, bool G, bool B) {
    digitalWrite(LED_R, !R);
    digitalWrite(LED_G, !G);
    digitalWrite(LED_B, !B);
  }

  void bltCommands(String bltData) {
    bltData.replace("\r", "\0");
    bltData.replace("\n", "\0");

    if (bltData.equalsIgnoreCase("stop")) {
      robotStop();
      return;
    }

    if (bltData.equalsIgnoreCase("run")) {
      robotRun = true;
      ledControl(0, 0, 1);
      return;
    }

    if (bltData.equalsIgnoreCase("print")) {
      printValues();
      return;
    }

    if (bltData.equalsIgnoreCase("PRTENC")) {
      printFileContent();
      return;
    }

    if (bltData.equalsIgnoreCase("DLENC")) {
      clearFileContent("/encoder.txt");
      return;
    }

    if (bltData.startsWith("KP:")) {
      String value = bltData.substring(3);
      Serial.print(value);
      Kp = value.toFloat();
      printValues();
      return;
    }
    if (bltData.startsWith("KI:")) {
      String value = bltData.substring(3);
      Ki = value.toFloat();
      printValues();
      return;
    }
    if (bltData.startsWith("KD:")) {
      String value = bltData.substring(3);
      Kd = value.toFloat();
      printValues();
      return;
    }
    if (bltData.startsWith("KPWM:")) {
      PWM = (bltData.substring(5)).toInt();
      printValues();
      return;
    }
    if (bltData.startsWith("KPSUC")) {
      String value = bltData.substring(6);
      int pot = value.toInt();
      PotSuc = pot;
      SerialBlt.printf("KpSuc: %i", pot);
      return;
    }
  }

  void printValues() {
    SerialBlt.printf("Valores PID:\nKp: %.4f\nKi: %.4f\nKd: %.4f\n", Kp, Ki, Kd);
  }

  void initializeSPIFFS() {
    if (!SPIFFS.begin(true)) {
      SerialBlt.println("Falha ao montar SPIFFS.");
      return;
    }

    if (!SPIFFS.exists("/encoder.txt")) {
      File file = SPIFFS.open("/encoder.txt", FILE_WRITE);
      if (file) {
        file.println("Início dos dados do encoder");
        file.close();
        SerialBlt.println("Arquivo encoder.txt criado.");
      } else {
        SerialBlt.println("Falha ao criar arquivo encoder.txt.");
      }
    }
  }

  void printFileContent() {
    File file = SPIFFS.open("/encoder.txt", FILE_READ);
    if (!file) {
      SerialBlt.println("Falha ao abrir o arquivo.");
      return;
    }

    Serial.println("Conteúdo do arquivo:");
    while (file.available()) {
      Serial.write(file.read());
    }

    file.close();
  }

  void recordRobotEncoder(int leftEncoder, int rightEncoder) {
    encoderLeftCounts[encoderLeftIndex] = leftEncoder;
    encoderRigthCounts[encoderRightIndex] = rightEncoder;
    encoderLeftIndex++;
    encoderRightIndex++;
  }

  void recordSPIFFSencoderValue() {
    File file = SPIFFS.open("/encoder.txt", FILE_APPEND);
    if (file) {
      for (int i = 0; i < 10000; i++) {
        if (encoderLeftCounts[i] == 0 && encoderRigthCounts[i] == 0) {
          continue;
        }
        file.print(encoderLeftCounts[i]);
        file.print(":");
        file.println(encoderRigthCounts[i]);
      }
      file.close();
    } else {
      SerialBlt.println("Falha ao abrir o arquivo encoder.txt.");
    }
  }

  void clearFileContent(const char* fileName) {
    File file = SPIFFS.open(fileName, FILE_WRITE);

    if (!file) {
      SerialBlt.println("Falha ao abrir o arquivo para apagar.");
      return;
    }

    file.close();
    SerialBlt.println("Arquivo apagado com sucesso.");
  }
};

PeripheralUnit* PeripheralUnit::instance = nullptr;  

#endif
