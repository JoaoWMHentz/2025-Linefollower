#include "BltCommands.h"
#include <BluetoothSerial.h>
#include <Arduino.h>
#include "Definitions.h"
void BltCommands::begin(){
	SerialBlt.begin("Follower");
    Kp = 0;
    Kd = 0;
    Ki = 0;
    robotRun = 0;
    PWM = 0;
}

bool BltCommands::getRobotRun() {
    return robotRun;
}

void BltCommands::robotBLT() {
    if (SerialBlt.available()) {
        char receivedData;
        String receivedString;
        do {
            receivedData = SerialBlt.read();
            receivedString += receivedData;
        } while (receivedData != '\n');
        if (receivedString.indexOf(";") >= 0) {
            String CommandsArray[QTDEPARAM];
            for(int j = 0; j < QTDEPARAM; j++){
              CommandsArray[j]= "";
            }
            splitString(receivedString, ';', CommandsArray, QTDEPARAM);
            for (int i = 0; i < QTDEPARAM; i++) {
                CommandsArray[i].replace(';', '/0');
                bltCommands(CommandsArray[i]);
            }
        }
        bltCommands(receivedString);
    }
}

void BltCommands::splitString(const String& input, char separator, String* outputArray, int outputArraySize) {
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

void BltCommands::bltCommands(String bltData) {
    bltData.replace("\r", "\0");
    bltData.replace("\n", "\0");

    if (bltData.equalsIgnoreCase("stop")) {
        robotRun = false;
        SerialBlt.printf("Robo parado %i", robotRun);
    }

    if (bltData.equalsIgnoreCase("run")) {
        robotRun = true;
    }

    if (bltData.equalsIgnoreCase("print")) {
        robotRun = true;
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

void BltCommands::printValues() {
    SerialBlt.print("Valores PID:\n");
    SerialBlt.printf("Kp: %.4f\n", Kp);
    SerialBlt.printf("Ki: %.4f\n", Ki);
    SerialBlt.printf("Kd: %.4f\n", Kd);
}