#include "PID.h"
#include "Locomotion.h"
#include "Definitions.h"
#include "JsumoSensor.h"
#include "BltCommands.h"
#include <BluetoothSerial.h>

#define BUTTON_PIN 4  // Defina o pino do botão

Locomotion locomotion = Locomotion();
JsumoSensor sensor = JsumoSensor();
PID pid = PID();
BltCommands blt = BltCommands();
TaskHandle_t BLTReadHandle;
long timePID = 0;
bool robotRunning = false;    // Estado atual do robô
bool lastButtonState = HIGH;  // Estado anterior do botão

void setupRobotTask() {
  // Criação da tarefa de leitura dos sensores do robô
  xTaskCreatePinnedToCore(
    robotSecundaryTask,  // Função da tarefa
    "BLTRead",           // Nome da tarefa
    10000,               // Tamanho da memória da tarefa
    NULL,                // Parâmetros
    1,                   // Prioridade
    &BLTReadHandle,      // Handler da tarefa
    1                    // Core onde a tarefa está pendurada (0 ou 1)
  );

  delay(200);  // Delay para iniciar as tarefas corretamente
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
  setupRobotTask();
  printWhiteThresholds();

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Configura o botão com resistor pull-up interno
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
  //debugLoop();
  robotLoop();
}

void debugLoop(){
  for (uint8_t i = 0; i < S_QTD; i++){
    Serial.print(sensor.isWhite(i));
    Serial.print(" ");
  }
  Serial.println();
  delay(100);
}

void robotLoop() {
  bool robotRUn = blt.getRobotRun();
  //locomotion.motorControl(500, 500);
  if (robotRUn) {
    locomotion.ledControl(0, 0, 1);
    locomotion.setPotSuc(blt.PotSuc);
    timePID = millis();
    pid.KD = blt.Kd;
    pid.KP = blt.Kp;
    pid.KI = blt.Ki;
    int output = pid.calcPid(sensor.calculateOffset());
    if (output > MAX_PWM) output = MAX_PWM;
    if (output < -MAX_PWM) output = -MAX_PWM;
    locomotion.motorControl(blt.PWM + -output, blt.PWM + output);
  } else {
    locomotion.motorControl(0, 0);
    //locomotion.brake();
    locomotion.ledControl(0, 0, 0);
    locomotion.setPotSuc(0);
  }
}

void robotSecundaryTask(void* pvParameters) {
  for (;;) {
    blt.robotBLT();
  }
}