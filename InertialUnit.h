#include <Wire.h>

// Endereço I2C do LSM6DSRTR
#define LSM6DSRTR_ADDRESS 0x6A

class LSM6DSRTR {
public:
    void begin() {
        Wire.begin();
        // Configuração básica do LSM6DSRTR
        writeRegister(0x10, 0x60); // CTRL1_XL: Acelerômetro a 104 Hz, +/- 2g
        writeRegister(0x11, 0x60); // CTRL2_G: Giroscópio a 104 Hz, 245 dps
    }

    void readAcceleration(float &ax, float &ay, float &az) {
        uint8_t buffer[6];
        readRegisters(0x28, 6, buffer); // Lê os registros de saída do acelerômetro

        // Converte os valores brutos para aceleração em g
        ax = (int16_t)((buffer[1] << 8) | buffer[0]) * 0.061 / 1000.0;
        ay = (int16_t)((buffer[3] << 8) | buffer[2]) * 0.061 / 1000.0;
        az = (int16_t)((buffer[5] << 8) | buffer[4]) * 0.061 / 1000.0;
    }

      void readGyroscope(float &gx, float &gy, float &gz) {
          uint8_t buffer[6];
          readRegisters(0x22, 6, buffer); 

          gx = (int16_t)((buffer[1] << 8) | buffer[0]) * 0.070 / 1000.0;
          gy = (int16_t)((buffer[3] << 8) | buffer[2]) * 0.070 / 1000.0;
          gz = (int16_t)((buffer[5] << 8) | buffer[4]) * 0.070 / 1000.0;
      }

    float getRoll(float ax, float ay, float az) {
        return atan2(ay, az) * RAD_TO_DEG;
    }

    float getPitch(float ax, float ay, float az) {
        return atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    }

private:
    void writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(LSM6DSRTR_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    void readRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
        Wire.beginTransmission(LSM6DSRTR_ADDRESS);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(LSM6DSRTR_ADDRESS, count);
        for (uint8_t i = 0; i < count; i++) {
            data[i] = Wire.read();
        }
    }
};

class KalmanFilter {
public:
    KalmanFilter() : angle(0), bias(0), P{{0, 0}, {0, 0}} {}

    float update(float newAngle, float newRate, float dt) {
        // Predição
        float rate = newRate - bias;
        angle += dt * rate;

        // Atualização da matriz de covariância
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_gyro * dt;

        // Cálculo do ganho de Kalman
        float S = P[0][0] + R_angle;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Atualização do ângulo e viés
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        // Atualização da matriz de covariância
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }

private:
    float angle; // Ângulo estimado
    float bias;  // Viés do giroscópio
    float P[2][2]; // Matriz de covariância

    // Constantes do filtro de Kalman
    const float Q_angle = 0.001;
    const float Q_gyro = 0.003;
    const float R_angle = 0.03;
};

// Instâncias das classes
LSM6DSRTR imu;
KalmanFilter kalmanRoll;
KalmanFilter kalmanPitch;

void setup() {
    Serial.begin(115200);
    imu.begin();
}

void loop() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Intervalo de tempo em segundos
    lastTime = currentTime;

    // Ler dados do acelerômetro e giroscópio
    float ax, ay, az;
    float gx, gy, gz;
    imu.readAcceleration(ax, ay, az