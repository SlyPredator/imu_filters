// Custom Linear Kalman Filter for MPU-6050 on ESP32

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define PI 3.141592f

unsigned long microsPrevious;

typedef struct {
  float angle;     // Estimated angle (radians)
  float bias;      // Estimated gyro bias (rad/s)
  float P[2][2];   // Error covariance matrix
  float Q_angle;   // Process noise covariance for angle
  float Q_bias;    // Process noise covariance for bias
  float R_measure; // Measurement noise covariance
} KalmanFilter;

KalmanFilter kalmanRoll, kalmanPitch, kalmanYaw;

// Kalman functions to be used later

void initKalmanFilter(KalmanFilter* kf, float Q_angle, float Q_bias, float R_measure) {
  kf->angle = 0.0f;
  kf->bias = 0.0f; 
  
  // err covariance matrix
  kf->P[0][0] = 0.0f;
  kf->P[0][1] = 0.0f;
  kf->P[1][0] = 0.0f;
  kf->P[1][1] = 0.0f;
  
  kf->Q_angle = Q_angle;
  kf->Q_bias = Q_bias;
  kf->R_measure = R_measure;
}

float updateKalman(KalmanFilter* kf, float measurement, float rate, float dt) {

  kf->angle += (rate - kf->bias) * dt;
  
  kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
  kf->P[0][1] -= dt * kf->P[1][1];
  kf->P[1][0] -= dt * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt;

  float S = kf->P[0][0] + kf->R_measure; 
  float K[2];
  K[0] = kf->P[0][0] / S;
  K[1] = kf->P[1][0] / S;

  float y = measurement - kf->angle;

  kf->angle += K[0] * y;
  kf->bias += K[1] * y;

  float P00_temp = kf->P[0][0];
  float P01_temp = kf->P[0][1];
  
  kf->P[0][0] -= K[0] * P00_temp;
  kf->P[0][1] -= K[0] * P01_temp;
  kf->P[1][0] -= K[1] * P00_temp;
  kf->P[1][1] -= K[1] * P01_temp;
  
  return kf->angle;
}

void accelAngles(float ax, float ay, float az, float* accelRoll, float* accelPitch) {
  float norm = sqrt(ax * ax + ay * ay + az * az);
  
  if (norm > 0.1f) { 
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    *accelRoll = atan2(ay, az);
    *accelPitch = -atan2(ax, sqrt(ay * ay + az * az));
  }
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin(21, 22);  // Pins SDA=21, SCL=22
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  initKalmanFilter(&kalmanRoll, 0.001f, 0.003f, 0.03f);
  initKalmanFilter(&kalmanPitch, 0.001f, 0.003f, 0.03f);
  
  initKalmanFilter(&kalmanYaw, 0.001f, 0.003f, 1000.0f);
  
  microsPrevious = micros();
  
  Serial.println("Kalman Filter ready. Output: Roll,Pitch,Yaw (degrees)");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long microsNow = micros();
  float dt = (microsNow - microsPrevious) / 1000000.0f;
  microsPrevious = microsNow;

  if (dt > 0.1f) dt = 0.01f;
  if (dt < 0.0001f) dt = 0.01f;

  float gx = g.gyro.x * PI / 180.0f;
  float gy = g.gyro.y * PI / 180.0f;
  float gz = g.gyro.z * PI / 180.0f;
  
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float accelRoll, accelPitch;
  accelAngles(ax, ay, az, &accelRoll, &accelPitch);
  
  // (no magnetometer)
  // So we'll use a pseudo-measurement that's just the previous estimate
  // This effectively makes it an open-loop integration with bias tracking

  float roll = updateKalman(&kalmanRoll, accelRoll, gx, dt);
  float pitch = updateKalman(&kalmanPitch, accelPitch, gy, dt);
  float yaw = updateKalman(&kalmanYaw, kalmanYaw.angle, gz, dt);

  Serial.print(roll * 180.0f / PI);
  Serial.print(",");
  Serial.print(pitch * 180.0f / PI);
  Serial.print(",");
  Serial.println(yaw * 180.0f / PI);

  delay(10);
}