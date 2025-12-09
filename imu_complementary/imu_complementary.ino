// Custom Complementary Filter for MPU-6050 on ESP32

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define ALPHA 0.98
#define PI 3.141592f
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
unsigned long microsPrevious;

// Complementary functions to be used later

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

void complementaryFilterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float accelRoll, accelPitch;
  accelAngles(ax, ay, az, &accelRoll, &accelPitch);

  float gyroRoll = roll + gx * dt;
  float gyroPitch = pitch + gy * dt;
  float gyroYaw = yaw + gz * dt;

  roll = ALPHA * gyroRoll + (1.0f - ALPHA) * accelRoll;
  pitch = ALPHA * gyroPitch + (1.0f - ALPHA) * accelPitch;
  yaw = gyroYaw;

  if (roll > PI) roll -= 2 * PI;
  if (roll < -PI) roll += 2 * PI;
  if (pitch > PI) pitch -= 2 * PI;
  if (pitch < -PI) pitch += 2 * PI;
  if (yaw > PI) yaw -= 2 * PI;
  if (yaw < -PI) yaw += 2 * PI;
}

void setup(void) {
  Serial.begin(115200);
    // Pins 21, 22 for SCL and SDA
  Wire.begin(21, 22);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  microsPrevious = micros();
  
  Serial.println("Complementary Filter ready. Output: Roll,Pitch,Yaw (degrees)");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long microsNow = micros();
  float dt = (microsNow - microsPrevious) / 1000000.0f;
  microsPrevious = microsNow;
  
  if (dt > 0.1f) dt = 0.01f; 
  
  float gx = g.gyro.x * PI / 180.0f;
  float gy = g.gyro.y * PI / 180.0f;
  float gz = g.gyro.z * PI / 180.0f;
  
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  
  complementaryFilterUpdate(gx, gy, gz, ax, ay, az, dt);

  Serial.print(roll * 180.0f / PI);
  Serial.print(",");
  Serial.print(pitch * 180.0f / PI);
  Serial.print(",");
  Serial.println(yaw * 180.0f / PI);
  
  delay(10);
}