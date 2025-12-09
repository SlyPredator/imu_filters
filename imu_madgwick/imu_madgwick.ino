// Custom Madgwick Filter implementation for MPU-6050 on ESP32

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define MADGWICK_BETA 0.041f 
#define PI 3.141592f

// quaternions
volatile float q0 = 1.0f; // w
volatile float q1 = 0.0f; // x
volatile float q2 = 0.0f; // y
volatile float q3 = 0.0f; // z

unsigned long microsPrevious;

// Madgwick functions to be used later

void coreMadgwick(float gx, float gy, float gz, float ax, float ay, float az, float deltat) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;  
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient descent algorithm correction (Jacobian step)
		float Fg = 2.0f * (q1 * q3 - q0 * q2) - ax;
		float Fh = 2.0f * (q0 * q1 + q2 * q3) - ay;
		float Fi = 2.0f * (0.5f - q1q1 - q2q2) - az; 
		
		// Jacobian terms (partial derivatives)
		float Jg0 = -_2q2;
		float Jg1 = _2q3;
		float Jg2 = -_2q0;
		float Jg3 = _2q1;
		
		float Jh0 = _2q1;
		float Jh1 = _2q0;
		float Jh2 = _2q3;
		float Jh3 = _2q2;
		
		float Ji0 = 0.0f;
		float Ji1 = -_4q1;
		float Ji2 = -_4q2;
		float Ji3 = 0.0f;

		s0 = Jg0 * Fg + Jh0 * Fh + Ji0 * Fi;
		s1 = Jg1 * Fg + Jh1 * Fh + Ji1 * Fi;
		s2 = Jg2 * Fg + Jh2 * Fh + Ji2 * Fi;
		s3 = Jg3 * Fg + Jh3 * Fh + Ji3 * Fi;

		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		qDot1 -= MADGWICK_BETA * s0;
		qDot2 -= MADGWICK_BETA * s1;
		qDot3 -= MADGWICK_BETA * s2;
		qDot4 -= MADGWICK_BETA * s3;
	}

	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

float invSqrt(float x) {
	union {
		float f;
		long i;
	} conv;
	conv.f = x;
	conv.i = 0x5f3759df - (conv.i >> 1);
	return conv.f * (1.5f - x * 0.5f * conv.f * conv.f);
}

void eulerAngles(float* roll, float* pitch, float* yaw) {
    *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    *pitch = -asin(2.0f * (q0 * q2 - q1 * q3)) * 180.0f / PI;
    *yaw = atan2(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
}

void setup(void) {
  Serial.begin(115200);
	// Pins 21 and 22, for SCL and SDA
  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); 

  microsPrevious = micros();

  Serial.println("Starting stream (Roll,Pitch,Yaw)...");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long microsNow = micros();
  float deltat = (microsNow - microsPrevious) / 1000000.0f;
  microsPrevious = microsNow;

  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  coreMadgwick(gx, gy, gz, ax, ay, az, deltat);

  float roll, pitch, yaw;
  eulerAngles(&roll, &pitch, &yaw);

  Serial.print(roll);
  Serial.print(","); 
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);

  delay(1); 
}