#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Servo servo;
Adafruit_MPU6050 mpu;

float setpoint = 0;
float errorSum = 0;
float lastError = 0;
float kp = 0.1;
float ki = 0.0;
float kd = 0.0;
unsigned long lastTime = 0;

float alpha = 0.98;
float angleY = 0;

void setup(void) {
    Serial.begin(9600);
    servo.attach(3);
    Wire.begin();

    if (!mpu.begin()){
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 está pronto!");

    delay(100);
    lastTime = millis();
}

void loop(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float xAccel = a.acceleration.x;
    float yAccel = a.acceleration.y;
    float zAccel = a.acceleration.z;

    float accelAngleY = atan2(xAccel, sqrt(yAccel * yAccel + zAccel * zAccel)) * 180 / PI;
    float gyroAngleY = g.gyro.y * 180 / PI;
    float angleY = alpha * (accelAngleY + gyroAngleY) + (1 - alpha) * lastAngleY;
    lastAngleY = angleY;

    unsigned long currentTime = millis();
    float deltaTime = millis();
    float error = setpoint - angleY;
    lastTime = currentTime;

    float gyroRateY = g.gyro.y;
    float p = kp * error;
    float i = ki * error * deltaTime;
    float gyroAngleY = gyroAngleY * deltaTime;

    angleY = alpha * (angleY + gyroAngleY) + (1.0 - alpha) * accelAngleY;
    float d = kd * (gyroRateY - lastGyroRateY);
    lastGyroRateY = gyroRateY;

    float pinOutput = Kp * error + Ki * errorSum + Kd * dError;
    errorSum += error * dt;
    dError = (gyroRateY - lastGyroRateY) / dt;
    lastGyroRateY = gyroRateY;

    int servoValue = map(pinOutput, -10, 10, 0, 180);
    servo.attach(servoPin);
    servoValue = constrain(servoValue, 0, 180);
    servo.write(servoValue);

    //prints
}