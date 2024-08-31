#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//#define OUTPUT_READABLE_QUATERNION

#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// Complementary filter parameters
float alpha = 0.98; // Weighting factor for gyroscope data
float beta = 0.02;  // Weighting factor for accelerometer data


void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        dmpReady = true;
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // Apply complementary filter for yaw
            float gyro_yaw = ypr[0] * 180 / M_PI;
            float accel_yaw = atan2(-gravity.x, sqrt(gravity.y * gravity.y + gravity.z * gravity.z)) * 180 / M_PI;
            float filtered_yaw = alpha * (gyro_yaw + filtered_yaw) + beta * accel_yaw;

            // Apply complementary filter for pitch
            float gyro_pitch = ypr[1] * 180 / M_PI;
            float accel_pitch = atan2(gravity.y, sqrt(gravity.x * gravity.x + gravity.z * gravity.z)) * 180 / M_PI;
            float filtered_pitch = alpha * (gyro_pitch + filtered_pitch) + beta * accel_pitch;

            // Apply complementary filter for roll
            float gyro_roll = ypr[2] * 180 / M_PI;
            float accel_roll = atan2(gravity.z, sqrt(gravity.x * gravity.x + gravity.y * gravity.y)) * 180 / M_PI;
            float filtered_roll = alpha * (gyro_roll + filtered_roll) + beta * accel_roll;

            // Print filtered angles
            Serial.print(filtered_roll);
            Serial.print(",");
            Serial.print(filtered_pitch);
            Serial.print(",");
            Serial.println(filtered_yaw);
        #endif
    }
}


