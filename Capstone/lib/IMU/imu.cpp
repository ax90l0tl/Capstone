#include "imu.h"

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
    mpuInterrupt = true;
}

IMU::IMU(uint8_t interrupt_pin)
{
    interrupt_pin_ = interrupt_pin;
    initialize();
    dmpInitialize();
    // Don't change these constants
    setXAccelOffset(3177);
    setYAccelOffset(1911);
    setZAccelOffset(935);
    setXGyroOffset(3);
    setYGyroOffset(4);
    setZGyroOffset(-4);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        CalibrateAccel(6);
        CalibrateGyro(6);
        PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(interrupt_pin_));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(interrupt_pin_), dmpDataReady, RISING);
        mpuIntStatus = getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


bool IMU::getData()
{
    if (dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        // display Euler angles in degrees
        dmpGetQuaternion(&q, fifoBuffer);
        dmpGetGravity(&gravity, &q);
        dmpGetYawPitchRoll(ypr, &q, &gravity);
        return true;
    }
    return false;
}

void IMU::printData()
{
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
}
