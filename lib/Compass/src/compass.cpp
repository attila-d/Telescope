#include "compass.h"

#include <EEPROM.h>
#include <MPU9250.h>
#include <quaternionFilters.h>

#include "debug.h"
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 // Use either this line or the next to select which I2C address your device is using
// #define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

bool compassInited = false;

MPU9250 &getMPU9250()
{
    return myIMU;
}

float north = 0.;

#define MAG_EEPROM_OFFSET 0

void saveMagToEEPROM()
{
    if (compassInited)
    {
        // (myIMU.magBias, myIMU.magScale)
        EEPROM.put(MAG_EEPROM_OFFSET + 0 * sizeof(float), myIMU.magBias[0]);
        EEPROM.put(MAG_EEPROM_OFFSET + 1 * sizeof(float), myIMU.magBias[1]);
        EEPROM.put(MAG_EEPROM_OFFSET + 2 * sizeof(float), myIMU.magBias[2]);
        EEPROM.put(MAG_EEPROM_OFFSET + 3 * sizeof(float), myIMU.magScale[0]);
        EEPROM.put(MAG_EEPROM_OFFSET + 4 * sizeof(float), myIMU.magScale[1]);
        EEPROM.put(MAG_EEPROM_OFFSET + 5 * sizeof(float), myIMU.magScale[2]);
        // EEPROM.put(MAG_EEPROM_OFFSET + 6 * sizeof(float), myIMU.accelBias[0]);
        // EEPROM.put(MAG_EEPROM_OFFSET + 7 * sizeof(float), myIMU.accelBias[1]);
        // EEPROM.put(MAG_EEPROM_OFFSET + 8 * sizeof(float), myIMU.accelBias[2]);
        // EEPROM.put(MAG_EEPROM_OFFSET + 9 * sizeof(float), myIMU.gyroBias[0]);
        // EEPROM.put(MAG_EEPROM_OFFSET + 10 * sizeof(float), myIMU.gyroBias[1]);
        // EEPROM.put(MAG_EEPROM_OFFSET + 11 * sizeof(float), myIMU.gyroBias[2]);

        DEBUGLN("Saved to EEPROM:");
        DEBUG3LN("bias0", myIMU.magBias[0], 4);
        DEBUG3LN("bias1", myIMU.magBias[1], 4);
        DEBUG3LN("bias2", myIMU.magBias[2], 4);
        DEBUG3LN("scale0", myIMU.magScale[0], 4);
        DEBUG3LN("scale1", myIMU.magScale[1], 4);
        DEBUG3LN("scale2", myIMU.magScale[2], 4);
        // DEBUG3LN("accelBias0", myIMU.accelBias[0], 4);
        // DEBUG3LN("accelBias1", myIMU.accelBias[1], 4);
        // DEBUG3LN("accelBias2", myIMU.accelBias[2], 4);
        // DEBUG3LN("gyroBias0", myIMU.gyroBias[0], 4);
        // DEBUG3LN("gyroBias1", myIMU.gyroBias[1], 4);
        // DEBUG3LN("gyroBias2", myIMU.gyroBias[2], 4);
    }
}

void readMagFromEEPROM()
{
    // (myIMU.magBias, myIMU.magScale)
    EEPROM.get(MAG_EEPROM_OFFSET + 0 * sizeof(float), myIMU.magBias[0]);
    EEPROM.get(MAG_EEPROM_OFFSET + 1 * sizeof(float), myIMU.magBias[1]);
    EEPROM.get(MAG_EEPROM_OFFSET + 2 * sizeof(float), myIMU.magBias[2]);
    EEPROM.get(MAG_EEPROM_OFFSET + 3 * sizeof(float), myIMU.magScale[0]);
    EEPROM.get(MAG_EEPROM_OFFSET + 4 * sizeof(float), myIMU.magScale[1]);
    EEPROM.get(MAG_EEPROM_OFFSET + 5 * sizeof(float), myIMU.magScale[2]);
    EEPROM.get(MAG_EEPROM_OFFSET + 6 * sizeof(float), myIMU.accelBias[0]);
    EEPROM.get(MAG_EEPROM_OFFSET + 7 * sizeof(float), myIMU.accelBias[1]);
    EEPROM.get(MAG_EEPROM_OFFSET + 8 * sizeof(float), myIMU.accelBias[2]);
    EEPROM.get(MAG_EEPROM_OFFSET + 9 * sizeof(float), myIMU.gyroBias[0]);
    EEPROM.get(MAG_EEPROM_OFFSET + 10 * sizeof(float), myIMU.gyroBias[1]);
    EEPROM.get(MAG_EEPROM_OFFSET + 11 * sizeof(float), myIMU.gyroBias[2]);

    // DEBUGLN("Read from EEPROM:");
    // DEBUG3LN("bias0",myIMU.magBias[0],4);
    // DEBUG3LN("bias1",myIMU.magBias[1],4);
    // DEBUG3LN("bias2",myIMU.magBias[2],4);
    // DEBUG3LN("scale0",myIMU.magBias[0],4);
    // DEBUG3LN("scale1",myIMU.magBias[1],4);
    // DEBUG3LN("scale2",myIMU.magBias[2],4);
}

void initMPU9250()
{
    // Read the WHO_AM_I register, this is a good test of communication
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (c == 0x71) // WHO_AM_I should always be 0x71
    {
        DEBUGLN(F("MPU9250 is online..."));
        myIMU.MPU9250SelfTest(myIMU.selfTest);
        myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

        myIMU.initMPU9250();

        // Read the WHO_AM_I register of the magnetometer, this is a good test of
        // communication
        byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

        // DEBUG("AK8963 ");
        // DEBUG("I AM 0x");
        // DEBUG2(d, HEX);
        // DEBUG(" I should be 0x");
        // DEBUG2LN(0x48, HEX);

        if (d != 0x48)
        {
            // Communication failed, stop here
            DEBUGLN(F("Communication failed, abort!"));
            // Serial.flush();
            abort();
        }

        // Get magnetometer calibration from AK8963 ROM
        myIMU.initAK8963(myIMU.factoryMagCalibration);
        // Initialize device for active mode read of magnetometer
        // DEBUGLN("AK8963 initialized for active data mode....");

        // Get sensor resolutions, only need to do this once
        myIMU.getAres();
        myIMU.getGres();
        myIMU.getMres();

        // The next call delays for 4 seconds, and then records about 15 seconds of
        // data to calculate bias and scale.

        compassInited = true;

        // Serial.println("The next call delays for 4 seconds, and then records about 15 seconds of data to calculate bias and scale.");
        //  myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
        readMagFromEEPROM();

        // DEBUGLN("AK8963 mag biases (mG)");
        // DEBUGLN(myIMU.magBias[0]);
        // DEBUGLN(myIMU.magBias[1]);
        // DEBUGLN(myIMU.magBias[2]);

        // DEBUGLN("AK8963 mag scale (mG)");
        // DEBUGLN(myIMU.magScale[0]);
        // DEBUGLN(myIMU.magScale[1]);
        // DEBUGLN(myIMU.magScale[2]);
        //    delay(2000); // Add delay to see results before serial spew of data
    }
}

void tickMPU9250()
{
    if (compassInited)
    {
        // If intPin goes high, all data registers have new data
        // On interrupt, check if data ready interrupt
        if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
        {
            myIMU.readAccelData(myIMU.accelCount); // Read the x/y/z adc values

            // Now we'll calculate the accleration value into actual g's
            // This depends on scale being set
            myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
            myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
            myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

            myIMU.readGyroData(myIMU.gyroCount); // Read the x/y/z adc values

            // Calculate the gyro value into actual degrees per second
            // This depends on scale being set
            myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
            myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
            myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

            myIMU.readMagData(myIMU.magCount); // Read the x/y/z adc values

            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental
            // corrections
            // Get actual magnetometer value, this depends on scale being set
            myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
            myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
            myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];

            north = atan2(myIMU.my, myIMU.mx) * 180 / PI;
        } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

        // Must be called before updating quaternions!
        myIMU.updateTime();

        // // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
        // // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
        // // (+ up) of accelerometer and gyro! We have to make some allowance for this
        // // orientationmismatch in feeding the output to the quaternion filter. For the
        // // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
        // // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
        // // modified to allow any convenient orientation convention. This is ok by
        // // aircraft orientation standards! Pass gyro rate as rad/s
        // MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
        //                        myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
        //                        myIMU.mx, myIMU.mz, myIMU.deltat);

        // // ahrs
        // // Serial print and/or display at 0.5 s rate independent of data rates
        // myIMU.delt_t = millis() - myIMU.count;

        // // update LCD once per half-second independent of read rate
        // if (myIMU.delt_t > 1000) {
        //     //   if(SerialDebug)
        //     {
        //         // Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        //         // Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        //         // Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        //         // Serial.println(" mg");

        //         // Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        //         // Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        //         // Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        //         // Serial.println(" deg/s");

        //         // DEBUG3("mx = ",(int)myIMU.mx,DEC);
        //         // DEBUG3(" my = ",(int)myIMU.my,DEC);
        //         // DEBUG3(" mz = ",(int)myIMU.mz,DEC);
        //         // DEBUGLN(" mG");

        //         // Serial.print("q0 = ");  Serial.print(*getQ());
        //         // Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        //         // Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        //         // Serial.print(" qz = "); Serial.println(*(getQ() + 3));
        //     }

        //     // Define output variables from updated quaternion---these are Tait-Bryan
        //     // angles, commonly used in aircraft orientation. In this coordinate system,
        //     // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
        //     // x-axis and Earth magnetic North (or true North if corrected for local
        //     // declination, looking down on the sensor positive yaw is counterclockwise.
        //     // Pitch is angle between sensor x-axis and Earth ground plane, toward the
        //     // Earth is positive, up toward the sky is negative. Roll is angle between
        //     // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
        //     // arise from the definition of the homogeneous rotation matrix constructed
        //     // from quaternions. Tait-Bryan angles as well as Euler angles are
        //     // non-commutative; that is, the get the correct orientation the rotations
        //     // must be applied in the correct order which for this configuration is yaw,
        //     // pitch, and then roll.
        //     // For more see
        //     // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        //     // which has additional links.
        //     myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
        //     myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
        //     myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
        //     myIMU.pitch *= RAD_TO_DEG;
        //     myIMU.yaw *= RAD_TO_DEG;

        //     // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
        //     // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
        //     // - http://www.ngdc.noaa.gov/geomag-web/#declination
        //     // myIMU.yaw -= 8.5;
        //     myIMU.yaw += 5.3;   // Budapest, 5o22'
        //     myIMU.roll *= RAD_TO_DEG;

        //     //   if(SerialDebug)
        //     {
        //         // Serial.print("Yaw, Pitch, Roll: ");
        //         // Serial.print(myIMU.yaw, 2);
        //         // Serial.print(", ");
        //         // Serial.print(myIMU.pitch, 2);
        //         // Serial.print(", ");
        //         // Serial.println(myIMU.roll, 2);

        //         // Serial.print("rate = ");
        //         // Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        //         // Serial.print(" Hz");

        //         north = atan2(myIMU.my, myIMU.mx) * 180 / PI;
        //         // DEBUG3LN("compass = ", north, 2);
        //     }

        //     myIMU.count = millis();
        //     myIMU.sumCount = 0;
        //     myIMU.sum = 0;
        // }  // if (myIMU.delt_t > 500)
    }
}

float getNorth()
{
    return north;
}

// magnetometer calibration

// uint16_t ii = 0;
// uint16_t sample_count = 0;
int16_t mag_max[3] = {-32768, -32768, -32768}, // Wrote out decimal (signed) values to remove a conversion warning
    mag_min[3] = {32767, 32767, 32767};
// float bias_dest[3] = {0., 0., 0.};
// float scale_dest[3] = {0., 0., 0.};

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
// void MPU9250::magCalMPU9250(float * bias_dest, float * scale_dest)
// {
void beginMagCalibration()
{
    DEBUGLN("Starting Mag Calibration");
    // Make sure resolution has been calculated
    myIMU.getMres();
    if (myIMU.Mmode == myIMU.M_8HZ)
    {
        DEBUGLN("8MHz");
    }
    else if (myIMU.Mmode == myIMU.M_100HZ)
    {
        DEBUGLN("100MHz");
    }

    // Serial.println(F("Mag Calibration: Wave device in a figure 8 until done!"));
    // Serial.println(
    //     F("  4 seconds to get ready followed by 15 seconds of sampling)"));
    // delay(4000);

    // shoot for ~fifteen seconds of mag data
    // at 8 Hz ODR, new mag data is available every 125 ms
    // if (myIMU.Mmode == myIMU.M_8HZ)
    // {
    //   sample_count = 128;
    // }
    // // at 100 Hz ODR, new mag data is available every 10 ms
    // if (myIMU.Mmode == myIMU.M_100HZ)
    // {
    //   sample_count = 1500;
    // }
}

long prevMillis = 0;

void tickMagCalibration()
{
    // for (ii = 0; ii < sample_count; ii++)
    // {
    long actMillis = millis();
    if (actMillis > (prevMillis + 200))
    {
        prevMillis = actMillis; // in every 200 millisec

        int16_t mag_temp[3] = {0, 0, 0};
        myIMU.readMagData(mag_temp); // Read the mag data
        // DEBUG3("T",mag_temp[0],DEC);
        // DEBUG3(",",mag_temp[1],DEC);
        // DEBUG3LN(",",mag_temp[2],DEC);

        for (int jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
            {
                mag_max[jj] = mag_temp[jj];
            }
            if (mag_temp[jj] < mag_min[jj])
            {
                mag_min[jj] = mag_temp[jj];
            }
        }
    }

    // if (Mmode == M_8HZ)
    // {
    //   delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
    // }
    // if (Mmode == M_100HZ)
    // {
    //   delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
    // }
    // }

    // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // }
}

void endMagCalibration()
{
    int32_t mag_bias[3] = {0, 0, 0},
            mag_scale[3] = {0, 0, 0};

    // Get hard iron correction
    // Get 'average' x mag bias in counts
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
    // Get 'average' y mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
    // Get 'average' z mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

    // Save mag biases in G for main program
    myIMU.magBias[0] = (float)mag_bias[0] * myIMU.mRes * myIMU.factoryMagCalibration[0];
    myIMU.magBias[1] = (float)mag_bias[1] * myIMU.mRes * myIMU.factoryMagCalibration[1];
    myIMU.magBias[2] = (float)mag_bias[2] * myIMU.mRes * myIMU.factoryMagCalibration[2];

    // Get soft iron correction estimate
    // Get average x axis max chord length in counts
    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
    // Get average y axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
    // Get average z axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    myIMU.magScale[0] = avg_rad / ((float)mag_scale[0]);
    myIMU.magScale[1] = avg_rad / ((float)mag_scale[1]);
    myIMU.magScale[2] = avg_rad / ((float)mag_scale[2]);

    DEBUGLN("Mag Calibration done!");
    saveMagToEEPROM();
}