// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*
 FS_SEL | Full Scale Range   | LSB Sensitivity (dividers)
 -------+--------------------+----------------
 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 
  AFS_SEL | Full Scale Range | LSB Sensitivity (dividers)
 --------+------------------+----------------
 0       | +/- 2g           | 8192 LSB/mg
 1       | +/- 4g           | 4096 LSB/mg
 2       | +/- 8g           | 2048 LSB/mg
 3       | +/- 16g          | 1024 LSB/mg
 
 */
 

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


#define OUTPUT_READABLE_ACCELGYRO
#define OUTPUT_READABLE_ROTATION
#define OUTPUT_READABLE_ACCEL
#define OUTPUT_READABLE_VELOCITY
#define OUTPUT_READABLE_DISPLACEMENT


//#define LED_PIN RED_LED
//#define BLINK() { blinkState = !blinkState; digitalWrite(LED_PIN, blinkState); }
#define BLINK()

#define NCALIB 32
#define SCALE_A (2*8192) // 1g = (9.80665 m/s^2)
#define SCALE_G 131      // 
#define G_FORCE 9.80665

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t base_ax, base_ay, base_az;
int16_t base_gx, base_gy, base_gz;


int16_t ax, ay, az;
int16_t gx, gy, gz;

int32_t dwax, dway, dwaz;
int32_t dwgx, dwgy, dwgz;

float vx, vy, vz, x, y, z, rx, ry, rz;

uint32_t t;

//bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    //Wire.pins(0, 2); // sda, sdl
    //Wire.begin();
    Wire.begin(0, 2); // sda, sdl
    
    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */


    //accelgyro.getFullScaleAccelRange() 	//AFS_SEL
    //accelgyro.getFullScaleGyroRange() 	//FS_SEL

    //accelgyro.setFullScaleAccelRange(1);   //AFS_SEL
    //accelgyro.setFullScaleGyroRange(1);   //FS_SEL

    Serial.print("AFS_SEL:\t"); Serial.print(accelgyro.getFullScaleAccelRange()); Serial.print("\tFS_SEL:\t"); Serial.println(accelgyro.getFullScaleGyroRange());

    // calibrate 
    Serial.print("Calibrating");
    calibrate(NCALIB);
    
    Serial.print("Base:\t");
    Serial.print(base_ax); Serial.print("\t"); Serial.print(base_ay); Serial.print("\t"); Serial.print(base_az); Serial.print("\t");
    Serial.print(base_gx); Serial.print("\t"); Serial.print(base_gy); Serial.print("\t"); Serial.println(base_gz);
    Serial.print("NormA:"); Serial.println(sqrt(base_ax*base_ax+base_ay*base_ay+base_az*base_az));
        
    // configure Arduino LED for
    //pinMode(LED_PIN, OUTPUT);
    x=y=z=0.0;
    vx=vy=vz=0.0;
    rx=ry=rz=0.0;
    t=millis();
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax-=base_ax; ay-=base_ay; az-=base_az;
    gx-=base_gx; gy-=base_gy; gz-=base_gz;
    uint32_t t1=millis(); 
    float dt=(t1-t)/1000.0;
    rx+=(float)gx/SCALE_G*dt; ry+=(float)gy/SCALE_G*dt; rz+=(float)gz/SCALE_G*dt;
    float vx0=vx, vy0=vy, vz0=vz;
    float arx=(float)ax/SCALE_A*G_FORCE, ary=(float)ay/SCALE_A*G_FORCE, arz=(float)az/SCALE_A*G_FORCE;
    vx+=arx*G_FORCE*dt; vy+=ary*dt; vz+=arz*dt; 
    x+=vx0*dt+arx*dt*dt/2; y+=vy0*dt+ary*dt*dt/2; z+=vz0*dt+arz*dt*dt/2;
    // integration - better to make average (val0+val)/2 ...
    Serial.print("a/g:\tDT=");Serial.print(t1-t);Serial.print(":\t"); 
#ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
    Serial.print("A=(");
    Serial.print(ax); Serial.print("\t");Serial.print(ay); Serial.print("\t");Serial.print(az);Serial.print(")\t G=(");
    Serial.print(gx); Serial.print("\t");Serial.print(gy); Serial.print("\t");Serial.print(gz);Serial.print(")\t");
#endif
#ifdef OUTPUT_READABLE_ACCEL
    Serial.print("A=(");
    Serial.print((int16_t)arx); Serial.print("\t");Serial.print((int16_t)ary); Serial.print("\t");Serial.print((int16_t)arz);Serial.print(") m/s^2\t"); //deg
#endif
#ifdef OUTPUT_READABLE_ROTATION
    Serial.print("R=(");
    Serial.print((int16_t)rx); Serial.print("\t");Serial.print((int16_t)ry); Serial.print("\t");Serial.print((int16_t)rz);Serial.print(") deg\t");
#endif
#ifdef OUTPUT_READABLE_VELOCITY
    Serial.print("V=(");
    Serial.print((int16_t)(vx*10)); Serial.print("\t");Serial.print((int16_t)(vy*10)); Serial.print("\t");Serial.print((int16_t)(vz*10));Serial.print(") cm/s\t"); //cm/s
#endif    
#ifdef OUTPUT_READABLE_DISPLACEMENT
    Serial.print("D=(");    
    Serial.print((int16_t)(x*10)); Serial.print("\t");Serial.print((int16_t)(y*10)); Serial.print("\t");Serial.print((int16_t)(z*10)); ;Serial.print(") cm"); //cm    
#endif

    Serial.println(""); 
    BLINK();
    
    t = t1;
}

void calibrate(uint16_t nsamp) {
    if(!nsamp) return;
    uint16_t isamp=nsamp;
    dwax=dway=dwaz=dwgx=dwgy=dwgz=0;
    while(isamp--) {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      dwax+=ax; dway+=ay; dwaz+=az;
      dwgx+=gx; dwgy+=gy; dwgz+=gz;
#ifdef OUTPUT_READABLE_ACCELGYRO_CALIB
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
#endif
      BLINK();
      Serial.print(".");
      delay(10);
    }
    Serial.println();
    base_ax=dwax/nsamp;
    base_ay=dway/nsamp;
    base_az=dwaz/nsamp;
    base_gx=dwgx/nsamp;
    base_gy=dwgy/nsamp;
    base_gz=dwgz/nsamp;
}
