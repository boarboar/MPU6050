#include <Wire.h>
#include "I2Cdev.h"

const int MPU_SDA=0;
const int MPU_SDL=2;
const int DEV_ID=4;

void setup() {
    delay(2000);
    Serial.begin(115200);
    Wire.begin(MPU_SDA, MPU_SDL);

    Serial.print("Sending...");
    bool res = I2Cdev::writeByte(DEV_ID, 0, 0);
    Serial.println(res);
}

void loop() {
  // put your main code here, to run repeatedly:

}
