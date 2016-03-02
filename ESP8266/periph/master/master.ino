#include <Wire.h>
#include "I2Cdev.h"

const int MPU_SDA=0;
const int MPU_SDL=2;
const int DEV_ID=4;

void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("Init Wire...");

    Wire.begin(MPU_SDA, MPU_SDL);

    Serial.println("Sending...");
    bool res = I2Cdev::writeByte(DEV_ID, 0x01, 0x02);
    Serial.println(res);
}

void loop() {
  // put your main code here, to run repeatedly:

}
