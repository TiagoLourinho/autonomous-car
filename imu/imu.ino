#include "ICM20600.h"
#include <Wire.h>
 
ICM20600 icm20600(true);
int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
 
void setup()
{
    Wire.begin();
    icm20600.initialize();
    Serial.begin(9600);
    Serial.println("Starting");
}
 
void loop()
{
  acc_x = icm20600.getAccelerationX();
  acc_y = icm20600.getAccelerationY();
  gyro_z = icm20600.getGyroscopeZ();

  Serial.print(acc_x);
  Serial.print(" ");
  Serial.print(acc_y);
  Serial.print(" ");
  Serial.println(gyro_z);

  delay(100);
}
