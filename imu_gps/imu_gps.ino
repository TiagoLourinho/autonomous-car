#include "ICM20600.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

int RXPin = 2;
int TXPin = 3;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
 
ICM20600 icm20600(true);
int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
 
void setup()
{
    Wire.begin();
    icm20600.initialize();
    Serial.begin(9600);
    gpsSerial.begin(9600);
}
 
void loop()
{
  acc_x = icm20600.getAccelerationX();
  acc_y = icm20600.getAccelerationY();
  gyro_z = icm20600.getGyroscopeZ();

  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isValid())
      {
        Serial.print(acc_x);
        Serial.print(" ");
        Serial.print(acc_y);
        Serial.print(" ");
        Serial.print(gyro_z);
        Serial.print(" ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" ");
        Serial.println(gps.location.lng(), 6);
      }
      else
      {
        Serial.println("Not available");
      }
      delay(500);
    }

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
}