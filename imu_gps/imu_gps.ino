#include "ICM20600.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define g 9.81

int RXPin = 2;
int TXPin = 3;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
 
ICM20600 icm20600(true);
uint32_t last_time = 0, gps_time = 0;
double acc_offset[] = {0.0, 0.0, 0.0};
double gyro_offset[] = {0.0, 0.0, 0.0};
double rot[] = {0.0, 0.0, 0.0};

void calibrate(uint32_t timeout, double* acc_offset, double* gyro_offset)
{
    double temp_acc_offset_x = 0, temp_acc_offset_y = 0, temp_acc_offset_z = 0;
    double temp_gyro_offset_x = 0, temp_gyro_offset_y = 0, temp_gyro_offset_z = 0;

    uint32_t timeStart = 0, count = 0;
    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
      count++;
      temp_acc_offset_x += icm20600.getAccelerationX();
      temp_acc_offset_y += icm20600.getAccelerationY();
      temp_acc_offset_z += icm20600.getAccelerationZ();
      temp_gyro_offset_x += icm20600.getGyroscopeX();
      temp_gyro_offset_y += icm20600.getGyroscopeY();
      temp_gyro_offset_z += icm20600.getGyroscopeZ();

      Serial.print(".");
      delay(100);
    }

    temp_acc_offset_x = -temp_acc_offset_x / count;
    temp_acc_offset_y = -temp_acc_offset_y / count;
    temp_acc_offset_z = -temp_acc_offset_z / count;
    temp_gyro_offset_x = -temp_gyro_offset_x / count;
    temp_gyro_offset_y = -temp_gyro_offset_y / count;
    temp_gyro_offset_z = -temp_gyro_offset_z / count;

    acc_offset[0] = temp_acc_offset_x;
    acc_offset[1] = temp_acc_offset_y;
    acc_offset[2] = temp_acc_offset_z;
    gyro_offset[0] = temp_gyro_offset_x;
    gyro_offset[1] = temp_gyro_offset_y;
    gyro_offset[2] = temp_gyro_offset_z;
}
 
void setup()
{
  Wire.begin();
  icm20600.initialize();
  Serial.begin(115200);
  gpsSerial.begin(9600);

  Serial.println("Starting calibration after 2 seconds.");
  delay(2000);
  calibrate(10000, acc_offset, gyro_offset);
  Serial.println();
}
 
void loop()
{
  // get acceleration
  int64_t raw_acc_x = icm20600.getAccelerationX();
  int64_t raw_acc_y = icm20600.getAccelerationY();
  int64_t raw_acc_z = icm20600.getAccelerationZ();
  double acc_x = raw_acc_x*1e-3*g;
  double acc_y = raw_acc_y*1e-3*g;
  double acc_z = raw_acc_z*1e-3*g;

  // get orientation
  double gyro_x = (double) icm20600.getGyroscopeX() + gyro_offset[0];
  double gyro_y = (double) icm20600.getGyroscopeY() + gyro_offset[1];
  double gyro_z = (double) icm20600.getGyroscopeZ() + gyro_offset[2];

  if (last_time == 0) {
    last_time = millis();
  }
  else {
    uint32_t now = millis();
    double elapsed = 1e-3 * (now - last_time);
    rot[0] += elapsed * gyro_x;
    rot[1] += elapsed * gyro_y;
    rot[2] += elapsed * gyro_z;
    last_time = now;
  }

  double force_magnitude = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);

  if (force_magnitude > 9.7 && force_magnitude < 9.9) {
    // roll/pitch in radian
    double roll = atan2(raw_acc_y, raw_acc_z);
    double pitch = atan2(-raw_acc_x, sqrt(raw_acc_y * raw_acc_y + raw_acc_z * raw_acc_z));

    rot[0] = degrees(roll);
    rot[1] = degrees(pitch);
  }

  // Correct acceleration using roll and pitch?
  // acc_x = acc_x*1e-3*g + sin(radians(rot[1])) * g;
  // acc_y = acc_y*1e-3*g - sin(radians(rot[0])) * g;

  // Print results
  Serial.print("r ");
  Serial.print(rot[0]);
  Serial.print(" ");
  Serial.print(rot[1]);
  Serial.print(" ");
  Serial.println(rot[2]);

  Serial.print("a ");
  Serial.print(acc_x);
  Serial.print(" ");
  Serial.print(acc_y);
  Serial.print(" ");
  Serial.println(acc_z);

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print("g ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(" ");
        Serial.println(gps.location.lng(), 6);
      }
      else {
        Serial.println("Not available");
      }
    }
  }

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis()-gps_time > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
    gps_time = millis();
  }
}