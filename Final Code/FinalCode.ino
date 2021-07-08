/* includes required libraries */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <SD.h>
#include <BluetoothSerial.h>
// Date and time using a RTC connected via I2C and Wire lib
#include "RTClib.h"


#define BNO055_SAMPLE_DELAY_MS (100)
#define chipSelect 33
RTC_PCF8523 rtc;

//variable declaration:
String dataString = " ";
int button = 27;
int LED = 12;
int buttonvalue;
int buttonvalueold = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

int previousorix;
int previousoriy;
int previousoriz;

int currentorix;
int currentoriy;
int currentoriz;

/* creating object to write data and for the IMU */
Adafruit_BNO055 IMU = Adafruit_BNO055();
File IMUdata;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it // BT error check borrowed from arduino example library
#endif

BluetoothSerial SerialBT;

char val = '0';
char lastVal = '0';


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  delay(1000);
  /* checks if the IMU is connected */
  Serial.println("Initializing IMU");
  if (!IMU.begin())
  {
    /* displays if IMU is not connected */
    Serial.print("The IMU was not detected, confirm the device is connected!");
    while (1);
  }
  Serial.println("Done..."); Serial.println("");

  delay(1000);

  /* checks if the SD is connected */
  Serial.println("Initializing SD Card");
  if (!SD.begin(chipSelect)) {
    Serial.println("Initializing SD Card FAILED! Please do not forget to connect the SD Card."); Serial.println("");
    while (1);
  }
  Serial.println("Done..."); Serial.println("");

  /* starts the IMU */
  IMU.begin();
  /* starts the SD */
  SD.begin(chipSelect);

  /* IMUdata = SD.open("/"); */

  delay(1000);

  /* I was going to have it record the temperature, but I did not think it would be relevant */
  /* creates variable for temp*/


  /* for better results uses the external piezo crystal as a time reference*/

  IMU.setExtCrystalUse(true);

  Serial.println("Datalogging Quaternion Test"); Serial.println("");

  IMUdata = SD.open("/MovementData.txt", FILE_WRITE);

  SerialBT.begin("IMU Datalogger");
  Serial.println("IMU Datalogger has started, ready to pair with bluetooth"); Serial.println("");

  pinMode(button, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(button, LOW);
  digitalWrite(LED, LOW);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.start();


}



void loop() {
  int8_t temp = IMU.getTemp();
  DateTime now = rtc.now();
  // put your main code here, to run repeatedly:
  buttonvalue = digitalRead(button);

  /* creates variable for callibration of gyroscope/accelerometer/magnetometer */
  /* magnetometer needs to be initialized at 0 */
  uint8_t systemcal, gyrocal, accelcal, magcal = 0;

  /* associates those variables with the correct calibration references from the BNO055 */
  IMU.getCalibration(&systemcal, &gyrocal, &accelcal, &magcal);

  /* creates the vector that holds the rotation,x,y,z quaternions*/
  imu::Quaternion quatern = IMU.getQuat();

  /* prints data in serial, relevant for vpython*/

  if ( SerialBT.available() )      // if data is available to read
  {
    ;
  }
  val = SerialBT.read();         // read it and store it in 'val'
  if (val == '1' || val == '0') {
    lastVal = val;
  }

  if ( lastVal == '0' )              // if '1' was received
  {

    delay(1000);                  // waits for a second
  }

  else if (lastVal == '1')

  {
    sensors_event_t event;
    IMU.getEvent(&event);
    dataString = "[" + String(event.orientation.z, 4) + ", " + String(event.orientation.y, 4) + ", " + String(event.orientation.z, 4) + "]"; //can modify this later if we want raw sensor data
    SerialBT.println(dataString);
    delay(100);
  }

  sensors_event_t event;
  previousorix = currentorix;
  previousoriy = currentoriy;
  previousoriz = currentoriz;
  IMU.getEvent(&event);
  currentorix = event.orientation.x;
  currentoriy = event.orientation.y;
  currentoriz = event.orientation.z;


  if (IMUdata) {

    Serial.print(quatern.w());
    Serial.print(",");
    Serial.print(quatern.x());
    Serial.print(",");
    Serial.print(quatern.y());
    Serial.print(",");
    Serial.print(quatern.z());
    Serial.print(",");
    Serial.print(accelcal);
    Serial.print(",");
    Serial.print(gyrocal);
    Serial.print(",");
    Serial.print(magcal);
    Serial.print(",");
    Serial.print(systemcal);
    Serial.print(", ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(", ");
    Serial.print(temp);
    Serial.print("C");

    if ((previousorix != currentorix) && (previousoriy != currentoriy) && (previousoriz != currentoriz)) {
      Serial.print(", Subject is moving");
      if ((previousorix - currentorix) >= abs(80) &&  (previousoriy - currentoriy) <= abs(19) && (previousoriz - currentoriz) <= abs(19)) {
        Serial.print(" and jumped");
      }
      if ((previousoriz - currentoriz) >= abs(80) &&  (previousoriy - currentoriy) <= abs(19) && (previousorix - currentorix) <= abs(19)) {
        Serial.print(" and jumped");
      }
      Serial.println(".");
    } else {
      Serial.println(", Subject is not moving.");
    }




    /* records data ito the file we created before*/
    if (buttonvalue == 1 && buttonvalueold == 0) {
      digitalWrite(LED, HIGH);
      IMUdata.print("W: ");
      IMUdata.print(quatern.w());
      IMUdata.print(", ");
      IMUdata.print("\tX: ");
      IMUdata.print(quatern.x());
      IMUdata.print(", ");
      IMUdata.print("\tY: ");
      IMUdata.print(quatern.y());
      IMUdata.print(", ");
      IMUdata.print("\tZ: ");
      IMUdata.print(quatern.z());
      IMUdata.print(",\t ");
      IMUdata.print(accelcal);
      IMUdata.print(",\t ");
      IMUdata.print(gyrocal);
      IMUdata.print(",\t ");
      IMUdata.print(magcal);
      IMUdata.print(",\t ");
      IMUdata.print(systemcal);

      IMUdata.print(now.year(), DEC);
      IMUdata.print('/');
      IMUdata.print(now.month(), DEC);
      IMUdata.print('/');
      IMUdata.print(now.day(), DEC);
      IMUdata.print(" (");
      IMUdata.print(daysOfTheWeek[now.dayOfTheWeek()]);
      IMUdata.print(") ");
      IMUdata.print(now.hour(), DEC);
      IMUdata.print(':');
      IMUdata.print(now.minute(), DEC);
      IMUdata.print(':');
      IMUdata.print(now.second(), DEC);
      IMUdata.print(temp);
      IMUdata.print("C");
      IMUdata.print(", ");


      if ((previousorix != currentorix) && (previousoriy != currentoriy) && (previousoriz != currentoriz)) {
        IMUdata.print(", Subject is moving");
        if ((previousorix - currentorix) >= abs(80) &&  (previousoriy - currentoriy) <= abs(19) && (previousoriz - currentoriz) <= abs(19)) {
          IMUdata.print(" and jumped");
        }
        if ((previousoriz - currentoriz) >= abs(80) &&  (previousoriy - currentoriy) <= abs(19) && (previousorix - currentorix) <= abs(19)) {
          IMUdata.print(" and jumped");
        }
        IMUdata.println(".");
      } else {
        IMUdata.println(", Subject is not moving.");
      }
    } else {
      digitalWrite(LED, LOW);
      if (buttonvalue == buttonvalueold) {
        buttonvalueold = 1;
      } else {
        buttonvalueold = 0;
      }
    }
  } else {

    Serial.println("Could not create MovementData.txt!");

  }

}
