#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <dht.h>
#include "SparkFun_VL53L1X.h"
#include <SPI.h>
#include <SD.h>
#include <DS3231.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT setup
dht DHT;
#define DHT11_PIN 7

// VL53L1X setup
SFEVL53L1X distanceSensor;

// RTC setup
DS3231 rtc(SDA, SCL);

// SD card setup
File myFile;
const int chipSelect = 10; // CS pin for SD card module

void setup() {
  // Begin Serial communication
  Serial.begin(9600);

  // Initialize the LCD
  lcd.begin();
  lcd.setBacklight((uint8_t)1);
  lcd.clear();

  // Initialize DHT11
  // Note: DHT does not require an explicit begin call

  // Initialize VL53L1X distance sensor
  Wire.begin();
  if (distanceSensor.begin() != 0) {
    Serial.println("VL53L1X sensor initialization failed");
    while (1);
  }

  // Initialize RTC
  rtc.begin();

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // Print initial message on LCD
  lcd.print("System Ready");
}

void loop() {
  // Read temperature and humidity from DHT11
  int chk = DHT.read11(DHT11_PIN);
  float temperature = DHT.temperature;
  float humidity = DHT.humidity;

  // Display temperature and humidity on LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");

  // Get distance measurement from VL53L1X
  distanceSensor.startRanging();
  while (!distanceSensor.checkForDataReady()) {
    delay(1);
  }
  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  // Output the sensor readings to the serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C, Humidity: ");
  Serial.print(humidity);
  Serial.print("%, Distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  // Get the current date and time from RTC
  String currentDate = rtc.getDateStr();
  String currentTime = rtc.getTimeStr();

  // Log data to the SD card
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(currentDate);
    myFile.print(",");
    myFile.print(currentTime);
    myFile.print(",");
    myFile.print(temperature);
    myFile.print(",");
    myFile.print(humidity);
    myFile.print(",");
    myFile.println(distance);
    myFile.close();
  } else {
    Serial.println("Error opening log.txt");
  }

  // Update LCD every 2 seconds
  delay(2000);
  lcd.clear();
}
