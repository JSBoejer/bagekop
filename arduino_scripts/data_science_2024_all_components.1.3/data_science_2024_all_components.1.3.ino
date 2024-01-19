#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <dht.h>
#include "SparkFun_VL53L1X.h"
#include <SPI.h>
#include <SD.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT setup
dht DHT;
#define DHT11_PIN 7

// VL53L1X setup
SFEVL53L1X distanceSensor;

// RTC setup
ThreeWire myWire(4,5,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

// SD card setup
File myFile;
const int chipSelect = 10;

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);  // Light sensor setup

  // LCD
  lcd.init();
  lcd.setBacklight((uint8_t)1);

  // DHT11 (no explicit initialization required)

  // VL53L1X distance sensor
  Wire.begin();
  if (distanceSensor.begin() != 0) {
    Serial.println("VL53L1X sensor initialization failed");
    while (1);
  }

  // SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {   //Tallet i parentesen skal være det pinnummer, I har tildelt CS fra modulet til jeres SD-læser
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");


  // RTC
  Rtc.Begin();


  // Log headers to SD card
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Date");
    myFile.print("\t");
    myFile.print("Time");
    myFile.print("\t");
    myFile.print("Temperature (C)");
    myFile.print("\t");
    myFile.print("Humidity (%)");
    myFile.print("\t");
    myFile.print("Distance (mm)");
    myFile.print("\t");
    myFile.print("Light (0-100)");
    myFile.print("\n");
    myFile.close();
  } else {
    Serial.println("Error opening log.txt");
  }
}

void loop() {
  // Read from DHT11
  DHT.read11(DHT11_PIN);
  float temperature = DHT.temperature;
  float humidity = DHT.humidity;

  // Read from VL53L1X
  distanceSensor.startRanging();
  while (!distanceSensor.checkForDataReady()) {
    delay(1);
  }
  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  // Read light level
  int lightRaw = analogRead(A0);
  int light = map(lightRaw, 0, 1023, 0, 100);

  // Get current date and time from RTC
  RtcDateTime now = Rtc.GetDateTime();

  // Update LCD for temperature and humidity
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
  delay(2000);

  // Update LCD for distance and light
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print("mm");
  lcd.setCursor(0, 1);
  lcd.print("Light: ");
  lcd.print(light);
  lcd.print("%");

  // Log data to SD card
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(now.Day());
    myFile.print('/');
    myFile.print(now.Month());
    myFile.print('/');
    myFile.print(now.Year());
    myFile.print("\t");
    myFile.print(now.Hour());
    myFile.print(':');
    myFile.print(now.Minute());
    myFile.print(':');
    myFile.print(now.Second());
    myFile.print("\t");
    myFile.print(temperature);
    myFile.print("\t");
    myFile.print(humidity);
    myFile.print("\t");
    myFile.print(distance);
    myFile.print("\t");
    myFile.println(light);
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("Error opening log.txt");
    // You might also want to display an error on the LCD here
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error opening");
    lcd.setCursor(0, 1);
    lcd.print("log.txt");
  }

  // Delay before next loop iteration
  delay(2000);
}