# Samlede kode til alle komponenter.

```c++
// Downloader alle nødvendige biblotekker.

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <SPI.h>
#include <SD.h>




// LCD setup
// Først skal vi initialisere LCD-displayet og angive dets dimensioner. Her er det 16 karakterer i bredden og 2 linjer i højden.
LiquidCrystal_I2C lcd(0x27, 16, 2);



#define DHTPIN 7     // Digital udgang forbundet til DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);


#define sensor A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog) - Definerer analog indgang for afstandssensor


// SD kort setup
File myFile; // Her opretter vi en variabel "myFile", som kan bruges til at referere til vores SD kort læser.
const int chipSelect = 10;

void setup() {

  Serial.begin(9600); //OBS baud rate 

  // Starter LCD-display
  lcd.init();
  lcd.setBacklight((uint8_t)1); // Sætter baggrundslyset

  // Starter dht
  dht.begin();


  // SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {   //Tallet i parentesen skal være det pinnummer, I har tildelt CS fra modulet til jeres SD-læser
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");


  // Skriver kolonne navne ind i logfilen
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Temperatur (C)");
    myFile.print("\t");
    myFile.print("Luftfugtighed (%)");
    myFile.print("\t");
    myFile.print("Afstand (mm)");
    myFile.print("\t");
    //myFile.print("Lys intensitet (0-100)");
    myFile.print("\n");
    myFile.close();
  } else {
    Serial.println("Kan ikke åbne log fil");
  }
}

void loop() {


  // Måler temperatur og luftfugtighed
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();


  // Vis temperatur og luftfugtighed på LCD-displayet.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");

  // Måler distance med IR-afstandsmåler
  float volts = analogRead(sensor)*0.0048828125;  // Omregner værdi fra bit til volt: Værdi fra sensor * (5/1024)
  float distance = 13*pow(volts, -1); // Omregner volt til afstand (cm) vha. funktion for standardkurve

  delay(2000);


if (distance >= 4 && distance <= 30) {
    // Update LCD for distance
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print("cm");
    lcd.setCursor(0, 1);
  } else {
    // if the file didn't open, print an error:
    Serial.println("Distance out of range");
    // You might also want to display an error on the LCD here
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance");
    lcd.setCursor(0, 1);
    lcd.print("out of range");
  }

  // Log data to SD card
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(temperature);
    myFile.print("\t");
    myFile.print(humidity);
    myFile.print("\t");
    myFile.println(distance);
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


```
