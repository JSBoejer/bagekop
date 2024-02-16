# Samlede kode til alle komponenter.

Der er indsat en pære med 220 Ohm modstand, som vist i guiden her. Input signalet i pæren skal gå til port 8.
Den pære blinker og er en indikator for om arduinonen stadig måler. Hvis pæren ikke blinker, så er arduinonen frosset.


```c++
// Downloader alle nødvendige biblotekker.

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <dht.h>
#include "SparkFun_VL53L1X.h"
#include <SPI.h>
#include <SD.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>




// LCD setup
// Først skal vi initialisere LCD-displayet og angive dets dimensioner. Her er det 16 karakterer i bredden og 2 linjer i højden.
LiquidCrystal_I2C lcd(0x27, 16, 2);


// DHT setup
dht DHT; // Her opretter vi en variabel "DHT", som kan bruges til at referere til vores temperature og fugtighedsmåler.
#define DHT11_PIN 7 //Outputtet fra måleren skal sendes ind i digital input port 7


// VL53L1X setup
SFEVL53L1X distanceSensor; // Her opretter vi en variabel "distanceSensor", som kan bruges til at referere til vores afstandsmåler.


// RTC setup
ThreeWire myWire(4,5,2); // Forbindelser til klokmodulet angives her.
RtcDS1302<ThreeWire> Rtc(myWire); // Her opretter vi en variabel "Rtc", som kan bruges til at referere til vores klokmodul.


// SD kort setup
File myFile; // Her opretter vi en variabel "myFile", som kan bruges til at referere til vores SD kort læser.
const int chipSelect = 10;


// Blinkede LED pære setup
int LEDpin = 8; //Input signalet til pæren skal komme fra digital output port 8



void setup() {

  Serial.begin(9600); OBS baud rate 


  pinMode(A0, INPUT);  // Definerer input porten til lyssensoren

  pinMode(LEDpin, OUTPUT); //Input signalet til pæren skal komme fra digital output port 8 (LEDpin's værdi)

  // Starter LCD-display
  lcd.init();
  lcd.setBacklight((uint8_t)1); // Sætter baggrundslyset



  // VL53L1X distance sensor
  // Hvis sensoren finder en fejl under opstart, vil den fryse og udskrive en fejlmeddelelse.
  if (distanceSensor.begin() != 0)
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");



  // SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {   //Tallet i parentesen skal være det pinnummer, I har tildelt CS fra modulet til jeres SD-læser
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");


  // RTC
  Rtc.Begin();
  //Hvis nedstående linjer indsættet i koden, så vil klokmodulets tid nulstilles til det tidspunkt, hvor denne sketch uploades til arduino.
  //RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  //Rtc.SetDateTime(compiled);

  // Skriver kolonne navne ind i logfilen
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Dato");
    myFile.print("\t");
    myFile.print("Tid");
    myFile.print("\t");
    myFile.print("Temperatur (C)");
    myFile.print("\t");
    myFile.print("Luftfugtighed (%)");
    myFile.print("\t");
    myFile.print("Afstand (mm)");
    myFile.print("\t");
    myFile.print("Lys intensitet (0-100)");
    myFile.print("\n");
    myFile.close();
  } else {
    Serial.println("Kan ikke åbne log fil");
  }
}

void loop() {


  // Måler temperatur og luftfugtighed
  DHT.read11(DHT11_PIN);
  float temperature = DHT.temperature;
  float humidity = DHT.humidity;


  // Måler distance med IR-afstandmåler
  int distance = getDistance(); // getDistance henviser til function i bunden.
  float distanceCM = distance / 10;


  // Måler lysintensitet
  int lightRaw = analogRead(A0);
  int light = map(lightRaw, 0, 1023, 0, 100);


  // Læser dato og tid i ISO Get current date and time from RTC in standard IUPAC time format
  String iupacTime = getIUPACTime();


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

  digitalWrite(LEDpin, HIGH);
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
    myFile.print(iupacTime);
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

  digitalWrite(LEDpin, LOW);
  // Delay before next loop iteration
  delay(2000);
}



// Function til at omformattere tidsformattet til ISO 8601

String getIUPACTime() {
    // Get current date and time from RTC
    RtcDateTime now = Rtc.GetDateTime();
    String dateTime;

    // Year
    dateTime += String(now.Year());
    dateTime += '-';

    // Month
    if (now.Month() < 10) dateTime += '0';
    dateTime += String(now.Month());
    dateTime += '-';

    // Day
    if (now.Day() < 10) dateTime += '0';
    dateTime += String(now.Day());
    dateTime += 'T'; // T is used to separate the date and time in ISO 8601 format

    // Hour
    if (now.Hour() < 10) dateTime += '0';
    dateTime += String(now.Hour());
    dateTime += ':';

    // Minute
    if (now.Minute() < 10) dateTime += '0';
    dateTime += String(now.Minute());
    dateTime += ':';

    // Second
    if (now.Second() < 10) dateTime += '0';
    dateTime += String(now.Second());

    return dateTime;
}


// Funktion til at måle afstand

int getDistance() {
  distanceSensor.startRanging();
  while (!distanceSensor.checkForDataReady()) {
    delay(1);
  }
  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  return distance;
}

```
