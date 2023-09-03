#include <WiFi.h>
#include "DHT.h"
#include "ThingSpeak.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
BH1750 bh1750_b;
Adafruit_BMP280 bmp; // I2C
#define DHTPIN 17     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
const char* ssid = "";   // your network SSID (name) 
const char* password = "";   // your network password
WiFiClient  client;
unsigned long myChannelNumber = 1;
const char * myWriteAPIKey = "27MFNV9UT5I3KO3A";
unsigned long lastTime = 0;
unsigned long timerDelay = 15000;

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);}
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println(F("DHTxx test!"));
  dht.begin();
     WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);  // Initialize ThingSpeak
   Wire1.begin(33,32);
    bh1750_b.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire1);
}

void loop() {
  
  if ((millis() - lastTime) > timerDelay) {
    
    // Connect or reconnect to WiFi
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect");
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password); 
        delay(5000);     
      } 
      Serial.println("\nConnected.");
    } 
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  float t=bmp.readTemperature();
  float p=bmp.readPressure();
float lux = bh1750_b.readLightLevel();
int s = analogRead(35); //Sensörden analog değer okuyoruz.
delay(1);
 int TN=analogRead(34);
 delay(1);
  // Check if any reads failed and exit early (to try again).
 if (isnan(h) || isnan (t) || isnan (p)){
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
 }
  
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.println(" %");
   Serial.print(F("Temperature: "));
    Serial.print(t);
     Serial.println(F(" *C"));
      Serial.print(F("Pressure: "));
       Serial.print(p/100);
        Serial.println(" hPa");
     Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
 Serial.print("Smoke: ");
  Serial.println(s);
 Serial.print("Soil Dryness: ");
  Serial.println(TN);
  
  ThingSpeak.setField(1, t);
    ThingSpeak.setField(2, h);
    ThingSpeak.setField(3, p);
    ThingSpeak.setField(4, lux);
     ThingSpeak.setField(5, s);
     ThingSpeak.setField(6, TN);
 int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey); 

   if(x == 200){
     Serial.println("Channel update successful.");
   }
   else{
     Serial.println("Problem updating channel. HTTP error code " + String(x));
   }
     
   lastTime = millis();
}
}
