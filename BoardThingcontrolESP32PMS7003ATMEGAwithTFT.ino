#include "HardwareSerial_NB_BC95.h"

#include <Adafruit_GFX.h>    // Core graphics library by Adafruit
#include <Arduino_ST7789.h> // Hardware-specific library for ST7789 (with or without CS pin)
#include <SPI.h>
#include <BME280I2C.h>
#include <Wire.h>



// Board Thingcontrol
#define TFT_DC    19
#define TFT_RST   5
#define TFT_MOSI  23   // for hardware SPI data pin (all of available pins)
#define TFT_SCLK  18   // for hardware SPI sclk pin (all of available pins)

Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK); //for display without CS pin

String json = "";

HardwareSerial hwSerial(1);
#define SERIAL1_RXPIN 25
#define SERIAL1_TXPIN 26
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms


String deviceToken = "091DTcf3hjLcklhVwPnx";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;

HardwareSerial_NB_BC95 AISnb;
const long interval = 20000;  //millisecond
unsigned long previousMillis = 0;
float temp(NAN), hum(NAN), pres(NAN);




struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm01_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};


struct pms7003data data;

void _initBME280()
{
  while (!Serial) {} // Wait

  //  Wire.begin();
  //  Wire.begin(27,14);
  pinMode(32, OUTPUT); // on BME280
  
  digitalWrite(32, HIGH); // on BME280
  delay(200);
  Wire.begin(21, 22);

  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }
}

void _initLCD() {
  tft.init(240, 240);   // initialize a ST7789 chip, 240x240 pixels
  //  uint16_t time = millis();
  tft.fillScreen(BLACK);
  tft.setTextColor(GREEN);
  tft.setTextSize(3);
  tft.setCursor(0, 110);
  tft.println("  Starting..");
}

void setup() {
  pinMode(15, OUTPUT); // turn on PMS7003
 
  digitalWrite(15, HIGH); // turn on PMS7003
   delay(200);
  Serial.begin(115200);
  _initLCD();
  hwSerial.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);

  AISnb.debug = true;

  AISnb.setupDevice(serverPort);
  //
  String ip1 = AISnb.getDeviceIP();
  delay(1000);
  //
  pingRESP pingR = AISnb.pingIP(serverIP);
  previousMillis = millis();


  _initBME280();

}


void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    readPMSdata(&hwSerial);
    printBME280Data();

    // tft print function
    tft.fillScreen(BLACK);

    tft.setCursor(0, 30);
    tft.setTextColor(GREEN);
    tft.setTextSize(3);
    tft.println("  Air Report");
    tft.println(" -----------");
    tft.setTextColor(YELLOW);
    tft.print(" Temp: ");
    tft.println(temp);
    tft.print(" Pres: ");
    tft.println(pres);
    tft.print(" PM1.0: ");
    tft.print(data.pm01_env); tft.println(" ug");
    tft.print(" PM2.5: ");
    tft.print(data.pm25_env); tft.println(" ug");
    tft.print(" PM 10:");
    tft.print(data.pm100_env); tft.println(" ug");
    //    //delay(1000);

    composeJson();
    Serial.println(json);
    getAQI() ;
    // Send data in String
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);

    previousMillis = currentMillis;
    UDPReceive resp = AISnb.waitResponse();

  }
}
void composeJson() {
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"temp\":");
  json.concat(temp);
  json.concat(",\"hum\":");
  json.concat(hum);
  json.concat(",\"pres\":");
  json.concat(pres);
  json.concat(",\"pm1\":");
  json.concat(data.pm01_env);
  json.concat(",\"pm2.5\":");
  json.concat(data.pm25_env);
  json.concat(",\"pm10\":");
  json.concat(data.pm100_env);

  json.concat(",\"pn03\":");
  json.concat(data.particles_03um);
  json.concat(",\"pn05\":");
  json.concat(data.particles_05um);
  json.concat(",\"pn10\":");
  json.concat(data.particles_10um);
  json.concat(",\"pn25\":");
  json.concat(data.particles_25um);
  json.concat(",\"pn50\":");
  json.concat(data.particles_50um);
  json.concat(",\"pn100\":");
  json.concat(data.particles_100um);
  json.concat("}");


}

void getAQI() {

  // reading data was successful!
  //  Serial.println();
  //  Serial.println("---------------------------------------");
  //  Serial.println("Concentration Units (standard)");
  //  Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
  //  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
  //  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
  //  Serial.println("---------------------------------------");
  //  Serial.println("Concentration Units (environmental)");
  Serial.print("PM1.0:"); Serial.print(data.pm01_env);
  Serial.print("\tPM2.5:"); Serial.print(data.pm25_env);
  Serial.print("\tPM10:"); Serial.println(data.pm100_env);
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
  Serial.println("---------------------------------------");

}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  memcpy((void *)&data, (void *)buffer_u16, 30);
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  if (sum != data.checksum) {
    //    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
void printBME280Data()
{
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  //delay(500);
}
