/**
 * M5Stack Weather Station
 * by Andy Page
 * Version 7.0
 * 21st September 2021
 * Added UV & Visible light calculations and display.
 * 
 * 30th December 2021
 * 
 * 
 */


#include <M5Stack.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <String.h>
#include "time.h"

//MODBUS CRC Calculator Table (Polynomial x^16+x^15+x^2+1 with starting value 0xFFFF)
const uint16_t wCRCTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

// Constants for calculating external temperature
const float RCALC = 0.1220703;
const float CONSTANT1 = 3.9083;
const float CONSTANT2 = 15.2725;
const float CONSTANT3 = 0.00231;
const float RTC0R = 1000.0;
const float CONSTANT4 = 0.001155;
const float RALPHA = 0.00385; //Simple slope constant
const float tempOffset = 1.2; //Add this to final temperature for correct calibration

// Constants for calculated external RH
const float RHV_CONSTANT = 0.912611;
const float RHM1 = 48.23;
const float RHC1 = -23.82;
const float RHC2 = 1.0546;
const float RHM2 = 0.00216;
const float DIV_RATIO = 1.28455;
const float RH_SUPPLY_VOLTAGE = 3.32; //Fixed value on this sensor as it is derived from a voltage regulator
const float RHCONSTANT1 = 0.1515;
const float RHCONSTANT2 = 157.2327;

//UV & Visible light constants
const float uvAlpha = 1.0;
const float uvBeta = 1.0;
const float uvGamma = 1.0;
const float uvDelta = 1.0;
const float uvAco = 1.986;
const float uvBco = 0.514;
const float uvCco = 2.91;
const float uvDco = 0.681;
const float uvaResp = 0.050933;
const float uvbResp = 0.060095;
const float visLightScale = 1.468;

//Dewpoint calculation constants
#define DEWPOINT_A 17.625
#define DEWPOINT_B 243.12


//Weather and weather sensor variables
Adafruit_BME280 bme;
float intTempC=0;
float presshPa=0;
float slphPa=0;
float intHum=0;
float extTemp=0;
float extRH=0;
float dpC = 0; //External dewpoint in degrees Celsius
float uva=0;
float uvb=0;
float uvi=0;
float vis=0;
float uvIndex=0;
uint32_t sfCount=0;
uint32_t sfDayStartCount=0;
uint32_t sfDayCount=0;

//Old values are used to write over the LCD values before writing new ones
float oldIntTempC=0;
float oldslphPa=0;
float oldIntHum=0;
float oldRain=0;
float oldExtTemp=0;
float oldExtRH=0;
float oldUVA=0;
float oldUVB=0;
float oldUVI=0;
float oldVis=0;
float oldUVIndex=0;
uint32_t oldSFDayCount=0;

//Pressure calibration
float altitude = 10;
float pressureCorrection = 0.45; //Pressure correction offset added to reading from sensor
#define PRESSURE_FACTOR1 44330.0f   //Used for sea level pressure correction
#define PRESSURE_FACTOR2 5.255f     //Used for sea level pressure correction

//Sensor keys.  These must match to process the message further.
uint8_t rainKey[8] = {0xE6,0xBA,0x08,0xFB,0x3A,0x4F,0x5E,0xCE};
uint8_t uvlKey[8] = {0x6E,0xDA,0x82,0x33,0x33,0x66,0xF5,0xE6};
uint8_t trhKey[8] = {0x6C,0x27,0xDA,0x88,0x2F,0xE4,0x1A,0xF0};
uint8_t sfKey[8] = {0xCD,0xEA,0xCB,0x09,0x33,0x6B,0x39,0x5A};

//LoRa variables
uint32_t loraMessageCount=0;
uint32_t oldLoraMessageCount=0;
float rssi=0;
float oldrssi=0;
float snr=0;
float oldSnr=0;
extern portMUX_TYPE mux;
extern volatile uint8_t rxFlag;
uint8_t loraData[255]; //Fixed LoRa buffer

uint8_t cycleCount=0;

uint8_t sdCardPresent=1; //1=present, 0=not present.  Checked at setup()

struct RainGauge{
  uint32_t tips;
  uint8_t rssiValue;
  float sensorTemperature;
  float mcuVoltage;
  float snr;
  float rainCal;
  float rainMM;
  uint32_t rainDayStartTips;
} rainGauge;


//WIFI
//VIRGINMEDIA
//const char* ssid = "VM8390324";
//const char* password = "x4twVgsXgqgn";

//NETGEAR
const char* ssid = "NETGEAR94";
const char* password = "Z1le94Apib7V";

const long gmtOffset_sec = 0; //GMT timezone
const int daylightOffset_sec = 3600; //Daylight saving active at the moment
const char* ntpServer = "pool.ntp.org";
struct tm timeinfo;

//Time
uint8_t oldDate=0;
uint8_t oldMonth=0;
uint16_t oldYear=0;
uint8_t oldSecond=0;
uint8_t oldMinute=0;
uint8_t oldHour=0;
uint8_t oldMin=0;
uint8_t oldDay=0;

//Sensors received message.  Will go to 1 when a message is received from that sensor.
//Prevents wunderground from receiving false zero data when unit is first run.
uint8_t trhRx=0;
uint8_t rainRx=0;
uint8_t uvlRx=0;

//Indicates if this is the first message received since turning on the receiver.  If it is, then reset the dayStartCount to the received value.
uint8_t sfFirst=0;
uint8_t rainFirst=0;

//Sensor response time counts in minutes
uint8_t trhTimerCount=0;
uint8_t rainTimerCount=0;
uint8_t uvlTimerCount=0;
uint8_t sensorTimeout = 20; //Sensor time out after 20 minutes (data becomes invalid)

//Display
#define BACKGROUND BLUE
#define FOREGROUND WHITE
uint8_t backlightIsOn=1;


void setup() {
  M5.begin();
  M5.Power.begin();
  if(!SD.begin()){
    Serial.println("SD Card failed or not present, logging disabled, defaults used.");
    sdCardPresent=0; //Note that SD card is not present.
  }
  else{
    Serial.println("SD Card present and initialised.");
  }

  log("M5 Started\r\n");
  
  doWifiStuff();
  log("M5 Wifi Started\r\n");
  oldMin = timeinfo.tm_min; //oldMin used for looking for new minute to perform end of minute actions
  oldDay = timeinfo.tm_mday; //oldDay used for looking for a new day to perform end of day actions
  M5.Lcd.fillScreen(BACKGROUND);
  M5.Lcd.setBrightness(100);
  startBME280();
  log("BME280 started\r\n");
  LoRaBegin(866500000);
  LoRaContinuousReceive(); //Set to continuous receive mode
  delay(10);
  LoRaClearIRQFlags();
  delay(5);
  log("LoRa started\r\n");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(FOREGROUND);
  M5.Lcd.setCursor(2,2);
  M5.Lcd.printf("RX");
  M5.Lcd.setCursor(2,20);
  M5.Lcd.printf("OUT");
  M5.Lcd.setCursor(2,200);
  M5.Lcd.printf("IN");
  M5.Lcd.setCursor(2,40);
  M5.Lcd.printf("RAIN");
  initialiseRainGauge();
  M5.Lcd.setCursor(2,60);
  M5.Lcd.printf("UVA");
  M5.Lcd.setCursor(150,60);
  M5.Lcd.print("UVB");
  M5.Lcd.setCursor(2,80);
  M5.Lcd.print("UVI");
  M5.Lcd.setCursor(150,80);
  M5.Lcd.print("UVX");
  M5.Lcd.setCursor(2,100);
  M5.Lcd.print("VIS");
  M5.Lcd.setCursor(2,120);
  M5.Lcd.print("SF");
}

void loop() {
  cycleCount++;
  if(cycleCount>100){
    getIntTempHumPressure();
    updateDisplay();
    //printLocalTime();
    updateLCDTimeDate();
    cycleCount=0;
    uint8_t min = timeinfo.tm_min;
    uint8_t day = timeinfo.tm_mday;

    if(min!=oldMin){
        checkAndUpdateSensorTimers(); //Checks if sensors are active or have not responded for the timeout period
        float tempF = CelsiusToFarenheit(extTemp);
        float dpF = CelsiusToFarenheit(dpC);
        float pressureInHG = HectoPascalsToInMercury(slphPa);
        float windDir = 0;
        float wsMPH=0;
        float wgMPH=0;
        float solarIr = luxToWattsPerSquareMetre(vis);
        float rain1hIn = 0;
        float dayRainIn = millimetresToInches(rainGauge.rainMM);
        float indoorTempF = CelsiusToFarenheit(intTempC);
        log("wunderground upload\r\n");
        wunderground(tempF, extRH, pressureInHG, windDir, wsMPH, wgMPH, dpF, solarIr, uvIndex, rain1hIn, dayRainIn, indoorTempF, intHum); //Weather underground upload
        oldMin = min;
    }
    if(day!=oldDay){
      //End of day
      rainGauge.rainDayStartTips = rainGauge.tips;
      rainGauge.rainMM=0;
      sfDayStartCount = sfCount;
      oldDay = day;
    }
  }
  if(rxFlag){
    loraMessageCount++;
    portENTER_CRITICAL(&mux);
    rxFlag=0;
    portEXIT_CRITICAL(&mux);
    Serial.print("LoRa MSG ");
    Serial.println(loraMessageCount);
    Serial.print("Available ");
    uint8_t bytesAvailable = LoRaAvailable();
    Serial.println(bytesAvailable);
    if(bytesAvailable>0){
      LoRaReadMessage(loraData, bytesAvailable);
      dumpMessage(loraData, bytesAvailable);
      processLoRaMessage(loraData, bytesAvailable);
    }
    LoRaClearIRQFlags();
    rssi = LoRaReadPacketRSSI();
    snr = LoRaReadPacketSNR(); 
    M5.Lcd.setTextColor(BACKGROUND);
    M5.Lcd.setCursor(28,2);
    M5.lcd.printf("%d",oldLoraMessageCount); 
    M5.Lcd.setCursor(110,2);
    M5.lcd.printf("%4.1fdB",oldSnr);
    M5.lcd.setCursor(210,2);
    M5.lcd.printf("%4.1fdBm",oldrssi);
    M5.Lcd.setTextColor(FOREGROUND);
    M5.Lcd.setCursor(28,2);
    M5.Lcd.printf("%d",loraMessageCount);
    M5.Lcd.setCursor(110,2);
    M5.lcd.printf("%4.1fdB",snr);
    M5.lcd.setCursor(210,2);
    M5.lcd.printf("%4.1fdBm",rssi); 
    oldLoraMessageCount=loraMessageCount;
    oldSnr = snr;
    oldrssi = rssi;
  }
  delay(10);
  if(M5.BtnA.wasPressed()){
    //Reset max/min/totals
    rainGauge.rainDayStartTips = rainGauge.tips;
    rainGauge.rainMM = (rainGauge.tips - rainGauge.rainDayStartTips) * rainGauge.rainCal;
    sfDayStartCount = sfCount;
    sfDayCount = 0;
    updateDisplay();
  }
  if(M5.BtnB.wasPressed()){
    if(backlightIsOn>0){
      backlightIsOn=0;
      M5.Lcd.setBrightness(0);
    }
    else{
      M5.Lcd.setBrightness(100);
      backlightIsOn=1;
    }
  }
  M5.update();

}

void updateLCDTimeDate(){
    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time!");
    }
    else{
        M5.lcd.setTextColor(BACKGROUND);
        M5.lcd.setCursor(2,220);
        M5.lcd.printf("%02d/%02d/%04d %02d:%02d:%02d", oldDate, oldMonth, oldYear, oldHour, oldMinute,oldSecond);
        M5.Lcd.setTextColor(FOREGROUND);
        M5.lcd.setCursor(2,220);
        uint8_t date = timeinfo.tm_mday;
        uint8_t month = 1 + timeinfo.tm_mon;
        uint16_t year = 1900 + timeinfo.tm_year;
        uint8_t hour = timeinfo.tm_hour;
        uint8_t minute = timeinfo.tm_min;
        uint8_t second = timeinfo.tm_sec;
        M5.lcd.printf("%02d/%02d/%04d %02d:%02d:%02d", date, month, year, hour, minute,second);
        oldDate = date;
        oldMonth = month;
        oldYear = year;
        oldHour = hour;
        oldMinute = minute;
        oldSecond = second;
    }  
}

void doWifiStuff(){
    Serial.println("Doing Wifi stuff...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
//    int ssidLength = 10;
//    int n = WiFi.scanNetworks();
//    for(int i=0;i<n;i++){
//      String ssid = (WiFi.SSID(i).length() > ssidLength) ? (WiFi.SSID(i).substring(0, ssidLength) + "...") : WiFi.SSID(i);
//      Serial.println(") " + ssid + " (" + WiFi.RSSI(i) + ");\n");
//    }
    Serial.print("WiFi connecting...");
    while(WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
    log(WiFi.localIP());
    Serial.println("Configuring RTC...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
}

void printLocalTime(){
    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time!");
        return;
    }
    Serial.println(&timeinfo, "%A, %B, %d, %Y, %H:%M:%S");
}

void updateDisplay(){
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(30,200);
  M5.Lcd.setTextColor(BACKGROUND);
  M5.Lcd.printf("%4.1f",oldIntTempC);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("o");
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("C ");
  M5.Lcd.printf("%4.1f%% ",oldIntHum);
  M5.Lcd.printf("%6.1fhPa",oldslphPa);
  M5.Lcd.setCursor(50,40);
  M5.Lcd.printf("%4.1fmm",oldRain);
  M5.Lcd.setCursor(50,20);
  M5.Lcd.printf("%4.1f", oldExtTemp);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("o");
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("C  ");
  M5.Lcd.printf("%4.1f%% ", oldExtRH);
  M5.Lcd.setCursor(50,60);
  M5.Lcd.printf("%4.1fW/m", oldUVA);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("2");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(200,60);
  M5.Lcd.printf("%4.1fW/m", oldUVB);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("2");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(50,80);
  M5.Lcd.printf("%4.1fW/m", oldUVI);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("2");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(200,80);
  M5.Lcd.printf("%4.1f", oldUVIndex);
  M5.Lcd.setCursor(50,100);
  M5.Lcd.printf("%4.1flux", oldVis);
  M5.Lcd.setCursor(50,120);
  M5.Lcd.printf("%d",oldSFDayCount);

  M5.Lcd.setCursor(30,200);
  M5.Lcd.setTextColor(FOREGROUND);
  M5.Lcd.printf("%4.1f",intTempC);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("o");
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("C ");
  M5.Lcd.printf("%4.1f%% ",intHum);
  M5.Lcd.printf("%6.1fhPa",slphPa);
  M5.Lcd.setCursor(50,40);
  M5.Lcd.printf("%4.1fmm",rainGauge.rainMM);
  M5.Lcd.setCursor(50,20);
  M5.Lcd.printf("%4.1f", extTemp);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("o");
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("C  ");
  M5.Lcd.printf("%4.1f%% ", extRH);
  M5.Lcd.setCursor(50,60);
  M5.Lcd.printf("%4.1fW/m", uva);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("2");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(200,60);
  M5.Lcd.printf("%4.1fW/m", uvb);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("2");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(50,80);
  M5.Lcd.printf("%4.1fW/m", uvi);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("2");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(200,80);
  M5.Lcd.printf("%4.1f", uvIndex);
  M5.Lcd.setCursor(50,100);
  M5.Lcd.printf("%4.1flux", vis);
  M5.Lcd.setCursor(50,120);
  M5.Lcd.printf("%d",sfDayCount);
    
  oldIntTempC=intTempC;
  oldslphPa=slphPa;
  oldIntHum=intHum;
  oldRain = rainGauge.rainMM;
  oldExtTemp = extTemp;
  oldExtRH = extRH;
  oldUVA = uva;
  oldUVB = uvb;
  oldUVI = uvi;
  oldUVIndex = uvIndex;
  oldVis = vis;
  oldSFDayCount = sfDayCount;
}

void getIntTempHumPressure(){
    intTempC = bme.readTemperature();
    presshPa = bme.readPressure() / 100.0F;
    slphPa = BME280_seaLevelPressure(presshPa, altitude);
    intHum = bme.readHumidity();
    //Serial.println(intTempC);
    //Serial.println(slphPa);
    //Serial.println(intHum);
}

/**
 * Applies compensation for altitude to absolute pressure value
 */
float BME280_seaLevelPressure(float press, float altitude_metres){
    float p = press / pow(1.0f - ((float)altitude_metres / PRESSURE_FACTOR1), PRESSURE_FACTOR2);
    return p + pressureCorrection; //Add on calibration offset
}

void startBME280(){
  Serial.print("Starting BME280... ");
  unsigned status = bme.begin(0x76, &Wire);
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      Serial.print("Program Halted");
      M5.Lcd.setTextColor(RED);
      M5.Lcd.printf("No BME280 - Program Halted!");
      while (1) delay(10);
  }  
  Serial.println("Done.");  
}

void processLoRaMessage(uint8_t *message, uint8_t length){
    Serial.print("Processing LoRa message of length ");
    Serial.println(length);
    if(message[0]==50){
        uint16_t crcResult = calculateCRC(message, length);
        Serial.print("CRC Check:");
        Serial.println(crcResult);
        if(crcResult == 0){
            Serial.println("CRC OK, processing further...");
            if(message[1]==0){
                if(message[2]==1){
                    processRainGaugeMessage(message);
                }
                else if(message[2]==2){
                    processUVLMessage(message);
                }
                else if(message[2]==3){
                    processTempHumMessage(message);
                }
                else if(message[2]==6){
                    processSFMessage(message);
                }
            }

        }
        else{
            Serial.println("CRC error - data rejected");
        }
    }
    else{
      Serial.println("Message not recognised");
    }
}

void dumpMessage(uint8_t *message, uint8_t length){
    for(uint8_t i=0;i<length;i++){
      Serial.print(message[i]);
      Serial.print(",");
    }
    Serial.println(" ");
}

/**
 * Calculates CRC16 for the number of characters in 
 * the message supplied from 0 to wLength
 */
uint16_t calculateCRC(uint8_t *nData, uint16_t wLength){
    uint16_t wCRCWord=0xFFFF;
    uint8_t nTemp;

    while (wLength--){
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
    }

    return wCRCWord;
}

void processSFMessage(uint8_t *message){
    log("SF message\r\n");
    uint8_t keyOK = keyMatch(message, sfKey);
    if(keyOK>0){
      Serial.println("SF key match");
      sfCount = 16777216*message[24] + 65536*message[25] + 256*message[26] + message[27];
      if(sfFirst==0){
        sfDayStartCount = sfCount;
        sfFirst=1;
      }
      Serial.print("SF:");
      Serial.println(sfCount);
      sfDayCount = sfCount - sfDayStartCount;

    }  
}



void processUVLMessage(uint8_t *message){
    log("UVL message\r\n");
    uint8_t keyOK = keyMatch(message, uvlKey);
    if(keyOK>0){
      Serial.println("UVL key match");
      uint16_t uvaRaw = message[24]*256 + message[25];
      uint16_t uvbRaw = message[26]*256 + message[27];
      uint16_t comp1Raw = message[28]*256 + message[29];
      uint16_t comp2Raw = message[30]*256 + message[31];
      uint16_t visRaw = message[32]*256 + message[33];
      float uvaCalc = (float)uvaRaw - (uvAco * uvAlpha * (float)comp1Raw/uvGamma - (uvBco * uvAlpha * (float)comp2Raw)/uvDelta);
      float uvbCalc = (float)uvbRaw - (uvCco * uvBeta * (float)comp1Raw/uvGamma - (uvDco * uvBeta * (float)comp2Raw)/uvDelta);
      if(uvaCalc<0){
        uvaCalc = 0;
      }
      if(uvbCalc<0){
        uvbCalc = 0;
      }
      uva = uvaCalc * (1.0/uvAlpha) * uvaResp;
      uvb = uvbCalc * (1.0/uvBeta) * uvbResp;
      uvi = (uva + uvb)/2.0;
      uvIndex = uvi/25;

      vis = visRaw * visLightScale;
      Serial.print("UVA:");
      Serial.println(uva);
      Serial.print("UVB:");
      Serial.println(uvb);
      Serial.print("UVI:");
      Serial.println(uvi);
      Serial.print("UVIndex:");
      Serial.println(uvIndex);
      Serial.print("VIS:");
      Serial.println(vis);
      uvlRx=1; //Set flag to show data received
      uvlTimerCount=0; //Reset timeout timer
    }
    else{
      Serial.println("UVL no key match");
    }
}

void processTempHumMessage(uint8_t *message){
    log("TRH message\r\n");
    uint8_t keyOK = keyMatch(message, trhKey);
    if(keyOK>0){
      Serial.println("TRH key match");
      uint16_t tempRaw = message[24] * 256 + message[25];
      uint16_t rhRaw = message[26] * 256 + message[27];
      float resistance = tempRaw * RCALC;
      extTemp = ((-CONSTANT1 + sqrt(CONSTANT2 + CONSTANT3 * (RTC0R - resistance)))/-CONSTANT4)+tempOffset;
      float Vatod = rhRaw * 2.048/32768.0;
      float Vsensor = Vatod * DIV_RATIO;
      float rh_uncompensated = (Vsensor/RH_SUPPLY_VOLTAGE - RHCONSTANT1) * RHCONSTANT2;
      extRH = rh_uncompensated/(RHC2 - RHM2 * extTemp);
      if(extRH>100){
         extRH=100;
      }
      if(extRH<0){
         extRH=0;
      }
      float alpha = log(extRH/100.0) + DEWPOINT_A * extTemp/(DEWPOINT_B + extTemp);
      dpC = (DEWPOINT_B * alpha)/(DEWPOINT_A-alpha);
      trhRx=1; //Set flag to show data received
      trhTimerCount=0; //Reset timeout timer
    }
    else{
      Serial.println("TRH no key match");
    }
}

/**
 * Checks if the key within the message matches the specified key
 * Returns 1 for match, 0 for no match
 */
uint8_t keyMatch(uint8_t *message, uint8_t *key){
    uint8_t returnMatch = 1;
    //All 8 bytes must match to avoid setting the match to 0.
    for(uint8_t i=0;i<8;i++){
        if(message[3+i]!=key[i]){
            returnMatch=0;
        }
    }
    return returnMatch;
}

void initialiseRainGauge(){
  rainGauge.rainCal = 0.33;
  rainGauge.rainDayStartTips = 0;
}

void processRainGaugeMessage(uint8_t *message){
    log("RAIN message\r\n");
    uint8_t keyOK = keyMatch(message, rainKey);
    if(keyOK>0){
      Serial.println("Rain gauge key match");
      rainGauge.tips = 16777216*message[24] + 65536*message[25] + 256*message[26] + message[27];
      if(rainFirst==0){
        rainGauge.rainDayStartTips = rainGauge.tips;
        rainFirst=1;
      }
      rainGauge.rainMM = (rainGauge.tips - rainGauge.rainDayStartTips) * rainGauge.rainCal;
      Serial.print("Rain: ");
      Serial.println(rainGauge.rainMM);
      rainRx=1; //Set flag to show data received
      rainTimerCount=0; //Reset timeout timer
    }
    else{
      Serial.println("Rain gauge no key match");
    }
}

/**
 * Logs messages to the SD card
 */
void log(const char *logMessage){
//  if(sdCardPresent>0){
//      File f = SD.open("/errorLog.txt",FILE_APPEND);
//      if(f){
//        if(!getLocalTime(&timeinfo)){
//          f.print("00/00/00 00:00:00  ");
//        }
//        else{
//          f.print(&timeinfo, "%A, %B, %d, %Y, %H:%M:%S  ");
//        }
//        f.print(logMessage);  
//        f.close();
//      }
//      else{
//        Serial.println("Error writing to SD card.");
//      }
//  }
}

/**
 * Go here once per minute to update timer counts for sensor responses.
 * Any that haven't responded for the timeout period will be set as RX=0 to prevent the data being used by other processes
 */
void checkAndUpdateSensorTimers(){
  trhTimerCount+=1;
  rainTimerCount+=1;
  uvlTimerCount+=1;
  if(trhTimerCount>sensorTimeout){
      trhRx=0;  
      trhTimerCount=sensorTimeout+1; //Prevent further counting with possible rollover to 0
  }
  if(uvlTimerCount>sensorTimeout){
      uvlRx=0;
      uvlTimerCount=sensorTimeout+1; //Prevent further counting with possible rollover to 0
  }
  if(rainTimerCount>sensorTimeout){
      rainRx=0;
      rainTimerCount=sensorTimeout+1; //Prevent further counting with possible rollover back to 0 (would be good again)
  }
//  Serial.println("Timeout counts:");
//  Serial.print("TRH:");
//  Serial.println(trhTimerCount);
//  Serial.print("RAIN:");
//  Serial.println(rainTimerCount);
//  Serial.print("UVL:");
//  Serial.println(uvlTimerCount);
}
