#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <EEPROM.h>

#define GPSRxPin 4 //gps' tx connected to GPSRxpin
#define GPSTxPin 3 //gps' rx
#define XbeeTxPin 7 // xbee's tx
#define XbeeRxPin 8 // xbee's rx 
#define SD_Pin 10

#define LegMotor 6
#define DrillMotor1 5
#define DrillMotor2 9
#define LegsButton 2

#define BMP_HEIGHT 350 // these are heights from sea level
#define GPS_HEIGHT 350 // the height at the location of the launch was 170m


struct telemetry_data1{
  double longtitude;
  double latitude;
  double height;
  double pressure;
  double temperature;
  double check_sum;
};

struct telemetry_data2{
  double longtitude;
  double latitude;
  double UV;
  double SoilMoisture;
  double check_sum;
};

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;
String telemetry_package;


telemetry_data1 telemetryDataOnAir;
telemetry_data2 telemetryDataOnGround;

unsigned long int timeMark;
unsigned long int packetCount = 0;
int input_value;
int normalized_value;

int bmpBelowHeightCounter,gpsBelowHeightCounter;
int bmpAboveHeightCounter,gpsAboveHeightCounter;


int UVOut = A1; //analog output from the uv sensor
int hygrometer = A0; // analog output soil moisture
int REF_3V3 = A3; //3.3V power on the Funduino board
bool onGround;

SoftwareSerial GPSserial = SoftwareSerial(GPSRxPin,GPSTxPin);
SoftwareSerial XbeeSerial = SoftwareSerial(XbeeRxPin,XbeeTxPin);

Adafruit_BMP280 bmp;

void setup() {
  GPSserial.begin(38400);
  XbeeSerial.begin(9600);
  
  
  if (!bmp.begin()) {  
    XbeeSerial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
  
  //uv sensor
  pinMode(UVOut, INPUT);
  pinMode(REF_3V3, INPUT);

  //motors
  pinMode(LegMotor,OUTPUT);
  pinMode(DrillMotor1,OUTPUT);
  pinMode(DrillMotor2,OUTPUT);

  digitalWrite(LegMotor,LOW); 
  digitalWrite(DrillMotor1,LOW);
  digitalWrite(DrillMotor2,LOW);

  pinMode(LegsButton,INPUT);

  bmpBelowHeightCounter = 0;
  gpsBelowHeightCounter = 0;
  bmpAboveHeightCounter=0;
  gpsAboveHeightCounter=0;
  
  if(digitalRead(LegsButton)==HIGH){
    onGround=true;
  }
  else {
    onGround = false;
  }
  
}

void loop() {

   while(!onGround){

    timeMark = millis();
    
    processGPS();

// <PACKET_COUNT>,<STATUS>,<LATITUDE>,<LONGTITUDE>,<ALTITUDE>,<PRESSURE>,<TEMPERATURE>,<CHECK_SUM> : STATUS=1 for on-flight , 2 for ground operation
    telemetryDataOnAir.latitude = (double) posllh.lat/10000000;
    telemetryDataOnAir.longtitude = (double) posllh.lon/10000000;
    telemetryDataOnAir.height = bmp.readAltitude(1013.25);//(double) posllh.height/1000.0;
    telemetryDataOnAir.pressure = bmp.readPressure();
    telemetryDataOnAir.temperature = bmp.readTemperature();

    packetCount++;
    
    telemetryDataOnAir.check_sum = packetCount
    + 1
    + telemetryDataOnAir.latitude 
    + telemetryDataOnAir.longtitude 
    + telemetryDataOnAir.height 
    + telemetryDataOnAir.pressure 
    + telemetryDataOnAir.temperature; 

    
    telemetry_package= String(packetCount) + "," 
    + String(1) + "," 
    + String(telemetryDataOnAir.latitude) + "," 
    + String(telemetryDataOnAir.longtitude) + "," 
    + String(telemetryDataOnAir.height) + "," 
    + String(telemetryDataOnAir.pressure) + "," 
    + String(telemetryDataOnAir.temperature) + "," 
    + String(telemetryDataOnAir.check_sum); 

    XbeeSerial.println(telemetry_package);
   
    if(telemetryDataOnAir.height <= BMP_HEIGHT ){
      bmpBelowHeightCounter++;
      bmpAboveHeightCounter=0;
    }
    else if (telemetryDataOnAir.height < 3000){
      bmpBelowHeightCounter = 0;
      bmpAboveHeightCounter++;
    }

    double gps_height = (double) posllh.height/1000.0;
    
    if(gps_height <= GPS_HEIGHT && gps_height!=0){
      gpsBelowHeightCounter++;
      gpsAboveHeightCounter=0;
    }
    else if(gps_height!=0) {
      gpsBelowHeightCounter = 0;
      gpsAboveHeightCounter++;
    }

    File dataFile = SD.open("datalog.txt" , FILE_WRITE );
    
    if(dataFile){
      dataFile.println(telemetry_package);
      dataFile.close();
    }

    if( didWeLaunch() ){
      EEPROM.write(0,73);
    }
    
    if(didWeLand() && EEPROM.read(0)==73){
      onGround=true;
    }
    
    delay(1000-(millis()-timeMark)); // processing power wasted

  }

  while(onGround){

    if( digitalRead(LegsButton) == LOW ){
      digitalWrite(LegMotor,HIGH);
      while(digitalRead(LegsButton)== LOW);
      digitalWrite(LegMotor,LOW);


      digitalWrite(DrillMotor1,HIGH);
      delay(30000);
      digitalWrite(DrillMotor1,LOW);
    
      digitalWrite(DrillMotor2,HIGH);
      delay(30000);
      digitalWrite(DrillMotor2,LOW);
      
    }

    while(73){
      timeMark = millis();
      processGPS();
      
      int uvlevel = averageAnalogRead(UVOut);
      int refLevel = averageAnalogRead(REF_3V3);
      float outputVoltage = 3.3 / refLevel * uvlevel;
      float uvIntensity = mapfloat(outputVoltage, 0.96, 2.9, 0.0, 15.0);

      input_value = analogRead(hygrometer);
      normalized_value = map(input_value, 0, 800, 0, 100);
      if (normalized_value > 800){
        normalized_value = 800;
      }
       
      packetCount++;

      telemetryDataOnGround.latitude = (double) posllh.lat/10000000;
      telemetryDataOnGround.longtitude = (double) posllh.lon/10000000;
      telemetryDataOnGround.UV = uvIntensity;
      telemetryDataOnGround.SoilMoisture = normalized_value;

      telemetryDataOnGround.check_sum = packetCount
      + 2
      + telemetryDataOnGround.latitude
      + telemetryDataOnGround.longtitude
      + telemetryDataOnGround.UV
      + telemetryDataOnGround.SoilMoisture;

      //<PACKET_COUNT>,2,<LATITUDE>,<LONGTITUDE>,<UV_RADIATION>,<SOIL MOISTURE>,<CHK_SUM>
      telemetry_package= String(packetCount) + "," 
      + String(2) + "," 
      + String(telemetryDataOnGround.latitude) + "," 
      + String(telemetryDataOnGround.longtitude) + "," 
      + String(telemetryDataOnGround.UV) + ","
      + String(telemetryDataOnGround.SoilMoisture) + ","
      + String(telemetryDataOnGround.check_sum); 

      XbeeSerial.println(telemetry_package);

      
      File dataFile = SD.open("datalog.txt" , FILE_WRITE );
    
      if(dataFile){
       dataFile.println(telemetry_package);
       dataFile.close();
      }
      delay(1000-(millis()-timeMark));
    }
  }

}

bool didWeLaunch(){
  return ( (bmpAboveHeightCounter>5 && gpsAboveHeightCounter>5 ) || (bmpAboveHeightCounter>10) || (gpsAboveHeightCounter>10) );
}

bool didWeLand(){
  return ( ( bmpBelowHeightCounter>=60 && gpsBelowHeightCounter>=60 ) || ( bmpBelowHeightCounter>=100 ) || ( gpsBelowHeightCounter>=100 ) );
}

//////////-Functions for UV-//////////
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//////////

//////////- Functions for GPS data parsing-//////////
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( GPSserial.available() ) {
    byte c = GPSserial.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}
//////////
