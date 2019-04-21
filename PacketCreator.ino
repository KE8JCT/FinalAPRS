

// Arduino APRS Tracker (aat) with Arduino Pro Mini 3.3V/8 MHz
// Based on https://github.com/sh123/aprs_tracker
// 
#include <SoftwareSerial.h>
//#include <SimpleTimer.h>
#include <TinyGPS.h>
#include "PacketTX.h"

// GPS SoftwareSerial
// Shares pins with (MISO 12/ MOSI 11) used for SPI
#define GPS_RX_PIN 6//4//possibly 4 or 6
#define GPS_TX_PIN 4//6

// APRS settings
String APRSPacket = "";
String APRS_CALLSIGN="KE8JCT";
String wide1 ="WIDE1";
String wide2="WIDE2";
String APRS_SSID="10";
String APRS_SYMBOL="="; //pointer & print buffer ax25

// Timer
//#define TIMER_DISABLED -1

TinyGPS gps;
SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);
//SimpleTimer timer;

//long instead of float for latitude and longitude
long lat = 0;
long lon = 0;

int year=0;
byte month=0, day=0, hour=0, minute=0, second=0, hundredths=0;
unsigned long age=0;

// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

void setup()  
{
  Serial.begin(115200);
  GPSSerial.begin(4800);
  
  Serial.println("Arduino APRS Tracker");

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);

  //APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  //APRS_setCallsign(APRS_CALLSIGN,APRS_SSID);
  //APRS_setSymbol(APRS_SYMBOL);
  //APRS_setPath1(wide1, 1);
  //APRS_setPath2(wide2, 2);

}

void loop()
{
  bool newData = false;

  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPSSerial.available())
    {
      char c = GPSSerial.read();
       //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  Serial.println("Checking for new data");
  if (newData)
  {
    Serial.println("New data");
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &age);
    gps.get_position(&lat, &lon, &age);

    //Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(year);
    //Serial.print(" "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));Serial.print(F(" "));
   
    //Serial.println("Time: " + time1);
  
    //Serial.print(F("LAT="));Serial.print(lat);
    Serial.print(F(" LON="));Serial.print(lon);

    //Serial.print(F(" "));
    //Serial.print(deg_to_nmea(lat, true));
    //Serial.print(F("/"));

    //Serial.println(deg_to_nmea(lon, false));
  }
  locationUpdate();
  delay(10000);
}

void locationUpdate() {
  //char comment []= "Honk if you love pointers";
  String time1 = String(day)+String(static_cast<int>(hour))+String(static_cast<int>(minute))+"z";
  String latstring = (String)deg_to_nmea(lat, true);
  String lonstring = (String)deg_to_nmea(lon, false);
//  Serial.println(time1+latstring+lonstring);
//  Serial.println(time1);
//  Serial.println(latstring);
//  Serial.println(lonstring);
  //APRSPacket = "KE8JCT-10/APZMDM,WIDE1-1:/" + time1 + lonstring + "/" + latstring + "C";
  //String APRSPacket = "KE8JCT-10/APZMDM,WIDE1-1:/212158z07407.30W/4126.23NC";
  String APRSPacket = "TEST";
  // turn off SoftSerial to stop interrupting tx
  delay(50);
  GPSSerial.end();
  
  // TX
  //APRS_printSettings();
  Serial.println("Sending Data");
  //APRS_sendLoc(comment, strlen(comment));
  //Serial.println(msg);
  Serial.println(APRSPacket);
  Serial.println("Data Sent");
  // read TX LED pin and wait till TX has finished (PB5) digital write 13 LED_BUILTIN
  //while(bitRead(PORTB,5));

  // start SoftSerial again
  GPSSerial.begin(4800); //changed from 9600
}

/*
**  Convert degrees in long format to APRS string format
**  DDMM.hhN for latitude and DDDMM.hhW for longitude
**  D is degrees, M is minutes and h is hundredths of minutes.
**  http://www.aprs.net/vm/DOS/PROTOCOL.HTM
*/
char* deg_to_nmea(long deg, boolean is_lat) {
  bool is_negative=0;
  if (deg < 0) is_negative=1;

  // Use the absolute number for calculation and update the buffer at the end
  deg = labs(deg);

  unsigned long b = (deg % 1000000UL) * 60UL;
  unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
  b = (b % 1000000UL) / 10000UL;

  conv_buf[0] = '0';
  // in case latitude is a 3 digit number (degrees in long format)
  if( a > 9999) { snprintf(conv_buf , 6, "%04u", a);} else snprintf(conv_buf + 1, 5, "%04u", a);

  conv_buf[5] = '.';
  snprintf(conv_buf + 6, 3, "%02u", b);
  conv_buf[9] = '\0';
  if (is_lat) {
    if (is_negative) {conv_buf[8]='S';}
    else conv_buf[8]='N';
    return conv_buf+1;
    // conv_buf +1 because we want to omit the leading zero
    }
  else {
    if (is_negative) {conv_buf[8]='W';}
    else conv_buf[8]='E';
    return conv_buf;
    }
}
