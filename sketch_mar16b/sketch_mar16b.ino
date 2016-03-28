
#include <SoftwareSerial.h>
//#include <FastSerial.h>
#include "Arduino.h"


#include <BetterStream.h>
#include <AP_Common.h>

#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

#define GPRMC_TERM "$GPRMC,"    //定义要解析的指令，因为这条指令包含定位和时间信息

char nmeaSentence[68];
String latitude;    //纬度
String longitude;   //经度
String lndSpeed;    //速度
String gpsTime;     //UTC时间，本初子午线经度0度的时间，和北京时间差8小时
String beiJingTime;   //北京时间

static boolean      mavbeat = 0;
static float        lastMAVBeat = 0;
static boolean      waitingMAVBeats = 1;
static uint8_t      apm_mav_type;
static uint8_t      apm_mav_system; 
static uint8_t      apm_mav_component;
static boolean      enable_mav_request = 0;



// software serial #1: TX = digital pin 10, RX = digital pin 11
SoftwareSerial portOne(10, 11);


void setup() {
    Serial.begin(9600);
    portOne.begin(9600);
  
    // setup mavlink port
    mavlink_comm_0_port = (BetterStream*)&Serial;

    
}

void loop() {
   //Serial.println("Hello World!");
   
  // put your main code here, to run repeatedly:
//mavlink_msg_set_position_target_local_ned_send(MAVLINK_COMM_0, 963497464, apm_mav_system, apm_mav_component,
  //    MAV_FRAME_LOCAL_NED, 4088/*0b0000111111111000*/, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

      /**
 * @brief Send a gps_raw_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */

mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, 963497464, 3, 900000000, 1400000000, 1000, 1, 1, 1, 1,255);

}


String getBeiJingTime(String s)
{
  int hour = s.substring(0,2).toInt();
  int minute = s.substring(2,4).toInt();
  int second = s.substring(4,6).toInt();

  hour += 8;

  if(hour > 24)
    hour -= 24;
  s = String(hour) + String(minute) + String(second);
  return s;
}

//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Latitude
String parseGprmcLat(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lat;
  /*make sure that we are parsing the GPRMC string. 
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 5; i++)
    {
      if(i < 3) 
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 3, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      if(i == 3)
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lat = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 3, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
      else
      {
        dEndLoc = s.indexOf(',', lEndLoc+1);
        lat = lat + " " + s.substring(lEndLoc+1, dEndLoc);
        /*Serial.print("i = 4, lEndLoc: ");
         Serial.println(lEndLoc);
         Serial.print("dEndLoc: ");
         Serial.println(dEndLoc);*/
      }
    }
    return lat; 
  }
  //}
  //}
}

//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Longitude
String parseGprmcLon(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lon;

  /*make sure that we are parsing the GPRMC string. 
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 7; i++)
    {
      if(i < 5) 
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 3, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      if(i == 5)
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lon = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 3, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
      else
      {
        dEndLoc = s.indexOf(',', lEndLoc+1);
        lon = lon + " " + s.substring(lEndLoc+1, dEndLoc);
        /*Serial.print("i = 4, lEndLoc: ");
         Serial.println(lEndLoc);
         Serial.print("dEndLoc: ");
         Serial.println(dEndLoc);*/
      }
    }
    return lon; 
  }
}

//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Longitude
String parseGprmcSpeed(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String lndSpeed;

  /*make sure that we are parsing the GPRMC string. 
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 8; i++)
    {
      if(i < 7) 
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 8, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      else
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        lndSpeed = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 8, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
    }
    return lndSpeed; 
  }
}


//Parse GPRMC NMEA sentence data from String
//String must be GPRMC or no data will be parsed
//Return Longitude
String parseGprmcTime(String s)
{
  int pLoc = 0; //paramater location pointer
  int lEndLoc = 0; //lat parameter end location
  int dEndLoc = 0; //direction parameter end location
  String gpsTime;

  /*make sure that we are parsing the GPRMC string. 
   Found that setting s.substring(0,5) == "GPRMC" caused a FALSE.
   There seemed to be a 0x0D and 0x00 character at the end. */
  if(s.substring(0,4) == "GPRM")
  {
    //Serial.println(s);
    for(int i = 0; i < 2; i++)
    {
      if(i < 1) 
      {
        pLoc = s.indexOf(',', pLoc+1);
        /*Serial.print("i < 8, pLoc: ");
         Serial.print(pLoc);
         Serial.print(", ");
         Serial.println(i);*/
      }
      else
      {
        lEndLoc = s.indexOf(',', pLoc+1);
        gpsTime = s.substring(pLoc+1, lEndLoc);
        /*Serial.print("i = 8, pLoc: ");
         Serial.println(pLoc);
         Serial.print("lEndLoc: ");
         Serial.println(lEndLoc);*/
      }
    }
    return gpsTime; 
  }
}

// Turn char[] array into String object
String charToString(char *c)
{

  String val = "";

  for(int i = 0; i <= sizeof(c); i++) 
  {
    val = val + c[i];
  }

  return val;
}

