#ifndef GPS_H
#define GPS_H

// All the GPS bullshit rolled up into a header file so that it isn't an eyesore
// when not in use, not that we should ever use it lmao

#include <Arduino.h>

// // More functions
// void init_GPS();
// String parse_rmc(String msg);                  // Parse RMC string function
// String parse_gga(String msg);                  // Prase GGA string im tired
// float degree_to_decimal(float num, byte sign); // GPS conversion function

// Some global vars
// String gps_id = "";           // Whatever ID you want for your fake can msg
// String input_serial4 = "";    // a string to hold incoming data
// boolean ready_serial4 = true; // whether the string is complete

struct decimal_coords_t
{
  int32_t latitude;
  int32_t longitude;
};
class adafruit_gps
{
public:
  adafruit_gps(HardwareSerial &gpsSerial) : serial(&gpsSerial) {}
  HardwareSerial *serial;
  void init();
  void read();
  void update();
  void run();
  decimal_coords_t get_coords()
  {
    return decimal_coords;
  }
private:
// Want to convert DDMM.MMMM to a decimal number DD.DDDDD? Slap it into this.
float degree_to_decimal(float num, byte sign)
{

  int intpart = (int)num;
  float decpart = num - intpart;

  int degree = (int)(intpart / 100);
  int mins = (int)(intpart % 100);

  if (sign == 'N' || sign == 'E')
  {
    // Return positive degree
    return (degree + (mins + decpart) / 60);
  }
  else
  {
    // Return negative degree
    return -(degree + (mins + decpart) / 60);
  }
}
  bool string_ready = false;
  String gps_data_buf = "";
  float utc;
  char valid_data;
  float latitude;
  char north_or_south;
  float longitude;
  char east_or_west;
  float velocity;
  float degrees_true;
  char date;
  float degrees_magnetic;
  char east_or_west_degrees;
  char mode;
  char status;
  decimal_coords_t decimal_coords;
};




#endif
