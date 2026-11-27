#include <gps.hpp>
void adafruit_gps::init()
{
    // Wait for GPS UART to start
  serial->begin(9600);
  // Set timeout to 0 so that (theoretically) we never block while waiting for Serial
  serial->setTimeout(0);

  // page 12 of https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
  // checksum generator https://nmeachecksum.eqth.net/
  // you can set a value from 0 (disable) to 5 (output once every 5 pos fixes)
  // 0  NMEA_SEN_GLL,  // GPGLL interval - Lat & long
  // 1  NMEA_SEN_RMC,  // GPRMC interval - Recommended Minimum Specific GNSS
  // 2  NMEA_SEN_VTG,  // GPVTG interval - Course over Ground and Ground Speed
  // 3  NMEA_SEN_GGA,  // GPGGA interval - GPS Fix Data
  // 4  NMEA_SEN_GSA,  // GPGSA interval - GNSS DOPS and Active Satellites
  // 5  NMEA_SEN_GSV,  // GPGSV interval - GNSS Satellites in View
  // 6-17           ,  // Reserved
  // 18 NMEA_SEN_MCHN, // PMTKCHN interval – GPS channel status
  serial->println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
  // Set update loop to 10hz
  serial->println("$PMTK220,100*2F");
}
void adafruit_gps::run()
{
    this->read();
    if (string_ready)
    {
        this->update();
        string_ready = false;
    }
}
void adafruit_gps::read()
{
    if (serial->available() && !string_ready)
    {
        char c = char(serial->read());
        gps_data_buf += c;
        if (c == '\n')
        {
            string_ready = true;
        }
    }
};
void adafruit_gps::update()
{
    
    // Get length of str
    int len = strlen(gps_data_buf.c_str());

    // Check that the incoming string is RMC
    if (!strstr(gps_data_buf.c_str(), "GPRMC"))
    {
        return;
    }

    // Replace commas with end character '\0' to seperate into single strings
    for (int j = 0; j < len; j++)
    {
        if (gps_data_buf[j] == ',' || gps_data_buf[j] == '*')
        {
            gps_data_buf[j] = '\0';
        }
    }

    // A lil working var
    int i = 0;

    // Go to string i and rip things
    // UTC time
    i += strlen(&gps_data_buf[i]) + 1;
    utc = atof(&gps_data_buf[i]);

    // Is data valid (A) or not (V)
    i += strlen(&gps_data_buf[i]) + 1;
    valid_data = gps_data_buf[i];

    // Raw lattitude in degrees
    i += strlen(&gps_data_buf[i]) + 1;
    latitude = atof(&gps_data_buf[i]);

    // North or South char
    i += strlen(&gps_data_buf[i]) + 1;
    north_or_south = gps_data_buf[i];

    // Raw longitude in degrees
    i += strlen(&gps_data_buf[i]) + 1;
    longitude = atof(&gps_data_buf[i]);

    // East or West char
    i += strlen(&gps_data_buf[i]) + 1;
    east_or_west = gps_data_buf[i];

    // spped
    i += strlen(&gps_data_buf[i]) + 1;
    velocity = atof(&gps_data_buf[i]);

    // Degrees true
    i += strlen(&gps_data_buf[i]) + 1;
    degrees_true = atof(&gps_data_buf[i]);

    // Date in ddmmyy
    i += strlen(&gps_data_buf[i]) + 1;
    date = gps_data_buf[i];

    // Degrees magnetic
    i += strlen(&gps_data_buf[i]) + 1;
    degrees_magnetic = atof(&gps_data_buf[i]);

    // East or West char
    i += strlen(&gps_data_buf[i]) + 1;
    east_or_west_degrees = gps_data_buf[i];

    // FAA mode
    i += strlen(&gps_data_buf[i]) + 1;
    mode = gps_data_buf[i];

    // A=autonomous, D=differential, E=Estimated,
    // M=Manual input mode N=not valid, S=Simulator, V = Valid
    i += strlen(&gps_data_buf[i]) + 1;
    status = gps_data_buf[i];

    this->decimal_coords.latitude = static_cast<int32_t>(degree_to_decimal(latitude, north_or_south)*10000000);
    this->decimal_coords.longitude = static_cast<int32_t>(degree_to_decimal(longitude, east_or_west)*10000000);

    // Reset buf for next line
    this->gps_data_buf = "";
};
