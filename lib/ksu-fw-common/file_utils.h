#ifndef FILE_UTILS_H
#define FILE_UTILS_H
#include <Arduino.h>
#include <SD.h>
#include <TimeLib.h>

time_t getTeensy3Time()
{
    return Teensy3Clock.get();
}
// Take the current time and turn into a string for filename
String date_time(int time, String suffix = "")
{
    String minutes = "";
    if (minute(time) > 10)
    {
        minutes = String(minute(time));
    }
    else
    {
        minutes = "0" + String(minute(time));
    }
    String seconds = "";
    if (second(time) > 10)
    {
        seconds = String(second(time));
    }
    else
    {
        seconds = "0" + String(second(time));
    }
    String outString = suffix + "MDY_" + String(month(time)) + "-" + String(day(time)) +
                       "-" + String(year(time)) + "_HMS_" + String(hour(time)) +
                       "-" + minutes + "-" + seconds +
                       ".CSV";

    return outString;
}

void sd_date_time(uint16_t *date, uint16_t *time)
{
    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(year(), month(), day());
    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(hour(), minute(), second());
}

bool setupSD(SDClass *sdCard)
{
    return sdCard->begin(BUILTIN_SDCARD);
}

bool setupLogfile(File *newlog, SDClass *sdCard, String prefix = "")
{

    // Set up real-time clock
    // READ: Manual setting of the clock is not needed, if you are using the Teensy loader .exe, it will always update the RTC when you flash.
    //       Just make sure to observe that the RTC has updated correctly whenever you are handling it!
    // Teensy3Clock.set(1692325055); // set time (epoch) at powerup  (COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!)
    // Make name of current time
    Serial.println(prefix);
    const char *filename = date_time(Teensy3Clock.get(), prefix).c_str();
    if (sdCard->exists(filename))
    { // Print error if name is taken
        Serial.println("You generated a duplicate file name... Go check RTC.");
        return false;
    }
    else if (!sdCard->exists(filename))
    {
        Serial.println("Writing SD card File Name");
        Serial.println(filename);
        *newlog = sdCard->open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT); // Open file for writing
    }
    // TODO dont hard code writes to SD in here
    // // Set header of .csv

    // logger.print("Time,Steering,ShockFL,ShockFR,ShockRL,ShockRR,Accel-X,Accel-Y,Accel-Z,Heading,Pitch,Roll,Gyro-X,Gyro-Y,Gyro-Z\n");
    return true;
}

#endif