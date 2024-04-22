#ifdef NEO_GPS

#include "gps.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <SPI.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define GPS_SPI					SPI
#define GPS_LONG_FLOAT_DEGREES	10'000'000

struct gps_data{
	float latitudeDegrees;
	float longitudeDegrees;
	float altitude;
	float speed;
	float angle;
	uint16_t milliseconds;
	uint16_t year;
	uint8_t fix;
	uint8_t satellites;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t seconds;
} gps;

void gps_step();

static SFE_UBLOX_GNSS gps_device;

void gps_setup()
{
	if (gps_device.begin(GPS_SPI, PIN_NEOGPS_CS, 4'000'000, 10) == false) 
	{
		Serial.println(F("u-blox GNSS not detected on SPI bus. Please check wiring. Freezing."));
		abort();
	}

	gps_device.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX);	//Set the SPI port to output UBX only (turn off NMEA noise)
  	gps_device.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);	//Save (only) the communications port settings to flash and BBR
	gps_device.setNavigationFrequency(10);					// Set the frequency for gps to get new position

	// Query module for new positions every 25ms, should only get a response every 100ms
	scheduler_add(TaskId::Gps, Task(gps_step, 25'000, 300));
}

void gps_step()
{
	gps.latitudeDegrees = (float)gps_device.getLatitude() / GPS_LONG_FLOAT_DEGREES;
	gps.longitudeDegrees = (float)gps_device.getLongitude() / GPS_LONG_FLOAT_DEGREES;
	gps.altitude = (float)gps_device.getAltitude();
	gps.fix = gps_device.getFixType();
	gps.satellites = (float)gps_device.getSIV();
	// Ground speed in m/s
	gps.speed = (float)gps_device.getGroundSpeed() / 1'000;
	gps.angle = (float)gps_device.getHeading() / 100'000;

	// Times for printing and power save
	gps.year = gps_device.getYear();
	gps.month = gps_device.getMonth();
	gps.day = gps_device.getDay();
	gps.hour = gps_device.getHour();
	gps.minute = gps_device.getMinute();
	gps.seconds = gps_device.getSecond();
	gps.milliseconds = gps_device.getMillisecond();
}

float gps_get_lat() { return gps.latitudeDegrees; }
float gps_get_lon() { return gps.longitudeDegrees; }
float gps_get_alt() { return gps.altitude; }

void gps_print()
{
	Serial.print('[');
	Serial.print(gps.year);
	Serial.print('-');
	if (gps.month < 10) {
		Serial.print('0');
	}
	Serial.print(gps.month);
	Serial.print('-');
	if (gps.day < 10) {
		Serial.print('0');
	}
	Serial.print(gps.day);
	Serial.print('T');
	if (gps.hour < 10) {
		Serial.print('0');
	}
	Serial.print(gps.hour);
	Serial.print(':');
	if (gps.minute < 10) {
		Serial.print('0');
	}
	Serial.print(gps.minute);
	Serial.print(':');
	if (gps.seconds < 10) {
		Serial.print('0');
	}
	Serial.print(gps.seconds);
	Serial.print('.');
	if (gps.milliseconds < 100) {
		Serial.print('0');
	}
	if (gps.milliseconds < 10) {
		Serial.print('0');
	}
	Serial.print(gps.milliseconds);
	Serial.print(F("] "));

	if (gps.fix) {
		Serial.print(F("Fix("));
		Serial.print(gps.fix);
		Serial.print(F("): "));
		Serial.print(gps.latitudeDegrees, 7);
		Serial.print(F(", ")); 
		Serial.print(gps.longitudeDegrees, 7);

		Serial.print(F("; Speed: "));
		Serial.print(gps.speed);  // Convert knots to m/s
		Serial.print(F("m/s; Course: "));
		Serial.print(gps.angle);
		Serial.print(F("deg; Altitude: "));
		Serial.print(gps.altitude);
		Serial.print(F("m; Satellites: "));
		Serial.println((int)gps.satellites);
	} else {
		Serial.println(F("No fix."));
	}
}

#endif