#pragma once
#include "util.hpp"

#include <HardwareSerial.h>

struct __attribute__((__packed__)) Packet {
	// Should be sorted in order of size in order to preserve alignment without
	// wasing space for padding.  The start of the sync sequence should also be
	// an invalid value for the first value listed so that receivers don't get
	// confused and interpret a valid packet as a sync word.
	uint32_t millis;
	float apogee;
	float kalman_alt, kalman_vel, kalman_acc;
	float raw_alt, raw_pressure;
	float raw_acc_x, raw_acc_y, raw_acc_z;
	float raw_gyro_x, raw_gyro_y, raw_gyro_z;
	float lat, lon;
	uint16_t temp;
	uint16_t batt_v, vref_v;
	FlightPhase phase;
	uint8_t checksum;

	Packet(
		uint32_t millis,
		float apogee,
		float kalman_alt, float kalman_vel, float kalman_acc,
		float raw_alt, float raw_pressure,
		float raw_acc_x, float raw_acc_y, float raw_acc_z,
		float raw_gyro_x, float raw_gyro_y, float raw_gyro_z,
		float lat, float lon,
		int16_t temp,
		uint16_t batt_v, uint16_t vref_v,
		FlightPhase phase) :
		millis(millis),
		apogee(apogee),
		kalman_alt(kalman_alt), kalman_vel(kalman_vel), kalman_acc(kalman_acc),
		raw_alt(raw_alt), raw_pressure(raw_pressure),
		raw_acc_x(raw_acc_x), raw_acc_y(raw_acc_y), raw_acc_z(raw_acc_z),
		raw_gyro_x(raw_gyro_x), raw_gyro_y(raw_gyro_y), raw_gyro_z(raw_gyro_z),
		lat(lat), lon(lon),
		temp(temp),
		batt_v(batt_v), vref_v(vref_v),
		phase(phase), checksum(0)
	{
		checksum = struct_checksum(*this);
	}

	size_t getLen() const { return sizeof(Packet); }
};

static_assert(sizeof(Packet) == 68, "Packet size changed.");
// static_assert(sizeof(Packet) <= SERIAL_TX_BUFFER_SIZE,
// 		"Packet should fit within serial TX buffer to minimize delay.");

void radio_setup();
void radio_send(const Packet &p);
