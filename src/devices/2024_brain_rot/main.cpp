#include "config.hpp"
#include "log.hpp"
#include "scheduler.hpp"
#include "util.hpp"
#include "gps.hpp"

#include <SPI.h>
#include <Wire.h>

#include <cmath>

#if defined (USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

// Prototypes
void blink_step();
void print_step();
void channel_step();
void channel_fire(Channel chan);
void test_pyrochannels();

static bool launched = false;

#ifdef KALMAN_GAINS
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f, {KALMAN_GAINS});
#else
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f,
		ALTITUDE_SIGMA, ACCELERATION_SIGMA, MODEL_SIGMA);
#endif

void setup()
{
	for (const ChannelConfig &c : channel_config) {
		pinMode(c.fire_pin, OUTPUT);
		digitalWrite(c.fire_pin, LOW);
	}

#ifdef PIN_LAUNCH
	pinMode(PIN_LAUNCH, OUTPUT);
	digitalWrite(PIN_LAUNCH, LOW);
#endif

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	pinMode(PIN_NEOGPS_CS, OUTPUT);
	digitalWrite(PIN_NEOGPS_CS, HIGH);

	pinMode(PIN_BATT_V, INPUT_ANALOG);
	// pinMode(PIN_SYS_V, INPUT_ANALOG);
	analogReadResolution(12);  // Enable full resolution
	analogWriteResolution(12);  // Enable full resolution

#if defined (USBCON) && defined(USBD_USE_CDC)
	usb_serial.begin();
#else
	Serial.begin(9'600);
#endif

	Serial.println(F("Flight Computer " __DATE__ " " __TIME__));

	Wire.begin();

	SPI.begin();

	// scheduler_add(TaskId::Command, Task(command_step, 100'000L, 10));
	scheduler_add(TaskId::Print, Task(print_step, 3'000'000L, 3000));
	scheduler_add(TaskId::Blink, Task(blink_step, (KALMAN_PERIOD / 2) * 1000L, 20));
	
}

void loop()
{
	uint32_t wait_time = schedule();
	if (wait_time > 4) {
		delayMicroseconds(wait_time - 4);
	}
}

void command_step()
{
	switch (Serial.read()) {
	case 'r':
		log_print_all();
		break;
	default:
		// Serial.println("Unrecognized command.");
		break;
	}

	// Radio control
	if (!launched) {
		switch (RADIO_SERIAL.read()) {
			case 'd':
				channel_fire(Channel::Drogue);
				break;
			case 'm':
				channel_fire(Channel::Main);
				break;
			default:
				break;
		}
	}
}

void blink_step()
{
	static bool on = false;
	digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
	on = !on;
}

void print_step()
{
	gps_print();
}
