#include "config.hpp"
#include "log.hpp"
#include "scheduler.hpp"
#include "util.hpp"
#include "gps.hpp"
#include "radio.hpp"

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
void deployment_step();

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

	while (!Serial) {}

	Serial.println(F("Flight Computer " __DATE__ " " __TIME__));

	gps_setup();
	radio_setup();

	Wire.begin();

	SPI.begin();

	// scheduler_add(TaskId::Command, Task(command_step, 100'000L, 10));
	scheduler_add(TaskId::Deployment, Task(deployment_step, 200'000L, 100));
	scheduler_add(TaskId::Print, Task(print_step, 3'000'000L, 3000));
	scheduler_add(TaskId::Blink, Task(blink_step, (KALMAN_PERIOD / 2) * 1000L, 20));
	
}

void deployment_step()
{
	static uint32_t land_time = 0;
	static FlightPhase phase = FlightPhase::Startup;
	static AvgHistory<float, EST_HISTORY_SAMPLES, 3> gravity_est_state;
	static AvgHistory<float, EST_HISTORY_SAMPLES, 3> ground_level_est_state;
	static kfloat_t apogee = 0;
	float accel[] = {1.0f, 2.0f, 3.0f};
	float raw_alt = 0;
	// Only send telemetry every other step to avoid saturating the link and
	// increasing latency.  This variable can be temporarily overridden to
	// immediately send important updates (e.g., launch and apogee detection).
	static bool send_now = true;
	uint32_t step_time = millis();

	if (std::isnan(raw_alt) || std::isnan(accel[0])) {
		// Wait until the next run, by which time we may have new data.
		return;
	}

	float accel_mag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

	if (phase < FlightPhase::Launched) {
		gravity_est_state.add(accel_mag);
		ground_level_est_state.add(raw_alt);
	}

	if (phase == FlightPhase::Startup) {
		if (!ground_level_est_state.full() ||
				!gravity_est_state.full()) {
			return;
		}
		
		phase = FlightPhase::Idle;
		// buzzer_ready();
	}

	accel_mag -= gravity_est_state.old_avg();
	float alt = raw_alt - ground_level_est_state.old_avg();

	uint32_t batt_v = analogRead(PIN_BATT_V);
	batt_v = map(batt_v, BATT_MIN_READING, BATT_MAX_READING, 0, BATT_MAX_VOLTAGE);

	uint32_t sys_v = 0;
	// uint32_t sys_v = analogRead(PIN_SYS_V);
	// sys_v = map(sys_v, SYS_MIN_READING, SYS_MAX_READING, 0, SYS_MAX_VOLTAGE);

	float gyro[] = {1.0f, 2.0f, 3.0f};


	if (send_now) {
		radio_send(Packet(step_time, apogee, kf.pos(), kf.rate(), kf.accel(),
				alt, 0.0f,
				accel[0], accel[1], accel[2],
				gyro[0], gyro[1], gyro[2],
				gps_get_lat(), gps_get_lon(),
				0, batt_v, 0.0f, phase));
	}

	send_now = !send_now;
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
