#include "radio.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>

void radio_sync()
{
	RADIO_SERIAL.write("\xdb\x69\xc0\x78", 4);
}

void radio_send(const Packet &pkt)
{
	const byte *pkt_buf = reinterpret_cast<const byte *>(&pkt);
	RADIO_SERIAL.write(pkt_buf, pkt.getLen());
}

void radio_setup()
{
	RADIO_SERIAL.begin(115200);
	scheduler_add(TaskId::PacketSync, Task(radio_sync, 20 * KALMAN_PERIOD * 1000L, 20));
}
