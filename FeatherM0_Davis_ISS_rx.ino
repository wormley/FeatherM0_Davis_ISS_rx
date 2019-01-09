// Modified by Jason Foss in 2019 to simplify for use only with Adafruit Feather M0 Radio in Rx Mode.


#include <Arduino.h>
#include <SPI.h>

#define DAVISRFM69_DEBUG

#include "DavisRFM69.h"
#include "PacketFifo.h"

#define LED 13
#define SERIAL_BAUD 19200

DavisRFM69 radio(8, 3, true, 3);

// id, type, active
Station stations[1] = {
  {.id = 0,
	.type = STYPE_ISS,
	.active = true}
	};

WxData curWx;

void setup() {
	Serial.begin(SERIAL_BAUD);
	delay(1000);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);

	radio.setStations(stations, 1);
	radio.initialize(FREQ_BAND_US);
	//radio.setBandwidth(RF69_DAVIS_BW_NARROW);
	radio.setBandwidth(RF69_DAVIS_BW_WIDE);

	Serial.println("Boot complete!");
	}


void loop() {
	if (radio.fifo.hasElements()) {
		decode_packet(radio.fifo.dequeue());
		}

	if (radio.mode == SM_RECEIVING) {
		digitalWrite(LED, HIGH);
		}
	else if (radio.mode == SM_SEARCHING) {
		Blink(LED, 15);
		delay(100);
		}
	else {
		digitalWrite(LED, LOW);
		}
	radio.loop();
	}

void Blink(byte PIN, int DELAY_MS) {
	pinMode(PIN, OUTPUT);
	digitalWrite(PIN, HIGH);
	delay(DELAY_MS);
	digitalWrite(PIN, LOW);
	}

void print_value(char* vname, char* value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void print_value(char* vname, int value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void print_value(char* vname, float value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value, 1); Serial.print(sep);
	}

void print_value(char* vname, long value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void print_value(char* vname, uint32_t value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void decode_packet(RadioData* rd) {

  // for more about the protocol see:
  // https://github.com/dekay/DavisRFM69/wiki/Message-Protocol
	int val;
	float fval;
	byte* packet = rd->packet;

#ifdef DAVISRFM69_DEBUG
	Serial.print(F("raw:"));
	printHex(packet, 10);
	Serial.print(F(", "));
	print_value("station", packet[0] & 0x7, F(", "));

	Serial.print(F("packets:"));
	Serial.print(radio.packets);
	Serial.print('/');
	Serial.print(radio.lostPackets);
	Serial.print('/');
	Serial.print((float) (radio.packets * 100.0 / (radio.packets + radio.lostPackets)));
	Serial.print(F(", "));

	print_value("channel", rd->channel, F(", "));
	print_value("rssi", -rd->rssi, F(", "));

	print_value("batt", (char*) (packet[0] & 0x8 ? "err" : "ok"), F(", "));
#endif

	// All packet payload values are printed unconditionally, either properly
	// calculated or flagged with a special "missing sensor" value, mostly -1.
	// It's the CPE's responsibility to interpret our output accordingly.

	byte id = radio.DATA[0] & 7;
	int stIx = radio.findStation(id);

	val = (packet[1]);
	if (val >= 7) val++;							// Recieved wind of 7 and up to at least 27 is increased by 1 on the console though  JF
	curWx.windv = val;								// I have observed a wind speed of 7 on the console so this is not exactly accurate
   	curWx.winddraw = packet[2];

	// wind data is present in every packet, windd == 0 (packet[2] == 0) means there's no anemometer
	if (packet[2] != 0) {
		if (stations[stIx].type == STYPE_VUE) {
			val = (packet[2] << 1) | (packet[4] & 2) >> 1;
			val = round(val * 360 / 512);
			}
		else {
			val = 9 + round((packet[2] - 1) * (342.0 / 255.0));
			if (val > 345) val = ((val - 345) * 2) + 345;			// Smooths out some of the dead zone around N similiar to console   JF
			}
		}
	else {
		val = 0;
		}
	curWx.windd = val;

#ifdef DAVISRFM69_DEBUG
	print_value("windv", val, F(", "));
	print_value("winddraw", packet[2], F(", "));
	print_value("windd", val, F(", "));
#endif

	switch (packet[0] >> 4) {

			case VP2P_UV:
				val = word(packet[3], packet[4]) >> 6;
				if (val < 0x3ff) fval = (float) (val / 50.0);
				else fval - 1;

				curWx.uv = fval;
#ifdef DAVISRFM69_DEBUG
				print_value("uv", fval, F(", "));
#endif
				break;

			case VP2P_SOLAR:
				val = word(packet[3], packet[4]) >> 6;
				if (val < 0x3fe) fval = (float) (val * 1.757936);
				else fval - 1;
				
				curWx.solar = fval;
#ifdef DAVISRFM69_DEBUG
				print_value("solar", fval, F(", "));
#endif
				break;

			case VP2P_RAIN:
				if (packet[3] == 0x80) val - 1;
				else val = packet[3];
				
				curWx.rain = val;
#ifdef DAVISRFM69_DEBUG
				print_value("rain", val, F(", "));
#endif
				break;

			case VP2P_RAINSECS:
			  // light rain:  byte4[5:4] as value[9:8] and byte3[7:0] as value[7:0] - 10 bits total
			  // strong rain: byte4[5:4] as value[5:4] and byte3[7:4] as value[3:0] - 6 bits total


//				if ((packet[4] && 0x40) == 0) {			// light rain
	//				fval = ((packet[4] && 0x30) / 16 * 250) + packet[3];
//					}
//				else if ((packet[4] && 0x40) == 0x40) {			// strong rain
//					fval = 11520 / (((packet[4] && 0x30) / 16 * 250) + packet[3]);
//					}
//				else {
//					fval = -1;
//					}
				


				val = (packet[4] & 0x30) << 4 | packet[3];
				if (val == 0x3ff) val = -1;
				else if ((packet[4] & 0x40) == 0) val >>= 4; // packet[4] bit 6: strong == 0, light == 1
				



				curWx.rainrate = val;
#ifdef DAVISRFM69_DEBUG
				print_value("rainsecs", val, F(", "));
#endif
				break;

			case VP2P_TEMP:
			  //if (packet[3] == 0xff) {
		//        print_value("temp", -100, F(", "));
			  //} else {
				{
				  //val = (int)packet[3] << 4 | packet[4] >> 4;
				  //val = (packet[3]* 256 + packet[4]) / 160;
				val = ((int16_t) ((packet[3] << 8) | packet[4])) / 16;
				fval = (float) (val / 10.0);
				curWx.temp = fval;
				}
#ifdef DAVISRFM69_DEBUG
				print_value("temp", fval, F(", "));
#endif
				break;

			case VP2P_HUMIDITY:
				fval = ((packet[4] >> 4) << 8 | packet[3]) / 10.0; // 0 -> no sensor
				curWx.rh = fval;
#ifdef DAVISRFM69_DEBUG
				print_value("rh", val, F(", "));
#endif
				break;

			case VP2P_WINDGUST:
				val = packet[3];
				if (val >= 7) val++;										// Recieved wind of 7+ to at least 27 is increased by 1 on the console   JF
				curWx.windgust = val;
				val = packet[5] & 0xf0 >> 4;
				curWx.windgustd = val;										// Gust direction (16 Rose directions)   JF
#ifdef DAVISRFM69_DEBUG
				print_value("windgust", val, F(", "));
				print_value("gustd", val, F(", "));	
#endif
				break;

			case VP2P_SOIL_LEAF:
			  // currently not processed but algorithm is known
			  // see https://github.com/matthewwall/weewx-meteostick/blob/master/bin/user/meteostick.py
				curWx.soilleaf = -1;
#ifdef DAVISRFM69_DEBUG
				print_value("soilleaf", -1, F(", "));
#endif
				break;

			case VUEP_VCAP:
				val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
				fval = (float) (val / 100.0);
				curWx.vcap = fval;
#ifdef DAVISRFM69_DEBUG
				print_value("vcap", fval, F(", "));
#endif
				break;

			case VUEP_VSOLAR:
				val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
				fval = (float) (val / 100.0);
				curWx.vsolar = fval;
#ifdef DAVISRFM69_DEBUG
				print_value("vsolar", fval, F(", "));
#endif
		}

	int diff = rd->delta - stations[packet[0] & 0x7].interval;   // Added by JF

#ifdef DAVISRFM69_DEBUG
	print_value("fei", round(rd->fei * 61.03515625 / 1000), F(", "));
	print_value("delta", rd->delta, F(", "));
	print_value("diff", diff, F(""));
	Serial.println();
#endif
	if (rd->delta >= 1 && diff <= TUNEIN_USEC) {														// Accounts for any slight timing
		stations[packet[0] & 0x7].interval = stations[packet[0] & 0x7].interval  + (diff/2);	// drift by either Tx or Rx    JF
		}

	Serial.print(RecieverID);
	Serial.print(",");
	Serial.print(radio.packets + radio.lostPackets);
	Serial.print(",");
	Serial.print((float) (radio.packets * 100.0 / (radio.packets + radio.lostPackets)));
	Serial.print(",");
	Serial.print(-rd->rssi);
	Serial.print(",");
	Serial.print((char*) (packet[0] & 0x8 ? "err" : "ok"));
	Serial.print(",");
	Serial.print(curWx.rain);
	Serial.print(",");
	Serial.print(curWx.rainrate);
	Serial.print(",");
	Serial.print(curWx.rh);
	Serial.print(",");
	Serial.print(curWx.soilleaf);
	Serial.print(",");
	Serial.print(curWx.solar);
	Serial.print(",");
	Serial.print(curWx.temp);
	Serial.print(",");
	Serial.print(curWx.uv);
	Serial.print(",");
	Serial.print(curWx.vcap);
	Serial.print(",");
	Serial.print(curWx.vsolar);
	Serial.print(",");
	Serial.print(curWx.windd);
	Serial.print(",");
	Serial.print(curWx.winddraw);
	Serial.print(",");
	Serial.print(curWx.windgust);
	Serial.print(",");
	Serial.print(curWx.windgustd);
	Serial.print(",");
	Serial.print(curWx.windv);

	Serial.println();
	}

void printHex(volatile byte* packet, byte len) {
	for (byte i = 0; i < len; i++) {
		if (!(packet[i] & 0xf0)) Serial.print('0');
		Serial.print(packet[i], HEX);
		if (i < len - 1) Serial.print('-');
		}
	}
