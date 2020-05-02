// Modified by Jason Foss in 2019 to simplify for use only with Adafruit Feather M0 Radio in Rx Mode.

#include <Arduino.h>
#include <SPI.h>

#include "DavisRFM69.h"
#include "RFM69registers.h"
#include <Wire.h>
#include <SparkFunBME280.h>
#include <math.h>

#define BME_DEBUG

BME280 mySensor;
boolean bme_valid = false;
DavisRFM69 radio(SPI_CS,RF69_IRQ_PIN,RF69_IRQ_NUM);
unsigned long time=0;
int bme_loop = 0;
float temperature=NAN;
float pressure=NAN;
float humidity=NAN;


#define BME_DELAY 2000

Station stations[2] = { 
	{ .id = 0, .type = ISS_TYPE, .active = true },
	{ .id = 2, .type = ISS_TYPE, .active = true }
 }; 

WxData curWx;

//  Report the amount of memory between the heap and the stack. Call freeMemory() to get the amount at that point.
//  From: https://github.com/mpflaga/Arduino-MemoryFree				 Added by JF

#ifdef DAVISRFM69_DEBUG
extern "C" char* sbrk(int incr);

int freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}
#endif

void bme_setup() {
	  mySensor.setFilter(2); //0 to 4 is valid. Filter coefficient. See 3.4.4
  mySensor.setStandbyTime(4); //0 to 7 valid. Time between readings. See table 27.

  mySensor.setTempOverSample(4); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  mySensor.setPressureOverSample(4); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  mySensor.setHumidityOverSample(4); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  
  mySensor.setMode(MODE_NORMAL); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
}


void setup() {
	Serial.begin(SERIAL_BAUD);
// Give us time to get to the serial port
	delay(5000);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);

	DavisRFM69::stations = stations;
	DavisRFM69::numStations = sizeof(stations)/sizeof(Station);

	radio.initialize(FREQ_BAND_US);
	//radio.setBandwidth(RF69_DAVIS_BW_NARROW);
	radio.setBandwidth(RF69_DAVIS_BW_WIDE);
//	mySensor.setI2CAddress(0x76);
//  if (mySensor.beginI2C() == false)  {
//        Serial.println("Could not find a valid BME280 sensor, check wiring!");
//  } else {
//	  bme_valid = true;
//	bme_setup();
//  }
 

	Serial.println("Boot complete!");
	}

void Blink(byte PIN, int DELAY_MS) {
	pinMode(PIN, OUTPUT);
	digitalWrite(PIN, HIGH);
	delay(DELAY_MS);
	digitalWrite(PIN, LOW);
	}
boolean led_state=false;
void Toggle_LED() {
	led_state = !led_state;
	pinMode(LED, OUTPUT);
	digitalWrite(LED, led_state);
	}


void print_value(const char* vname, char* value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void print_value(const char* vname, int value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void print_value(const char* vname, float value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value, 1); Serial.print(sep);
	}

void print_value(const char* vname, long value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void print_value(const char* vname, uint32_t value, const __FlashStringHelper* sep) {
	Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
	}

void decode_packet() {
	radio.qLen--;
	RadioData* rd = &radio.packetFifo[radio.packetOut];
	byte* packet = rd->packet;
	if (++radio.packetOut == FIFO_SIZE) radio.packetOut = 0;


  // for more about the protocol see:
  // https://github.com/dekay/DavisRFM69/wiki/Message-Protocol

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

	curWx.windv = (packet[1]);
	if (curWx.windv >= 7) curWx.windv++;				// Recieved wind of 7 and up to at least 28 is increased by 1 on the console though  JF
   	curWx.winddraw = packet[2];							// The console occasionaly shows a speed of 7 with a raw of 6, somewhat random?

	// wind data is present in every packet, windd == 0 (packet[2] == 0) means there's no anemometer
	if (packet[2] != 0) {
		if (stations[packet[0] & 0x7].type == STYPE_VUE) curWx.windd = round(((packet[2] << 1) | (packet[4] & 2) >> 1) * 0.703125);
		else {
			curWx.windd = 9 + round((packet[2] - 1) * 1.341176);
			if (curWx.windd > 345) curWx.windd = ((curWx.windd - 345) * 2) + 345;			// Smooths out some of the dead zone around N similiar to console   JF
			}
		}
	else {
		curWx.windd = 0;
		}

#ifdef DAVISRFM69_DEBUG
	print_value("windv", curWx.windv, F(", "));
	print_value("winddraw", packet[2], F(", "));
	print_value("windd", curWx.windd, F(", "));
#endif

	switch (packet[0] >> 4) {

			case VP2P_UV:
				curWx.uv = word(packet[3], packet[4]) >> 6;
				if (curWx.uv < 0x3ff) curWx.uv = (curWx.uv / 50.0);
				else curWx.uv =  -1;

#ifdef DAVISRFM69_DEBUG
				print_value("uv", curWx.uv, F(", "));
#endif
				break;

			case VP2P_SOLAR:
				curWx.solar = word(packet[3], packet[4]) >> 6;
				if (curWx.solar < 0x3fe) curWx.solar = (curWx.solar * 1.757936);
				else curWx.solar = -1;
				
#ifdef DAVISRFM69_DEBUG
				print_value("solar", curWx.solar, F(", "));
#endif
				break;

			case VP2P_RAIN:
				if (packet[3] == 0x80) curWx.rain = -1;
				else curWx.rain = packet[3];
				
#ifdef DAVISRFM69_DEBUG
				print_value("rain", curWx.rain, F(", "));
#endif
				break;

			case VP2P_RAINSECS:
				// Seconds between last 2 tips or since last tip, whichever is greater
				// light rain:  byte4[5:4] as value[9:8] and byte3[7:0] as value[7:0] - 10 bits total
				// strong rain: byte4[5:4] as value[5:4] and byte3[7:4] as value[3:0] - 6 bits total
				// packet[4] bit 6: strong == 0, light == 1

				curWx.rainrate = (packet[4] & 0x30) << 4 | packet[3];
				if (curWx.rainrate == 0x3ff) curWx.rainrate = 0;
				else if ((packet[4] & 0x40) == 0) curWx.rainrate >>= 4;
				curWx.rainrate = round(3600.0 / curWx.rainrate); // Equals 0.01"/hr
				
#ifdef DAVISRFM69_DEBUG
				print_value("rainsecs", curWx.rainrate, F(", "));
#endif
				break;

			case VP2P_TEMP:
				curWx.temp = (((int16_t) ((packet[3] << 8) | packet[4])) / 16);
				
#ifdef DAVISRFM69_DEBUG
				print_value("temp", curWx.temp, F(", "));
#endif
				break;

			case VP2P_HUMIDITY:
				curWx.rh = ((packet[4] >> 4) << 8 | packet[3]); // 1/10ths

#ifdef DAVISRFM69_DEBUG
				print_value("rh", curWx.rh, F(", "));
#endif
				break;

			case VP2P_WINDGUST:
				curWx.windgust = packet[3];
				if (curWx.windgust >= 7) curWx.windgust++;			// Recieved wind of 7+ to at least 28 is increased by 1 on the console   JF
				curWx.windgustd = packet[5] & 0xf0 >> 4;			// Gust direction (16 Rose directions)   JF
								
#ifdef DAVISRFM69_DEBUG
				print_value("windgust", curWx.windgust, F(", "));
				print_value("gustd", curWx.windgustd, F(", "));
#endif
				break;

			case VP2P_SOIL_LEAF:
			  // currently not processed but algorithm is known
			  // see https://github.com/matthewwall/weewx-meteostick/blob/master/bin/user/meteostick.py
				curWx.soilleaf = -1;
#ifdef DAVISRFM69_DEBUG
				print_value("soilleaf", curWx.soilleaf, F(", "));
#endif
				break;

			case VUEP_VCAP:
				curWx.vcap = ((packet[3] << 2) | (packet[4] & 0xc0) >> 6); // 1/100ths

#ifdef DAVISRFM69_DEBUG
				print_value("vcap", curWx.vcap, F(", "));
#endif
				break;

			case VUEP_VSOLAR:
				curWx.vsolar = ((packet[3] << 2) | (packet[4] & 0xc0) >> 6); // 1/100ths

#ifdef DAVISRFM69_DEBUG
				print_value("vsolar", curWx.vsolar, F(", "));
#endif
		}


#ifdef DAVISRFM69_DEBUG
	int diff = rd->delta - stations[packet[0] & 0x7].interval;					// Added by JF
	print_value("fei", round(rd->fei * 61.03515625 / 1000), F(", "));
	print_value("delta", rd->delta, F(", "));
	print_value("diff", diff, F(", "));
	Serial.print(freeMemory());													// Added by JF
	Serial.println();
#endif

//	Fine tunes timing but not needed as diff is usually small anyway   JF
//	if (rd->delta >= 1 && diff <= TUNEIN_USEC) stations[packet[0] & 0x7].interval = stations[packet[0] & 0x7].interval  + (diff/2);	


	Serial.print("c:");
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

#ifdef DAVISRFM69_DEBUG
void printHex(volatile byte* packet, byte len) {
	for (byte i = 0; i < len; i++) {
		if (!(packet[i] & 0xf0)) Serial.print('0');
		Serial.print(packet[i], HEX);
		if (i < len - 1) Serial.print('-');
		}
	}
#endif

void update_bme() {

	if (! bme_valid) return;
	bme_loop++;
	switch (bme_loop)
	{
		case 1:
		  temperature= mySensor.readTempC();
		  break;
		case 2:
		  pressure= mySensor.readFloatPressure();
		  break;
		case 3:
			humidity = mySensor.readFloatHumidity();
			break;
	    case 4:
			if (! (isnan(temperature) ||
					isnan(pressure) ||
					isnan(humidity))) {
						Serial.print("bme:");
						Serial.print(temperature);
						Serial.print(",");
						Serial.print(pressure);
						Serial.print(",");
						Serial.print(humidity);
						Serial.println();
					}
			bme_loop=0;
			break;
		default:
			bme_loop=0;
	}
	return;
};




unsigned int blinky=0;
void loop() {
	unsigned long timenow;
	if (radio.qLen > 0) decode_packet();
	if (radio.mode == SM_RECEIVING)	digitalWrite(LED, HIGH);
	else if (radio.mode == SM_SEARCHING) {
		blinky++;
		if ((blinky % 100) == 0 ) Toggle_LED();
		delay(1);
		}
	else digitalWrite(LED, LOW);
	timenow = millis();
	// Lazy wraparound check
	if (timenow > time || time == 0 || (time > 100000 && timenow < 100000) ) {
		time=timenow+BME_DELAY;
		if (Serial.read() == 'r' ) NVIC_SystemReset();
	//Serial.println(micros());
	//	update_bme();
	//Serial.println(micros());
	};



	radio.loop();
	}

 