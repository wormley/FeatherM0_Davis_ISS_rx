// Driver definition for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// In accordance with the CC-BY-SA, many modifications by GitHub user "kobuki".
//
// Modified by Jason Foss in 2019 to simplify for use only with Adafruit Feather M0 Radio in Rx Mode.


#define DAVISRFM69_DEBUG

#ifndef DAVISRFM69_h
#define DAVISRFM69_h

// Davis VP2 standalone station types
#define STYPE_ISS         0x0 // ISS
#define STYPE_TEMP_ONLY   0x1 // Temperature Only Station
#define STYPE_HUM_ONLY    0x2 // Humidity Only Station
#define STYPE_TEMP_HUM    0x3 // Temperature/Humidity Station
#define STYPE_WLESS_ANEMO 0x4 // Wireless Anemometer Station
#define STYPE_RAIN        0x5 // Rain Station
#define STYPE_LEAF        0x6 // Leaf Station
#define STYPE_SOIL        0x7 // Soil Station
#define STYPE_SOIL_LEAF   0x8 // Soil/Leaf Station
#define STYPE_SENSORLINK  0x9 // SensorLink Station (not supported for the VP2)
#define STYPE_OFF         0xA // No station OFF
#define STYPE_VUE         0x10 // pseudo station type for the Vue ISS
							   // since the Vue also has a type of 0x0

#define ISS_TYPE	STYPE_VUE // Change to STYPE_VUE to correctly display wind for VUE

#define LED 13
#define SERIAL_BAUD 115200
#define SPI_CS          8 // SS is the SPI slave select pin, for instance D10 on atmega328
#define RF69_IRQ_PIN    3
#define RF69_IRQ_NUM    3
// This is the default, our caller can override.
#define NUMSTATIONS			  1
#define DAVIS_PACKET_LEN     10 // ISS has fixed packet lengths of eight bytes including CRC and two bytes trailing repeater info

#define RF69_MODE_SLEEP       0 // XTAL OFF
#define RF69_MODE_STANDBY     1 // XTAL ON
#define RF69_MODE_RX          2 // RX MODE
#define RF69_MODE_TX          3 // TX MODE
#define RF69_MODE_INIT		0xff // USED ONLY FOR INIT THIS IS NOT VALID OTHERWISE

#define RF69_DAVIS_BW_NARROW  1
#define RF69_DAVIS_BW_WIDE    2

#define RESYNC_THRESHOLD	 49 // max. number of lost packets from a station before rediscovery
#define LATE_PACKET_THRESH 7500L // packet is considered missing after this many micros
// This must include enough time to get on-channel, tune AND receive the entire packet
#define TUNEIN_USEC		 15000L // 10 milliseconds, amount of time before an expect TX to tune the radio in.	    							
								// this includes possible radio turnaround tx->rx or sleep->rx transitions
								// 10 ms is reliable, should be able to get this faster but
								// the loop is polled, so slow loop calls will cause missed packets

#define DISCOVERY_STEP   150000000L	// 150 seconds
#define FIFO_SIZE			8

#define FREQ_TABLE_LENGTH_US 51
#define FREQ_TABLE_LENGTH_AU 51
#define FREQ_TABLE_LENGTH_EU 5
#define FREQ_TABLE_LENGTH_NZ 51

#define FREQ_BAND_US 0
#define FREQ_BAND_AU 1
#define FREQ_BAND_EU 2
#define FREQ_BAND_NZ 3

// added these here because upstream removed them
#define REG_TESTAFC        0x71

#define RF_FDEVMSB_9900    0x00
#define RF_FDEVLSB_9900    0xa1

#define RF_AFCLOWBETA_ON   0x20
#define RF_AFCLOWBETA_OFF  0x00 // Default

// Below are known, publicly documented packet types for the VP2 and the Vue.

// VP2 packet types
#define VP2P_UV           0x4 // UV index
#define VP2P_RAINSECS     0x5 // seconds between rain bucket tips
#define VP2P_SOLAR        0x6 // solar irradiation
#define VP2P_TEMP         0x8 // outside temperature
#define VP2P_WINDGUST     0x9 // 10-minute wind gust
#define VP2P_HUMIDITY     0xA // outside humidity
#define VP2P_RAIN         0xE // rain bucket tips counter
#define VP2P_SOIL_LEAF    0xF // soil/leaf station

// Vue packet types
#define VUEP_VCAP         0x2 // supercap voltage
#define VUEP_VSOLAR       0x7 // solar panel voltage

/** DavisRFM69 state machine modes */
enum sm_mode {
	SM_IDLE = 0,  				// no stations configured
	SM_SEARCHING = 1, 			// searching for station(s)
	SM_SYNCHRONIZED = 2, 		// in sync with all stations
	SM_RECEIVING = 3,			// receiving from a station
	};

	// Station data structure for managing radio reception
struct __attribute__((packed)) Station {
	byte id;                	// station ID (set with the DIP switch on original equipment)
							  // set it ONE LESS than advertised station id, eg. 0 for station 1 (default) etc.
	byte type;              	// STYPE_XXX station type, eg. ISS, standalone anemometer transmitter, etc. 
	bool active;            	// true when the station is actively listened and will queue packets
	byte repeaterId;        	// repeater id when packet is coming via a repeater, otherwise 0
							  // repeater IDs A..H are stored as 0x8..0xf here

	uint32_t lastRx;   	 	// last time a packet is seen or should have been seen when missed
	uint32_t lastSeen; 	 	// last factual reception time
	uint32_t interval;    	// packet transmit interval for the station: (41 + id) / 16 * 1M microsecs
	uint32_t numResyncs;  	// number of times discovery of this station started because of packet loss
	uint32_t packets; 		// total number of received packets after (re)restart
	uint32_t lostPackets;     // missed packets since a packet was last seen from this station
	uint32_t syncBegan; 		// time sync began for this station.
	uint32_t recvBegan; 		// time we tuned in to receive
	uint32_t earlyAmt;		// microseconds from when we turned on rx to when the last packet was rx'ed (for tuning, we want this small)
	byte progress;			// search(sync) progress in percent.
	byte channel;           	// rx channel the next packet of the station is expected on (moved by amm for packing on 32 bit machines)
	};

struct __attribute__((packed)) WxData {
	byte rain = 0;
	uint16_t rainrate = 0;
	uint16_t rh = 0;
	int16_t soilleaf = -1;
	float solar = -1;
	int16_t temp = 0;
	float uv = -1;
	int16_t vcap = -1;
	int16_t vsolar = -1;
	uint16_t windd = 0;
	uint8_t winddraw = 0;
	uint8_t windgust = 0;
	byte windgustd = 0;
	uint16_t windv = 0;
	};

struct __attribute__((packed)) RadioData {
	byte packet[DAVIS_PACKET_LEN];
	byte channel;
	byte rssi;
	int16_t fei;
	uint32_t delta;
	};

class DavisRFM69 {
public:
	static volatile byte packetOut, qLen;
	RadioData packetFifo[FIFO_SIZE];
	static volatile uint32_t lostPackets;
	static volatile uint32_t packets;
	static volatile byte numStations;
	static volatile enum sm_mode mode;
	static Station *stations;

	DavisRFM69(byte slaveSelectPin, byte interruptPin, byte interruptNum) {
		_slaveSelectPin = slaveSelectPin;
		_interruptPin = interruptPin;
		_interruptNum = interruptNum;
		_mode = RF69_MODE_STANDBY;
		}

	void initialize(byte freqBand);
	void setBandwidth(byte bw);
	void loop();

protected:
	static volatile byte packetIn;
	static volatile byte DATA[DAVIS_PACKET_LEN];
	static volatile byte _mode;
	static volatile byte CHANNEL;
	static volatile int RSSI;
	static volatile int16_t FEI;
	static volatile byte band;
	static volatile uint32_t numResyncs;
	static volatile uint32_t lostStations;
	static volatile byte stationsFound;
	static volatile byte curStation;

	static DavisRFM69* selfPointer;

	byte _slaveSelectPin;
	byte _interruptPin;
	byte _interruptNum;

	void setChannel(byte channel);
	static uint16_t crc16_ccitt(volatile byte *buf, byte len, uint16_t initCrc = 0);
	byte readReg(byte addr);
	void writeReg(byte addr, byte val);
	byte nextChannel(byte channel);
	int findStation(byte id);
	void handleRadioInt();
	uint32_t difftime(uint32_t after, uint32_t before);
	void nextStation();
	void(*userInterrupt)();
	void virtual interruptHandler();
	byte reverseBits(byte b);
	static void isr0();
	void setMode(byte mode);
	void select();
	void unselect();
	};

	// FRF_MSB, FRF_MID, and FRF_LSB for the 51 North American, Australian, New Zealander & 5 European channels
	// used by Davis in frequency hopping
#endif  // DAVISRFM_h
