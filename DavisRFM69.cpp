// Driver implementation for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// Modified for the Adafruit feather M0 by Alan Marchiori 2016 amm042@bucknell.edu.
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work for now from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// In accordance with the CC-BY-SA, many modifications by GitHub user "kobuki".
//
// Modified by Jason Foss in 2019 to simplify for use only with Adafruit Feather M0 Radio in Rx Mode.

#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater
#include <SPI.h>
#define DavisRFM69_local

#include "RFM69registers.h"
#include "DavisRFM69.h"
#include "DavisRFM69_frequencies.h"

volatile byte  DavisRFM69::DATA[DAVIS_PACKET_LEN];
volatile byte DavisRFM69::_mode = RF69_MODE_INIT;       // current transceiver state
volatile byte DavisRFM69::CHANNEL = 0;
volatile byte DavisRFM69::band = 0;
volatile int DavisRFM69::RSSI = 0;   // RSSI measured immediately after payload reception
volatile int16_t DavisRFM69::FEI = 0;

volatile uint32_t DavisRFM69::packets = 0;
volatile uint32_t DavisRFM69::lostPackets = 0;
volatile uint32_t DavisRFM69::numResyncs = 0;
volatile uint32_t DavisRFM69::lostStations = 0;
volatile byte DavisRFM69::stationsFound = 0;
volatile byte DavisRFM69::curStation = 0;
volatile byte DavisRFM69::numStations = NUMSTATIONS;
//volatile uint32_t DavisRFM69::lastDiscStep;
volatile uint32_t rfm69_mode_timer = 0;
volatile byte  DavisRFM69::packetIn, DavisRFM69::packetOut, DavisRFM69::qLen;

int freeMemory();


volatile enum sm_mode DavisRFM69::mode = SM_IDLE;
//PacketFifo DavisRFM69::fifo;
Station *DavisRFM69::stations;
DavisRFM69* DavisRFM69::selfPointer;

void DavisRFM69::initialize(byte freqBand) {
	const byte CONFIG[][2] =
		{
		/* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
		/* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_10 }, // Davis uses Gaussian shaping with BT=0.5
		/* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_19200}, // Davis uses a datarate of 19.2 KBPS
		/* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_19200},
		/* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_9900}, // Davis uses a deviation of 9.9 kHz
		/* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_9900},
		/* 0x0B */ { REG_AFCCTRL, RF_AFCLOWBETA_OFF }, // TODO: Should use LOWBETA_ON, but having trouble getting it working
		/* 0x12 */ { REG_PARAMP, RF_PARAMP_25 }, // xxx
		/* 0x18 */ { REG_LNA, RF_LNA_ZIN_50 | RF_LNA_GAINSELECT_AUTO }, // Not sure which is correct!
		/* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4 }, // Use 25 kHz BW (BitRate < 2 * RxBw)
		/* 0x1A */ { REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3 }, // Use double the bandwidth during AFC as reception
		/* 0x1E */ { REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON },
		/* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, //DIO0 is the only IRQ we're using
		/* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // Reset the FIFOs. Fixes a problem I had with bad first packet.
		/* 0x29 */ { REG_RSSITHRESH, 150 }, // real dBm = -(REG_RSSITHRESH / 2) -> 190 raw = -95 dBm
		/* 0x2d */ { REG_PREAMBLELSB, 0x4 }, // Davis has four preamble bytes 0xAAAAAAAA -- use 6 for TX for this setup
		/* 0x2e */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2 },  // Allow a couple erros in the sync word
		/* 0x2f */ { REG_SYNCVALUE1, 0xcb }, // Davis ISS first sync byte. http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html
		/* 0x30 */ { REG_SYNCVALUE2, 0x89 }, // Davis ISS second sync byte.
		/* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF }, // Fixed packet length and we'll check our own CRC
		/* 0x38 */ { REG_PAYLOADLENGTH, DAVIS_PACKET_LEN }, // Davis sends 8 bytes of payload, including CRC that we check manually.
		/* 0x3c */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | 0x07 }, // TX on FIFO having more than seven bytes
		/* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_1BIT | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
		/* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // // TODO: Should use LOWBETA_ON, but having trouble getting it working
		/* 0x71 */ { REG_TESTAFC, 0 }, // AFC Offset for low mod index systems
		{255, 0}
		};

	digitalWrite(_slaveSelectPin, HIGH);
	pinMode(_slaveSelectPin, OUTPUT);
	SPI.begin();

	do writeReg(REG_SYNCVALUE1, 0xaa); while (readReg(REG_SYNCVALUE1) != 0xaa);
	do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55);

	for (byte i = 0; CONFIG[i][0] != 255; i++)
		writeReg(CONFIG[i][0], CONFIG[i][1]);

	setMode(RF69_MODE_STANDBY);
	while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
	attachInterrupt(_interruptNum, DavisRFM69::isr0, RISING);

	selfPointer = this;
	userInterrupt = NULL;
	mode = SM_IDLE;
	band = freqBand;
	setChannel(0);
	for (byte i = 0; i < numStations; i++) {
		stations[i].channel = 0;
		stations[i].lastRx = 0;
		stations[i].interval = 0;
		stations[i].lostPackets = 0;
		stations[i].lastRx = 0;
		stations[i].lastSeen = 0;
		stations[i].packets = 0;
		stations[i].syncBegan = 0;
		}
	}

	/**
	 * compute a 32bit time difference assuming the first argument
	 * happend after the second. (ie accounting for wrap around).
	 */
uint32_t DavisRFM69::difftime(uint32_t after, uint32_t before) {
	if (after >= before || !(after < 0x8fffffff  && before >0x8fffffff) ) {
		if (after< before) return 0;
		return after - before;
		}
	 // counter wrapped
	return (0xffffffff - before) + after + 1;
	}

	/**
	 * Called by the main arduino loop, used when not using a timer (handleTimerInt)
	 * this will check for lost packets and tune the radio to the proper channel
	 * hopefully at the proper time. Added by AMM.
	 */
void DavisRFM69::loop() {
	uint8_t i;

	// first see if we have tuned into receive a station previously and failed to actually receive a packet
	if (mode == SM_RECEIVING) {
	  // packet was lost
		if (difftime(micros(), stations[curStation].recvBegan) > (1 + stations[curStation].lostPackets)*(LATE_PACKET_THRESH + TUNEIN_USEC)
		&& mode == SM_RECEIVING ) {
#ifdef DAVISRFM69_DEBUG
			Serial.print(micros());
			Serial.print(": missed packet from station ");
			Serial.print(stations[curStation].id);
			Serial.print(" channel ");
			Serial.println(stations[curStation].channel);
#endif
			lostPackets++;
			stations[curStation].lostPackets++;
				stations[curStation].lastRx += stations[curStation].interval;
			    stations[curStation].channel = nextChannel(stations[curStation].channel);
			

			// lost a station
			if (stations[curStation].lostPackets > RESYNC_THRESHOLD) {
#ifdef DAVISRFM69_DEBUG
				Serial.print(micros());
				Serial.print(": station ");
				Serial.print(stations[curStation].id);
				Serial.println(" is lost.");
#endif
				stations[curStation].lostPackets = 0;
				stations[curStation].interval = 0;

				lostStations++;
				stationsFound--;
				}

			curStation = -1;
			mode = SM_IDLE;
			setMode(RF69_MODE_STANDBY);
		}
		else {
	   // waiting to receive and it hasn't timed out yet.
	   // do nothing.
			return;
			}
		}

		// next check for any station that's about to transmit
		// if found, tune in
	for (i = 0; i < numStations; i++) {
	  // interval is filled in once we discover a station
		if (stations[i].interval > 0) {
#ifdef DAVISRFM69_DEBUG_VERBOSE
				Serial.print(micros());
				Serial.print(" ");
				Serial.print(stations[i].lastRx);
				Serial.print(" ");
				Serial.println(difftime(stations[i].lastRx + stations[i].interval,micros()));
				Serial.print("id ");
				Serial.print(stations[i].id);
				Serial.print(" channel ");
				Serial.println(stations[i].channel);
#endif

			if (difftime(stations[i].lastRx + stations[i].interval,micros()) < ((1 + stations[i].lostPackets)*TUNEIN_USEC)) {
#ifdef DAVISRFM69_DEBUG
				Serial.print(micros());
				Serial.print(" ");
				Serial.print(stations[i].lastRx);
				Serial.print(" ");
				Serial.println(difftime(stations[i].lastRx + stations[i].interval,micros()));
				Serial.print(": tune to station ");
				Serial.print(stations[i].id);
				Serial.print(" channel ");
				Serial.println(stations[i].channel);
#endif
				stations[i].recvBegan = micros();
				setChannel(stations[i].channel);

				// we are now set to receive from this station.
				mode = SM_RECEIVING;
				curStation = i;
				return;
				}
			}
		}

		// if no transmitter is about to transmit, check for any
		// station that we are not synchonized with, if there are
		// any stations not synchronized, turn on the receiver and
		// hope we get lucky.
	bool all_sync = true;
	for (i = 0; i < numStations; i++) {
	  // unknown stations will have interval of zero, the radio interrupt will
	  // fill this in if we receive a packet from the given station.
		if (stations[i].interval == 0) {
			mode = SM_SEARCHING;
			all_sync = false;
			if (stations[i].syncBegan == 0) {
				// we have never tried to sync to this station
#ifdef DAVISRFM69_DEBUG
				Serial.print(micros());
				Serial.print(": begin sync to station ");
				Serial.print(stations[i].id);
				Serial.print(" channel ");
				Serial.println(stations[i].channel);
#endif
				stations[i].syncBegan = micros();
				stations[i].progress = 0;
				setChannel(stations[i].channel);
				}
			else if (difftime(micros(), stations[i].syncBegan) > DISCOVERY_STEP) {
			   // we tried and failed to sync, try the next channel

				stations[i].channel = nextChannel(stations[i].channel);
#ifdef DAVISRFM69_DEBUG
				Serial.print(micros());
				Serial.print(": sync fail, begin sync to station ");
				Serial.print(stations[i].id);
				Serial.print(" channel ");
				Serial.println(stations[i].channel);
#endif
				stations[i].syncBegan = micros();
				stations[i].progress = 0;
				setChannel(stations[i].channel);
				}
			else {
				if (CHANNEL != stations[i].channel) setChannel(stations[i].channel);
			 // we're waiting to hear from this station, don't tune away!
			 setMode(RF69_MODE_RX);

#ifdef DAVISRFM69_DEBUG
				byte p = difftime(micros(), stations[i].syncBegan) / (DISCOVERY_STEP / 100);

				if (stations[i].progress != p) {
					Serial.print(micros());
					Serial.print(": listen progress ");
					Serial.print(p);
					Serial.println("\%");
					stations[i].progress = p;
					Serial.println(freeMemory());									// Added by JF
					}
#endif
				break;
				}
			}
		}

	if (all_sync) {
		mode = SM_SYNCHRONIZED;

		// if we got here, no stations are about to TX and all stations are in sync,
		// we can disable our radio to save power.
		if (_mode != RF69_MODE_SLEEP) {
#ifdef DAVISRFM69_DEBUG
			Serial.print(micros());
			Serial.println(": Nothing to do, going to sleep");
#endif
			setMode(RF69_MODE_SLEEP);
			}
		}
	}

	// Handle received packets, called from RFM69 ISR
void DavisRFM69::handleRadioInt() {
	uint32_t lastRx = micros();
	uint16_t rxCrc = word(DATA[6], DATA[7]);  // received CRC
	uint16_t calcCrc = DavisRFM69::crc16_ccitt(DATA, 6);  // calculated CRC

	bool repeaterCrcTried = false;

	// repeater packets checksum bytes (0..5) and (8..9), so try this at mismatch
	if (calcCrc != rxCrc) {
		calcCrc = DavisRFM69::crc16_ccitt(DATA + 8, 2, calcCrc);
		repeaterCrcTried = true;
		}

	// packet passed crc?
	if (calcCrc == rxCrc && rxCrc != 0) {

	  // station id is byte 0:0-2
		byte id = DATA[0] & 7;
		int stIx = findStation(id);

		// if we have no station cofigured for this id (at all; can still be be !active), ignore the packet
		// OR packet passed the repeater crc check, but no repeater is set for the station
		// OR packet passed the normal crc check, and repeater is set for the station
		if (stIx < 0
			|| (repeaterCrcTried && stations[stIx].repeaterId == 0)
			|| (!repeaterCrcTried && stations[stIx].repeaterId != 0)) {
			setChannel(CHANNEL);
			return;
			}

		if (stationsFound < numStations && stations[stIx].interval == 0) {
			stations[stIx].interval = ((41 + id) * 1000000 / 16) ;
			//-(1000000* ((10+2+4)*8/19200)) ; 
			// Davis' official tx interval in us minus packet length
			// We don't get signaled till the end of the packet, hopping is from
			// start to start (4 preamble, 2 sync, 10 data at 19200 symbol/second(bits))

			stationsFound++;
#ifdef DAVISRFM69_DEBUG
			Serial.print("found station ");
			Serial.print(stIx);
			Serial.print(" interval ");
			Serial.println(stations[stIx].interval);
#endif
			if (lostStations > 0) lostStations--;
			}

		packets++;
		if (stations[stIx].active) {
			stations[stIx].packets++;
			if (qLen < FIFO_SIZE) {
				memcpy(&packetFifo[packetIn].packet, (byte*) DATA, DAVIS_PACKET_LEN);
				packetFifo[packetIn].channel = CHANNEL;
				packetFifo[packetIn].rssi = -RSSI;
				packetFifo[packetIn].fei = FEI;
				packetFifo[packetIn].delta = stations[curStation].lastSeen > 0 ? lastRx - stations[curStation].lastSeen : 0;
				if (++packetIn == FIFO_SIZE) packetIn = 0;
				qLen++;
				}
			}

#ifdef DAVISRFM69_DEBUG
		stations[stIx].earlyAmt = difftime(lastRx, stations[stIx].recvBegan);
		Serial.print("early amt = ");
		Serial.println(stations[stIx].earlyAmt);
#endif
		stations[stIx].channel = nextChannel(CHANNEL);
		stations[stIx].lostPackets = 0;
		stations[stIx].lastRx = lastRx;
		stations[stIx].lastSeen = lastRx;

	// no longer waiting to RX (if we were at all anwyay)
		mode = SM_IDLE;
		// standby the radio
		setMode(RF69_MODE_STANDBY);
		}
	else {
   // bad CRC, go back to RX on this channel
		setChannel(CHANNEL); // this always has to be done somewhere right after reception, even for ignored/bogus packets
		}
	}

	// Calculate the next hop of the specified channel
byte DavisRFM69::nextChannel(byte channel) {
	return ++channel % bandTabLengths[band];
	}

	// Find the station index in stations[] for station expected to tx the earliest and update curStation
void DavisRFM69::nextStation() {
	uint32_t earliest = 0xffffffff;
	uint32_t now = micros();
	for (int i = 0; i < numStations; i++) {
		uint32_t current = stations[i].lastRx + stations[i].interval - now;
		if (stations[i].interval > 0 && current < earliest) {
			earliest = current;
			curStation = i;
			}
		}
	}

	// Find station index in stations[] for a station ID (-1 if doesn't exist)
int DavisRFM69::findStation(byte id) {
	for (byte i = 0; i < numStations; i++) {
		if (stations[i].id == id) return i;
		}
	return -1;
	}

void DavisRFM69::interruptHandler() {
	RSSI = (-readReg(REG_RSSIVALUE)) >> 1;  // Read up front when it is most likely the carrier is still up
	if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
		FEI = word(readReg(REG_FEIMSB), readReg(REG_FEILSB));
		setMode(RF69_MODE_STANDBY);

		select();   // Select RFM69 module, disabling interrupts
		SPI.transfer(REG_FIFO & 0x7f);
		for (byte i = 0; i < DAVIS_PACKET_LEN; i++)
			DATA[i] = reverseBits(SPI.transfer(0));
		handleRadioInt();
		unselect();  // Unselect RFM69 module, enabling interrupts
		}
	}

void DavisRFM69::setChannel(byte channel) {

#ifdef DAVISRFM69_DEBUG
	Serial.print("CH->>");
	Serial.println(channel);
#endif

	CHANNEL = channel;
	if (CHANNEL > bandTabLengths[band] - 1) CHANNEL = 0;
	writeReg(REG_FRFMSB, pgm_read_byte(&bandTab[band][CHANNEL][0]));
	writeReg(REG_FRFMID, pgm_read_byte(&bandTab[band][CHANNEL][1]));
	writeReg(REG_FRFLSB, pgm_read_byte(&bandTab[band][CHANNEL][2]));

	if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
		writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	setMode(RF69_MODE_RX);
	}

	// The data bytes come over the air from the ISS least significant bit first. Fix them as we go. From
	// http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
byte DavisRFM69::reverseBits(byte b) {
	b = ((b & 0b11110000) >> 4) | ((b & 0b00001111) << 4);
	b = ((b & 0b11001100) >> 2) | ((b & 0b00110011) << 2);
	b = ((b & 0b10101010) >> 1) | ((b & 0b01010101) << 1);

	return(b);
	}

	// Davis CRC calculation from http://www.menie.org/georges/embedded/
uint16_t DavisRFM69::crc16_ccitt(volatile byte *buf, byte len, uint16_t crc) {
	while (len--) {
		crc ^= *(char *) buf++ << 8;
		for (int i = 0; i < 8; ++i) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
			}
		}
	return crc;
	}

void DavisRFM69::setMode(byte newMode) {
	if (newMode == _mode) return;

#ifdef DAVISRFM69_DEBUG
	Serial.print(_mode);
	Serial.print(" -> ");
	Serial.println(newMode);
#endif

	switch (newMode) {
			case RF69_MODE_TX:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
				break;
			case RF69_MODE_RX:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
				break;
			case RF69_MODE_STANDBY:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
				break;
			case RF69_MODE_SLEEP:
				writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);		// Changed from RF_OPMODE_SLEEP to RF_OPMODE_STANDBY
																							// Caused missed packets   JF
				break;
			default: return;
		}
	_mode = newMode;
	}

void DavisRFM69::isr0() { selfPointer->interruptHandler(); }

byte DavisRFM69::readReg(byte addr) {
	select();
	SPI.transfer(addr & 0x7F);
	byte regval = SPI.transfer(0);
	unselect();
	return regval;
	}

void DavisRFM69::writeReg(byte addr, byte value) {
	select();
	SPI.transfer(addr | 0x80);
	SPI.transfer(value);
	unselect();
	}

	/// Select the transceiver
void DavisRFM69::select() {
	noInterrupts();
	digitalWrite(_slaveSelectPin, LOW);
	}

	/// Unselect the transceiver chip
void DavisRFM69::unselect() {
	digitalWrite(_slaveSelectPin, HIGH);
	interrupts();
	}

void DavisRFM69::setBandwidth(byte bw) {
	switch (bw) {
			case RF69_DAVIS_BW_NARROW:
				writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4); // Use 25 kHz BW (BitRate < 2 * RxBw)
				writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use double the bandwidth during AFC as reception
				break;
			case RF69_DAVIS_BW_WIDE:
			  // REG_RXBW 50 kHz fixes console retransmit reception but seems worse for SIM transmitters (to be confirmed with more testing)
				writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use 50 kHz BW (BitRate < 2 * RxBw)
				writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_2); // Use double the bandwidth during AFC as reception
				break;
			default:
				return;
		}
	}
