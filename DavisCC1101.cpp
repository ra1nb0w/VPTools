// Driver implementation for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
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

#include "DavisCC1101.h"
#include "CC1101registers.h"
#include "TimerOne.h"
#include "MsTimer2.h"
#include "PacketFifo.h"

#include <SPI.h>

volatile byte DavisCC1101::DATA[DAVIS_PACKET_LEN];
volatile byte DavisCC1101::_mode; // current transceiver state
volatile bool DavisCC1101::_packetReceived = false;
volatile byte DavisCC1101::CHANNEL = 0;
volatile byte DavisCC1101::band = 0;
volatile int DavisCC1101::RSSI; // RSSI measured immediately after payload reception
volatile int16_t DavisCC1101::FEI;
volatile bool DavisCC1101::txMode = false;
volatile uint32_t DavisCC1101::lastTx = micros();
volatile uint32_t DavisCC1101::txDelay = 0;
volatile uint32_t DavisCC1101::realTxDelay = 0;
volatile byte DavisCC1101::txId = 255;
volatile byte DavisCC1101::txChannel = 255;
volatile byte DavisCC1101::rxRssi = 160; // -80 dB

volatile uint32_t DavisCC1101::packets = 0;
volatile uint32_t DavisCC1101::lostPackets = 0;
volatile uint32_t DavisCC1101::numResyncs = 0;
volatile uint32_t DavisCC1101::lostStations = 0;
volatile byte DavisCC1101::stationsFound = 0;
volatile byte DavisCC1101::curStation = 0;
volatile byte DavisCC1101::numStations = 0;
volatile byte DavisCC1101::discChannel = 2;
volatile uint32_t DavisCC1101::lastDiscStep;
volatile int16_t DavisCC1101::freqCorr = 0;
volatile uint32_t DavisCC1101::timeBase = 1000000;
volatile bool DavisCC1101::repeaterPT = false;

PacketFifo DavisCC1101::fifo;
Station *DavisCC1101::stations;
DavisCC1101 *DavisCC1101::selfPointer;

bool DavisCC1101::initialize(byte freqBand)
{
  /*
  const byte CONFIG[][2] =
      {
          // GDO2 Output Pin Configuration
          // The Rx FIFO threshold is kept at the default value of 32, which is
          // never reached due to a packet length of 8, so GDO2 will go high at
          // end of packet.
          {kIOCFG2, 0x01},

          // GDO0 Output Pin Configuration
          // Set when the sync word has been received. Cleared at end of packet
          // (and several other conditions not relevant here).
          {kIOCFG0, 0x06},

          // Sync Word, High Byte
          {kSYNC1, 0xCB},

          // Sync Word, Low Byte
          {kSYNC0, 0x89},

          // Packet Length
          // Davis uses 10 data bytes (not including preamble or sync bytes).
          {kPKTLEN, 0x0A},

          // Packet Automation Control
          // Preamble quality threshold = 16.
          // CRC_AUTOFLUSH = 0. APPEND_STATUS = 0.  No address check.
          {kPKTCTRL1, 0x80},

          // Packet Automation Control
          // Whitening off.  Use FIFOs for RX and TX data.
          // CRC_EN = 0.  Fixed packet length mode.
          {kPKTCTRL0, 0x00},

          // Frequency Synthesizer Control
          // IF is 152 kHz.  This value is from SmartRF Studio.
          {kFSCTRL1, 0x06},

          // Frequency Synthesizer Control
          // Frequency offset is 0.
          {kFSCTRL0, 0},

          // Modem Configuration
          // Channel bandwidth is 101.6 kHz.
          // Data rate is 19192 Hz (target 19.2 kHz).
          {kMDMCFG4, 0xC9},

          // Modem Configuration
          // Data rate is 19192 Hz (target 19.2 kHz).
          {kMDMCFG3, 0x83},

          // Modem Configuration
          // Enable DC blocking filter.  GFSK Modulation.
          // Manchester encoding off.  Need 16/16 bits correct in sync word
          // to begin packet reception.
          {kMDMCFG2, 0x12},

          // Modem Configuration
          // No forward error correction.  4 preamble bytes (for TX I believe).
          // 199.951 kHz channel spacing (not used for selecting frequency).
          {kMDMCFG1, 0x22},

          // Modem Deviation Setting
          // 9.5 kHz deviation.
          {kDEVIATN, 0x24},

          // Main Radio Control State Machine Configuration
          // RX_TIME_RSSI = 0.  RX_TIME_QUAL = 0.
          // No RX timeout (that's implemented here).
          {kMCSM2, 0x07},

          // Main Radio Control State Machine Configuration
          // CCA_MODE = 3 (clear channel indication behavior).
          // Switch to IDLE mode after receiving a packet.
          {kMCSM1, 0x30},

          // Main Radio Control State Machine Configuration
          // Calibrate when switching from IDLE to RX, TX or FSTXON.
          // PO_TIMEOUT = 2.  PIN_CTRL_EN = 0.  XOSC_FORCE_ON = 0.
          {kMCSM0, 0x18},

          // Frequency Offset Compensation Configuration
          // FOC_BS_CS_GATE = 1.  FOC_PRE_K = 2.  FOC_POST_K = 1.
          // FOC_LIMIT = 2.
          {kFOCCFG, 0x36},

          // Frequency Synthesizer Calibration
          // Value from SmartRF Studio.
          {kFSCAL3, 0xE9},

          // Frequency Synthesizer Calibration
          // Value from SmartRF Studio.
          {kFSCAL2, 0x2A},

          // Frequency Synthesizer Calibration
          // Value from SmartRF Studio.
          {kFSCAL1, 0x00},

          // Frequency Synthesizer Calibration
          // Value from SmartRF Studio.
          {kFSCAL0, 0x1F},

          // Various Test Settings
          // Value from SmartRF Studio.
          {kTEST2, 0x81},

          // Various Test Settings
          // Value from SmartRF Studio.
          {kTEST1, 0x35},

          // Various Test Settings
          // Value from SmartRF Studio.
          {kTEST0, 0x09},

          // default
          {255, 0}};
        */

  const byte CONFIG[][2] =
      {
          {kIOCFG2, 0x2E},
          {kIOCFG0, 0x01},
          {kFIFOTHR, 0x41},
          {kSYNC1, 0xCB},
          {kSYNC0, 0x89},
          {kPKTLEN, 0x0A},
          {kPKTCTRL1, 0x00},
          {kPKTCTRL0, 0x00},
          {kFSCTRL1, 0x06},
          {kFREQ2, 0x21},
          {kFREQ1, 0x63},
          {kFREQ0, 0x1E},
          {kMDMCFG4, 0xC9},
          {kMDMCFG3, 0x83},
          {kMDMCFG2, 0x12},
          {kMDMCFG1, 0x33},
          {kMDMCFG0, 0x22},
          {kDEVIATN, 0x24},
          {kMCSM1, 0x30}, //0x3C
          {kMCSM0, 0x18},
          {kFOCCFG, 0x16},
          {kAGCCTRL2, 0x43},
          {kWORCTRL, 0xFB},
          {kFSCAL3, 0xE9},
          {kFSCAL2, 0x2A},
          {kFSCAL1, 0x00},
          {kFSCAL0, 0x1F},
          {kTEST2, 0x81},
          {kTEST1, 0x35},
          {kTEST0, 0x09},
          {255, 0}};

#ifdef DEBUG
  Serial.println("Initiliaze CC1101 interface...");
#endif

  pinMode(_slaveSelectPin, OUTPUT);
  //setup AVR GPIO ports
  pinMode(CC1101_IRQ_PIN, INPUT);
  pinMode(CC1101_IRQ2_PIN, INPUT);

  digitalWrite(_slaveSelectPin, HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  // probably can be removed with 8MHz clock
  //SPI.setClockDivider(SPI_CLOCK_DIV2); // max speed, except on Due which can run at system clock speed
  SPI.begin();

  // The CC1101 includes an automatic power-on reset circuit.  See section
  // 19.1.1 of the datasheet for details.  Here we wait for MISO to go low,
  // indicating that the crystal oscillator has stabilized.
  uint32_t start_time = millis();
  select();
  while (digitalRead(MISO))
  { // Wait for XOSC Stable.
    // millis() will not overflow since a power-on/reset just occurred.
    if (millis() - start_time > 50)
    {
#ifdef DEBUG
      Serial.println(F("error: CC1101 is not responding."));
#endif
      return false;
    }
  }
  unselect();

  // If the Arduino was not reset by power-on, the CC1101 will not
  // automatically reset itself. We reset so that if the user messed up any
  // registers with the debug commands they will be set to their defaults.
  // select();
  // SPI.transfer(kSRES);
  // unselect();

  sendStrobe(kSFTX); //flush the TX_fifo content
  delayMicroseconds(100);
  sendStrobe(kSFRX); //flush the RX_fifo content
  delayMicroseconds(100);

  byte partnum = readReg(kPARTNUM); //reads CC1100 partnumber
  byte version = readReg(kVERSION); //reads CC1100 version number

  //checks if valid Chip ID is found. Usualy 0x03 or 0x14. if not -> abort
  if (version == 0x00 || version == 0xFF)
  {
#ifdef DEBUG
    Serial.println(F("no CC11xx found!"));
    Serial.println();
#endif
    return false;
  }
#ifdef DEBUG
  Serial.print(F("Partnumber: "));
  Serial.println(partnum, HEX);

  Serial.print(F("Version: "));
  Serial.println(version, HEX);
#endif

  for (byte i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  setMode(CC1101_MODE_IDLE);
  //  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)
  //    ; // Wait for ModeReady
  attachInterrupt(_interruptNum, DavisCC1101::isr0, RISING);

  selfPointer = this;

  setBand(freqBand);
  setChannel(discChannel);

  fifo.flush();
  initStations();
  lastDiscStep = micros();

#ifndef CC1101_TX_ONLY
  MsTimer2::set(1, DavisCC1101::handleTimerInt);
  MsTimer2::start();
#endif

  return true;
}

void DavisCC1101::stopReceiver()
{
  Timer1.detachInterrupt();
#ifndef CC1101_TX_ONLY
  MsTimer2::stop();
#endif
  setMode(CC1101_MODE_SLEEP);
}

void DavisCC1101::setStations(Station *_stations, byte n)
{
  stations = _stations;
  numStations = n;
}

void DavisCC1101::handleTxInt()
{
  uint32_t t = micros();
  realTxDelay = t - lastTx;
  (*selfPointer->txCallback)((byte *)DATA);
  selfPointer->send((byte *)DATA, txChannel);
  txChannel = selfPointer->nextChannel(txChannel);
  realTxDelay = (int32_t)(t - lastTx - txDelay);
  lastTx = t;
}

// Handle missed packets. Called from a timer ISR every ms
void DavisCC1101::handleTimerInt()
{
#ifdef DEBUGX
  Serial.print("CC1101 state machine register: ");
  Serial.println(selfPointer->readReg(kMARCSTATE), HEX);
  Serial.print("CC1101 pkt status register: ");
  Serial.println(selfPointer->readReg(kPKTSTATUS), HEX);
#endif
  if (txMode)
    return;

  uint32_t t = micros();
  bool readjust = false;

  if (stations[curStation].interval > 0 && stations[curStation].lastRx + stations[curStation].interval - t < DISCOVERY_GUARD && CHANNEL != stations[curStation].channel)
  {
    selfPointer->setChannel(stations[curStation].channel);
    return;
  }

  if (t - lastDiscStep > DISCOVERY_STEP)
  {
    discChannel = selfPointer->nextChannel(discChannel);
    lastDiscStep = t;
  }

  // find and adjust 'last seen' rx time on all stations with older reception timestamp than their period + threshold
  // that is, find missed packets
  for (byte i = 0; i < numStations; i++)
  {
    if (stations[i].interval > 0 && (t - stations[i].lastRx) > stations[i].interval + LATE_PACKET_THRESH)
    {
      if (stations[curStation].active)
        lostPackets++;
      stations[i].lostPackets++;
      stations[i].missedPackets++;
      if (stations[i].lostPackets > RESYNC_THRESHOLD)
      {
        stations[i].numResyncs++;
        stations[i].interval = 0;
        stations[i].lostPackets = 0;
        lostStations++;
        stationsFound--;
        if (lostStations == numStations)
        {
          numResyncs++;
          stationsFound = 0;
          lostStations = 0;
          selfPointer->initStations();
          selfPointer->setChannel(discChannel);
          return;
        }
      }
      else
      {
        stations[i].lastRx += stations[i].interval;                          // when packet should have been received
        stations[i].channel = selfPointer->nextChannel(stations[i].channel); // skip station's next channel in hop seq
        readjust = true;
      }
    }
  }

  if (readjust)
  {
    selfPointer->nextStation();
    selfPointer->setChannel(stations[curStation].channel);
  }
}

// Handle received packets, called from RFM69 ISR
void DavisCC1101::handleRadioInt()
{
  uint32_t lastRx = micros();
  uint16_t rxCrc = word(DATA[6], DATA[7]);              // received CRC
  uint16_t calcCrc = DavisCC1101::crc16_ccitt(DATA, 6); // calculated CRC

  bool repeaterCrcTried = false;

  // repeater packets checksum bytes (0..5) and (8..9), so try this at mismatch
  if (calcCrc != rxCrc)
  {
    calcCrc = DavisCC1101::crc16_ccitt(DATA + 8, 2, calcCrc);
    repeaterCrcTried = true;
  }

  // fifo.queue((byte*)DATA, CHANNEL, -RSSI, FEI, lastRx - stations[curStation].lastSeen);

  // packet passed crc?
  if (calcCrc == rxCrc && rxCrc != 0)
  {
#ifdef DEBUG
    Serial.println("Good CRC");
#endif

    // station id is byte 0:0-2
    byte id = DATA[0] & 7;
    int stIx = findStation(id);

    // if we have no station cofigured for this id (at all; can still be be !active), ignore the packet
    // OR packet passed the repeater crc check, but no repeater is set for the station
    // OR packet passed the normal crc check, and repeater is set for the station
    if (stIx < 0 || (!repeaterPT && repeaterCrcTried && stations[stIx].repeaterId == 0) || (!repeaterPT && !repeaterCrcTried && stations[stIx].repeaterId != 0))
    {
#ifdef DEBUG
      Serial.println("Data received but station not configured");
#endif
      setChannel(CHANNEL);
      return;
    }

    if (stationsFound < numStations && stations[stIx].interval == 0)
    {
      stations[stIx].interval = (41 + id) * timeBase / 16; // Davis' official tx interval in us
      stationsFound++;
      if (lostStations > 0)
        lostStations--;
    }

    if (stations[stIx].active)
    {
      packets++;
      stations[stIx].packets++;
      fifo.queue((byte *)DATA, CHANNEL, -RSSI, FEI, stations[curStation].lastSeen > 0 ? lastRx - stations[curStation].lastSeen : 0);
    }

    stations[stIx].lostPackets = 0;
    stations[stIx].lastRx = stations[stIx].lastSeen = lastRx;
    stations[stIx].channel = nextChannel(CHANNEL);

    nextStation(); // skip curStation to next station expected to tx

    if (stationsFound < numStations && stations[curStation].lastRx + stations[curStation].interval - lastRx > DISCOVERY_MINGAP)
    {
      setChannel(discChannel);
    }
    else
    {
      setChannel(stations[curStation].channel); // reset current radio channel
    }
  }
  else
  {
#ifdef DEBUG
    Serial.println("Wrong CRC");
#endif
    setChannel(CHANNEL); // this always has to be done somewhere right after reception, even for ignored/bogus packets
  }
}

// Calculate the next hop of the specified channel
byte DavisCC1101::nextChannel(byte channel)
{
  return ++channel % getBandTabLength();
}

// Find the station index in stations[] for station expected to tx the earliest and update curStation
void DavisCC1101::nextStation()
{
  uint32_t earliest = 0xffffffff;
  uint32_t now = micros();
  for (int i = 0; i < numStations; i++)
  {
    uint32_t current = stations[i].lastRx + stations[i].interval - now;
    if (stations[i].interval > 0 && current < earliest)
    {
      earliest = current;
      curStation = i;
    }
  }
}

// Find station index in stations[] for a station ID (-1 if doesn't exist)
int DavisCC1101::findStation(byte id)
{
  for (byte i = 0; i < numStations; i++)
  {
    if (stations[i].id == id)
      return i;
  }
  return -1;
}

// Reset station array to safe defaults
void DavisCC1101::initStations()
{
  for (byte i = 0; i < numStations; i++)
  {
    stations[i].channel = 0;
    stations[i].lastRx = 0;
    stations[i].interval = 0;
    stations[i].lostPackets = 0;
    stations[i].lastRx = 0;
    stations[i].lastSeen = 0;
    stations[i].packets = 0;
    stations[i].missedPackets = 0;
  }
}

void DavisCC1101::interruptHandler()
{
#ifdef DEBUG
  Serial.println("interruptHandler");
#endif

  if (txMode)
    return;

  RSSI = readRSSI();           // Read up front when it is most likely the carrier is still up
  if (_mode == CC1101_MODE_RX) // && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    //FEI = word(readReg(REG_FEIMSB), readReg(REG_FEILSB));
    setMode(CC1101_MODE_IDLE);
#ifdef DEBUG
    Serial.print("RX FIFO length: ");
    Serial.println(readReg(kRXBYTES));
    Serial.print("Receive the packet from RX FIFO: ");
#endif
    select(); // Select RFM69 module, disabling interrupts
    SPI.transfer(kFIFO | kSPIHeaderRWBit | kSPIHeaderBurstBit);
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++)
    {
      DATA[i] = reverseBits(SPI.transfer(0));
#ifdef DEBUG
      Serial.print(DATA[i], HEX);
#endif
    }
#ifdef DEBUG
    Serial.println();
#endif
    _packetReceived = true;
    handleRadioInt();
    sendStrobe(kSFRX);
    unselect(); // Unselect CC1101 module, enabling interrupts
  }
}

bool DavisCC1101::canSend()
{
  if (_mode == CC1101_MODE_RX && readRSSI() < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(CC1101_MODE_IDLE);
    return true;
  }
  return false;
}

void DavisCC1101::setTxMode(bool mode)
{
  /*
  if (mode)
  {
    txMode = mode;
    writeReg(REG_FDEVMSB, RF_FDEVMSB_10000);
    writeReg(REG_FDEVLSB, RF_FDEVLSB_10000);
    writeReg(REG_PREAMBLELSB, 0);
    writeReg(REG_SYNCCONFIG, RF_SYNC_OFF);
    // +13 = 4 bytes "carrier" (0xff) + 6 bytes preamble (0xaa) + 3 bytes carrier
    writeReg(REG_PAYLOADLENGTH, DAVIS_PACKET_LEN + 13);
    writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | (DAVIS_PACKET_LEN + 13 - 1));
  }
  else
  {
    writeReg(REG_FDEVMSB, RF_FDEVMSB_9900);
    writeReg(REG_FDEVLSB, RF_FDEVLSB_9900);
    writeReg(REG_PREAMBLELSB, 4);
    writeReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2);
    writeReg(REG_PAYLOADLENGTH, DAVIS_PACKET_LEN);
    txMode = mode;
  }
  */
}

// IMPORTANT: make sure buffer is at least 10 bytes
void DavisCC1101::send(const byte *buffer, byte channel)
{
#ifndef CC1101_TX_ONLY
  MsTimer2::stop();
#endif
  setTxMode(true);
  //writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  setChannel(channel);
  sendFrame(buffer);
  setTxMode(false);
  setChannel(stations[curStation].channel);
#ifndef CC1101_TX_ONLY
  MsTimer2::start();
#endif
}

// IMPORTANT: make sure buffer is at least 6 bytes
void DavisCC1101::sendFrame(const byte *buffer)
{
  setMode(CC1101_MODE_IDLE); // turn off receiver to prevent reception while filling fifo
  uint32_t t = micros();
  // Wait for ModeReady using 15 ms timeout
  while ((readReg(kMARCSTATE) & 0x01) == 0x00)
  {
    if (micros() - t > SPI_OP_TIMEOUT)
      return;
  }
  //writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"

  // calculate crc on first 6 bytes (no repeater info)
  uint16_t crc = crc16_ccitt((volatile byte *)buffer, 6);
  // write to FIFO
  select();
  SPI.transfer(kFIFO | kSPIHeaderBurstBit);

  // The following byte sequence is an attempt to emulate the original Davis packet structure.

  // // carrier on/start
  // SPI.transfer(0xff);
  // SPI.transfer(0xff);
  // SPI.transfer(0xff);
  // SPI.transfer(0xff);

  // // preamble
  // SPI.transfer(0xaa);
  // SPI.transfer(0xaa);
  // SPI.transfer(0xaa);
  // SPI.transfer(0xaa);
  // SPI.transfer(0xaa);
  // SPI.transfer(0xaa);

  // // sync word
  // SPI.transfer(0xcb);
  // SPI.transfer(0x89);

  // make sure we use the correct transmitter ID
  byte byte0 = buffer[0] & 0xf0 | txId;
  SPI.transfer(reverseBits(byte0));

  // transmit the remaining bytes of the buffer
  for (byte i = 1; i < 6; i++)
    SPI.transfer(reverseBits(buffer[i]));

  // transmit crc of first 6 bytes
  SPI.transfer(reverseBits(crc >> 8));
  SPI.transfer(reverseBits(crc & 0xff));

  // transmit dummy repeater info (always 0xff, 0xff for a normal packet without repeater)
  SPI.transfer(0xff);
  SPI.transfer(0xff);

  // add one extra for stability
  SPI.transfer(0xff);

  unselect();

  setMode(CC1101_MODE_TX);
  t = micros(); // wait SPI_OP_TIMEOUT ms max (transmission time is about 9.5 ms)
  while (digitalRead(_interruptPin) == 0 && micros() - t < SPI_OP_TIMEOUT)
    ; // wait for DIO0 to turn HIGH signalling transmission finish
  setMode(CC1101_MODE_IDLE);
  //writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
}

void DavisCC1101::setChannel(byte channel)
{
  CHANNEL = channel;

  byte a = pgm_read_byte(&bandTab[band][CHANNEL][0]);
  byte b = pgm_read_byte(&bandTab[band][CHANNEL][1]);
  byte c = pgm_read_byte(&bandTab[band][CHANNEL][2]);
  /*
  if (freqCorr != 0)
  {
    uint32_t x = ((uint32_t)a << 16) | ((uint32_t)b << 8) | (uint32_t)c;
    x += freqCorr;
    c = x & 0x0000ff;
    b = (x & 0x00ff00) >> 8;
    a = (x & 0xff0000) >> 16;
  }
*/
  writeReg(kFREQ2, a);
  writeReg(kFREQ1, b);
  writeReg(kFREQ0, c);

  // Clear the RX FIFO in case it was somehow underflowed.
  sendStrobe(kSFRX);
  sendStrobe(kSRX);

  if (!txMode)
  {
    receiveBegin();
    setRssiThresholdRaw(rxRssi);
  }
}

// The data bytes come over the air from the ISS least significant bit first. Fix them as we go. From
// http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
byte DavisCC1101::reverseBits(byte b)
{
  b = ((b & 0b11110000) >> 4) | ((b & 0b00001111) << 4);
  b = ((b & 0b11001100) >> 2) | ((b & 0b00110011) << 2);
  b = ((b & 0b10101010) >> 1) | ((b & 0b01010101) << 1);

  return (b);
}

// Davis CRC calculation from http://www.menie.org/georges/embedded/
uint16_t DavisCC1101::crc16_ccitt(volatile byte *buf, byte len, uint16_t initCrc)
{
  uint16_t crc = initCrc;
  while (len--)
  {
    int i;
    crc ^= *(char *)buf++ << 8;
    for (i = 0; i < 8; ++i)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

void DavisCC1101::setMode(byte newMode)
{
  if (newMode == _mode)
    return;
#ifdef DEBUG
  Serial.print("Set mode: ");
  Serial.println(newMode, HEX);
#endif

  switch (newMode)
  {
  case CC1101_MODE_TX:
    sendStrobe(kSFTX);
    sendStrobe(kSTX);
  case CC1101_MODE_RX:
    sendStrobe(kSFRX);
    sendStrobe(kSRX);
    break;
  case CC1101_MODE_SYNTH:
    sendStrobe(kSFSTXON);
    break;
  case CC1101_MODE_IDLE:
    sendStrobe(kSIDLE);
    break;
  case CC1101_MODE_SLEEP:
    sendStrobe(kSPWD);
    break;
  default:
    return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == CC1101_MODE_SLEEP && (readReg(kMARCSTATE) & 0x00) == 0x00)
    ; // Wait for ModeReady

  _mode = newMode;
}

void DavisCC1101::sleep()
{
  setMode(CC1101_MODE_SLEEP);
}

void DavisCC1101::isr0() { selfPointer->interruptHandler(); }

void DavisCC1101::receiveBegin()
{
  _packetReceived = false;
  // if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
  //   writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

  //writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(CC1101_MODE_RX);
}

bool DavisCC1101::receiveDone()
{
  return _packetReceived;
}

void DavisCC1101::setRssiThreshold(int rssiThreshold)
{
  setRssiThresholdRaw(abs(rssiThreshold) << 1);
}

void DavisCC1101::setRssiThresholdRaw(int rssiThresholdRaw)
{
  /*
  writeReg(REG_RSSIVALUE, rssiThresholdRaw);
  rxRssi = rssiThresholdRaw;
  */
}

int DavisCC1101::readRSSI(bool forceTrigger)
{
  int rssi = 0;
  /*
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00)
      ; // Wait for RSSI_Ready

  }
        */
  // check
  // 1) Read the RSSI status register
  // 2) Convert the reading from a hexadecimal
  // number to a decimal number (RSSI_dec)
  // 3) If RSSI_dec ≥ 128 then RSSI_dBm = (RSSI_dec - 256)/2 – RSSI_offset
  // 4) Else if RSSI_dec < 128 then RSSI_dBm = (RSSI_dec)/2 – RSSI_offset
  rssi = -readReg(kRSSI);
  rssi >>= 1;
  return rssi;
}

byte DavisCC1101::readReg(byte addr)
{
  select();
  byte cc1101_status_byte_ = SPI.transfer(addr | kSPIHeaderRWBit);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

void DavisCC1101::writeReg(byte addr, byte value)
{
  select();
  byte cc1101_status_byte_ = SPI.transfer(addr);
  SPI.transfer(value);
  unselect();
}

void DavisCC1101::sendStrobe(byte strobe)
{
  select();
  byte regval = SPI.transfer(strobe);
  unselect();
}

void DavisCC1101::sendStrobeWithReadBit(byte strobe)
{
  select();
  byte regval = SPI.transfer(strobe | kSPIHeaderRWBit);
  unselect();
}

// Select the transceiver
void DavisCC1101::select()
{
  noInterrupts();
  digitalWrite(_slaveSelectPin, LOW);
}

// Unselect the transceiver chip
void DavisCC1101::unselect()
{
  digitalWrite(_slaveSelectPin, HIGH);
  interrupts();
}

// from -30 to 10 dBm
void DavisCC1101::setPowerLevel(byte dBm)
{
  uint8_t pa = 0xC0;
  _powerLevel = dBm;

  if (dBm <= -30)
    pa = 0x00;
  else if (dBm <= -20)
    pa = 0x01;
  else if (dBm <= -15)
    pa = 0x02;
  else if (dBm <= -10)
    pa = 0x03;
  else if (dBm <= 0)
    pa = 0x04;
  else if (dBm <= 5)
    pa = 0x05;
  else if (dBm <= 7)
    pa = 0x06;
  else if (dBm <= 10)
    pa = 0x07;

  writeReg(kFREND0, pa);
}

// return stored _powerLevel
uint8_t DavisCC1101::getPowerLevel()
{
  return _powerLevel;
}

void DavisCC1101::setCS(byte newSPISlaveSelect)
{
  _slaveSelectPin = newSPISlaveSelect;
  pinMode(_slaveSelectPin, OUTPUT);
}

void DavisCC1101::readAllRegs()
{
  byte regVal;
  for (byte regAddr = 1; regAddr <= 0x3F; regAddr++)
  {
    select();
    regVal = readReg(regAddr);
    unselect();

    if (!(regAddr & 0xf0))
      Serial.print('0');
    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    if (!(regVal & 0xf0))
      Serial.print('0');
    Serial.print(regVal, HEX);
    Serial.print(" - ");
    Serial.println(regVal, BIN);
  }
}

byte DavisCC1101::readTemperature(byte calFactor) // returns centigrade
{
  // PTEST but only on RX/TX otherwise
  // write 0xBF to PTEST, read and
  // then write PTEST to 0x7F
  /*
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING))
    Serial.print('*');
  // 'complement'corrects the slope, rising temp = rising val
  // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor;
  */
}

void DavisCC1101::rcCalibration()
{
  /*
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00)
    ;
    */
}

void DavisCC1101::setBand(byte newBand)
{
  band = newBand;
}

void DavisCC1101::setBandwidth(byte bw)
{
  /*
  switch (bw)
  {
  case RF69_DAVIS_BW_NARROW:
    writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4);  // Use 25 kHz BW (BitRate < 2 * RxBw)
    writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use double the bandwidth during AFC as reception
    break;
  case RF69_DAVIS_BW_WIDE:
    // REG_RXBW 50 kHz fixes console retransmit reception but seems worse for SIM transmitters (to be confirmed with more testing)
    writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3);  // Use 50 kHz BW (BitRate < 2 * RxBw)
    writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_2); // Use double the bandwidth during AFC as reception
    break;
  default:
    return;
  }
  */
}

byte DavisCC1101::getBandTabLength()
{
  return bandTabLengths[band];
}

void DavisCC1101::setFreqCorr(int16_t value)
{
  freqCorr = value * 1000.0 / CC1101_FSTEP;
}

void DavisCC1101::enableTx(void (*function)(byte *buffer), byte ID)
{
  txCallback = function;
  txId = ID;
  txChannel = 0;
  txDelay = (41 + txId) * timeBase >> 4;
  lastTx = micros();
  Timer1.initialize(txDelay);
  Timer1.attachInterrupt(DavisCC1101::handleTxInt, txDelay);
}

void DavisCC1101::disableTx()
{
  Timer1.detachInterrupt();
  txCallback = NULL;
  txId = 255;
  txChannel = 255;
  txDelay = 0;
}

void DavisCC1101::setTimeBase(uint32_t value)
{
  timeBase = value;
}

void DavisCC1101::setRepeaterPT(bool value)
{
  repeaterPT = value;
}
