#ifndef DAVISCC1101registers_h
#define DAVISCC1101registers_h

// The CC1101-CC1190 EM has a 26 MHz crystal.
#define kCrystalFrequency 26e6
#define kSPIHeaderRWBit 0x80
#define kSPIHeaderBurstBit 0x40

// CC1101 Registers
#define kIOCFG2 0x00
#define kIOCFG1 0x01
#define kIOCFG0 0x02
#define kFIFOTHR 0x03
#define kSYNC1 0x04
#define kSYNC0 0x05
#define kPKTLEN 0x06
#define kPKTCTRL1 0x07
#define kPKTCTRL0 0x08
#define kADDR 0x09
#define kCHANNR 0x0A
#define kFSCTRL1 0x0B
#define kFSCTRL0 0x0C
#define kFREQ2 0x0D
#define kFREQ1 0x0E
#define kFREQ0 0x0F
#define kMDMCFG4 0x10
#define kMDMCFG3 0x11
#define kMDMCFG2 0x12
#define kMDMCFG1 0x13
#define kMDMCFG0 0x14
#define kDEVIATN 0x15
#define kMCSM2 0x16
#define kMCSM1 0x17
#define kMCSM0 0x18
#define kFOCCFG 0x19
#define kBSCFG 0x1A
#define kAGCCTRL2 0x1B
#define kAGCCTRL1 0x1C
#define kAGCCTRL0 0x1D
#define kWOREVT1 0x1E
#define kWOREVT0 0x1F
#define kWORCTRL 0x20
#define kFREND1 0x21
#define kFREND0 0x22
#define kFSCAL3 0x23
#define kFSCAL2 0x24
#define kFSCAL1 0x25
#define kFSCAL0 0x26
#define kRCCTRL1 0x27
#define kRCCTRL0 0x28
#define kFSTEST 0x29
#define kPTEST 0x2A
#define kAGCTEST 0x2B
#define kTEST2 0x2C
#define kTEST1 0x2D
#define kTEST0 0x2E
#define kFIFO 0x3F
// Status Registers.
// Same addresses as strobes, but with the burst bit set.  Read only.
#define kPARTNUM 0xF0
#define kVERSION 0xF1
#define kFREQEST 0xF2
#define kLQI 0xF3
#define kRSSI 0xF4
#define kMARCSTATE 0xF5
#define kWORTIME1 0xF6
#define kWORTIME0 0xF7
#define kPKTSTATUS 0xF8
#define kVCO_VC_DAC 0xF9
#define kTXBYTES 0xFA
#define kRXBYTES 0xFB
#define kRCCTRL1_STATUS 0xFC
#define kRCCTRL0_STATUS 0xFD

// Command Strobes
#define kSRES 0x30
#define kSFSTXON 0x31
#define kSXOFF 0x32
#define kSCAL 0x33
#define kSRX 0x34
#define kSTX 0x35
#define kSIDLE 0x36
#define kSWOR 0x38
#define kSPWD 0x39
#define kSFRX 0x3A
#define kSFTX 0x3B
#define kSWORRST 0x3C
#define kSNOP 0x3D

#endif