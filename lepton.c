/*=========================================================================
| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
| FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
| COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
| BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
| LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
| CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
| LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
| ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
| POSSIBILITY OF SUCH DAMAGE.
 ========================================================================*/
/*
 * lepton.c
 *
 *  Created on: Jun 6, 2013
 *      Author: rbaltzer
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "aardvark.h"
#include "main.h"
#include "lepton.h"
#include "crc.h"


#define BUFFER_SIZE      	(65535)
#define SLAVE_RESP_SIZE     (26)


#define VOSPI_PACKET_SIZE  	(164)
#define VOSPI_HEADER_BYTES	(4)
#define H_LINE_BYTES		(VOSPI_PACKET_SIZE - VOSPI_HEADER_BYTES) // 80 pixels per line, 2 bytes each = 160
#define MAX_PAYLOAD_SIZE	(164)

#define FRAMES_PER_SECOND   (9)
#define USECONDS_PER_FRAME	(1000000/FRAMES_PER_SECOND)

#define H_PIXELS			(80)
#define V_LINES				(60)

#define STATE(x) 			leptonData.state=x
#define IN_DATA				leptonData.bytesIn
#define OUT_DATA			leptonData.bytesOut
#define IDX					leptonData.inIndex
#define ODX					leptonData.outIndex
#define FB					leptonData.frameBuffer
#define FBX					leptonData.frameBufferIndex
#define LINE				leptonData.line

//
// VoSPI IDs (0xFF00 - 0xFFFF)
//
// 		0xFFFF - Start of Frame (SOF)
//		0xFFFE - End of Frame (EOF)
//		0xFFFD - Alternate stream
//		0xFFFC - Discard packet
//		0xFFF8 - 0xFFFB - Pallette Header Packet
//		0xFF00 - 0xFFF7 Reserved

// VoSPI Packet IDs
typedef enum {
	idPallette 			= 0xFFF8, // Thru 0xFFFB?
	idDiscard 			= 0xFFFC,
	idAlternateStream 	= 0xFFFD,
	idEOF 				= 0xFFFE,
	idSOF 				= 0xFFFF,
} VoSpiPacketId;

#define ID_BYTE_POSITION	(0)
#define CRC_BYTE_POSITION	(2)

enum {
	sofID 				= 0,
	sofCRC 				= 2,
	sofNumPayloadBytes 	= 4,
	sofHeaderType	 	= 6,
	sofPixelFormat	 	= 7,
} SofBytePosition;



enum {
    spiMODE_POL0_PHASE0 = 0,
    spiMODE_POL0_PHASE1 = 1,
    spiMODE_POL1_PHASE0 = 2,
    spiMODE_POL1_PHASE1 = 3,
} SpiMode;

/*
 * VoSPI Physical Implementation
 *
 * VoSPI uses a modified Serial Peripheral Interface (SPI) to carry the data.
 * The SCK (Serial Clock), /CS (Chip Select, active low), and MISO (Master In/Slave Out)
 * signals are present, but VoSPI does not use the MOSI (Master Out/Slave In) signal.
 *
 * Implementations are generally restricted to a single master and a single slave,
 * due to the strength of the line drivers in the hardware.
 * The Lepton operates as a slave port only. The host controller must operate as an
 * SPI master to generate the SCK (Serial Clock) signal.
 *
 * The Lepton uses SPI Mode 3 (CPOL=1, CPHA=1). SCK is high when idle.
 * Data is set up by the Lepton on the falling edge of SCK, and should be sampled by the
 * host controller on the rising edge of SCK.
 *
 * Data is transferred most-significant byte first and in big-endian order
 */

//
//	Psuedocode:
//
//  Note: In general, if an error occurs (e.g. CRC is wrong), the state machine will go to step 1 and
//        start over. This might be changed later depending on data reliability.
//
//	1. Start reading bytes until we see SOF ID (0xFFFF).
//	2. Once we see the ID, capture the 164 bytes, calc CRC and compare, if valid,
//		continue otherwise back to step 1.
//  3. Capture # (Total Number of Lines from SOF) Video Packets
//  4. Capture EOF.
//  5. Alert any observers that a frame is complete
//  5. Deal with timing and go to step 2.
//

static void readSpiBytes(u08 NumBytes);
static bool m_stateInit(void);
static void words2bytes(u16* words, u08* bytes, u16 numWords);
static void buildStream(void);
static void calcCrc(void); //debug

typedef enum {
	hdrIgnore 	= 0,
	hdrSOF	  	= 1,
	hdrEOF    	= 2,
	hdrPallet0  = 16,
	hdrPallet1,
	hdrPallet2,
	hdrPallet3,
	hdrPallet4,
	hdrPallet5,
	hdrPallet6,
	hdrPallet7,
	hdrPallet8,
	hdrPallet9,
	hdrPallet10,
	hdrPallet11,
	hdrPallet12,
	hdrPallet13,
	hdrPallet14,
	hdrPallet15,
}HeaderType;

typedef struct {
	s16 port;
    Aardvark handle;
	u08 state;
	u08* bytesIn;	// TODO: Make sure pointed at buffer VOSPI_PACKET_SIZE
	u16 crc;
	u16 payloadBytes;
	HeaderType headerType;

	u16 inIndex;
	u16 outIndex;
	u08* bytesOut;	// Note: This is used primarily as a debug tool w/ MOSI & MISO tied together the data loops back.

	u08 frameBuffer[H_PIXELS*V_LINES];
	u16 frameBufferIndex;
	u08 line;
}LeptonData;

static u08 inData[VOSPI_PACKET_SIZE];

// LoopBack SOF Packet (VoSPI is big-endian)
static u08 sofPacket[VOSPI_PACKET_SIZE] =
{
		0xFF, 0xFF,		// ID  				= 0xFFFF
		0xDE, 0xAD,		// CRC 				= 0xDEAD
		0x00, 0xA4,		// PAYLOAD BYTES 	= 0xA4 = 164
		hdrSOF			// HEADER TYPE		= hdrSOF = 1
};

static u16 line0Packet[VOSPI_PACKET_SIZE/2] =
{
	0x3000, 0x8c08, 0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x000e,
	0x000f, 0x0010, 0x0011, 0x0012, 0x0013, 0x0014, 0x0015, 0x0016, 0x0017, 0x0018, 0x0019, 0x001a, 0x001b, 0x001c, 0x001d, 0x001e, 0x001f,
	0x0020, 0x0021, 0x0022, 0x0023, 0x0024, 0x0025, 0x0026, 0x0027, 0x0028, 0x0029, 0x002a, 0x002b, 0x002c, 0x002d, 0x002e, 0x002f, 0x0030,
	0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037, 0x0038, 0x0039, 0x003a, 0x003b, 0x003c, 0x003d, 0x003e, 0x003f, 0x0040, 0x0041,
	0x0042, 0x0043, 0x0044, 0x0045, 0x0046, 0x0047, 0x0048, 0x0049, 0x004a, 0x004b, 0x004c, 0x004d, 0x004e, 0x004f };

static u16 line59Packet[VOSPI_PACKET_SIZE/2] =
{
	0x003b, 0x48d0, 0x170c, 0x170d, 0x170e, 0x170f, 0x1710, 0x1711, 0x1712, 0x1713, 0x1714, 0x1715, 0x1716, 0x1717, 0x1718, 0x1719, 0x171a,
	0x171b, 0x171c, 0x171d, 0x171e, 0x171f, 0x1720, 0x1721, 0x1722, 0x1723, 0x1724, 0x1725, 0x1726, 0x1727, 0x1728, 0x1729, 0x172a, 0x172b,
	0x172c, 0x172d, 0x172e, 0x172f, 0x1730, 0x1731, 0x1732, 0x1733, 0x1734, 0x1735, 0x1736, 0x1737, 0x1738, 0x1739, 0x173a, 0x173b, 0x173c,
	0x173d, 0x173e, 0x173f, 0x1740, 0x1741, 0x1742, 0x1743, 0x1744, 0x1745, 0x1746, 0x1747, 0x1748, 0x1749, 0x174a, 0x174b, 0x174c, 0x174d,
	0x174e, 0x174f, 0x1750, 0x1751, 0x1752, 0x1753, 0x1754, 0x1755, 0x1756, 0x1757, 0x1758, 0x1759, 0x175a, 0x175b
};

static u16 discardPacket[VOSPI_PACKET_SIZE/2] =
{
	0x1fff, 0xffff, 0xdcd0, 0xdcad, 0xdcd2, 0xdcad, 0xdcd4, 0xdcad, 0xdcd6, 0xdcad, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2ebc, 0x3b00,
	0x1042, 0x085e, 0x0100, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

// Five packets with 4 random bytes prepended used to emulate
// a "real world" byte stream coming from a Lepton
static u08 emulationStream[(VOSPI_PACKET_SIZE*5) + 4];

LeptonData leptonData;

void leptonInit(void)
{
	// MOSI is tied to MISO so we can feed bytes we want to see
	IN_DATA  = inData;
//	OUT_DATA = sofPacket;

	STATE(stateIdle);
}

void leptonStart(void)
{
	// Build emulation stream that gets looped back MOSI->MISO
	buildStream();
	OUT_DATA = emulationStream;

	STATE(stateInit);
	ODX = 0;
	printf("leptonStart\r\n");
}

// The state machine starts by trying to identify a SOF packet and syncing to it

void leptonStateMachine(void)
{
	switch (leptonData.state) {
	case stateIdle:
		// Do nothing
		break;

	case stateInit:
		if (m_stateInit()) {
			STATE(stateFindIDFirst);
		}
		else {
			STATE(stateError);
		}
		break;

	case stateDelayFourFrames:
		usleep(4*USECONDS_PER_FRAME);
		STATE(stateFindIDFirst);
		break;

	// Look for XF in first byte
	case stateFindIDFirst:
		IDX = 0;
		readSpiBytes(1);
		if ((IN_DATA[ID_BYTE_POSITION] & 0x0F) == 0x0F) {
			STATE(stateFindIDSecond);
		}
		else {
			STATE(stateDelayFourFrames);
		}
		break;

	// Look for FX in second byte
	case stateFindIDSecond:
		readSpiBytes(1);
		if ((IN_DATA[ID_BYTE_POSITION+1] & 0xF0) == 0xF0) {
			STATE(stateGetRemaining);
		}
		else {
			STATE(stateDelayFourFrames);
		}
		break;

	// Read the rest of the packet
	case stateGetRemaining:
		readSpiBytes(VOSPI_PACKET_SIZE-2);
		STATE(stateReadPacket);
		break;

	// Read next packet
	case stateReadPacket:
		IDX = 0;
		readSpiBytes(VOSPI_PACKET_SIZE);

		// Check if ID field looks like video packet on HLine 0
		if (((IN_DATA[ID_BYTE_POSITION] & 0x0F) == 0) &&
				((IN_DATA[ID_BYTE_POSITION+1]) == 0)) {
			LINE = 0;
			STATE(stateCheckCRC);
		}

		// Check if ID field looks like discard packet
		else if (((IN_DATA[ID_BYTE_POSITION] & 0x0F) == 0x0F) &&
				((IN_DATA[ID_BYTE_POSITION+1] & 0xF0) == 0xF0)) {
			// Keep reading packets
		}
		else {
			STATE(stateDelayFourFrames);
		}

		// If Discard, read another packet,
		// If video packet, check CRC, store first line and state = readvideopackets
		// else goto stateDelayFourFrames
		break;

	case stateCheckCRC:
	{
		u16 calcCRC, packetCRC;

		packetCRC = (u16) IN_DATA[CRC_BYTE_POSITION] << 8;
		packetCRC = packetCRC + (u16) IN_DATA[CRC_BYTE_POSITION + 1];

		//Zero out bytes
		IN_DATA[0] = 0;		// T TODO: Do something with T
		IN_DATA[2] = 0;		// CRC16 MSB
		IN_DATA[3] = 0;		// CRC16 LSB

		calcCRC = fast_crc16(0x0, IN_DATA, VOSPI_PACKET_SIZE);
		if (packetCRC != calcCRC) {
			STATE(stateDelayFourFrames);
		}
		else {
			memcpy(&FB[LINE*H_LINE_BYTES], &IN_DATA[VOSPI_HEADER_BYTES], H_LINE_BYTES);
			LINE = LINE + 1;
//			if (LINE == V_LINES) {	// TODO: switch back once we have a real camera
			if (LINE == 2) {
				STATE(stateDelayBetweenFrames);
			}
			else {
				STATE(stateReadLineOfVideo);
			}
		}
	}
		break;

	case stateReadLineOfVideo:
		IDX = 0;
		readSpiBytes(VOSPI_PACKET_SIZE);
		STATE(stateCheckCRC);
		break;

	case stateDelayBetweenFrames:
		// TODO: Let someone know we have a frame
		usleep(USECONDS_PER_FRAME); // TODO: Really?
		STATE(stateReadPacket);
		printf("Frame RX'd lines %d \r\n", LINE);
		exit(0);
		break;

	case stateError:
		printf("LeptonStateMachine: Ended in ERROR");
		STATE(stateIdle);
		break;

	default:
		printf("ERROR: stateMachine(): Hit default state\r");
		break;
	}
}

///////////////////////////////////
// State Machine Functions
///////////////////////////////////

static bool m_stateInit(void){
    int mode       = spiMODE_POL1_PHASE1;
    u08 i, slave_resp[SLAVE_RESP_SIZE];

	leptonData.port = aadetect();

	if (leptonData.port == -1) {
		return false;
	}

    // Open the device
    leptonData.handle = aa_open(leptonData.port);
    if (leptonData.handle <= 0) {
        printf("Unable to open Aardvark device on port %d\n", leptonData.port);
        printf("Error code = %d\n", leptonData.handle);
        return false;
    }

    // Ensure that the SPI subsystem is enabled
    aa_configure(leptonData.handle, AA_CONFIG_SPI_I2C);

    // Disable the Aardvark adapter's power pins.
    // This command is only effective on v2.0 hardware or greater.
    // The power pins on the v1.02 hardware are not enabled by default.
    aa_target_power(leptonData.handle, AA_TARGET_POWER_NONE);

    // Setup the clock phase
    aa_spi_configure(leptonData.handle, mode >> 1, mode & 1, AA_SPI_BITORDER_MSB);

    // Set the slave response
    for (i=0; i<SLAVE_RESP_SIZE; ++i)
        slave_resp[i] = 'A' + i;

    aa_spi_slave_set_response(leptonData.handle, SLAVE_RESP_SIZE, slave_resp);

    // Enable the slave
    aa_spi_slave_enable(leptonData.handle);
    return true;
}

///////////////////////////////////
// Helper Functions
///////////////////////////////////

// Read NumBytes bytes via SPI
static void readSpiBytes(u08 NumBytes)
{
    aa_spi_write(leptonData.handle, NumBytes, &OUT_DATA[ODX], NumBytes, &IN_DATA[IDX]);
    IDX += NumBytes;
    ODX += NumBytes;
}


void getVoSPIPacket(void) {
	//Reads 164 bytes of data
}

// Big endian
static void words2bytes(u16* words, u08* bytes, u16 numWords)
{
	u16 i;

	for (i = 0 ; i < numWords ; i++) {
		bytes[(i*2)] = (*words) >> 8;
		bytes[(i*2) + 1] = (u08) ((*words++) & 0xff);
	}
}

static void buildStream(void)
{
	u08 discardPacketBytes[VOSPI_PACKET_SIZE];
	u08 line0PacketBytes[VOSPI_PACKET_SIZE];
	u08 line59PacketBytes[VOSPI_PACKET_SIZE];
	u08 randomBytes[4] = {0xf1, 0xfe, 0, 9};

	words2bytes(line0Packet, line0PacketBytes, VOSPI_PACKET_SIZE/2);
	words2bytes(line59Packet, line59PacketBytes, VOSPI_PACKET_SIZE/2);
	words2bytes(discardPacket, discardPacketBytes, VOSPI_PACKET_SIZE/2);

	// Build the emulation stream
	memcpy(&emulationStream[(0*VOSPI_PACKET_SIZE)], randomBytes, 4);
	memcpy(&emulationStream[(0*VOSPI_PACKET_SIZE)+4], discardPacketBytes, VOSPI_PACKET_SIZE);
	memcpy(&emulationStream[(1*VOSPI_PACKET_SIZE)+4], discardPacketBytes, VOSPI_PACKET_SIZE);
	memcpy(&emulationStream[(2*VOSPI_PACKET_SIZE)+4], line0PacketBytes, VOSPI_PACKET_SIZE);
	memcpy(&emulationStream[(3*VOSPI_PACKET_SIZE)+4], line59PacketBytes, VOSPI_PACKET_SIZE);
	printf("done\r\n");
}

// Debug function to make sure CRC-16 is working.
// Packets are calc'd w/ CRC and T fields set to zero.
static void calcCrc(void)
{
	u08 localPacket[VOSPI_PACKET_SIZE];
	u16 crc;

	// Convert words to bytes
	words2bytes(line0Packet, localPacket, VOSPI_PACKET_SIZE/2);

	//Zero out bytes
	localPacket[0] = 0;		// T
	localPacket[2] = 0;		// CRC16 MSB
	localPacket[3] = 0;		// CRC16 LSB

	crc = fast_crc16(0x0, localPacket, VOSPI_PACKET_SIZE);
	printf("crc = 0x%x\r\n", crc);
}
