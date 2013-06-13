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

#include "aardvark.h"
#include "main.h"
#include "lepton.h"

#define BUFFER_SIZE      	(65535)
#define SLAVE_RESP_SIZE     (26)
#define VOSPI_PACKET_SIZE  	(164)
#define MAX_PAYLOAD_SIZE	(164)

#define STATE(x) 			leptonData.state=x
#define IN_DATA				leptonData.bytesIn
#define OUT_DATA			leptonData.bytesOut
#define IDX					leptonData.inIndex
#define ODX					leptonData.outIndex

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

static u08 line59Packet[VOSPI_PACKET_SIZE] =
{

};

static u08 discardPacket[VOSPI_PACKET_SIZE] =
{

};

LeptonData leptonData;

void leptonInit(void)
{
	// MOSI is tied to MISO so we can feed bytes we want to see
	IN_DATA  = inData;
	OUT_DATA = sofPacket;

	STATE(stateIdle);
}

void leptonStart(void)
{
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
			STATE(stateSyncFindIDFirst);
		}
		else {
			STATE(stateError);
		}
		break;

	case stateSyncFindIDFirst:
		IDX = 0;
		readSpiBytes(1);
		if (IN_DATA[sofID] == (u08) (idSOF >> 8)) {
			STATE(stateSyncFindIDSecond);
		}
		break;

	case stateSyncFindIDSecond:
		readSpiBytes(1);
		if (IN_DATA[sofID+1] == (u08) (idSOF & 0x00FF)) {
			// Read 2 bytes to get the SOF packet CRC
			readSpiBytes(2);
			leptonData.crc = ((u16) IN_DATA[sofCRC]<<8) +
						      (u16) IN_DATA[sofCRC+1];
			if (leptonData.crc >= 0xFFF0) {
				STATE(stateSyncFindIDFirst);
			}
			else {
				printf("crc = 0x%x\r\n", leptonData.crc);
				STATE(stateSyncCheckPayload);
			}
		}
		else {
			STATE(stateSyncFindIDFirst);
		}
		break;

	case stateSyncCheckPayload:
		readSpiBytes(2);
		leptonData.payloadBytes = leptonData.crc = ((u16) IN_DATA[sofNumPayloadBytes]<<8) +
													(u16) IN_DATA[sofNumPayloadBytes+1];
		printf("Payload bytes %d\r\n", leptonData.payloadBytes);
		if (leptonData.payloadBytes <= MAX_PAYLOAD_SIZE) {
			STATE(stateSyncCheckHeaderType);
		}
		else {
			STATE(stateSyncFindIDFirst);
		}
		break;

	case stateSyncCheckHeaderType:
		readSpiBytes(1);
		leptonData.headerType = (HeaderType) IN_DATA[sofHeaderType];
		if (leptonData.headerType == hdrSOF) {
			STATE(stateSyncCheckPixelFormat);
		}
		else {
			STATE(stateSyncFindIDFirst);
		}
		break;

	case stateSyncCheckPixelFormat:
		printf("stateSyncCheckPixelFormat\r");
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
