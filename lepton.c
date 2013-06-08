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

//=========================================================================
// CONSTANTS
//=========================================================================
#define BUFFER_SIZE      65535

#define SLAVE_RESP_SIZE     26

//
// VoSPI IDs (0xFF00 - 0xFFFF)
//
// 		0xFFFF - Start of Frame (SOF)
//		0xFFFE - End of Frame (EOF)
//		0xFFFD - Alternate stream
//		0xFFFC - Discard packet
//		0xFFF8 - 0xFFFB - Pallette Header Packet
//		0xFF00 - 0xFFF7 Reserved

//
//	Psuedocode:
//
//  Note: In general, if an error occurs (e.g. CRC is wrong), the state machine will go to step 1 and
//        start over. This might be changed later depending on data reliablity.
//
//	1. Start reading bytes until we see SOF ID (0xFFFF).
//	2. Once we see the ID, capture the 164 bytes, calc CRC and compare, if valid,
//		continue otherwise back to step 1.
//  3. Capture # (Total Number of Lines from SOF) Video Packets
//  4. Capture EOF.
//  5. Alert any observers that a frame is complete
//  5. Deal with timing and go to step 2.
//

static u08 readSpiByte(void);
static bool m_stateInit(void);

typedef struct {
	s16 port;
    Aardvark handle;
	u08 state;
}LeptonData;

typedef enum {
	stateIdle,
	stateInit,
	stateSyncFindID,
	stateSyncCheckPayload,
	stateSyncCheckHeaderType,
	stateSyncCheckPixelFormat,
	stateError,
}LeptonState;

LeptonData leptonData;


void leptonInit(void)
{
	leptonData.state = stateIdle;
}

void leptonStart(void)
{
	leptonData.state = stateInit;
	printf("leptonStart\r\n");
}

void leptonStateMachine(void)
{
	switch (leptonData.state) {
	case stateIdle:
		// Do nothing
		break;

	case stateInit:
		printf("$");
		if (m_stateInit()) {
			leptonData.state = stateSyncFindID;
		}
		break;

	case stateSyncFindID:
		break;

	case stateSyncCheckPayload:
		break;

	case stateSyncCheckHeaderType:
		break;

	case stateSyncCheckPixelFormat:
		break;

	default:
		printf("ERROR: stateMachine(): Hit default state\r\n");
		break;
	}
}

///////////////////////////////////
// State Machine Functions
///////////////////////////////////

static bool m_stateInit(void){
    int mode       = 1;
    int timeout_ms = 500;
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

static u08 readSpiByte(void)
{

	return 0;
}

void getVoSPIPacket(void) {
	//Reads 164 bytes of data
}
