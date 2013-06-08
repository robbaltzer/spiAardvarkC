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

typedef struct {
	u08 state;
}LeptonData;

typedef enum {
	stateSyncFindID,
	stateSyncCheckPayload,
	stateSyncCheckHeaderType,
	stateSyncCheckPixelFormat,
}LeptonState;

LeptonData leptonData;

void stateMachine(void) {

	switch (leptonData.state) {
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

static u08 readSpiByte(void)
{

}

void getVoSPIPacket(void) {
	//Reads 164 bytes of data
}
