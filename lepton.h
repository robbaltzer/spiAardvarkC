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
 * lepton.h
 *
 *  Created on: Jun 6, 2013
 *      Author: rbaltzer
 */

#ifndef __lepton_h
#define __lepton_h

typedef enum {
	stateIdle,
	stateStart,
	stateDelayFourFrames,
	stateFindIDFirst,
	stateFindIDSecond,
	stateGetRemaining,
	stateReadPacket,
	stateReadLineOfVideo,
	stateCheckCRC,
	stateDelayBetweenFrames,
	stateError,
}LeptonState;

void leptonStateMachine(void);
void leptonInit(void);
void leptonStart(void);
bool spiInit(void);
#endif
