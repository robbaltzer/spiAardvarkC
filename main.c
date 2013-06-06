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

//=========================================================================
// INCLUDES
//=========================================================================
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

static s16 m_port;

static void dump (Aardvark handle, int timeout_ms);

int main (int argc, char *argv[])
{
	m_port = aadetect();
	if (m_port != -1) {
		readReadSPI(m_port);
	}

    return 0;
}

s16 aadetect(void)
{
    u16 ports[16];
    u32 unique_ids[16];
    int nelem = 16;

    // Find all the attached devices
    int count = aa_find_devices_ext(nelem, ports, nelem, unique_ids);
    int i;

    printf("%d device(s) found:\n", count);

    // Print the information on each device
    if (count > nelem)  count = nelem;
    for (i = 0; i < count; ++i) {
        // Determine if the device is in-use
        const char *status = "(avail) ";
        if (ports[i] & AA_PORT_NOT_FREE) {
            ports[i] &= ~AA_PORT_NOT_FREE;
            status = "(in-use)";
        }

        // Display device port number, in-use status, and serial number
        printf("    port=%-3d %s (%04d-%06d)\n",
               ports[i], status,
               unique_ids[i] / 1000000,
               unique_ids[i] % 1000000);
    }

    if (count > 0) {
    	return ports[0];
    }
    else {
    	return -1;
    }
}

static u08 data_in1[500];
static u08 data_out1[500];

void readReadSPI(u16 port)
{
    Aardvark handle;

    int mode       = 1;
    int timeout_ms = 500;

    u08 slave_resp[SLAVE_RESP_SIZE];
    int i;
    data_out1[0] = 0x01;
    data_out1[1] = 0x02;
    data_out1[2] = 0x03;
    data_out1[3] = 0x04;
    data_out1[4] = 0xDE;
    data_out1[5] = 0xAD;

//    if (argc < 4) {
//        printf("usage: aaspi_slave PORT MODE TIMEOUT_MS\n");
//        printf("  mode 0 : pol = 0, phase = 0\n");
//        printf("  mode 1 : pol = 0, phase = 1\n");
//        printf("  mode 2 : pol = 1, phase = 0\n");
//        printf("  mode 3 : pol = 1, phase = 1\n");
//        printf("\n");
//        printf("  The timeout value specifies the time to\n");
//        printf("  block until the first packet is received.\n");
//        printf("  If the timeout is -1, the program will\n");
//        printf("  block indefinitely.\n");
//        return 1;
//    }

    // Open the device
    handle = aa_open(port);
    if (handle <= 0) {
        printf("Unable to open Aardvark device on port %d\n", port);
        printf("Error code = %d\n", handle);
//        return 1;
    }

    // Ensure that the SPI subsystem is enabled
    aa_configure(handle, AA_CONFIG_SPI_I2C);

    // Disable the Aardvark adapter's power pins.
    // This command is only effective on v2.0 hardware or greater.
    // The power pins on the v1.02 hardware are not enabled by default.
    aa_target_power(handle, AA_TARGET_POWER_NONE);

    // Setup the clock phase
    aa_spi_configure(handle, mode >> 1, mode & 1, AA_SPI_BITORDER_MSB);

    // Set the slave response
    for (i=0; i<SLAVE_RESP_SIZE; ++i)
        slave_resp[i] = 'A' + i;

    aa_spi_slave_set_response(handle, SLAVE_RESP_SIZE, slave_resp);

    // Enable the slave
    aa_spi_slave_enable(handle);


    // Write the data to the bus
    aa_spi_write(handle, 500, data_out1, 500, data_in1);

    // Watch the SPI port
    dump(handle, timeout_ms);

    // Disable the slave and close the device
    aa_spi_slave_disable(handle);
    aa_close(handle);

//    return 0;
}

static u08 data_in[BUFFER_SIZE];

static void dump (Aardvark handle, int timeout_ms)
{
    int trans_num = 0;
    int result;

    printf("Watching slave SPI data...\n");

    // Wait for data on bus
    result = aa_async_poll(handle, timeout_ms);
    if (result != AA_ASYNC_SPI) {
        printf("No data available.\n");
        return;
    }

    printf("\n");

    // Loop until aa_spi_slave_read times out
    for (;;) {
        int num_read;

        // Read the SPI message.
        // This function has an internal timeout (see datasheet).
        // To use a variable timeout the function aa_async_poll could
        // be used for subsequent messages.
        num_read = aa_spi_slave_read(handle, BUFFER_SIZE, data_in);

        if (num_read < 0 && num_read != AA_SPI_SLAVE_TIMEOUT) {
            printf("error: %s\n", aa_status_string(num_read));
            return;
        }
        else if (num_read == 0 || num_read == AA_SPI_SLAVE_TIMEOUT) {
            printf("No more data available from SPI master.\n");
            return;
        }
        else {
            int i;
            // Dump the data to the screen
            printf("*** Transaction #%02d\n", trans_num);
            printf("Data read from device:");
            for (i = 0; i < num_read; ++i) {
                if ((i&0x0f) == 0)      printf("\n%04x:  ", i);
                printf("%02x ", data_in[i] & 0xff);
                if (((i+1)&0x07) == 0)  printf(" ");
            }
            printf("\n\n");

            ++trans_num;
        }
    }
}
