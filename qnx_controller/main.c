/*
 * Alex Baraker
 * Nick Blachowicz
 * SWEN.563.02 - Real Time & Embedded Systems
 * Project #6
 * 2175
 *
 * Description: Part a system with the STM eval board. The purplebox's
 * role is to record ADC samples, map their amplitudes from the full-scale
 * input range of the ADC to the full degree travel of a servo motor, and
 * transmit that degree calculation serially to the STM eval board.
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>		/* for sleep() */
#include <stdint.h>
#include <hw/inout.h>	/* for in*() and out*() functions */
#include <sys/neutrino.h>	/* for ThreadCtl() */
#include <sys/mman.h>	/* for mmap_device_io() */

/* The Neutrino IO port used here corresponds to a */
/* single register which is one byte long */
#define PORT_LENGTH_ADC 	(16)

// The base memory address of the GPIO registers
#define ADC_BASE			(0x280)

// Memory offsets for the various peripherals
#define COMMAND_REG		(0)
#define AD_LSB			(0)
#define PAGE_REG		(1)
#define AD_MSB			(1)
#define AD_CHAN_REG		(2)
#define GAIN_SCAN_CTRL	(3)
#define INPUT_STATUS	(3)
#define INT_CNT_CTRL	(4)
#define AD_FIFO_STATUS	(6)
#define AD_MODE_CONFIG	(13)
#define DIO_PORT_A		(8)
#define DIO_PORT_B		(9)
#define DIO_DIR_REG		(11)

// How the GPIO define the data direction of the GPIO registers
#define OUTPUT			(0)
#define INPUT			(1)

// The GPIO pins we are using
#define CLK_PIN			(3)
#define DAT_PIN			(0)
#define ACK_PIN			(0)

// Global variables
unsigned short buffer;				// Used to store data that is to be shifted out.
unsigned char ack_flag = 0;			// Flag used to check if we are in the ACK state.
unsigned char transmit_flag = 0;	// Flag used to check if we are in the data send state.
/* -------------------------------------------- */
///////////////////////////////////////////////////////////////////
// FUNC:	set_bit
// INFO:	Sets or clears a specific bit in a byte, by masking that bit on the
//			current register contents
// PARAM:
//		unsigned char src:	The original contents of the register
//		unsigned char bit:  The bit to be changed
//		unsigned char val:  The value of the bit being set
// RETURN:
//		The modified register contents.
///////////////////////////////////////////////////////////////////
unsigned char set_bit(unsigned char src, unsigned char bit, unsigned char val)
{
	return val? (src | (1 << bit)): (src & ~(1 << bit));
}

///////////////////////////////////////////////////////////////////
// FUNC:	start_AD_conv
// INFO:	Sets the STRTAD bit, which starts and ADC conversion.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void start_AD_conv(unsigned int IO_module_handle) { out8(IO_module_handle + COMMAND_REG, 0x80); }

///////////////////////////////////////////////////////////////////
// FUNC:	reset_FIFO
// INFO:	Sets the RSTFIFO bit, which clears the contents of the FIFO.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void reset_FIFO(unsigned int IO_module_handle) { out8(IO_module_handle + COMMAND_REG, 0x10); }

///////////////////////////////////////////////////////////////////
// FUNC:	write_to_page_reg
// INFO:	Writes the lowest 2 bits of data into the page register,
//			which selects the register page (used to select a register
//			from multiple registers with the same address).
// PARAM:
//		char data : The value to be written into the register
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void write_to_page_reg(char data, unsigned int IO_module_handle) { out8(IO_module_handle + PAGE_REG, (data & 0x03)); }

///////////////////////////////////////////////////////////////////
// FUNC:	write_AD_channel_reg
// INFO:	Sets the start and end channel for the ADC to cycle through
//			while sampling. It will start at the value of lo_ch, and
//			increment up to hi_ch, then start the cycle over. If lo_ch == hi_ch
//			the ADC will only sample that one channel.
// PARAM:
//		unsigned char hi_ch	: The highest channel to sample.
//		unsigned char lo_ch : The lowest channel to sample.
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void write_AD_channel_reg(unsigned char hi_ch, unsigned char lo_ch, unsigned int IO_module_handle) { out8(IO_module_handle + AD_CHAN_REG, (hi_ch << 4) | lo_ch); }

///////////////////////////////////////////////////////////////////
// FUNC:	set_gain
// INFO:	Sets the gain of the ADC input.
// PARAM:
//		char gain : The lowest 2 bits correspond to the desired gain for the ADC input.
//					00 : gain = 1
//					01 : gain = 2
//					10 : gain = 4
//					11 : gain = 8
//					The selected gain affects the range of input voltages that the ADC can support.
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void set_gain(char gain, unsigned int IO_module_handle) { out8(IO_module_handle + GAIN_SCAN_CTRL, (0x03 & gain)); }

///////////////////////////////////////////////////////////////////
// FUNC:	clear_ainte_bit
// INFO:	Clears the AINTE bit, so that the ADC will not auto-sample, it will only
//			start a sample when the STRTAD is set.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void clear_ainte_bit(unsigned int IO_module_handle) { out8(IO_module_handle + INT_CNT_CTRL, in8(IO_module_handle + INT_CNT_CTRL) & 0xFE); }	// Clear AINTE bit, so that ADC will only trigger when ADSTART bit is set

///////////////////////////////////////////////////////////////////
// FUNC:	set_AD_single_ended_bipolar
// INFO:	Sets the ADC for single-ended input, bipolar mode. This means
//			that a single channel will be used for sampling, and the range
//			of input voltages extends into negative voltages.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void set_AD_single_ended_bipolar(unsigned int IO_module_handle) { out8(IO_module_handle + AD_MODE_CONFIG, 0x00); }

///////////////////////////////////////////////////////////////////
// FUNC:	read_AD_LSB
// INFO:	Reads the contents of the ADC's LSB register, which holds the
//			lowest 8 bits of the ADC sample.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		unsigned char : The lowest 8 bits of the ADC sample.
///////////////////////////////////////////////////////////////////
unsigned char read_AD_LSB(unsigned int IO_module_handle) { return in8(IO_module_handle + AD_LSB); }

///////////////////////////////////////////////////////////////////
// FUNC:	read_AD_MSB
// INFO:	Reads the contents of the ADC's MSB register, which holds the
//			highest 8 bits of the ADC sample.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		unsigned char : The highest 8 bits of the ADC sample.
///////////////////////////////////////////////////////////////////
unsigned char read_AD_MSB(unsigned int IO_module_handle) { return in8(IO_module_handle + AD_MSB); }

///////////////////////////////////////////////////////////////////
// FUNC:	read_AD_channel_FIFO_status
// INFO:	Returns the depth of the FIFO register, if the FIFO is not in enhanced mode.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		unsigned char : The current number of samples in the FIFO.
///////////////////////////////////////////////////////////////////
unsigned char read_AD_channel_FIFO_status(unsigned int IO_module_handle) { return in8(IO_module_handle + AD_FIFO_STATUS); }

///////////////////////////////////////////////////////////////////
// FUNC:	read_AD_conv_status
// INFO:	Returns the ADBUSY bit, which is high if the ADC is currently sampling.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		unsigned char : 1 if the ADC is currently sampling, 0 otherwise.
///////////////////////////////////////////////////////////////////
unsigned char read_AD_conv_status(unsigned int IO_module_handle) { return (in8(IO_module_handle + INPUT_STATUS) & 0x80); }

///////////////////////////////////////////////////////////////////
// FUNC:	read_AD_update_status
// INFO:	Returns 1 if the register settings for the ADC are currently settling in.
//			There is a hardware-defined 10 us wait period after changing any register setting.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		unsigned char : 1 if it has not been 10 us since the register contents were changed, 0 otherwise.
///////////////////////////////////////////////////////////////////
unsigned char read_AD_update_status(unsigned int IO_module_handle) { return (in8(IO_module_handle + INPUT_STATUS) & 0x20); }

///////////////////////////////////////////////////////////////////
// FUNC:	get_ADC_sample
// INFO:	Combines the LSB and MSB register contents into a single integer value.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		short : A signed value, corressponding to the full sample of the ADC.
///////////////////////////////////////////////////////////////////
short get_ADC_sample(unsigned int IO_module_handle) { return (in8(IO_module_handle + AD_LSB) + in8(IO_module_handle + AD_MSB) * 256); }

///////////////////////////////////////////////////////////////////
// FUNC:	conv_to_volts
// INFO:	Translates an integer representing the ADC value into a floating-point value.
// PARAM:
//		int sample : The ADC sample.
// RETURN:
//		float : The floating-point representation of the voltage.
///////////////////////////////////////////////////////////////////
float conv_to_volts(int sample) { return (((float)(sample)) / 32768.0 * 5.0); }

///////////////////////////////////////////////////////////////////
// FUNC:	set_DIO_dir
// INFO:	Sets the data direction of a DIO port.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
//		char port_A_dir : Sets the direction of port A (1 -> input, 0 -> output)
//		char port_B_dir : Sets the direction of port B (1 -> input, 0 -> output)
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void set_DIO_dir(unsigned int IO_module_handle, char port_A_dir, char port_B_dir) { out8(IO_module_handle + DIO_DIR_REG, ( (port_A_dir & 0x1) << 4) | ( (port_B_dir & 0x1) << 1) ); }

///////////////////////////////////////////////////////////////////
// FUNC:	read_DIO_port_A
// INFO:	Reads the contents of DIO port A.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		unsigned char : The data from DIO port A.
///////////////////////////////////////////////////////////////////
unsigned char read_DIO_port_A(unsigned int IO_module_handle) { return in8(IO_module_handle + DIO_PORT_A); }

///////////////////////////////////////////////////////////////////
// FUNC:	write_DIO_port_A
// INFO:	Writes data to DIO port A.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
//		unsigned char data : The data to be written to the port.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void write_DIO_port_A(unsigned int IO_module_handle, unsigned char pin, unsigned char data)
{
	unsigned char shift = 0;

	if ( (data & 0x1) == 0) shift = read_DIO_port_A(IO_module_handle) & ~(0x1 << pin);
	else					shift = read_DIO_port_A(IO_module_handle) | (0x1 << pin);

	out8(IO_module_handle + DIO_PORT_A, shift);
}

///////////////////////////////////////////////////////////////////
// FUNC:	read_DIO_port_B
// INFO:	Reads the contents of DIO port B.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		unsigned char : The data from DIO port B.
///////////////////////////////////////////////////////////////////
unsigned char read_DIO_port_B(unsigned int IO_module_handle) { return in8(IO_module_handle + DIO_PORT_B); }

///////////////////////////////////////////////////////////////////
// FUNC:	write_DIO_port_B
// INFO:	Writes data to DIO port B.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
//		unsigned char data : The data to be written to the port.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void write_DIO_port_B(unsigned int IO_module_handle, unsigned char pin, unsigned char data)
{
	unsigned char shift = 0;

	if ( (data & 0x1) == 0) shift = read_DIO_port_B(IO_module_handle) & ~(0x1 << pin);
	else					shift = read_DIO_port_B(IO_module_handle) | (0x1 << pin);

	out8(IO_module_handle + DIO_PORT_B, shift);
}

unsigned char volts_to_servo_degrees(float volts) { return (unsigned char)((150.0 - (volts + 5.0) * 15.0 )); }

void load_buffer(unsigned char data) { buffer = 0x01FF & ~(((unsigned short)data) << 1); }

void send_data(unsigned int IO_module_handle)
{
	write_DIO_port_B(IO_module_handle, DAT_PIN, ~((unsigned char)(buffer & 0xFF)));
	buffer >>= 1;
}

///////////////////////////////////////////////////////////////////
// FUNC:	AD_init
// INFO:	Sets the register contents of the ADC, to get it setup for sampling.
// PARAM:
//		unsigned int IO_module_handle : The base address of the ADC registers.
// RETURN:
//		nothing
///////////////////////////////////////////////////////////////////
void AD_init(unsigned int IO_module_handle)
{
	write_AD_channel_reg(2, 2, IO_module_handle);
	clear_ainte_bit(IO_module_handle);
	set_gain(0x01, IO_module_handle);
	write_to_page_reg(0x02, IO_module_handle);	// Sets us on page 2, for FIFO and AD/DA control
	set_AD_single_ended_bipolar(IO_module_handle);
	while ( read_AD_update_status(IO_module_handle) );	// Wait 10us for ADC registers to settle
}

///////////////////////////////////////////////////////////////////
// FUNC:	main
// INFO:	The main function.
///////////////////////////////////////////////////////////////////
int main()
{
	buffer = 0;						// Clear the buffer
	int sample, counter = 0;		// Initialize variables
	int privity_err;				// Used for memory access
	unsigned int IO_module_handle;	// Used for memory access

	/* Give this thread root permission to access */
	/* the hardware */
	privity_err = ThreadCtl( _NTO_TCTL_IO, NULL );
	if ( privity_err == -1)
	{
		printf("can't get root permissions\n");
		return -1;
	}

	// Define which registers we want access to
	IO_module_handle = mmap_device_io( PORT_LENGTH_ADC, ADC_BASE );
	if ( IO_module_handle == MAP_DEVICE_FAILED )
	{
		perror( "data map for ADC failed" );
		return EXIT_FAILURE;
	}

	// House-keeping
	printf("Program starting.\n");
	AD_init(IO_module_handle);							// Initialize ADC settings
	set_DIO_dir(IO_module_handle, OUTPUT, OUTPUT);		// Set ports A and B as outputs
	out8(IO_module_handle + DIO_PORT_A, 0x0);			// Clear port A

	// The main loop
	while(1)
	{
		if ( (!ack_flag) && (!transmit_flag) )	// Not in ACK state, not in send data state, therefore must be in idle state
		{
			set_DIO_dir(IO_module_handle, OUTPUT, OUTPUT); 		// Set port B to output

			// When the clk toggles 50 times, read new data
			if (counter%50 == 0)
			{
				start_AD_conv(IO_module_handle);
				while ( read_AD_conv_status(IO_module_handle) );	// Wait for ADC conversion to finish

				sample = get_ADC_sample(IO_module_handle);
				float volts = conv_to_volts(sample);
				unsigned char degrees = volts_to_servo_degrees(volts);
				load_buffer(degrees);
				transmit_flag = 1;	// Now in the data send state
				printf("Sample #%i:\t%i\t%f\t%i\t%08x\n", counter, sample, volts, degrees, degrees);
			}
		}
		else if ( (!ack_flag) && (transmit_flag) )	// Not in ACK state, but in send data state
		{
			set_DIO_dir(IO_module_handle, OUTPUT, OUTPUT); 		// Set port B to output
			if (counter%2 == 0) send_data(IO_module_handle);	// Put next data bit on data pin
			if (buffer == 0)
			{
				transmit_flag = 0;
				ack_flag = 1;
			}
		}
		else if ( (ack_flag) && (!transmit_flag) )	// In ACK state
		{
			set_DIO_dir(IO_module_handle, OUTPUT, INPUT); // Set port B to input
			if (read_DIO_port_B(IO_module_handle + DIO_PORT_B))
			{
				ack_flag = 0;
			}
		}
		else
		{
			printf("State error\r\n");
		}

		write_DIO_port_A(IO_module_handle, CLK_PIN, counter%2);	// Toggle CLK pin

		counter++;
		usleep(1);
	}
}






