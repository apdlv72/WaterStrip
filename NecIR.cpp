
#include "NecIR.h"

// Flags
#define REPEAT_FLAG   0
#define START_FLAG    1
#define NEWDATA_FLAG  2
#define ERROR_FLAG    3
#define TIMER_FLAG    4

// Some macros that will increase readability (order of arguments matters .. GCC will optimize constants)
#define APPROX(value, constant) (((constant)-len_uncert<=(value)) && ((value)<=(constant)+len_uncert))
#define APPROX_LONG(value, constant) (((constant)-len_uncert_long<=(value)) && ((value)<=(constant)+len_uncert_long))
#define SET_FLAG(FLAG)  flags |=  (1<<(FLAG))
#define RST_FLAG(FLAG)  flags &= ~(1<<(FLAG))
#define IS_FLAG(FLAG)  (flags &   (1<<(FLAG)))


// Real values observed at 10kHz start: 100-135, repeat: 109-110
// Using scaling 25 here, since LPD6803 code works at 25kHz by default (for 50% cpumax) 
// while IR code expected 10kHz originally:
// (Not real values, but computed from 10kHz values)
/*
#define PAUSE        (25*940/10)  // There seems to be a pause length. This is used mainly for a reset condition.
#define START        (25*135/10)  // Length of a start  bit: 9+4.5  = 13.5ms
#define REPEAT       (25*113/10)  // Length of a repeat bit: 9+2.25 = 11.2ms
#define UNCERT_LONG  (25* 11/10)  // Uncertainty: +-0.15ms for START and REPEAT
#define ONE          (25* 22/10)  // Length of a logical 1:  2.2ms
#define ZERO         (25* 11/10)  // Length of a logical 0:  1.1ms
#define UNCERT       (25*  5/10)  // Uncertainty: +-0.5ms for ONEs and ZEROs
 */

// Real values observed at 25kHz: start: 100-135, repeat: 109-110
/*
 675 280 29 28 27 29 28 27  29 55 57 55 56 57 55 56  57 27 28 29 27 28 29 55  29 55 56 57 55 56 57 27
 924 280 28 28 28 28 28 28  28 56 56 56 56 56 56 56  56 56 28 28 56 56 28 56  28 28 56 56 28 29 55 28
 336 29 28 27 29 28 27 29  28 56 55 57 56 55 57 56  55 28 29 27 28 29 27 57  27 57 55 56 45 55 56 28
 */
#define START        (336)  // Length of a start  bit: 9+4.5  = 13.5ms
#define REPEAT       (280)  // Length of a repeat bit: 9+2.25 = 11.2ms
#define ONE          (56)  // Length of a logical 1:  2.2ms
#define ZERO         (28)  // Length of a logical 0:  1.1ms
#define UNCERT       (13)  // Uncertainty: +-0.5ms for ONEs and ZEROs
#define UNCERT_LONG  (25* 11/10)  // Uncertainty: +-0.15ms for START and REPEAT


static uint16_t len_start, len_repeat, len_one, len_zero, len_uncert, len_uncert_long;

// a complete signal sequence consisting of pos/neg address and command code (from LSB to MSB)
typedef struct 
{
	uint8_t comm;  // negated command (for control) OR a real address for 16bit addresses
	uint8_t comm2; // actual command
	uint8_t addr;  // negated address (for control)
	uint8_t addr2; // actual address
} s_addr_comm;

// union used to shift in IR bits one by one at receive time and
// to access commands/addresses as bytes at decoding time
typedef union
{
	s_addr_comm bytes;   // negated control version and actual addr+command
	uint32_t    dword;   // 32 raw bits: 8 MSB are comm, 8 LSB are addr2
}
u_addr_comm;

// buffer for bits received
static volatile u_addr_comm buffer;

// flags and buffer for NEC IR command decipher
static volatile uint8_t    flags = 0;

// buffers for current command and address
static volatile uint8_t    command  = 0;

// buffers for command repetitions:
static volatile uint8_t  last_command;
static volatile uint16_t repetitions = 0;
static volatile uint16_t errors      = 0;


static volatile uint16_t edge_count  = 0;

// pointer to a counter incremented externally by a counter interrupt
static volatile  uint16_t * counter;


// forward declaration
static void necir_interrupt();

// how ofter per seconds the external counter is incremented
static uint32_t frequency;

#ifdef DEBUG_IRNEC_LENGTHS
#define DEBUG_LENGTHS_COUNT 36
volatile uint16_t lengths_buff[DEBUG_LENGTHS_COUNT+1];
#endif


void NecIR::setup(int INT0_pin, volatile uint16_t * cnt, uint32_t freq)
{
	counter = cnt;
        frequency = freq;      
        
        len_start       = freq * START       / 10000;
        len_repeat      = freq * REPEAT      / 10000;
        len_one         = freq * ONE         / 10000;
        len_zero        = freq * ZERO        / 10000;
        len_uncert      = freq * UNCERT      / 10000;
        len_uncert_long = freq * UNCERT_LONG / 10000;
        
        Serial.println("Length of one  bits: "); Serial.println(len_one);
        Serial.println("Length of zero bits: "); Serial.println(len_zero);
  
#ifdef DEBUG_IRNEC_LENGTHS
	memset((void*)lengths_buff, 0, sizeof(lengths_buff));
#endif

	// set up pin 2 for interrupt on falling edge
	pinMode(INT0_pin, INPUT_PULLUP);
	attachInterrupt(0, necir_interrupt, FALLING);
}


uint8_t NecIR::get_command(uint16_t * rep)
{
	if (IS_FLAG(REPEAT_FLAG))
	{
		RST_FLAG(REPEAT_FLAG);
		if (rep) *rep = repetitions;
		return last_command;
	}

	if (IS_FLAG(NEWDATA_FLAG))
	{
		RST_FLAG(NEWDATA_FLAG);
		if (rep) *rep = 0;
		return command;
	}

	return 0;
}


uint16_t NecIR::get_errors()
{
	return errors;
}


void NecIR::dump_lengths(void)
{
#ifdef DEBUG_IRNEC_LENGTHS
	uint8_t i=0;
	for (i=0; i<DEBUG_LENGTHS_COUNT; i++)
	{
                uint16_t l = lengths_buff[i];
		if (l<1)
		{
			break;
		}

                if      (APPROX_LONG(l,len_start))  Serial.print("S");             
                else if (APPROX_LONG(l,len_repeat)) Serial.print("R");             
                else if (APPROX(l,len_one))         Serial.print("H");             
                else if (APPROX(l,len_zero))        Serial.print("L");         
                else                                Serial.print("E");     
                
                Serial.print(":");          
                Serial.print(l);          
                Serial.print(" ");
		if (0==(i%8)) Serial.print(" ");                                                
	}
	if (i>0)
	{
		Serial.println();
	}
#endif
}


static void necir_interrupt()
{
	cli();
	uint16_t length = *counter;
	*counter  = 0;
	sei();

	// count total number of edges received as a good indicator to check
	// if this function is called at all (interrupt setup worked)
#ifdef DEBUG_UNRECOGNIZED_EDGED
	total_edge_count++;
#endif

	// received an implausible value? reset
	/*
	if (length>4*PAUSE)
   	{
   		RST_FLAG(TIMER_FLAG);
   		return;
   	}
	 */

	// first falling edge seen
	if (!IS_FLAG(TIMER_FLAG))
	{
		SET_FLAG(TIMER_FLAG);
		edge_count = 0;
		return;
	}

	// start bit seen
	if (APPROX_LONG(length, len_start))
	{
		edge_count = 0;
		buffer.dword = 0;
#ifdef DEBUG_IRNEC_LENGTHS
		lengths_buff[0] = length;
#endif
		return;
	}

#ifdef DEBUG_IRNEC_LENGTHS
	//  for debug: record length and exit
	if (edge_count<DEBUG_LENGTHS_COUNT)
	{
		// +1 because 0th contans start bit which would be overwritten by first
		// command bit otherwise because edge count was not yet incrementes here
		lengths_buff[edge_count+1] = length;
	}
#endif

	if (APPROX_LONG(length, len_repeat))
	{
		SET_FLAG(REPEAT_FLAG);
		repetitions++;
		return;
	}
	repetitions = 0;

	// TODO: test if optimization works: if (!(edge_count & 32)) // saves 4 bytes of code
	if (edge_count<32)
	{
		buffer.dword = (buffer.dword << 1); // shift one to the left

		if (APPROX(length,len_one))
		{
			buffer.dword = (buffer.dword | 1);
		}
		else if (APPROX(length,len_zero))
		{
			// leave LSB as is (code has no effect however optmized by gcc anyway so leave for clarity)
		}
		else
		{
			errors++;
		}

		edge_count++;

		if (edge_count==32)
		{
			// WARN: need to assign these to temp. variables, otherwise below check won't work ... why so???
			uint8_t actual_comm =   buffer.bytes.comm2;
			uint8_t check_comm  = (~buffer.bytes.comm);
			// positive and XOR of negated values should match
			// addresses however can differ (seen 0 and 8 with 22 key remote)
			if (actual_comm==check_comm)
			{
				last_command = command = buffer.bytes.comm2;
                                //last_address = address = buffer.bytes.addr2;
				SET_FLAG(NEWDATA_FLAG);
			}

			RST_FLAG(TIMER_FLAG);
		}
	}
}





