
#include "NecIR.h"

// Flags
#define REPEAT_FLAG   0
#define START_FLAG    1
#define NEWDATA_FLAG  2
#define ERROR_FLAG    3
#define TIMER_FLAG    4

// Some macros that will increase readability (order of arguments matters .. GCC will optimize constants)
#define APPROX(value, constant) (((constant)-UNCERT<=(value)) && ((value)<=(constant)+UNCERT))
#define APPROX_LONG(value, constant) (((constant)-UNCERT_LONG<=(value)) && ((value)<=(constant)+UNCERT_LONG))
#define SET_FLAG(FLAG)  flags |=  (1<<(FLAG))
#define RST_FLAG(FLAG)  flags &= ~(1<<(FLAG))
#define IS_FLAG(FLAG)  (flags &   (1<<(FLAG)))


#define START        (13500)  // Length of a start  bit: 9+4.5  ms = 13.5ms = 13500µs
//#define REPEAT       (11200)  // Length of a repeat bit: 9+2.25 ms = 11.2ms = 11200µs
#define REPEAT       (11200)  // Length of a repeat bit: 9+2.25 ms = 11.2ms = 11200µs
#define ONE          ( 2200)  // Length of a logical 1:    2.2  ms          =  2200µs
#define ZERO         ( 1100)  // Length of a logical 0:    1.1  ms          =  1100µs
#define UNCERT       (  500)  // Uncertainty: +-0.5ms for ONEs and ZEROs
#define UNCERT_LONG  ( 1500)  // Uncertainty: +-0.15ms for START and REPEAT


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


static volatile uint32_t edge_count  = 0;

// forward declaration
static void necir_interrupt();

#ifdef DEBUG_IRNEC_LENGTHS
#define DEBUG_LENGTHS_COUNT 36
volatile uint16_t lengths_buff[DEBUG_LENGTHS_COUNT+1];
#endif


void NecIR::setup(int INT0_pin)
{
#ifdef DEBUG_IRNEC_LENGTHS
	memset((void*)lengths_buff, 0, sizeof(lengths_buff));
#endif

	// set up pin 2 for interrupt on falling edge
	pinMode(INT0_pin, INPUT_PULLUP);
	attachInterrupt(0, necir_interrupt, FALLING);
}

#ifdef DEBUG_IRNEC_LENGTHS
uint32_t NecIR::get_edge_count()
{
  return edge_count;
}
#endif

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


uint16_t error_length;

uint16_t NecIR::get_error_length()
{
	return error_length;
}


#ifdef DEBUG_IRNEC_LENGTHS
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

                if      (APPROX_LONG(l,START))  Serial.print("S");             
                else if (APPROX_LONG(l,REPEAT)) Serial.print("R");             
                else if (APPROX(l,ONE))         Serial.print("H");             
                else if (APPROX(l,ZERO))        Serial.print("L");         
                else                            Serial.print("E");     
                
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
#endif

static long last = 0;

static void necir_interrupt()
{
	cli();
        long now = micros();
        long length = now-last;
        last = now;
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
	if (APPROX_LONG(length, START))
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

	if (APPROX_LONG(length, REPEAT))
	{
		SET_FLAG(REPEAT_FLAG);
		repetitions++;
		return;
	}

	// TODO: test if optimization works: if (!(edge_count & 32)) // saves 4 bytes of code
	if (edge_count<32)
	{
		buffer.dword = (buffer.dword << 1); // shift one to the left

		if (APPROX(length, ONE))
		{
			buffer.dword = (buffer.dword | 1);
		}
		else if (APPROX(length, ZERO))
		{
			// leave LSB as is (code has no effect however optmized by gcc anyway so leave for clarity)
		}
		else
		{
                        error_length = length;
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
                        repetitions = 0;
		}
	}
}






