#ifndef irnec_h
#define irnec_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG_IRNEC_LENGTHS

class NecIR {

public:
	void     setup(int INT0_pin, volatile uint16_t * counter, uint32_t frequency);
	uint8_t  get_command(uint16_t * repeat);
	uint16_t get_errors();
	void     dump_lengths(void);
};

#endif


