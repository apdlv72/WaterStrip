#ifndef irnec_h
#define irnec_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#define DEBUG_IRNEC_LENGTHS

class NecIR {

public:
	void     setup(int INT0_pin);
	uint8_t  get_command(uint16_t * repeat);
        uint16_t get_error_length();
	uint16_t get_errors();
        uint32_t get_edge_count();
	void     dump_lengths(void);
};

#endif



