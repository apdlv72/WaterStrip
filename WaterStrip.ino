// Board: Aduino Pro Mini 5V 16 MHz with AtMega328

#ifndef OSX
  #ifndef __AVR_ATmega328P__
    #error WRONG TARGET BOARD. EXPECTED __AVR_ATmega328P__
  #endif
#endif

#define STRIP_TYPE_6803
//#define STRIP_TYPE_RGB

#define WITH_HELP
#define WITH_UART_VALUES_OUTPUT
#define WITH_UART_PWM_OUTUT

/*********************************************************************/

#if defined(STRIP_TYPE_6803) && defined(STRIP_TYPE_RGB)
#error Only one strip type may be defined
#endif

#include <eEEPROM.h>

#ifdef STRIP_TYPE_6803
  #include <FastSPI_LED.h>
#endif

#include "ir_commands.h"
#include "NecIR.h"

// the actual RGB data
struct CRGB { unsigned char g; unsigned char r; unsigned char b; };
struct CRGB *leds;

// Number of RGB LEDs in strand:
#define WIDTH     50 // number of pixels/LEDs/sections????


#define SIGN(A)  ((A)<0 ? -s1 : 1  )
#define ABS(A)   ((A)<0 ? (-(A)) : (A))

#ifndef MAX
#define MAX(A,B) ((A)>(B) ? (A) : (B))
#endif

#ifndef MIN
#define MIN(A,B) ((A)<(B) ? (A) : (B))
#endif

// modes of operation
#define MODE_MIN         0
#define MODE_DEFAULT     MODE_MIN

#ifdef STRIP_TYPE_6803
  #define MODE_WATER     0 // water simulation,  monochromatic, brightness varies
  #define MODE_WATER2    1 // mostly monochromatic, but slight displacement on G/B channel causes some coloring
  #define MODE_FADE      2 // like WATER2, however base color fades over time
  #define MODE_RAINBOW   3 // displays whole rainbow range on the strip, otherwise like WATER
  #define MODE_RAINBOW2  4 // like RAINBOW, however displays only a subrange of the whole color range
  #define MODE_RAINBOW3  5 // like RAINBOW, however displays whole color range 2-3 times
  #define MODE_CONST     6 // constant color only, but brightness may be set by the user 
  #define MODE_RAINFLOW  7 // like RAINBOW2, ripple data used as color displacement
  #define MODE_RAINFLOW2 8 // like RAINBOW3, ripple data used as color displacement
  #define MODE_MAX       8
#endif

#ifdef STRIP_TYPE_RGB
  #define MODE_WATER     0 // water simulation,  monochromatic, brightness varies
  #define MODE_WATER2    1 // mostly monochromatic, but slight displacement on G/B channel causes some coloring
  #define MODE_FADE      2 // like WATER2, however base color fades over time
  #define MODE_RAINBOW   3 // displays whole rainbow range on the strip, otherwise like WATER
  #define MODE_CONST     4 // constant color only, but brightness may be set by the user 
  #define MODE_RAINFLOW  5 // like RAINBOW2, ripple data used as color displacement
  #define MODE_MAX       5
#endif


// define which parameter should be modified by keys QUICK/SLOW
#define PARAM_SPEED            1 // modify speed (FPS))
#define PARAM_RANDOM_TRESHOLD  2 // modify likeliness of disturbance
#define PARAM_DIST_STRENGTH    3 // modify strenght/apmlitude of disturbances
#define PARAM_FADE_SPEED       5 // fading speed for RAINBOW/FADE modes
#define PARAM_VISU_AMPLITUDE   6 // controls degree of influence of disturbance on lization

#define DIST_RAMP_UP_COUNT     8   // ramp up time (frames) until a disturbance fully applies to ripples buffer

#define RND_TRESHOLD_DEFAULT  50 // defines likeliness of disturbance to occur: 0 - never, 1000 - always (in terms of frames)
#define RND_STRENGTH_DEFAULT 300

// Chose pin where IR detector is connected (must be able to  INT0) 
int PIN_IR_INT0 =  2; 

// This is the pin where the on board LED is conencted to
int PIN_LED     = 13;

// PWM channels for RGB strip output: 
#define PIN_PWM_R 3
#define PIN_PWM_G 5
#define PIN_PWM_B 6

// Pins connected to key and state of BT module
#define PIN_BT_KEY   8 
#define PIN_BT_STATE 9


uint8_t paused = 0;

int16_t  dist_height[WIDTH];
uint8_t  dist_count[WIDTH];

int8_t   ripples1[WIDTH];
int8_t   ripples2[WIDTH];
int8_t * src = ripples1;
int8_t * dst = ripples2;


typedef struct preset_s
{
	uint8_t mode; // the mode of operation: WATER, RAINBOW etc.
	uint8_t randomTreshold; // defined likelihood for disturbances: 0-never, 100-always (in terms of frames)
	uint8_t disturbWidth;
	// 8 bits would be sufficient for moste of the below, but 16 makes computation much more convenient.
	// since this structure needs to fit into EEprom 7 times and consumes les than 20 bytes, this is not critical 
	int16_t disturbStrength; // aplitude/strength allows for random disturbances
	int16_t offsetR;    // base value for R channel (0-255)
	int16_t offsetG;    // base value for G channel
	int16_t offsetB;    // base value for B channel
	int16_t amplitude;     // factor 0-255 defines the impact of disturbances on the outpul values
	int16_t brightness; // dimming factor: 0-dark, 255-max. brightness
	
        uint16_t delayTime;  // main delay controls effecive Fps rate
	uint16_t fadePrescale; // defines after how many sub ticks a tick of the fading counter gets incremented	
	uint16_t magic;      // magic 4711 saved in EEprom to check if anything was saved at all in a slot before
        uint8_t  power;
} 
preset_s;

// the preset that should be loaded after a timeout
int8_t  presetToLoad = -1;

// timout to tell appart whether the DIY button was just pressed or held down 
uint32_t presetLoadTime  = 0;
uint32_t lastChangeSaveTime = 0;


static NecIR necIR(PIN_IR_INT0);

// all current values that control visualization and therefore must be saved in EEprom
preset_s preset;

uint16_t fadeCount     = 0;
uint16_t fadeSubCount  = 0;
uint8_t  paramSelected = PARAM_SPEED;

uint16_t frameCount = 0;

char      inputBuffer[128];         // a string to hold incoming serial data
char    * inputPos  = inputBuffer;
char    * inputEnd  = inputBuffer+sizeof(inputBuffer)-1;
boolean   stringComplete = false;  // whether the string is complete

// debug mode. 0=none, 1=normal, 2=verbose
uint8_t D = 1;

uint32_t old_pwm = 0;


/*********** prototypes ***********/
void serialEvent();
inline void toggle_pause();
void stripClear(uint8_t r, uint8_t g, uint8_t b);
void presetLoad(int n, struct preset_s * preset);
void sendCurrent();
void sendHello();
/******* end of prototypes ********/


void initDefaults(struct preset_s * preset)
{
	preset->mode             =  MODE_DEFAULT;
	preset->randomTreshold   =  RND_TRESHOLD_DEFAULT;
	preset->disturbStrength =    70; // max. value to add/subbstract as disturbance to ripple buffer
	preset->disturbWidth     =     6;
	preset->offsetR          =  0x70;
	preset->offsetG          =  0x70;
	preset->offsetB          =  0xff;
	preset->amplitude        =    60; // was: 196
	preset->brightness       =   255;
	preset->delayTime        =    25;
	preset->fadePrescale     =    20;	
	preset->magic            =  4711;
        preset->power            =     1;
}


// limit "val" to a value in 0,...,255 
inline uint8_t limit(int16_t val)
{
	if (val>=255) return (uint8_t)255;
	if (val<=0) return 0;
	return (uint8_t)val;
}

long restrict (long value, long low, long high)
{
  if (value<low)  return low;
  if (value>high) return high;
  return value;
}


int CYCLIC_DIST(int a, int b, int width)
{
	int d = ABS(a-b);
	if (d>width/2) d=ABS(d-width);
	return d;
}

/************** visualization rendering below *******************/

void applyBrightness(byte rgb[])
{
  rgb[0] = (byte) ((preset.brightness * rgb[0]) >> 8);
  rgb[1] = (byte) ((preset.brightness * rgb[1]) >> 8);
  rgb[2] = (byte) ((preset.brightness * rgb[2]) >> 8);
  // avoid flickering between 0 and 1,2,3 values
//  if (rgb[0]<4) rgb[0]=0;
//  if (rgb[1]<4) rgb[1]=0;
//  if (rgb[2]<4) rgb[2]=0;
}


void visualizeWater()
{
        //Serial.print("visualizeWater ");
	byte rgb[3]= {0,0,0};
	int16_t baseScaledR = (int16_t) ((240 * (uint16_t)preset.offsetR) >> 8);
	int16_t baseScaledG = (int16_t) ((240 * (uint16_t)preset.offsetG) >> 8);
	int16_t baseScaledB = (int16_t) ((240 * (uint16_t)preset.offsetB) >> 8);

	for (int x = 0; x < WIDTH; x++)
	{
		serialEvent();
		int16_t rippleScaled = (int16_t) ((preset.amplitude * dst[x]) >> 8);

		switch (preset.mode)
		{
		case MODE_WATER:
                        // add ripples only if channel is not completely dark to avoid flickering
			rgb[0] = baseScaledR<4 ? 0 : limit(baseScaledR + rippleScaled); 
			rgb[1] = baseScaledG<4 ? 0 : limit(baseScaledG + rippleScaled); 
			rgb[2] = baseScaledB<4 ? 0 : limit(baseScaledB + rippleScaled); 
			break;

    		case MODE_WATER2:
        		{
        			// compute displacement for where to look for ripple values for G/B
        			// might be negative!!!
        			int displace = rippleScaled / 4;
        			int xG = x + displace;
        			int xB = x - displace;
        
        			// make sure values are in valid range
        			while (xG < 0)
        				xG += WIDTH;
        			while (xB < 0)
        				xB += WIDTH;
        
        			int rippleG = dst[xG % WIDTH];
        			int rippleB = dst[xB % WIDTH];
        
                                // add ripples only if channel is not completely dark to avoid flickering
        			rgb[0] = baseScaledR<4 ? 0 : limit(baseScaledR + rippleScaled);
        			rgb[1] = baseScaledG<4 ? 0 : limit(baseScaledG + ((preset.amplitude * rippleG) >> 8));
        			rgb[2] = baseScaledB<4 ? 0 : limit(baseScaledB + ((preset.amplitude * rippleB) >> 8));        
        			break;
        		}
		}

		applyBrightness(rgb);
                leds[x].r = rgb[0];
                leds[x].g = rgb[1];
                leds[x].b = rgb[2];
	}
	return;
}

void visualizeRainbow(int pos, short rippleScaled, int width, int midR, int midG, int midB, byte rgb[])
{
	int i = (int) ((pos + fadeCount) % width);
//        if (D>1 && 0==pos)
//        {
//            Serial.print("DvisualizeRainbow:");
//            Serial.print(" rippleScaled="); Serial.println(rippleScaled); 
//            Serial.print(" fadeCount="); Serial.println(fadeCount); 
//            Serial.print(" width="); Serial.println(width); 
//            Serial.print(" midR="); Serial.println(midR); 
//            Serial.println();
//        }
	// this one
	// rgb[0] = limit(240*CYCLIC_DIST(i, midR, width)/(width/2) + rippleScaled);
	// is equivalent to the below but saves passing one argument: (with/2)
        // MIN is 1 ... never let the LED go off completely since betwenn 0 and is a big gap in PWM
	rgb[0] = restrict(240 * 2 * CYCLIC_DIST(i, midR, width) / width + rippleScaled, 4, 255);
	rgb[1] = restrict(240 * 2 * CYCLIC_DIST(i, midG, width) / width + rippleScaled, 4, 255);
	rgb[2] = restrict(240 * 2 * CYCLIC_DIST(i, midB, width) / width + rippleScaled, 4, 255);
}

// visualizeRainbow2(x, rippleScaled>>2, (3*WIDTH), 1*(3*WIDTH)/6, 3*(3*WIDTH)/6, 5*(3*WIDTH)/6, rgb);
void visualizeRainbow2(short pos, short rippleScaled, int width, int midR, int midG, int midB, byte rgb[])
{
        /*  
          if (0==pos && D>1)
          {
            Serial.print("D:pos:"); Serial.print(pos);
            Serial.print(" rippleScaled:"); Serial.print(rippleScaled);
            Serial.print(" w:"); Serial.print(width);
            Serial.print(" mR:"); Serial.print(midR);
            Serial.print(" mG:"); Serial.print(midG);
            Serial.print(" mB:"); Serial.print(midB);
            Serial.print(" RGB:"); Serial.print(rgb[0],16);
            Serial.print(" "); Serial.print(rgb[1],16);
            Serial.print(" "); Serial.println(rgb[2],16);
          }
        */  
	int i = (int) (((width<<3) + fadeCount + pos + rippleScaled) % width); // using ripple as displacement 
	// this one
	// rgb[0] = limit(240*CYCLIC_DIST(i, midR, width)/(width/2) + rippleScaled);
	// is equivalent to the below but saves passing one argument: (with/2)
	rgb[0] = restrict(240 * 2 * CYCLIC_DIST(i, midR, width)/width + rippleScaled, 4, 255);
	rgb[1] = restrict(240 * 2 * CYCLIC_DIST(i, midG, width)/width + rippleScaled, 4, 255);
	rgb[2] = restrict(240 * 2 * CYCLIC_DIST(i, midB, width)/width + rippleScaled, 4, 255);
}


void visualize()
{
	switch (preset.mode)
	{
	case MODE_WATER:
	case MODE_WATER2:
		visualizeWater();
		return;
	}

	fadeSubCount++;
	if (fadeSubCount > preset.fadePrescale)
	{
		fadeCount++;
		fadeSubCount = 0;
	}

	byte rgb[3]; // = new short[3];

	for (short x = 0; x < WIDTH; x++)
	{
		serialEvent();
		short rippleScaled = (short) ((preset.amplitude * dst[x]) >> 8);

		switch (preset.mode)
		{
		case MODE_FADE:
		{
			int displace = rippleScaled / 3;

			// modulo 510 will create a sequence 0,...,509,0,...,509,...
			// when fadeCount increases
			int oR = (int) ((preset.offsetR + fadeCount) + displace) % 510;
			// for values>255, difference 510-value will yield
			// 0,...,255,510-256=254,...,510-509=1,0,...
			if (oR > 255)
				oR = 510 - oR;

			int oG = (int) ((preset.offsetG + 2 * fadeCount / 3) - displace) % 510;
			if (oG > 255)
				oG = 510 - oG;

			int oB = (int) ((preset.offsetB + 1 * fadeCount / 4) - displace) % 510;
			if (oB > 255)
				oB = 510 - oB;

			rgb[0] = MAX((short) 30, limit(oR + rippleScaled));
			rgb[1] = MAX((short) 30, limit(oG + rippleScaled));
			rgb[2] = MAX((short) 30, limit(oB + rippleScaled));
		}
		break;

		case MODE_RAINBOW:
			visualizeRainbow(x, rippleScaled, (8 * WIDTH / 3), 1 * (8 * WIDTH / 3) / 6, 3 * (8 * WIDTH / 3) / 6, 5 * (8 * WIDTH / 3) / 6, rgb);
			break;
#ifdef STRIP_TYPE_6803
		case MODE_RAINBOW2:
			// many arguments here that could be computed within
			// visualizeRainbow however
			// this won't allow the compiler to optimze at compile time
			visualizeRainbow(x, rippleScaled, WIDTH, 1 * WIDTH / 6,	3 * WIDTH / 6, 5 * WIDTH / 6, rgb);
			break;

		case MODE_RAINBOW3:
			visualizeRainbow(x, rippleScaled, (2 * WIDTH / 3), 1 * (2 * WIDTH / 3) / 6, 3 * (2 * WIDTH / 3) / 6, 5 * (2 * WIDTH / 3) / 6, rgb);
			break;

		case MODE_RAINFLOW2:
			visualizeRainbow2(x, rippleScaled>>3, (2*WIDTH), 1*(2*WIDTH)/6, 3*(2*WIDTH)/6, 5*(2*WIDTH)/6, rgb);
			break;
#endif
		case MODE_RAINFLOW:
			visualizeRainbow2(x, rippleScaled>>3, (WIDTH), 1*(WIDTH)/6, 3*(WIDTH)/6, 5*(WIDTH)/6, rgb);
			break;


		case MODE_CONST: 
			// keep this here although optimization possible, since this is less time consuming
			// and therefore performance does not matter.
			rgb[0] = limit(preset.offsetR);
			rgb[1] = limit(preset.offsetG);
			rgb[2] = limit(preset.offsetB);
			break;
		}

		applyBrightness(rgb);
                leds[x].r = rgb[0];
                leds[x].g = rgb[1];
                leds[x].b = rgb[2];
	}
}


/************** water simulation below *******************/

void next()
{
	dst[0] =  (byte) (((src[(WIDTH-1)  ] + src[1])>>1) - dst[0]);
	dst[0] -= (dst[0]>>6);
	dst[(WIDTH-1)] =  (byte) (((src[(WIDTH-1)-1] + src[0])>>1) - dst[(WIDTH-1)]);
	dst[(WIDTH-1)] -= (dst[(WIDTH-1)]>>6);

	for (int i=1; i<(WIDTH-1); i++)
	{
		serialEvent();
		if (dist_count[i]>0)
		{
			int d  = (DIST_RAMP_UP_COUNT-dist_count[i]) * dist_height[i] / DIST_RAMP_UP_COUNT;
			src[i] += d;		    
			dst[i] -= d;		    
			dist_count[i]--;
		}
		dst[i] = (byte) (((src[i-1] + src[i+1])>>1) - dst[i]);
		dst[i] -= (dst[i]>>6);
	}
}


void disturb(int force) 
{
	if (!force && random(1000)>=preset.randomTreshold)
	{
		return;
	}

	int x       = random(WIDTH); // pixel position where disturbance starts

	int h_total = random(preset.disturbStrength<<2) - preset.disturbStrength; // 2*R(A)-A yields values -A,...,A
	if (true)
	{	
		int width = random(preset.disturbWidth)<<1; // pixel influenced: 1,...,6
		int h_pp  = h_total;
		if (width>0) h_pp/=width;

		for (int i=0; i<width; i++)
		{
			dist_height[(x+i)%WIDTH] += h_pp;
			dist_count [(x+i)%WIDTH]  = DIST_RAMP_UP_COUNT;
		}
	}
	/*
	else
	{
		int width   = 1+random(preset.disturbWidth); // pixel influenced: 1,...,6
		int h_pp = h_total/width;
		for (int i=0; i<width; i++)
		{
		// this was wrong but worked: 
		// dst[x%WIDTH] += height;
		dst[(x+i)%WIDTH] += h_pp;
		}
	}
	 */
}


// swap src and dst buffers
void swap()
{
	int8_t * tmp = src;  
	src = dst;
	dst = tmp;
}


/************** IR command handling from here on *******************/

inline void toggle_pause()
{
	paused = paused ? false : true;
        Serial.println(paused ? "I:PAUSE=1" : "I:PAUSE=0");
}

inline void modeToggle() 
{
	preset.mode = (short) ((preset.mode + 1) % (MODE_MAX + 1));
	fadeCount = fadeSubCount = 0;
}

void resetRipples() 
{
	for (int i=0; i<WIDTH; i++)
	{
		ripples1[i] = 0;
		ripples2[i] = 0;
		dist_height[i] = 0;
		dist_count[i] = 0;
	}	
	//if (D) { Serial.println("D:RIP_RST"); } 
}

void resetDefaults()
{
	paused = false;
	resetRipples();
	preset.randomTreshold = 0;
	initDefaults(&preset);
	//if (D) { Serial.println("D:DEF_RST"); } 
}

void brightnessIncrease() 
{
	preset.brightness = MIN(255, preset.brightness+20);
}

void brightnessDecrease() 
{
	preset.brightness = MAX(0, preset.brightness-20);
}

void rgbIncrease(uint32_t i)
{
	preset.offsetR = limit(preset.offsetR + ((0xff0000 & i)>>16));
	preset.offsetG = limit(preset.offsetG + ((0x00ff00 & i)>> 8));
	preset.offsetB = limit(preset.offsetB + ((0x0000ff & i)    ));	
}

void rgbDecrease(uint32_t i)
{
	preset.offsetR = limit(preset.offsetR - ((0xff0000 & i)>>16));
	preset.offsetG = limit(preset.offsetG - ((0x00ff00 & i)>> 8));
	preset.offsetB = limit(preset.offsetB - ((0x0000ff & i)    ));	
}

void setRGBBase(uint32_t i) 
{
	preset.offsetR = (0xff0000 & i)>>16;
	preset.offsetG = (0x00ff00 & i)>>8;
	preset.offsetB = (0x0000ff & i);	

	fadeCount = 0;

	switch (preset.mode)
	{
        // these modes use their own colors. if user sets a color, use a mode that allows this       
	case MODE_RAINBOW:
	case MODE_RAINFLOW:
#ifdef STRIP_TYPE_6803
	case MODE_RAINBOW2:
	case MODE_RAINBOW3:
	case MODE_RAINFLOW2:
#endif
		preset.mode = MODE_DEFAULT;
		break;
	}
}

void indicateFeedbackCmd()
{
	for (byte n=0; n<6; n++)
	{
		serialEvent();
	    stripClear((short)255,(short)255,(short)255);
	    delay(20);
	    
	    serialEvent();
	    stripClear((short)255,(short)0,(short)0);
	    delay(20);
	}
}

int presetsDiffer(struct preset_s * p, struct preset_s * q)
{
	int len = sizeof(*p);
	byte * a = (byte*)p;
	byte * b = (byte*)q;
	for (int i=0; i<len; i++)
	{
		if (*a!=*b) return true;
		a++;
		b++;
	}
	return false;
}

boolean presetSave(int n) 
{
	//if (D)  { Serial.print("D:PRE_SAVE("); Serial.print(n); Serial.println(")"); }
	struct preset_s old;
	presetLoad(n, &old);
	
        preset.magic = 4711;
	// save only if there is a difference
	if (presetsDiffer(&old, &preset))
	{
		int len = sizeof(preset);
		byte * b = (byte*)&preset;
		int addr = n*len;

		//if (D) { Serial.print("D:EE_WR(@"); Serial.print(addr,16); Serial.print(","); Serial.print(len); Serial.println(")"); } 
		for (int i=0; i<len; i++)
		{
			eEEPROM.write(addr, *b);
			b++;
			addr++;    
		}
                return true; // changes saved
	}
        return false;
}

void presetLoad(int n, struct preset_s * preset)
{
	//if (D>2) { Serial.print("D:PRE_LD("); Serial.print(n); Serial.println(")"); }
        struct preset_s temp;
	int len = sizeof(temp);
	byte * b = (byte*) &temp;
	int addr = n*len;

	//if (D)>1) { erial.print("D:EE_READ(@"); Serial.print(addr,16); Serial.print(","); Serial.print(len); Serial.println(")"); } 
	for (int i=0; i<len; i++)
	{
		*b = eEEPROM.read(addr);
		b++;
		addr++;    
	}

        if (temp.magic==4711)
        {
          memcpy(preset, &temp, len);
          // if (D>1) { Serial.println("D:MAGIC_FOUND"); }
        }
        else
        {
	   //if (D) { Serial.print("D:MAG_INV("); Serial.print(temp.magic); Serial.println(")"); }
           initDefaults(preset); 
           //if (D) { Serial.println("D:INIT_DEF"); }
        }

        //if (D>1) { Serial.println("D:PRESET_LOADED"); }
}

void handlePreset(int i, int repeat)
{
	if (repeat<8) 
	{
		// if (D) { Serial.print("D:PRE_SCHED("); Serial.print(i); Serial.println(")"); }
		presetToLoad = (byte) i;
		presetLoadTime = millis() + 100;	    
	}
	else if (8==repeat)
	{
		presetSave(i);
		indicateFeedbackCmd();
	}
}

void presetCheckLoad()
{
	if (presetToLoad>-1 && presetLoadTime>0 && millis()>presetLoadTime)
	{
	    if (D>0) { Serial.print("D:PRE_LD("); Serial.print(presetToLoad); Serial.println(")"); } 
	    presetLoad(presetToLoad, &preset);
            preset.power = 1;
    	    presetToLoad = -1;
            resetRipples();
	}
}

void changeCheckSave() 
{
	if (lastChangeSaveTime>0 && millis()>lastChangeSaveTime) 
	{
                if (D>0) { Serial.println("D:CHG_SAV"); }
                stripClear(0,0,0);
		if (presetSave(0))
                {
                  delay(10);
                  stripClear(255,255,255);
                  delay(10);
                }
  		lastChangeSaveTime = 0;
	}
}


inline void scheduleSavingOfChanges()
{
  if (D>0) { Serial.println("D:CHG_SCHED"); }
  // save changes after 5s of user inactivity
  lastChangeSaveTime = millis()+5000; 
}


inline void parameterSelect(int param) 
{
	paramSelected = param;
}


void parameterIncrease() 
{
	switch (paramSelected)
	{
	case PARAM_RANDOM_TRESHOLD: 
		preset.randomTreshold   = MIN(100, preset.randomTreshold+1);    
		break;
	case PARAM_DIST_STRENGTH:  
		preset.disturbStrength = MIN(400, preset.disturbStrength+5); 
		break;
	case PARAM_VISU_AMPLITUDE:
		preset.amplitude        = MIN(255, preset.amplitude+10);
		break;
	case PARAM_SPEED:           
		preset.delayTime        = MAX(0,   ((long)preset.delayTime)-10);        
		break;
	case PARAM_FADE_SPEED:      
		preset.fadePrescale     = MAX(0,   preset.fadePrescale-1);         
		break;
	}
}

void parameterDecrease() 
{
	switch (paramSelected)
	{
	case PARAM_RANDOM_TRESHOLD: 
		preset.randomTreshold   = MAX(0,   preset.randomTreshold-3);    
		break;
	case PARAM_DIST_STRENGTH:  
		preset.disturbStrength = MAX(1,   preset.disturbStrength-5); 
		break;
	case PARAM_VISU_AMPLITUDE:
		preset.amplitude        = MAX(0, preset.amplitude-10);
		break;
	case PARAM_SPEED:           
		preset.delayTime        = MIN(500, preset.delayTime+10);
		break;
	case PARAM_FADE_SPEED:      
		preset.fadePrescale     = MIN(20,  preset.fadePrescale+1);         
		break;
	}	
}


void setPower(int to)
{
   switch (to)
   {
     case 0:
     case 1:
       preset.power = to;
       break;
     default:
       preset.power = 0==preset.power ? 1 : 0;
   }
}


void handleIRCommand(int code, int repeat) 
{
        if (code>0)
        {
  	  if (D) { Serial.print("D:IR_CMD("); Serial.print(code,16);  Serial.print(","); Serial.print(repeat); Serial.println(")"); } 
	}

	switch (code)
	{
	case 0: // no command
		presetCheckLoad();   // load presets (DIYx) after button was released few millies ago
		return; // to avoid setting ticksUntilSave at end

	case CMD_POWER    : 
       		if (5==(repeat%6)) 
                {
                     setPower(2); // 2 = toggle˘
                }
                else if (preset.power>0)
                {
          	  disturb(true); 		    
                  preset.randomTreshold=RND_TRESHOLD_DEFAULT; 
                }
		break;
        default:
          if (0==preset.power)
          {
            Serial.println("D:OFF");
            return;
          }
        }

	switch (code)
	{
	case CMD_POWER: 
                // fall thru avoiding error message. handled above already
                break;
                
	case CMD_PAUSE    : 
		if (repeat<6) 
                {
			toggle_pause();
                }
		else 
                {
			resetDefaults();  
                }
                break;

	case CMD_AUTO     : modeToggle();	   	    break;
	case CMD_LIGHTER  : brightnessIncrease();	    break;
	case CMD_DARKER   : brightnessDecrease();	    break;	
	case CMD_QUICK    : parameterIncrease();	    break;
	case CMD_SLOW     : parameterDecrease();	    break;

	case CMD_JUMP3   : 
		parameterSelect(PARAM_RANDOM_TRESHOLD); 
		if (repeat>10) 
                {
                   resetRipples(); 
                   preset.randomTreshold = 0;
                }
		break;
	case CMD_JUMP7    : parameterSelect(PARAM_VISU_AMPLITUDE);	    break;
	case CMD_FADE3    : parameterSelect(PARAM_DIST_STRENGTH);  break;
	case CMD_FADE7    : parameterSelect(PARAM_FADE_SPEED);	    break;
	case CMD_FLASH    : parameterSelect(PARAM_SPEED);	    break;

	case CMD_B_UP     : rgbIncrease(0x00000f); break;
	case CMD_B_DOWN   : rgbDecrease(0x00000f); break;
	case CMD_G_UP     : rgbIncrease(0x000f00); break;
	case CMD_G_DOWN   : rgbDecrease(0x000f00); break;
	case CMD_R_UP     : rgbIncrease(0x0f0000); break;
	case CMD_R_DOWN   : rgbDecrease(0x0f0000); break;

	case CMD_DIY1     : handlePreset(1, repeat); break;
	case CMD_DIY2     : handlePreset(2, repeat); break;
	case CMD_DIY3     : handlePreset(3, repeat); break;
	case CMD_DIY4     : handlePreset(4, repeat); break;
	case CMD_DIY5     : handlePreset(5, repeat); break;
	case CMD_DIY6     : handlePreset(6, repeat); break;

	case CMD_R1       : setRGBBase(0xff0000); break;
	case CMD_G1       : setRGBBase(0x00ff00); break; 
	case CMD_B1       : setRGBBase(0x0000ff); break; 
	case CMD_WHITE    : setRGBBase(0xff96a5); break; // calibrated

	case CMD_R2       : setRGBBase(0xff2000); break;
	case CMD_G2       : setRGBBase(0x00ff40); break;
	case CMD_B2       : setRGBBase(0x4000ff); break;
	case CMD_PINKISH1 : setRGBBase(0xff7070); break;

	case CMD_R3       : setRGBBase(0xff3000); break;
	case CMD_G3       : setRGBBase(0x00ff80); break;
	case CMD_B3       : setRGBBase(0x8000ff); break;
	case CMD_PINKISH2 : setRGBBase(0xff5050); break;

	case CMD_R4       : setRGBBase(0xff4000); break;
	case CMD_G4       : setRGBBase(0x00ffc0); break;
	case CMD_B4       : setRGBBase(0xc000ff); break;
	case CMD_BLUISH1  : setRGBBase(0xf0f0ff); break;

	case CMD_R5       : setRGBBase(0xff6000); break;
	case CMD_G5       : setRGBBase(0x00ffff); break;
	case CMD_B5       : setRGBBase(0xff00ff); break;
	case CMD_BLUISH2  : setRGBBase(0xccccff); break;

	default:
		Serial.print("E:KEY:"); Serial.println(code, 16);
		return;

	}

	Serial.print("O:KEY:"); Serial.println(code, 16);
        scheduleSavingOfChanges();
	sendCurrent();
}


void show()
{
#ifdef STRIP_TYPE_6803        
        FastSPI_LED.show();
#endif  
#ifdef STRIP_TYPE_RGB
        // swap R and B like the LPD strip does so remote control works correctly
        analogWrite(PIN_PWM_R, leds[25].r);  
        analogWrite(PIN_PWM_G, leds[25].g);  
        analogWrite(PIN_PWM_B, leds[25].b);  
#endif
}

void stripClear(uint8_t r, uint8_t g, uint8_t b)
{
	byte rgb[3];
	rgb[0] = r;
	rgb[1] = g;
	rgb[2] = b;
	//scaleRGB888(rgb); 
	
	for (uint8_t x=0; x<WIDTH; x++)
	{
                leds[x].r = rgb[2];
                leds[x].g = rgb[1];
                leds[x].b = rgb[0];
          
	}
        show();
}


void fadeIn()
{
  short dR = (short) (255 - preset.offsetR);
  short dG = (short) (255 - preset.offsetG);
  short dB = (short) (255 - preset.offsetB);

  for (int s = 32; s > 0; s--)
  {
    short r = (short) (preset.offsetR + (s * dR) / 32);
    short g = (short) (preset.offsetG + (s * dG) / 32);
    short b = (short) (preset.offsetB + (s * dB) / 32);		
    stripClear(r,g,b);
    delay(50);
  }
}


void sendModeName(int mode)
{
        switch (mode)
        {
          case MODE_WATER     : Serial.print("Water");          break; // water simulation,  monochromatic, brightness varies
          case MODE_WATER2    : Serial.print("WaterColored");   break; // mostly monochromatic, but slight displacement on G/B channel causes some coloring
          case MODE_FADE      : Serial.print("Fading");         break; // like WATER2, however base color fades over time
          case MODE_RAINBOW   : Serial.print("Rainbow");        break; // displays whole rainbow range on the strip, otherwise like WATER
#ifdef STRIP_TYPE_6803
          case MODE_RAINBOW2  : Serial.print("RainbowLong");    break; // like RAINBOW, however displays only a subrange of the whole color range
          case MODE_RAINBOW3  : Serial.print("RainbowShort");   break; // like RAINBOW, however displays whole color range 2-3 times
          case MODE_RAINFLOW2 : Serial.print("RainFlowShort"); break; // like RAINBOW3, ripple data used as color displacement
#endif          
          case MODE_RAINFLOW  : Serial.print("RainFlow");       break; // like RAINBOW2, ripple data used as color displacement
          case MODE_CONST     : Serial.print("Constant");       break; // constant color only, but brightness may be set by the user
          default             : Serial.print("INVAL");          break;
        }
}


uint32_t rgbToUint32(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r)<<16) | (((uint32_t)g)<<8) | ((uint32_t)b);
}


void sendCurrent()
{
	Serial.print("G:");
	Serial.print("M="); Serial.print(preset.mode, 16); Serial.print(',');
        sendModeName(preset.mode);

        uint32_t rgb = rgbToUint32(preset.offsetR, preset.offsetG, preset.offsetB);
	Serial.print(" C="); Serial.print(rgb, 16);
	Serial.print(" S="); Serial.print(preset.delayTime,       16);
	Serial.print(" F="); Serial.print(preset.fadePrescale,    16);
	Serial.print(" B="); Serial.print(preset.brightness,      16);
	Serial.print(" A="); Serial.print(preset.amplitude,       16);
	Serial.print(" R="); Serial.print(preset.randomTreshold,  16);
	Serial.print(" T="); Serial.print(preset.disturbStrength, 16);
	Serial.println("");
}


#ifdef WITH_UART_VALUES_OUTPUT
void output_uart(int debug_level)
{
	if (0==debug_level)
		return;

	Serial.print("RGB "); Serial.print(frameCount);

	if (debug_level>1)
	{
		for (register uint8_t j=0; j<WIDTH; j++)
		{
                        uint32_t rgb = rgbToUint32(leds[j].r, leds[j].g, leds[j].b);
                        Serial.print(' '); Serial.print(rgb, 16);
		}
	}

	Serial.println("");
	Serial.flush();  

	if (debug_level>1)
	{
		delay(2);
	}
}
#endif

// the setup routine runs once when you press reset:
void setup() 
{                
	// initialize the digital pin as an output.
	Serial.begin(9600);        // set up Serial library at 9600 bps
	Serial.println("V:WaterStrip:1.0");  // prints verison with ending line break 
    
	// try to load saved presets. if there are none, fall back to defaults
	presetLoad(0, &preset);
	sendCurrent();

	// add some initial disturbance to ripple buffer
	for (int i=0; i<6; i++) disturb(1);

#ifdef STRIP_TYPE_6803        
        FastSPI_LED.setLeds(WIDTH);
        FastSPI_LED.setChipset(CFastSPI_LED::SPI_LPD6803);
        FastSPI_LED.init();
        FastSPI_LED.start();   
        leds = (struct CRGB*)FastSPI_LED.getRGBData();
#endif       
#ifdef STRIP_TYPE_RGB        
        int len = WIDTH*3;
        leds = (struct CRGB*)malloc(len);

        pinMode(PIN_PWM_R, OUTPUT);     
        pinMode(PIN_PWM_G, OUTPUT);     
        pinMode(PIN_PWM_B, OUTPUT);     
#endif 
        pinMode(PIN_LED,   OUTPUT);     

    // init IR receiver
	// done in constructor now: necIR.setup(PIN_IR_INT0);
        
	// strip is completely white initially. fade to start color now:
	//fadeIn();
	Serial.println("D:SETUP_COMPLETE");
	//Serial.println("D:INF_LOOP"); for (;;);

	sendHello();
}


void ackCmd(const char * cmd, long arg)
{
  Serial.print("O:"); Serial.print(cmd); Serial.print(":"); 
  Serial.print(arg,16); Serial.print(','); Serial.println(arg);
}


void sendHello()
{
  // Version, Kind, Power, Bright, Amplitude, Speed, Fade, Brightness, Random, sTrength, Color
  Serial.print("H:V=1 K=");
#if   defined STRIP_TYPE_6803
  Serial.print("LPD6803"); // drem/magic strip with LPD6803 controllers
#elif defined STRIP_TYPE_RGB
  Serial.print("ARGB"); // common Anode, RGB
#endif
  Serial.print(" P=0-2 B=0-FF A=0-FF S=0-FFF F=0-FFF R=0-FF T=0-FFF C=0-FFFFFF ");
  
  // supported mode numbers and names
  Serial.print("M=");
  for (int i=MODE_MIN; i<=MODE_MAX; i++)
  {
     Serial.print(i,16); Serial.print("=");
     sendModeName(i);
     Serial.print("\t"); 
  }
  Serial.println();
}

#ifdef WITH_HELP
void sendHelp()
{
  Serial.println("I:H|?|G - help,features,get curr");
  Serial.println("I:K=ir_key");
  Serial.println("I:P=power");
  Serial.println("I:M=mode");  
  Serial.println("I:C=color");
  Serial.println("I:B=brightn");
  Serial.println("I:A=ampl");
  Serial.println("I:S=speed");
  Serial.println("I:F=fade");
  Serial.println("I:R=random");
  Serial.println("I:T=strength");
  Serial.println("I:L=preset");
  Serial.println("I:W=preset");
  Serial.println("I:P=power");
}
#endif


long togglePin(byte pin, byte value)
{
  Serial.print("setting pin "); Serial.print(pin); Serial.print(" to "); Serial.println(value);
  analogWrite(pin, value);  
  return 1;
}


void checkSerialCommand()
{
      if (!stringComplete)
      {
        return;
      }

      //if (D>1) { Serial.print("D:LINE:'"); Serial.print(inputBuffer); Serial.println("'"); }

      long arg = 0;  
      if (('='==inputBuffer[1] || ':'==inputBuffer[1]) && 0!=inputBuffer[2])
      {
        char cmd = inputBuffer[0];
        char * c = inputBuffer+2;
        inputBuffer[1]=0;
        sscanf(c, "%lx", &arg);
        
        switch (cmd)
        {
          case 'P':
          setPower(arg);
          ackCmd("P", preset.power);
          break;
          
          default:
          if (preset.power==0)
          {
             Serial.println("EOFF");
             return;
          }
        }

        switch (cmd)
        {
        case 'K':
          handleIRCommand(arg, 0); 
          break;
        case 'M':
          preset.mode = restrict(arg, MODE_MIN, MODE_MAX);
          ackCmd("M", preset.mode);
          break;
        case 'C':
          arg &= 0xffffff;
          preset.offsetR = (arg & 0xff0000)>>16;
          preset.offsetG = (arg & 0x00ff00)>> 8;
          preset.offsetB = (arg & 0x0000ff);
          ackCmd("C", arg);
          break;
        case 'B':
          preset.brightness = restrict(arg, 0, 255);
          ackCmd("B", preset.brightness);
          break;
        case 'A':
          preset.amplitude = restrict(arg, 0, 255);
          ackCmd("A", preset.amplitude);
          break;
        case 'S':
          preset.delayTime = restrict(arg, 0, 1000);
          ackCmd("S", preset.delayTime);
          break;
        case 'F':
          preset.fadePrescale = restrict(arg, 0, 2000);
          ackCmd("F", preset.fadePrescale);
          break;
        case 'R':
          preset.randomTreshold = restrict(arg, 0, 100);
          ackCmd("R", preset.randomTreshold);
          break;
        case 'T':
          preset.disturbStrength = restrict(arg, 0, 400);
          ackCmd("S", preset.amplitude);
          break;
        case 'D':
          D = restrict(arg, 0, 2);
          ackCmd("D", D);
          break;
        case 'L':
          presetLoad(arg, &preset);
          preset.power = 1;
          ackCmd("L", D);
          break;
        case 'W':
          presetSave(arg);
          ackCmd("W", D);
          break;
        case 'Y':
        case 'y':
          {
            int pin, value;
            sscanf(c, "%i,%i", &pin, &value);
            long x = togglePin(pin, value);
            ackCmd("Y", x);
            break;
          }
        }

        scheduleSavingOfChanges();
      }
      else if (!strcmp("F", inputBuffer))
      {
        Serial.print("F:"); Serial.println(frameCount,16);
      }
      else if (!strcmp("H", inputBuffer))
      {
        sendHello();
      }
#ifdef WITH_HELP      
      else if (!strcmp("?", inputBuffer))
      {
        sendHelp();
      }
#endif        
      else if (!strcmp("G", inputBuffer))
      {
        sendCurrent();
      }
      else
      {
        Serial.print("E:INVAL:'"); Serial.print(inputBuffer); Serial.println("'");
      }
      
      // reset input buffer and ready flag:
      *(inputPos = inputBuffer) = 0;
      stringComplete=false;
}


void checkIRCommand()
{
	uint8_t  cmd;
	uint16_t rep;
	cmd = necIR.get_command(&rep);
  	handleIRCommand(cmd, rep);
}


void performSimulation()
{
	serialEvent(); // do frequent checks for serial data
    next();  // compute new frame
    
    serialEvent();
    visualize();     // compute RGB values from ripple data
    
    serialEvent();
    show();
    
    serialEvent();
    #ifdef WITH_UART_VALUES_OUTPUT
    output_uart(0);  // 0=silent, 1=only frame count, no delay, 2=values for simulation
    #endif
    
    serialEvent();
    swap();          // swap src and dst buffer
    disturb(false);  // add some random disturbance to current frame
    frameCount++;

    serialEvent();
}


void blink()
{
  long m = millis()/1000;
  digitalWrite(PIN_LED, 0==(m%2) ? HIGH : LOW);  
}


void serialEvent()
{
  if (Serial.available())
  {
    int n = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer)-1);
    if (0==n)
    {
      return;
    }
    else
    {
      inputBuffer[n] = 0;
      stringComplete = true;
    }
  }
  return; 
}

// the loop routine runs over and over again forever:
void loop() 
{
    blink();    
    
    #ifdef WITH_UART_PWM_OUTUT
    if (D>1)
    {
      uint32_t pwm = rgbToUint32(leds[25].r, leds[25].g, leds[25].b);
      if (pwm != old_pwm)
      {
        Serial.print("PWM:"); Serial.println(pwm, 16);
        old_pwm = pwm;
      }
    }
    #endif

    // handle IR commands
    #if 1
    serialEvent();
    checkIRCommand();
    #endif

    serialEvent();

    // do actual simulation and visualization
    if (preset.power>0)
    {
      if (!paused)
      {
        performSimulation();
      }
    }
    else
    {
      stripClear(0,0,0);
    }  
    
    serialEvent();

    for (uint16_t i=0; i<preset.delayTime; i++)
    {
      delay(1);
      serialEvent();
      checkSerialCommand();      
    }  
    
    // check at least once:
    checkSerialCommand();
    
    // if there were changes, save them after a while (after timeout expired)
    changeCheckSave();   
}


