#define DEBUG_IRNEC_LENGTHS

#include <RGBConverterFloat.h>
#include <EEPROM.h>
// required for LPD6803: TimerOne
#include <TimerOne.h>

#warning +---------------------------------------------------------------------------------+
#warning |Try to go with http://code.google.com/p/fastspi/source/browse/trunk/FastSPI_LED.h|
#warning +---------------------------------------------------------------------------------+
#include <LPD6803.h>
//#include <LPD8806.h>
//#include "SPI.h"

#include "ir_commands.h"
#include "NecIR.h"

// Number of RGB LEDs in strand:
#define WIDTH     50 // number of pixels/LEDs/sections????

#define SIGN(A)  ((A)<0 ? -1 : 1  )
#define ABS(A)   ((A)<0 ? (-(A)) : (A))
#define MAX(A,B) ((A)>(B) ? (A) : (B))    
#define MIN(A,B) ((A)<(B) ? (A) : (B))    

// modes of operation
#define MODE_WATER     0 // water simulation,  monochromatic, brightness varies
#define MODE_WATER2    1 // mostly monochromatic, but slight displacement on G/B channel causes some coloring
#define MODE_FUNKY     2 // very expensive yet colorful: convers tyo HSB and makes distortion shift hue
#define MODE_FADE      3 // like WATER2, however base color fades over time
#define MODE_RAINBOW   4 // displays whole rainbow range on the strip, otherwise like WATER
#define MODE_RAINBOW2  5 // like RAINBOW, however displays only a subrange of the whole color range
#define MODE_RAINBOW3  6 // like RAINBOW, however displays whole color range 2-3 times
#define MODE_RAINBOW4  7 // like RAINBOW2, ripple data used as color displacement
#define MODE_RAINBOW5  8 // like RAINBOW3, ripple data used as color displacement
#define MODE_CONST     9 // constant color only, but brightness may be set by the user 
#define MODE_MAX       9
#define MODE_DEFAULT   MODE_WATER

// define which parameter should be modified by keys QUICK/SLOW
#define PARAM_SPEED            1 // modify speed (FPS))
#define PARAM_RANDOM_TRESHOLD  2 // modify likeliness of disturbance
#define PARAM_DIST_AMPLITUDE   3 // modify strenght/apmlitude of disturbances
#define PARAM_FADE_SPEED       5 // fading speed for RAINBOW/FADE modes
#define PARAM_VISU_FACTOR      6 // controls degree of influence of disturbance on visualization

#define DIST_FACTOR_DEFAULT 196 // defines how much disturbances contribute to the LED brigtness
#define DIST_RAMP_UP_COUNT  8 // ramp up time (frames) until a disturbance fully applies to ripples buffer

#define RND_TRESHOLD_DEFAULT 5 // defines likeliness of disturbance to occur: 0 - never, 100 - always (in terms of frames)
#define RND_AMPLITUDE_DEFAULT 300

// Chose 2 pins for output; can be any valid output pins:
int dataPin  = 7;
int clockPin = 8;

// Chose pin where IR detector is connected (must fire INT0) 
int INT0_pin = 2; 

// First parameter is the number of LEDs in the strand.  The LED strips
// are 32 LEDs per meter but you can extend or cut the strip.  Next two
// parameters are SPI data and clock pins:
//LPD8806 strip = LPD8806(WIDTH, dataPin, clockPin);
LPD6803 strip = LPD6803(WIDTH, dataPin, clockPin);

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
	uint16_t disturbAmplitude; // aplitude/strength allows for random disturbances
	uint16_t offsetR;    // base value for R channel (0-255)
	uint16_t offsetG;    // base value for G channel
	uint16_t offsetB;    // base value for B channel
	uint16_t factor;     // factor 0-255 defines the impact of disturbances on the outpul values
	uint16_t brightness; // dimming factor: 0-dark, 255-max. brightness
	uint16_t delayTime;  // main delay controls effecive Fps rate
	uint16_t fadePrescale; // defines after how many sub ticks a tick of the fading counter gets incremented	
	uint16_t magic;      // magic 4711 saved in EEprom to check if anything was saved at all in a slot before
} 
preset_s;

// the preset that should be loaded after a timeout
int8_t  presetToLoad = -1;

// timout to tell appart whether the DIY button was just pressed or held down 
uint32_t presetLoadTime  = 0;
uint32_t lastChangeSaveTime = 0;


RGBConverterFloat rgbConverter;
NecIR necIR;

// all current values that control visialization and therefore must be saved in EEprom
preset_s preset;

uint16_t fadeCount     = 0;
uint16_t fadeSubCount  = 0;
uint8_t  paramSelected = PARAM_SPEED;

uint16_t frameCount = 0;


void initDefaults()
{
	preset.mode             = MODE_DEFAULT;
	preset.randomTreshold   = RND_TRESHOLD_DEFAULT;
	preset.disturbAmplitude =  150; // max. value to add as disturbance to ripple buffe
	preset.disturbWidth     =    6;
	preset.offsetR          =   80;
	preset.offsetG          =  120;
	preset.offsetB          =  220;
	preset.factor           = DIST_FACTOR_DEFAULT;
	preset.brightness       =  255;
	preset.delayTime        =   50;
	preset.fadePrescale     =   10;	
	preset.magic            = 4711;
}


// limit "val" to a value in 0,...,127 (strip uses 7 but values)
inline uint8_t limit(int16_t val)
{
	if (val>=255) return (uint8_t)255;
	if (val<=0) return 0;
	return (uint8_t)val;
}

int CYCLIC_DIST(int a, int b, int width)
{
	int d = ABS(a-b);
	if (d>width/2) d=ABS(d-width);
	return d;
}

/************** visualization rendering below *******************/

void scaleRGB888(byte rgb[]) 
{
	// convert to 5bit to mimic the limited color resolution of the real strip
	// (could mask also the three LSBs here)
	rgb[0]>>=3; rgb[1]>>=3; rgb[2]>>=3;
}

void applyBrightness(byte rgb[])
{
	rgb[0] = (byte) ((preset.brightness * rgb[0]) >> 8);
	rgb[1] = (byte) ((preset.brightness * rgb[1]) >> 8);
	rgb[2] = (byte) ((preset.brightness * rgb[2]) >> 8);
}

void visualizeWater()
{
	byte rgb[3]= {0,0,0};
	short baseScaledR = (short) ((230 * preset.offsetR) / 256);
	short baseScaledG = (short) ((230 * preset.offsetG) / 256);
	short baseScaledB = (short) ((230 * preset.offsetB) / 256);

	for (int x = 0; x < WIDTH; x++)
	{
		short rippleScaled = (short) ((preset.factor * dst[x]) / 256);

		switch (preset.mode)
		{
		case MODE_WATER:
			rgb[0] = limit(baseScaledR + rippleScaled);
			rgb[1] = limit(baseScaledG + rippleScaled);
			rgb[2] = limit(baseScaledB + rippleScaled);
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

			rgb[0] = limit(baseScaledR + rippleScaled);
			rgb[1] = limit(baseScaledG
					+ ((preset.factor * rippleG) >> 8));
			rgb[2] = limit(baseScaledB
					+ ((preset.factor * rippleB) >> 8));
			break;
		}
		}

		applyBrightness(rgb);

		scaleRGB888(rgb);
		// strip code assumes GBR but my strip is BGR
		strip.setPixelColor(x, rgb[2], rgb[1], rgb[0]);
	}
	return;
}

void visualizeFunky()
{
	byte  rgb[3]; // = new short[3];
	float hsv[3]; // = new float[3];
	// using static access to be in sync with C code. ignore warning
	rgbConverter.rgbToHsv(preset.offsetR, preset.offsetG, preset.offsetB, hsv);
	float hue = hsv[0]; 
	float sat = hsv[1];

	for (int x = 0; x < WIDTH; x++)
	{
		int rippleScaled = ((preset.factor * dst[x]) >> 8);
		float h = (float) (hue + 0.0025f * rippleScaled);

		rgbConverter.hslToRgb(h, sat, 0.5f, rgb);

		applyBrightness(rgb);

		scaleRGB888(rgb);
		// strip code assumes GBR but my strip is BGR:
		strip.setPixelColor(x, rgb[2], rgb[1], rgb[0]);
	}
}

void visualizeRainbow(short pos, short rippleScaled, int width, int midR, int midG, int midB, byte rgb[])
{
	int i = (int) ((pos + fadeCount) % width);
	// this one
	// rgb[0] = limit(240*CYCLIC_DIST(i, midR, width)/(width/2) + rippleScaled);
	// is equivalent to the below but saves passing one argument: (with/2)
	rgb[0] = limit(240 * 2 * CYCLIC_DIST(i, midR, width) / width + rippleScaled);
	rgb[1] = limit(240 * 2 * CYCLIC_DIST(i, midG, width) / width + rippleScaled);
	rgb[2] = limit(240 * 2 * CYCLIC_DIST(i, midB, width) / width + rippleScaled);
}


void visualizeRainbow2(short pos, short rippleScaled, int width, int midR, int midG, int midB, byte rgb[])
{
	int i = (int) (((width<<3) + fadeCount + pos + rippleScaled) % width); // using ripple as displacement 
	// this one
	// rgb[0] = limit(240*CYCLIC_DIST(i, midR, width)/(width/2) + rippleScaled);
	// is equivalent to the below but saves passing one argument: (with/2)
	rgb[0] = limit(240 * 2 * CYCLIC_DIST(i, midR, width)/width + rippleScaled);
	rgb[1] = limit(240 * 2 * CYCLIC_DIST(i, midG, width)/width + rippleScaled);
	rgb[2] = limit(240 * 2 * CYCLIC_DIST(i, midB, width)/width + rippleScaled);
}

void visualize()
{
	switch (preset.mode)
	{
	case MODE_FUNKY:
		visualizeFunky();
		return;
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
		short rippleScaled = (short) ((preset.factor * dst[x]) >> 8);

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
			visualizeRainbow(x, rippleScaled, (3 * WIDTH),
					1 * (3 * WIDTH) / 6, 3 * (3 * WIDTH) / 6,
					5 * (3 * WIDTH) / 6, rgb);
			break;

		case MODE_RAINBOW2:
			// many arguments here that could be computed within
			// visualizeRainbow however
			// this won't allow the compiler to optimze at compile time
			visualizeRainbow(x, rippleScaled, WIDTH, 1 * WIDTH / 6,	3 * WIDTH / 6, 5 * WIDTH / 6, rgb);
			break;

		case MODE_RAINBOW3:
			visualizeRainbow(x, rippleScaled, (2 * WIDTH / 3), 1 * (2 * WIDTH / 3) / 6, 3 * (2 * WIDTH / 3) / 6, 5 * (2 * WIDTH / 3) / 6, rgb);
			break;

		case MODE_RAINBOW4:
			visualizeRainbow2(x, rippleScaled>>3, (WIDTH), 1*(WIDTH)/6, 3*(WIDTH)/6, 5*(WIDTH)/6, rgb);
			break;

		case MODE_RAINBOW5:
			visualizeRainbow2(x, rippleScaled>>2, (3*WIDTH), 1*(3*WIDTH)/6, 3*(3*WIDTH)/6, 5*(3*WIDTH)/6, rgb);
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

		scaleRGB888(rgb);
		// strip code assumes GBR but my strip is BGR
		strip.setPixelColor(x, rgb[2], rgb[1], rgb[0]);
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
	if (!force && random(100)>=preset.randomTreshold)
	{
		return;
	}

	int x       = random(WIDTH); // pixel position where disturbance starts 	
	int h_total = (preset.disturbAmplitude>>1)-random(preset.disturbAmplitude);	
	if (true)
	{	
		int width = random(preset.disturbWidth)<<1; // pixel influenced: 1,...,6
		int h_pp  = h_total/width;
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
inline void pause() 
{
	paused = paused ? false : true;
}

inline void modeToggle() 
{
	preset.mode = (short) ((preset.mode + 1) % (MODE_MAX + 1));
	fadeCount = fadeSubCount = 0;
}

void resetRipples() 
{
	preset.randomTreshold = 0;
	for (int i=0; i<WIDTH; i++)
	{
		ripples1[i] = 0;
		ripples2[i] = 0;
		dist_height[i] = 0;
		dist_count[i] = 0;
	}	
	Serial.println("*** RIPPLES RESET ***");
}

void resetDefaults()
{
	paused = false;
	resetRipples();
	initDefaults();
	Serial.println("*** DEFAULTS RESET ***");
}

void brightnessIncrease() 
{
	preset.brightness = MIN(255, preset.brightness+20);
}

void brightnessDecrease() 
{
	preset.brightness = MAX(1, preset.brightness-20);
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

	preset.factor = DIST_FACTOR_DEFAULT; //-offsetR;

	fadeCount = 0;

	switch (preset.mode)
	{
	case MODE_RAINBOW:
		preset.mode = MODE_DEFAULT;
		break;
	}
}

void indicateFeedback()
{
	for (byte n=0; n<3; n++)
	{
	    stripClear((short)255,(short)255,(short)255);
	    delay(100);
	    
	    stripClear((short)0,(short)0,(short)0);
	    delay(100);
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

void presetSave(int n) 
{
	Serial.print("SAVING "); Serial.print(" PRESET "); Serial.println(n);  
	// load current presets saved below index n
	struct preset_s old;
	presetLoad(n, &old);
	
	// save only if there is a difference
	if (presetsDiffer(&old, &preset))
	{
		int len = sizeof(preset);
		byte * b = (byte*)&preset;
		int addr = n*len;

		Serial.print("WRITING "); Serial.print(len); Serial.print(" BYTES TO ADDR "); Serial.println(addr);  
		for (int i=0; i<len; i++)
		{
			EEPROM.write(addr, *b);
			b++;
			addr++;    
		}
	}
	else
	{
		Serial.println("NO CHANGES");
	}
}

void presetLoad(int n, struct preset_s * preset)
{
	Serial.print("LOADING "); Serial.print(" PRESET "); Serial.println(n);  
	int len = sizeof(*preset);
	byte * b = (byte*)preset;
	int addr = n*len;

	Serial.print("READING "); Serial.print(len); Serial.print(" BYTES FROM ADDR "); Serial.println(addr);  
	for (int i=0; i<len; i++)
	{
		*b = EEPROM.read(addr);
		b++;
		addr++;    
	}
}

void handlePreset(int i, int repeat)
{
	if (repeat<10) 
	{
		Serial.print("SCHEDULING "); Serial.println(i);
		presetToLoad = (byte) i;
		presetLoadTime = uptime_millies + 1000;	    
	}
	else if (10==repeat)
	{
		presetSave(i);
		indicateFeedback();
	}
}

void presetCheckLoad()
{
	if (presetToLoad>-1 && presetLoadTime>0 && uptime_millies>presetLoadTime)
	{
	    Serial.print("LOADING "); Serial.println(presetToLoad);
		presetLoad(presetToLoad, &preset);
		presetToLoad = -1;
	}
}

void changeCheckSave() 
{
	if (lastChangeSaveTime>0 && uptime_millies>lastChangeSaveTime) 
	{
		presetSave(0);  
		lastChangeSaveTime = 0;
	}
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
		preset.randomTreshold   = MIN(100, preset.randomTreshold+2);    
		break;
	case PARAM_DIST_AMPLITUDE:  
		preset.disturbAmplitude = MIN(400, preset.disturbAmplitude+5); 
		break;
	case PARAM_VISU_FACTOR:
		preset.factor           = MIN(255, preset.factor+10);
		break;
	case PARAM_SPEED:           
		preset.delayTime        = MAX(0,   preset.delayTime-10);        
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
	case PARAM_DIST_AMPLITUDE:  
		preset.disturbAmplitude = MAX(1,   preset.disturbAmplitude-5); 
		break;
	case PARAM_VISU_FACTOR:
		preset.factor           = MAX(0, preset.factor-10);
		break;
	case PARAM_SPEED:           
		preset.delayTime        = MIN(250, preset.delayTime+10);
		break;
	case PARAM_FADE_SPEED:      
		preset.fadePrescale     = MIN(20,  preset.fadePrescale+1);         
		break;
	}	
}

void handleCommand(int code, int repeat) 
{
	Serial.print("GOT CMD: "); Serial.print(code);  Serial.print(" RPT: "); Serial.println(repeat);
	
	switch (code)
	{
	case 0: // no command
		changeCheckSave();	 // save changes after timout expired
		presetCheckLoad();   // load presets (DIYx) after button was released few millies ago
		return; // to avoid setting ticksUntilSave at end

	case CMD_POWER    : 
		disturb(true); 
		preset.randomTreshold=RND_TRESHOLD_DEFAULT; 
		break;

	case CMD_PAUSE    : 
		if (repeat<6) 
			pause(); 
		else 
			resetDefaults();  break;

	case CMD_JUMP3   : 
		parameterSelect(PARAM_RANDOM_TRESHOLD); 
		if (repeat>10) resetRipples(); 
		break;

	case CMD_AUTO:	    	modeToggle();	   	    break;
	case CMD_LIGHTER:	brightnessIncrease();	    break;
	case CMD_DARKER:	brightnessDecrease();	    break;	
	case CMD_QUICK:	   	parameterIncrease();	    break;
	case CMD_SLOW:	    	parameterDecrease();	    break;
	case CMD_JUMP7:	    parameterSelect(PARAM_DIST_AMPLITUDE);  break;
	case CMD_FADE3:	    parameterSelect(PARAM_VISU_FACTOR);	    break;
	case CMD_FADE7:	    parameterSelect(PARAM_FADE_SPEED);	    break;
	case CMD_FLASH:	    parameterSelect(PARAM_SPEED);	    break;

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
	case CMD_WHITE    : setRGBBase(0xffffff); break;

	case CMD_R2       : setRGBBase(0xff4000); break;
	case CMD_G2       : setRGBBase(0x00ff40); break;
	case CMD_B2       : setRGBBase(0x4000ff); break;
	case CMD_PINKISH1 : setRGBBase(0xeeeeff); break;

	case CMD_R3       : setRGBBase(0xff8000); break;
	case CMD_G3       : setRGBBase(0x00ff80); break;
	case CMD_B3       : setRGBBase(0x8000ff); break;
	case CMD_PINKISH2 : setRGBBase(0xddddff); break;

	case CMD_R4       : setRGBBase(0xffc000); break;
	case CMD_G4       : setRGBBase(0x00ffc0); break;
	case CMD_B4       : setRGBBase(0xc000ff); break;
	case CMD_BLUISH1  : setRGBBase(0xffeeee); break;

	case CMD_R5       : setRGBBase(0xffff00); break;
	case CMD_G5       : setRGBBase(0x00ffff); break;
	case CMD_B5       : setRGBBase(0xff00ff); break;
	case CMD_BLUISH2  : setRGBBase(0xffdddd); break;

	default:
		char buf[32];
		sprintf(buf,"UNKNOWN CMD %02x", code);
		Serial.print(buf);
		return;

	}

	lastChangeSaveTime = uptime_millies+5000; // save after 5s

	dumpParameters();
}


void stripClear(uint8_t r, uint8_t g, uint8_t b)
{
	byte rgb[3];
	rgb[0] = r;
	rgb[1] = g;
	rgb[2] = b;
	scaleRGB888(rgb); 
	
	for (uint8_t i=0; i<WIDTH; i++)
	{
		strip.setPixelColor(i, rgb[2], rgb[1], rgb[0]);
	}
	strip.show();
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
		delay(10);
	}
}


void dumpParameters()
{
	char buf[64];
	Serial.print("PARAMS ");
	sprintf(buf, "M:%i ", preset.mode); Serial.print(buf);
	sprintf(buf, "RGB:%02x%02x%02x ", preset.offsetR, preset.offsetG, preset.offsetB); Serial.print(buf);
	sprintf(buf, "(%i) ",    preset.factor); Serial.print(buf);
	sprintf(buf, "BRI:%i",   preset.brightness); Serial.print(buf);
	sprintf(buf, "RND:%i",   preset.randomTreshold); Serial.print(buf);
	sprintf(buf, "AMP:%i",   preset.disturbAmplitude); Serial.print(buf);
	sprintf(buf, "DELAY:%i", preset.delayTime); Serial.print(buf);
	sprintf(buf, "FADE:%i",  preset.fadePrescale); Serial.print(buf);
	Serial.println("");
}


void output_uart(int debug_level)
{
	if (0==debug_level)
		return;

	char buf[64];
	sprintf(buf, "RGB %d", frameCount);
	Serial.print(buf);

	if (debug_level>1)
	{
		for (register uint8_t j=0; j<WIDTH; j++)
		{
			uint16_t grb = strip.getPixelColor(j);
			// grb is 5,5,5 bits = 0x7fff max
			sprintf(buf, " %04x", grb);
			Serial.print(buf);
		}
	}

	Serial.println("");
	Serial.flush();  

	if (debug_level>1)
	{
		delay(2);
	}
}


void debug_timer()
{
          long cpumax = 50;
          // dreisatz: ms = 100% * (benötigte zeit pro bit) / max%, 
          // jedes bit braucht 20us, bei 50% cpu -> 100*20/50 = 40us -> 25.000bit/s -> 25.000bit/50 LEDS/16bitz = 31.25 Hz
          // bei 40% cpu: -> 100*20/40 = 50us  = 20kHz -> 25Hz
          // bei 20$ cpu: -> 100*20/20 = 100us = 10kHz -> 12.5Hz
          // Achtung: 20us/bit nicht mehr aktuell durch uptime_millies und &ticks
          long time = 100;
          time *= 20;   // 20 microseconds per bitout
          time /= cpumax;    // how long between timers
          Serial.print("time: "); Serial.println(time);
          // time: 40
          
          //Timer1.initialize(microseconds = time);
          long microseconds = time;
          
          #define RESOLUTION 65536    // Timer1 is 16 bit

          //F_CPU = 16000000 ;
          long cycles = (F_CPU / 2000000) * microseconds;  // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
          Serial.print("cycles: "); Serial.println(cycles);
          // cycles: 320
                              
          const char * prescale = "?";
          if (cycles < RESOLUTION)             prescale = "full xtal";
          else if  ((cycles >>= 3) < RESOLUTION) prescale = "/8";
          else if((cycles >>= 3) < RESOLUTION) prescale = "/64";
          else if((cycles >>= 2) < RESOLUTION) prescale = "/256";
          else if((cycles >>= 2) < RESOLUTION) prescale = "/1024";
          else        cycles = RESOLUTION - 1, prescale = "max";  // request was out of bounds, set as maximum
        
          Serial.print("setPeriod: prescale: "); Serial.println(prescale); 
          Serial.print("setPeriod: cycles: "); Serial.println(cycles); 
}

// the setup routine runs once when you press reset:
void setup() 
{                
	// initialize the digital pin as an output.
	//pinMode(led, OUTPUT);     
	Serial.begin(57600);        // set up Serial library at 9600 bps
	Serial.println("Water1D");  // prints hello with ending line break 
    
        Serial.print("Timer1.cycles: "); Serial.println(Timer1.cycles);
        Serial.print("Timer1.clockSelectBits: "); Serial.println(Timer1.clockSelectBits);
        
        debug_timer();

	// try to load saved presets. if there are none, fall back to defaults
	Serial.println("Loading preset 0");
	presetLoad(0, &preset);
	if (4711!=preset.magic)
	{
		Serial.println("Magic not found");
		Serial.println("Init to defaults");
		initDefaults();
	}

	// add some initial disturbance to ripple buffer
	for (int i=0; i<6; i++) disturb(1);


	// Start up the LED strip
	#warning check if this changes the interrupt frequency and enable if not so
        #warning sure it does. check if the new frequency computation code works instead.
        // 20% should be enough for 12.5 Hz, 40% for 25Hz update freq (for 50 pixels)
	//strip.setCPUmax(20);  // start with 50% CPU usage. up this if the strand flickers or is slow
        uint32_t frequency = strip.getFrequency();
        Serial.print("Strip frequency: "); Serial.println(frequency);

	strip.begin();

	// set up data and clock for output
	pinMode(dataPin,  OUTPUT);     
	pinMode(clockPin, OUTPUT);     

        // init IR receiver
	necIR.setup(INT0_pin, &ticks, frequency);

	// strip is completely white initialy. fade to start color
	fadeIn();
}


// the loop routine runs over and over again forever:
void loop() 
{
#if 1
	long t=ticks; 
	ticks=0;

	uint8_t  cmd;
	uint16_t rep, err;
	cmd = necIR.get_command(&rep);
	err = necIR.get_errors();
	if (err)
	{
		char buf[128];
		sprintf(buf,"ERRS: %i", err);
		Serial.println(buf);
	}

	if (Serial.available()>0)
	{
		cmd = Serial.read();
	}

	if (cmd>0)
	{
		char buf[128];
		sprintf(buf,"CMD %02x, rep: %i, ticks: %ld", cmd, rep, t);
		Serial.println(buf);
	}

	necIR.dump_lengths();
	handleCommand(cmd, rep);
#endif

#if 1
	next();  // compute new frame  
	visualize();     // compute RGB values from ripple data
	strip.show();    // Refresh LED states
	output_uart(0);  // 0=silent, 1=only frame count, no delay, 2=values for simulation

	swap();          // swap src and dst buffer
	disturb(false);  // add some random disturbance to current frame
	frameCount++;
#endif

	delay(preset.delayTime);  
}



