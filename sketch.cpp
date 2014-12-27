/*
 * Copyright 2012 Adam Cooper, derived from the work of Elco Jacobs.
 * See http://www.elcojacobs.com/easy-to-use-atmel-studio-project-for-arduino-and-programming-the-arduino-from-python/
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ShiftPWM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#include <IRremote.cpp>

#include <EEPROM.cpp>
#include "EEPROMUtils.cpp"
#include "I2CUtils.cpp"
#include <Wire.cpp>
#include <twi.c>

#include "ShapedBrightnessController.cpp"

// Function prototypes go here (telling the compiler these functions exist).
void readSourceValues();
void updateTGM();
void programTriple(uint8_t rgb, uint8_t shape, int phase, uint8_t rateSrc, uint8_t scaleSrc, uint8_t tgSrc);
void programAll(uint8_t shape, uint8_t rateSrc, uint8_t scaleSrc, uint8_t tgSrc);
		
		void printPatch(uint8_t led);
		void printPatternBytes(uint8_t led);
		void loadProgram(uint8_t programNumber);
		uint8_t decodeNumIR(unsigned long irCode);

uint16_t getSrcVal(uint8_t src);
void stepButton(uint8_t src);


/*
 * Place a direct direct copy of sketch (*.ino or *.pde) beneath this comment block.... except that you:
 * 1: might have to add some function prototypes to the top, because the Arduino environment does this automatically but Atmel Studio does not.
 * 2: might have to change the #include to the .cpp rather than .h (check which one includes the other. Arduino IDE seems able to work it out!)
 */

//comment out to suppress debug messages to Serial and to exclude unit tests from compilation
//unit test mode is toggled with IR_TEST
#define DEBUG

//number of LEDS in use. ShapedBrightnessController has a compilation #define max value = 9
#define NUM_LEDS 3
//number of LEDs determins EEPROM program size, which in turn determines the number of programs available
#define PROG_BYTES (8+NUM_LEDS*8)
#define MAX_PROG_NUM (int)(2040/PROG_BYTES) //assumes 2k EEPROM with 8 bytes of space at the start (byte 0 stores num LEDs)

// Input wiring details. Which pin is connected to which logical input
#define PIN_AUDIO A0 //audio level (ADC0)
#define PIN_LEV1 A1 //VR1 (ADC1)
#define PIN_LEV2 A2 //VR2 or thermister (ADC2)
#define PIN_LEV3 A3 //VR3 or LDR (ADC3)
#define PIN_SW1 2 //switch 1. NB the numbering is ARDUINO-style
#define PIN_SW2 3 //switch 2
#define PIN_SW3 4 //switch 3
#define PIN_IR 10 //IR receiver
#define PIN_PROG 9 //switch to put into programming mode
#define PIN_ACT 5 //"active" LED output

// array to hold source values, e.g. ADC readings, and the indeces of each source.
//Most are re-populated periodically (but not necessarily on each loop) or on an event
// Va;lues are always in the range 0-1023
uint16_t srcVals[16+NUM_LEDS]={0,1023,512,0,0,0,0,0,0,0,512,512,512,512};
#define SRC_OFF 0 //permanently OFF, i.e. value = 0
#define SRC_ON 1 //always ON, i.e. value =1023
#define SRC_HALF 2 //always value = 512
#define SRC_AUDIO 3 //audio level (ADC0)
#define SRC_LEV1 4 //VR1 (ADC1)
#define SRC_LEV2 5 //VR2 or thermister (ADC2)
#define SRC_LEV3 6 //VR3 or LDR (ADC3)
//the next six should be continguous and in the same order
#define SRC_STEP1 7 //button 1 | IR controlled button #1 ... buttons step through values 0, 255, 510, 765, 1020
#define SRC_STEP2 8 //button 2 | IR controlled button #2
#define SRC_STEP3 9 //button 3 | IR controlled button #3
#define SRC_IR_INT1 0xA //IR controlled integer value #1
#define SRC_IR_INT2 0xB //IR controlled integer value #2
#define SRC_IR_INT3 0xC //IR controlled integer value #3
#define SRC_LFO 0xD //Low Freq osc (triangle wave)
#define SRC_RND_1S 0xE //Random number in range 0-1023, changing every second
#define SRC_RND_10S 0xF //Random number in range 0-1023, changing every 10 seconds
#define SRC_TG_MASK_BASE 0x10 //srcVals index at which the trigger/gate mask values reside (there are NUM_LEDS of them).
//NB TG_MASK is, in principle, available for rate and scale patches, but is NOT intended for use that way
#define SRC_CONST 0x80 //effectively a bit indicator that the (lowest 7 bits <<3) is a "constant value source". #defined here mostly as documentation

// array to hold the patches - i.e. the mapping from the values in srcVals to parameters passed to the ShapedBrightnessController
// each LED has 3 parameters, with the precise details of how they affect the brightness over time being determined by the pattern
//	setting in force for the LED (which may change over time if a sequence has been programmed)
uint8_t patches[NUM_LEDS][3];//the value in each cell is an index into srcVals
//the second index of patches:
#define PAR_RATE 0 // - see ShapedBrightnessController.setRate()
#define PAR_SCALE 1 //  - see ShapedBrightnessController.setScale()
#define PAR_TG_IP 2 //trigger or gate input - see ShapedBrightnessController.setTriggerIP()
// multiplier for patches[][PAR_RATE], +2 means bit shift the srcVal two places to the MSB, -2 means shift 2 places towards LSB
signed char rateFactor[NUM_LEDS];

//IR class and data
IRrecv irrecv(PIN_IR);
decode_results results;
//next 4 to store received commands
unsigned long irLast;//used to read in the last received value
unsigned long irCommand;//used to store the active command (i.e. the command that started a sequence of key presses)
uint8_t irSrc = SRC_STEP1;//which src is currently affected by PLUS/MINUS inputs (in run mode)
uint8_t irTens;//for numeric inputs, the decoded value in the tens column
uint8_t irUnits;//ditto units
// codes for remote control buttons. This is based on the ubiquitous "car MP3" sender, but the button names
// as used in this code are generalised. The "car MP3" button names appear in the comment
#define IR_N0 0xFF6897 //0
#define IR_N1 0xFF30CF //1
#define IR_N2 0xFF18E7 //2
#define IR_N3 0xFF7A85 //3
#define IR_N4 0xFF10EF //4
#define IR_N5 0xFF38C7 //5
#define IR_N6 0xFF5AA5 //6
#define IR_N7 0xFF42BD //7
#define IR_N8 0xFF4AB5 //8
#define IR_N9 0xFF52AD //9
#define IR_CH_PLUS 0xFFE21D //CH+
#define IR_CH_MINUS 0xFFA25D //CH-
#define IR_CH 0xFF629D //CH
#define IR_NEXT 0xFF02FD //NEXT
#define IR_PREV 0xFF22DD //PREV
#define IR_PLUS  0xFFA857 //VOL+
#define IR_MINUS 0xFFE01F //VOL-
#define IR_PLAY 0xFFC23D //Play/Pause
#define IR_PROG 0xFF906F //EQ
#define IR_CANCEL 0xFF9867 //100+
#define IR_OK 0xFFB04F //200+

//Reading/writing programs from EEPROM
uint8_t programCount;//number of programs in EEPROM
uint8_t currentProgram;//the index of the currently-loaded program.

//Program cycling. The program changes at intervals, indicated by pcRateSrc
//cycling uses currentProgram, which is fixed in the range 1..programCount
uint8_t pcRateSrc;//the contents of EEPROM address 0x02, which is a value from SRC_*
uint16_t pcCounter=0;////gets getSrcVal(pcRateSrc) added each tick. When exceeds 2048 the mask changes by one step and counter resets to 0
boolean pcActive = true;//cycling active

//object for time-varying LED controllers
ShapedBrightnessController sbc = ShapedBrightnessController(NUM_LEDS);

//Random changes
unsigned long lastRandChange1;
unsigned long lastRandChange10;

//A Low Frequency Osc (LFO) - triangle form - can be used as a SRC in a patch, but also has its freq controlled by SRC value via the special LFO patch
uint8_t lfoRateSrc;
uint16_t lfoCounter=0;

//The "trigger/gate mask" provides an on/off time-varying pattern (e.g. moving dot, bar, etc) that feeds SRC_TG_MASK_BASE+led
//and can be used as a patch source. Its rate can be set by other SRC inputs.
uint16_t tgMask=1;//stores the bit mask
uint8_t runLength = NUM_LEDS;
uint16_t tgMaskMask=(1<<NUM_LEDS)-1; //tgMask is forced to = tgMask & tgMaskMask in some cases
uint16_t tgmCounter=0;//gets tgmRate added each tick. When exceeds 2048 the mask changes by one step and counter resets to 0
uint16_t tgmRateSrc=1;//source for rate of change. Any valid SRC including CONST
uint8_t tgmPattern=0;// stores the active change pattern - see the following #defines. //NB: top nibble assumed to hold modifiers, low nibble to code for basic pattern
bool tgmAB=true;//some patterns alternate. This keeps whether A or B motion is in force.
//mask change patterns
#define TGM_DISABLED 0 //no not use trigger gate mask (this is not strictly necessary since, TGM only has effect if relevant elements in srcVals are patched to a LED controller
#define TGM_SINGLE 1 //single bit on
#define TGM_GROW 2 //bits progressively turn on to MSB then turn off from MSB
#define TGM_PASS 3 //bits progressively turn on to MSB then turn off from LSB
#define TGM_DOUBLE 4// like SINGLE 2 bits on
#define TGM_TRIPLIFY 128 //add this to treat the change pattern as an RGB pattern and replicate across all RGB triples

boolean lastButton[3];
unsigned long lastTickMillis;

void setup(){
	//pin modes
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
	pinMode(PIN_PROG,INPUT_PULLUP);
	pinMode(PIN_IR, INPUT);
	
	pinMode(PIN_ACT, OUTPUT);
	
	#ifdef DEBUG
	Serial.begin(9600);
	Serial.println("Start");
	#endif
	
	//initialise the pwm
	sbc.initialise();
	
	// Start the ir receiver
	irrecv.enableIRIn();
	
	//check the EEPROM for programs
	//first byte is number of LEDs in the programs. Must match NUM_LEDS otherwise there are 0 programs available.
	if(EEPROM.read(0)==NUM_LEDS){
		programCount = EEPROM.read(1);
		pcRateSrc = EEPROM.read(2);
	}else{
		//correct the error, and assert 0 programs
		EEPROM.write(0,NUM_LEDS);
		EEPROM.write(1,0);
		programCount =0;
		pcActive=false;
	}
	
	//Load program 1 if available, otherwise a default program
	if(programCount>0){
		currentProgram=1;
		loadProgram(1);
	}else{
		sbc.setPattern(0, SBC_WAVESHAPE_SAW + SBC_WSMOD_INVERT, 0);//saw wave
		patches[0][PAR_RATE] = SRC_LEV1;//VR1 controls rate
		rateFactor[0]=0;//rate is not shifted
		patches[0][PAR_SCALE] = SRC_ON;// half scale brightness range
		patches[0][PAR_TG_IP] =  SRC_ON;//the waveform change is un-gated
		//LED GREEN 1
		sbc.setPattern(1, SBC_WAVESHAPE_OFF, 0);
		patches[1][PAR_RATE] = SRC_OFF;
		rateFactor[1]=0;
		patches[1][PAR_SCALE] = SRC_OFF;
		patches[1][PAR_TG_IP] =  SRC_OFF;
		//LED BLUE 1
		sbc.setPattern(2, SBC_WAVESHAPE_OFF, 0);//phase shifted 240 deg
		patches[2][PAR_RATE] = SRC_LEV3;
		rateFactor[2]=0;
		patches[2][PAR_SCALE] = SRC_OFF;
		patches[2][PAR_TG_IP] =  SRC_OFF;
	}
	
	////LED RED 1
	//sbc.setPattern(0, SBC_WAVESHAPE_SAW, 0);//saw wave
	//patches[0][PAR_RATE] = SRC_LEV1;//VR1 controls rate
	//rateFactor[0]=0;//rate is not shifted
	//patches[0][PAR_SCALE] = SRC_ON;// full scale brightness range
	//patches[0][PAR_TG_IP] =  SRC_ON;//the waveform change is un-gated
	////LED GREEN 1
	//sbc.setPattern(1, SBC_WAVESHAPE_SAW, 683);//phase shifted 120 deg
	//patches[1][PAR_RATE] = SRC_LEV2;
	//rateFactor[1]=0;//rate is not shifted
	//patches[1][PAR_SCALE] = SRC_ON;// full scale brightness range
	//patches[1][PAR_TG_IP] =  SRC_ON;//the waveform change is un-gated
	////LED BLUE 1
	//sbc.setPattern(2, SBC_WAVESHAPE_SAW, 1365);//phase shifted 240 deg
	//patches[2][PAR_RATE] = SRC_LEV3;
	//rateFactor[2]=0;//rate is not shifted
	//patches[2][PAR_SCALE] = SRC_ON;// full scale brightness range
	//patches[2][PAR_TG_IP] =  SRC_ON;//the waveform change is un-gated	
	
	//flash activity pin to show we are alive
	digitalWrite(PIN_ACT, HIGH);
	delay(700);
	digitalWrite(PIN_ACT, LOW);
	
	lastTickMillis = millis();

}

void loop(){
	readSourceValues();
	
	//pass the latest source values AND trigger a brightness update "tick" at around 16Hz
	// note that it is NOT necessary to pass the source values each tick; the previous vals remain in force until changed
	uint16_t rate;
	if((millis()-lastTickMillis) > 62){
		
		//if not waiting for program load commands, and program cycling is active then process the program cycling rules
		if(pcActive && (irCommand == 0)){
			pcCounter+=(getSrcVal(pcRateSrc)>>5);
			if(pcCounter>=2048){
				currentProgram++;
				if(currentProgram>programCount){
					currentProgram = 1;
				}
				loadProgram(currentProgram);
				pcCounter = 0;
			}
		}
		
		//check IR receiver.
		if (irrecv.decode(&results)) {
			irLast = results.value;
			irrecv.resume(); // Receive the next value
			//only proceed if it was not a "repeat last value" code
			if(irLast!=0xFFFFFFFF){
				#ifdef DEBUG
				Serial.println(irLast, HEX);//value is an unsigned long
				#endif
				boolean loadOK=false;//gets set to true if prog is to be loaded (several routes to this situation)
				//if current command is PROG then need up to two numbers terminated by OK (or cancel to finish)
				if(irCommand == IR_PROG){
					if(irLast == IR_OK){//go and load a program from EEPROM if a valid prog number was entered
						uint8_t progNum=irTens*10 + irUnits;
						if(progNum>0 && progNum<=programCount){
							loadOK=true;
							currentProgram=progNum;
						}
						irCommand=0;//also cancel the active command
					}else if(irLast == IR_NEXT){
						currentProgram++;
						if(currentProgram>programCount){
							currentProgram=1;
						}
						loadOK = true;
					}else if (irLast == IR_PREV){
						currentProgram--;
						if(currentProgram==0){
							currentProgram=programCount;
						}				
						loadOK = true;		
					}else if(irLast == IR_CANCEL){
						irCommand = 0;
					}else if(irLast == IR_PLAY){
						pcActive = !pcActive;
						irCommand = 0;
					}else{
						uint8_t irNumber = decodeNumIR(irLast);//gets the decimal number for the last key. returns 255 if not a number
						#ifdef DEBUG
						Serial.println(irNumber, DEC);
						#endif
						if(irNumber!=255){
							irTens = irUnits;
							irUnits=irNumber;
							//blink off to ack the number
							digitalWrite(PIN_ACT, LOW);
							delay(200);
							digitalWrite(PIN_ACT, HIGH);
						}
					}
					//
					if(loadOK){						
						loadProgram(currentProgram);
						irCommand=0;//also cancel the active command
					}
				}
				
				//does the last value start a PROG sequence? Only do it if there are programs.
				if(irLast == IR_PROG){
					if(programCount>0){
						irCommand = IR_PROG;
						irTens = 0;
						irUnits = 0;
					}
				}else if(irLast == IR_CH_MINUS){
					irSrc--;
					if(irSrc<SRC_STEP1)irSrc = SRC_IR_INT3;
				}else if(irLast == IR_CH_PLUS){
					irSrc++;
					if(irSrc>SRC_IR_INT3)irSrc = SRC_STEP1;
				} else if((irLast == IR_MINUS) || (irLast == IR_PLUS)){
					//change STEP or IR_INT srcVals
					int step=(irSrc<SRC_IR_INT1)?255:32;
					int val = (int)srcVals[irSrc];
					if(irLast == IR_MINUS){
						val-=step;
						if(val<0) val = 0;
					}else{
						val+=step;
						if(val>=1024) val = 1023;
					}
					srcVals[irSrc] = (uint16_t)val;
						//flash activity pin to ack
						digitalWrite(PIN_ACT, HIGH);
						delay(100);
						digitalWrite(PIN_ACT, LOW);
					#ifdef DEBUG
					Serial.println(val);
					#endif
				}
				
				//some flashing when IR srcValue pointer changes (this does not count as a command sequence... see below)
				if((irLast == IR_CH_MINUS) || (irLast == IR_CH_PLUS)){
					int flashLength = (irSrc>SRC_STEP3?250:100);
					for(uint8_t ii=0;ii<1+(irSrc - SRC_STEP1)%3;ii++){
						digitalWrite(PIN_ACT, HIGH);
						delay(flashLength);			
						digitalWrite(PIN_ACT, LOW);
						delay(flashLength);
					}
				}
				
				//turn the activity LED on while a sequence is expected and off when not
				if(irCommand == 0){
					digitalWrite(PIN_ACT, LOW);
				}else{
					digitalWrite(PIN_ACT, HIGH);
				}
			}
		}

		lastTickMillis=millis();
		
		//update the LFO value
		lfoCounter+=(getSrcVal(lfoRateSrc)>>4);
		if(lfoCounter>=2048) lfoCounter -=2048;
		if(lfoCounter<1024){
			srcVals[SRC_LFO]=lfoCounter;
		}else{
			srcVals[SRC_LFO]=2048 - lfoCounter;
		}
		
		//update the trigger/gate mask
		if(!(tgmPattern==TGM_DISABLED)){
			updateTGM();
		}
		
		//use the patches to set the LED change rate, brightness scale, or trigger/gate input
		for(uint8_t led = 0; led< NUM_LEDS; led++){
			rate = getSrcVal(patches[led][PAR_RATE]);
			signed char rf=rateFactor[led];
			if(rf>0){//rateFactor scales the rate by factors of two
				rate = rate << rf;
			}else if(rf<0){
				rate = rate >> -rf;
			}
			sbc.setRate(led, rate);
			sbc.setScale(led, getSrcVal(patches[led][PAR_SCALE]));
			sbc.setTriggerIP(led, getSrcVal(patches[led][PAR_TG_IP]));
		}
		sbc.tick();
	}
}

//updates the trigger gate mask. This should be called each "tick" to update the mask internal counter,
// and to set srcVals accordingly. i.e. it should be called before the patches are processed.
void updateTGM(){
	uint8_t tgmPattern2 = tgmPattern & 0xF;
	//gets tgmRate added each tick. When exceeds 2048 the mask changes by one step and counter resets to 0
	 tgmCounter+=(getSrcVal(tgmRateSrc)>>2);
	 if(tgmCounter>=2048){
		 //change mask
		 switch (tgmPattern2){
			 case TGM_SINGLE:
				tgMask = tgMask<<1;
				if(tgMask==(uint16_t)(1<<runLength)){//top reset
					tgMask=1;
				}
				break;
			case TGM_GROW:
				if(tgmAB){
					tgMask = tgMaskMask & (1 + (tgMask<<1));
					if(tgMask==((uint16_t)(1<<runLength)-1)){//top bounce
						tgmAB = !tgmAB;
					}
				}else{
					tgMask = tgMask>>1;
					if(tgMask==1){//bottom bounce
						tgmAB = !tgmAB;
					}
				}
				break;
			case TGM_PASS:
				if(tgmAB){
					tgMask = tgMaskMask & (1 + (tgMask<<1));
					if(tgMask==((uint16_t)(1<<runLength)-1)){//top bounce
						tgmAB = !tgmAB;
					}
					}else{
						tgMask = (tgMask<<1) & tgMaskMask;
						if(tgMask==(uint16_t)(1<<(runLength-1))){//bottom bounce
							tgmAB = !tgmAB;
						}
					}
				break;			
			case TGM_DOUBLE:
				tgMask = tgMask<<1;
				if(tgMask==2){//fudge in bit0 if only bit1 set
					tgMask=3;
				}
				tgMask = tgMaskMask & tgMask;
				if(tgMask==0){//top reset (0 because tgMaskMask has already kicked in)
					tgMask=1;
				}
				break;
		 }
		 
		 //transfer mask to srcVals
		 for(uint8_t i=0; i<NUM_LEDS; i++){
			 uint8_t ii = i%runLength;
			srcVals[SRC_TG_MASK_BASE+i]=(tgMask&(1<<ii))?1023:0;		 
		 }
		 //reset counter
		 tgmCounter = 0;
	 }
}

//reads ADCs, check button events etc and updates the values in srcVals.
//does not include IR control
//excludes the trigger/gate mask
void readSourceValues(){
	srcVals[SRC_AUDIO] = 0; //TO DO	
	srcVals[SRC_LEV1] = analogRead(PIN_LEV1);
	srcVals[SRC_LEV2] = analogRead(PIN_LEV2);
	srcVals[SRC_LEV3] = analogRead(PIN_LEV3);
	boolean b = digitalRead(PIN_SW1);
	if(b^lastButton[0]){
		 lastButton[0] = b;
		 stepButton(SRC_STEP1);
	}
	b = digitalRead(PIN_SW2);
	if(b^lastButton[1]){
		lastButton[1] = b;
		stepButton(SRC_STEP2);
	}
	b = digitalRead(PIN_SW3);
	if(b^lastButton[2]){
		lastButton[2] = b;
		stepButton(SRC_STEP3);
	}
	
	//random changes at 1 and 10 seconds intervals
	unsigned long t = millis();
	if((t-lastRandChange1)>1000){
		srcVals[SRC_RND_1S] = random(1024);//srcVals is uint16_t, random returns long - should auto-cast
		lastRandChange1=t;
	}
	if((t-lastRandChange10)>10000){
		srcVals[SRC_RND_10S] = random(1024);
		lastRandChange10=t;
	}	
}

void stepButton(uint8_t src){
	uint16_t oldVal=srcVals[src];
	oldVal+=255;
	if(oldVal>1024) oldVal = 0;
	srcVals[src] = oldVal;
}

//gets the actual value that currently pertains for a given source.
//This copes with the CONST pseudo-sources, which simple look-up in srcVals could not ...
uint16_t getSrcVal(uint8_t src){
	if(src&SRC_CONST){
		return (src&(~SRC_CONST)) <<3;
	}else{
		return srcVals[src];
	}
}

// - ----------- programming helpers ------------
// replicate a specified program to all LEDs
// NB: if using the TG mask generator as a patch source, then use SRC_TG_MASK_BASE as a pseudo-source (the appropriate actual src that maps to the LED will be used)
void programAll(uint8_t shape, uint8_t rateSrc, uint8_t scaleSrc, uint8_t tgSrc){
	for(int led=0; led<NUM_LEDS; led++){
		//main shape
		sbc.setPattern(led, shape, 0);
		//patches
		patches[led][PAR_RATE] = rateSrc;
		patches[led][PAR_SCALE] = scaleSrc;
		if(tgSrc == SRC_TG_MASK_BASE){
			patches[led][PAR_TG_IP] = SRC_TG_MASK_BASE+led;
		}else{
			patches[led][PAR_TG_IP] = tgSrc;
		}
	}
}

// replicate a specified program to all 3 LEDS in a RGB triple, with specified relative phase shift
// NB: if using the TG mask generator as a patch source, then use SRC_TG_MASK_BASE as a pseudo-source (the appropriate actual src that maps to the LED will be used)
void programTriple(uint8_t rgb, uint8_t shape, int phase, uint8_t rateSrc, uint8_t scaleSrc, uint8_t tgSrc){
	uint8_t led = rgb*3;
	for(int i=0; i<3; led++){
		//main shape
		sbc.setPattern(led, shape, phase*i);
		//patches
		patches[led][PAR_RATE] = rateSrc;
		patches[led][PAR_SCALE] = scaleSrc;
		if(tgSrc == SRC_TG_MASK_BASE){
			patches[led][PAR_TG_IP] = SRC_TG_MASK_BASE+led;
			}else{
			patches[led][PAR_TG_IP] = tgSrc;
		}
		led++;
	}
}

void printPatch(uint8_t led){
	char outStr[5];
	sprintf(outStr, "%02X", patches[led][PAR_RATE]); 
	Serial.print(outStr);
	sprintf(outStr, "%02X", rateFactor[led]);
	Serial.print(outStr);
	Serial.print(" ");
	sprintf(outStr, "%02X", patches[led][PAR_SCALE]);
	Serial.print(outStr);
	sprintf(outStr, "%02X", patches[led][PAR_TG_IP]);
	Serial.println(outStr);
}

void printPatternBytes(uint8_t led){
	uint8_t buff[4];
	sbc.getPatternProgBytes(led,buff);
			
	char outStr[5];
	sprintf(outStr, "%02X", buff[0]);
	Serial.print(outStr);
	sprintf(outStr, "%02X", buff[1]);
	Serial.print(outStr);
	Serial.print(" ");
	sprintf(outStr, "%02X", buff[2]);
	Serial.print(outStr);
	sprintf(outStr, "%02X", buff[3]);
	Serial.println(outStr);
}

void loadProgram(uint8_t programNumber){
	#ifdef DEBUG
	Serial.print("Load Prog:");
	Serial.println(programNumber);
	#endif
	
	//program start address
	word eAddr = 8 + PROG_BYTES*(programNumber-1);
	//block buffer
	uint8_t buff[4];
	
	//read in LFO and Trigger/gate mask settings
	EEPROMUtils::loadBytes(&eAddr, buff, 4);
	lfoRateSrc = (uint16_t)buff[0];
	EEPROMUtils::loadBytes(&eAddr, buff, 4);
	tgmRateSrc = (uint16_t)buff[0];
	tgmPattern = buff[1];
	tgMask = 1;
	runLength = (tgmPattern&TGM_TRIPLIFY)?3:NUM_LEDS;
	tgMaskMask=(1<<runLength)-1;
	
	//loop over LEDS for shape data
	for(uint8_t led=0; led<NUM_LEDS; led++){		
		EEPROMUtils::loadBytes(&eAddr, buff, 4);
		sbc.setPatternFromProgBytes(led,buff);
		#ifdef DEBUG
		printPatternBytes(led);
		#endif
		
		//EEPROMUtils::loadBytes(&eAddr, buff, 2);
		//phase = EEPROMUtils::loadInt(&eAddr);
		//sbc.setPattern(led, buff[0] , phase);
	}

	//loop over LEDs for patches
	for(uint8_t led=0; led<NUM_LEDS; led++){
		EEPROMUtils::loadBytes(&eAddr, buff, 4);
		patches[led][PAR_RATE] = buff[0];
		rateFactor[led]=buff[1];
		patches[led][PAR_SCALE] = buff[2];
		patches[led][PAR_TG_IP] = buff[3];
		#ifdef DEBUG
		printPatch(led);
		#endif
	}
	
}

//convert an IR code into a decimal number, or return255 if code does not mean a number
uint8_t decodeNumIR(unsigned long irCode){
	switch (irCode){
		case IR_N0:
			return 0;
		case IR_N1:
			return 1;
		case IR_N2:
			return 2;
		case IR_N3:
			return 3;
		case IR_N4:
			return 4;
		case IR_N5:
			return 5;
		case IR_N6:
			return 6;
		case IR_N7:
			return 7;
		case IR_N8:
			return 8;
		case IR_N9:
			return 9;
		default:
			return 255;
	}
}
