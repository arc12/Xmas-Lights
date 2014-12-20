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

#include "I2CUtils.cpp"
#include <Wire.cpp>
#include <twi.c>

#include "ShapedBrightnessController.cpp"

// Function prototypes go here (telling the compiler these functions exist).
void readSourceValues();
void updateTGM();
void programTriple(uint8_t rgb, uint8_t shape, int phase, uint8_t rateSrc, uint8_t scaleSrc, uint8_t tgSrc);
void programAll(uint8_t shape, uint8_t rateSrc, uint8_t scaleSrc, uint8_t tgSrc);

/*
 * Place a direct direct copy of sketch (*.ino or *.pde) beneath this comment block.... except that you:
 * 1: might have to add some function prototypes to the top, because the Arduino environment does this automatically but Atmel Studio does not.
 * 2: might have to change the #include to the .cpp rather than .h (check which one includes the other. Arduino IDE seems able to work it out!)
 */

//comment out to suppress debug messages to Serial and to exclude unit tests from compilation
//unit test mode is toggled with IR_TEST
#define DEBUG

//number of LEDS in use. ShapedBrightnessController has a compilation #define max value = 9
#define SBC_NUM_LEDS 3

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
uint16_t srcVals[16+SBC_NUM_LEDS]={0,1023};//only set the values for ON and OFF, which are constants
#define SRC_OFF 0 //permanently OFF, i.e. value = 0
#define SRC_ON 1 //always ON, i.e. value =1023
#define SRC_AUDIO 2 //audio level (ADC0)
#define SRC_LEV1 3 //VR1 (ADC1)
#define SRC_LEV2 4 //VR2 or thermister (ADC2)
#define SRC_LEV3 5 //VR3 or LDR (ADC3)
#define SRC_SW1 6 //switch 1
#define SRC_SW2 7 //switch 2
#define SRC_SW3 8 //switch 3
#define SRC_IR_INT1 9 //IR controlled integer value #1
#define SRC_IR_INT2 0xA //IR controlled integer value #2
#define SRC_IR_INT3 0xB //IR controlled integer value #3
#define SRC_IR_SW1 9 //IR controlled boolean #1
#define SRC_IR_SW2 0xA //IR controlled boolean #2
#define SRC_LFO 0xE //Low Freq osc (triangle wave)
#define SRC_RND 0xF //Random number in range 0-1023
#define SRC_TG_MASK_BASE 0x10 //srcVals index at which the trigger/gate mask values reside (there are SBC_NUM_LEDS of them).
//NB TG_MASK is, in principle, available for rate and scale patches, but is NOT intended for use that way

// array to hold the patches - i.e. the mapping from the values in srcVals to parameters passed to the ShapedBrightnessController
// each LED has 3 parameters, with the precise details of how they affect the brightness over time being determined by the pattern
//	setting in force for the LED (which may change over time if a sequence has been programmed)
uint8_t patches[SBC_NUM_LEDS][3];//the value in each cell is an index into srcVals
//the second index of patches:
#define PAR_RATE 0 // - see ShapedBrightnessController.setRate()
#define PAR_SCALE 1 //  - see ShapedBrightnessController.setScale()
#define PAR_TG_IP 2 //trigger or gate input - see ShapedBrightnessController.setTriggerIP()
// multiplier for patches[][PAR_RATE], +2 means bit shift the srcVal two places to the MSB, -2 means shift 2 places towards LSB
signed char rateFactor[SBC_NUM_LEDS];

//IR class and data
IRrecv irrecv(PIN_IR);
decode_results results;
#define IR_TYPE "NEC" //IR message type
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
#define IR_TEST 0xFF906F //EQ
//100+,NEC,FF9867
//200+,NEC,FFB04F

//object for time-varying LED controllers
ShapedBrightnessController sbc = ShapedBrightnessController(SBC_NUM_LEDS);

//A Low Frequency Osc (LFO) - triangle form - can be used as a SRC in a patch, but also has its freq controlled by SRC value via the special LFO patch

//The "trigger/gate mask" provides an on/off time-varying pattern (e.g. moving dot, bar, etc) that feeds SRC_TG_MASK_BASE+led
//and can be used as a patch source. Its rate can be set by other SRC inputs.
uint16_t tgMask=1;//stores the bit mask
uint16_t tgMaskOver=1<<(SBC_NUM_LEDS-1); //if the mask ever gets >= this value then it has passed the top. consequent action varies
uint16_t tgMaskMask=(1<<SBC_NUM_LEDS)-1; //tgMask is forced to = tgMask & tgMaskMask in some cases
uint16_t tgmCounter=0;//gets tgmRate added each tick. When exceeds 2048 the mask changes by one step and counter resets to 0
uint16_t tgmRate=1;//rate of change.
uint8_t tgmPattern=0;// stores the active change pattern - see the following #defines. //NB: top nibble assumed to hold modifiers, low nibble to code for basic pattern
bool tgmAB=true;//some patterns alternate. This keeps whether A or B motion is in force.
//mask change patterns
#define TGM_DISABLED 0 //no not use trigger gate mask (this is not strictly necessary since, TGM only has effect if relevant elements in srcVals are patched to a LED controller
#define TGM_SINGLE 1 //single bit on
#define TGM_GROW 2 //bits progressively turn on to MSB then turn off from MSB
#define TGM_PASS 3 //bits progressively turn on to MSB then turn off from LSB
#define TGM_DOUBLE 4// like SINGLE 2 bits on
#define TGM_TRIPLIFY 128 //add this to treat the change pattern as an RGB pattern and replicate across all RGB triples

unsigned long lastTickMillis;

void setup(){
	//pin modes
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
	pinMode(PIN_PROG,INPUT_PULLUP);
	pinMode(PIN_IR, INPUT);
	
	pinMode(PIN_ACT, OUTPUT);
	
	//initialise the pwm
	sbc.initialise();
	
	// Start the ir receiver
	irrecv.enableIRIn();
	
	#ifdef DEBUG
		Serial.begin(9600);
		Serial.println("Start");
	#endif
	
	//set a basic program. There are two parts, both remain unchanged at runtime.
	//(a) defines the LED behaviour, is stored in the ShapedBrightnessController
	//(b) defines the patching of "input" values to LED control parameters
	//LED RED 1
	sbc.setPattern(0, SBC_WAVESHAPE_SAW, 0);//saw wave
	patches[0][PAR_RATE] = SRC_LEV1;//VR1 controls rate
	rateFactor[0]=0;//rate is not shifted
	patches[0][PAR_SCALE] = SRC_ON;// full scale brightness range
	patches[0][PAR_TG_IP] =  SRC_ON;//the waveform change is un-gated
	//LED GREEN 1
	sbc.setPattern(1, SBC_WAVESHAPE_SAW, 683);//phase shifted 120 deg
	patches[1][PAR_RATE] = SRC_LEV2;
	rateFactor[1]=0;//rate is not shifted
	patches[1][PAR_SCALE] = SRC_ON;// full scale brightness range
	patches[1][PAR_TG_IP] =  SRC_ON;//the waveform change is un-gated
	//LED BLUE 1
	sbc.setPattern(2, SBC_WAVESHAPE_SAW, 1365);//phase shifted 240 deg
	patches[2][PAR_RATE] = SRC_LEV3;
	rateFactor[2]=0;//rate is not shifted
	patches[2][PAR_SCALE] = SRC_ON;// full scale brightness range
	patches[2][PAR_TG_IP] =  SRC_ON;//the waveform change is un-gated
	
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
		lastTickMillis=millis();
		//update the LFO value
		//-- to do
		
		//update the trigger/gate mask
		if(!(tgmPattern==TGM_DISABLED)){
			updateTGM();
		}
		
		//use the patches to set the LED change rate, brightness scale, or trigger/gate input
		for(uint8_t led = 0; led< SBC_NUM_LEDS; led++){
			rate = srcVals[patches[led][PAR_RATE]];
			if(rateFactor>0){//rateFactor scales the rate by factors of two
				rate = rate << rateFactor[led];
			}else if(rateFactor<0){
				rate = rate >> (-rateFactor[led]);
			}
			//Serial.print(led);
			//Serial.print(" ");
			//Serial.println(rate);
			sbc.setRate(led, rate);
			sbc.setScale(led, srcVals[patches[led][PAR_SCALE]]);
			sbc.setTriggerIP(led, srcVals[patches[led][PAR_TG_IP]]);
		}
		sbc.tick();
	}
}

//updates the trigger gate mask. This should be called each "tick" to update the mask internal counter,
// and to set srcVals accordingly
void updateTGM(){
	uint8_t tgmPattern2 = tgmPattern & 0xF;
	//gets tgmRate added each tick. When exceeds 2048 the mask changes by one step and counter resets to 0
	 tgmCounter+=tgmRate;
	 if(tgmCounter>=2048){
		 //change mask
		 switch (tgmPattern2){
			 case TGM_SINGLE:
				tgMask = tgMask<<1;
				break;
			case TGM_GROW:
				if(tgmAB){
					tgMask = 1 + (tgMask<<1);
				}else{
					tgMask = tgMask>>1;
				}
				break;
			case TGM_PASS:
				if(tgmAB){
					tgMask = 1 + (tgMask<<1);
				}else{
					tgMask = (tgMask<<1) & tgMaskMask;
				}
				break;			
			case TGM_DOUBLE:
				tgMask = tgMask<<1;
				if(tgMask==2){//fudge in bit0 if only bit1 set
					tgMask=3;
				}
				break;
		 }
		 //check if the pattern needs a reset or an AB bounce
		 if( (tgMask>=tgMaskOver) || ((tgmPattern&TGM_TRIPLIFY) && (tgMask>=8) ) ){
			 if( (tgmPattern2==TGM_SINGLE) || (tgmPattern2==TGM_DOUBLE) ){
				tgMask = 1;			 
			 }else{
				 if(tgmAB){
					 tgMask = 1;
				 }
				 tgmAB = !tgmAB;
			 }
		 }
		 if(tgMask==0){//bottom bounce
			 tgmAB = !tgmAB;
		 }
		 //transfer mask to srcVals
		 uint8_t j;
		 for(uint8_t i=0; i<SBC_NUM_LEDS; i++){
			if(tgmPattern&TGM_TRIPLIFY){
				j=i%3;
			}else{
				j=i;
			}
			srcVals[SRC_TG_MASK_BASE+i]=(tgMask&(1<<j))?1023:0;		 
		 }
		 //reset counter
		 tgmCounter = 0;
	 }
}

//reads ADCs, check for IR and button events etc and updates the values in srcVals.
//excludes the trigger/gate mask
void readSourceValues(){
	srcVals[SRC_AUDIO] = 0; //TO DO	
	srcVals[SRC_LEV1] = analogRead(PIN_LEV1);
	srcVals[SRC_LEV2] = analogRead(PIN_LEV2);
	srcVals[SRC_LEV3] = analogRead(PIN_LEV3);
	srcVals[SRC_SW1] = digitalRead(PIN_SW1)?1023:0;
	srcVals[SRC_SW2] = digitalRead(PIN_SW2)?1023:0;
	srcVals[SRC_SW3] = digitalRead(PIN_SW3)?1023:0;
	
	// IR to do
	
	srcVals[SRC_RND] = random(1024);//srcVals is uint16_t, random returns long - should auto-cast
}

// - ----------- programming helpers ------------
// replicate a specified program to all LEDs
// NB: if using the TG mask generator as a patch source, then use SRC_TG_MASK_BASE as a pseudo-source (the appropriate actual src that maps to the LED will be used)
void programAll(uint8_t shape, uint8_t rateSrc, uint8_t scaleSrc, uint8_t tgSrc){
	for(int led=0; led<SBC_NUM_LEDS; led++){
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