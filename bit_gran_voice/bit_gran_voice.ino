

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Bounce.h>

template <class X, class M, class N, class O, class Q>
X map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void blink(int led, int time_mms){
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(time_mms);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
}

// GUItool: begin automatically generated code
AudioInputAnalog         adc1;           //xy=332,625
AudioFilterBiquad        biquad1;        //xy=471,625
AudioPlaySerialflashRaw  playFlashRaw1;  //xy=612,762
//AudioFilterStateVariable filter1;        //xy=635,632
AudioAnalyzePeak         peak1;          //xy=636,698
//AudioMixer4              mixer1;         //xy=787,639
AudioEffectBitcrusher    bitcrusher1;    //xy=929,644
AudioEffectGranular      granular1;      //xy=1072,642
AudioOutputAnalog        dac1;           //xy=1248,640
AudioConnection          patchCord1(adc1, biquad1);
//AudioConnection          patchCord2(biquad1, 0, filter1, 0);
AudioConnection          patchCord2(biquad1, bitcrusher1);
AudioConnection          patchCord3(biquad1, peak1);
//AudioConnection          patchCord5(filter1, 0, mixer1, 0);
//AudioConnection          patchCord6(filter1, 2, mixer1, 1);
//AudioConnection          patchCord7(mixer1, bitcrusher1);
AudioConnection          patchCord8(bitcrusher1, granular1);
AudioConnection          patchCord9(granular1, dac1);
// GUItool: end automatically generated code

// GUItool: end automatically generated code



// Pins
const int FLASH_CS = 6;               // Serial flash chip select
const int AMP_ENABLE = 5;             // Amplifier enable pin
const int pink1 = A15 ; //=26
const int pink2 = A19 ; //=30


//const int pink1 = A5 ; // =19
//const int pink1 = A6 ;
//const int inter = 19 ;


const int led_b1 = 2 ;
const int pinb1 = 3 ;

const int led_b2 = 5 ;
const int pinb2 = 4 ;

const int led_inter = 6 ; 
const int inter = 7 ; // 19 ;


//const int pinb3 = A6 ;  //=20

const bool DEBUG = false;
const bool FEEDBACK_SUPPRESSION = true;  // Enables input filter
const unsigned int LOWPASS_CUTOFF = 2000; // Hz
const unsigned int CROSSOVER_FREQ = 1800; // Filter center freq
const float BASS_GAIN_ON = 0.5;
const float BASS_GAIN_OFF = 0.0;
const float TREBLE_GAIN_ON = 0.75;    // Voice output volume
const float TREBLE_GAIN_OFF = 0.0;
const float SQUELCH_CUTOFF = 0.15;    // Voice threshold
const int HYSTERESIS_TIME_ON = 20;    // Milliseconds
const int HYSTERESIS_TIME_OFF = 400;  // Milliseconds
const int button_time = 15 ; // Milliseconds 


Bounce button0 = Bounce(pinb1, 15);
Bounce button1 = Bounce(pinb2, 15);
//Bounce inter1  = Bounce (inter,15) ;

unsigned long SerialMillisecondCounter;


//BitCrusher
int current_CrushBits = 16; //this defaults to passthrough.
int current_SampleRate = 44100; // this defaults to passthrough.

#define GRANULAR_MEMORY_SIZE 12800  // enough for 290 ms at 44.1 kHz
int16_t granularMemory[GRANULAR_MEMORY_SIZE];


typedef enum volState {
  QUIET,
  QUIET_TO_LOUD,
  LOUD,
  LOUD_TO_QUIET,
} VolState;

// Global variables
elapsedMillis fps; // Sample peak only if we have available cycles
VolState state = QUIET;
unsigned long timer;



float ratio =1  ;
float msec = 1; 


float tmp = 0 ;
void setup() {

  Serial.begin(9600);

  pinMode(pinb1, INPUT_PULLUP);
  pinMode(pinb2, INPUT_PULLUP);
  pinMode(inter, INPUT_PULLUP);

  pinMode(led_b1, OUTPUT);
  pinMode(led_b2, OUTPUT);
  pinMode(led_inter, OUTPUT);
  //pinMode(pinb3, INPUT_PULLUP);

  // Initialize amplifier
  //AudioMemory(20);
  AudioMemory(40); //this is WAY more tha nwe need
  dac1.analogReference(EXTERNAL ); // much louder!            // time for DAC voltage stable
  delay(50);
  pinMode(AMP_ENABLE, OUTPUT);

  // wait up to 10 seconds for Arduino Serial Monitor
  unsigned long startMillis = millis();
  if ( DEBUG ) {
    while ( !Serial && ( millis() - startMillis < 10000 ) );
  }

  
  if ( !SerialFlash.begin(FLASH_CS) ) {
      Serial.println( "Unable to access SPI Flash chip" );
  }

 if ( FEEDBACK_SUPPRESSION ) {
   biquad1.setLowpass(0, LOWPASS_CUTOFF, 0.707);
 } else {
   biquad1.setLowpass(0, 8000, 0.707);
  }

  // Configure the State Variable filter
  //filter1.frequency(CROSSOVER_FREQ);
  //filter1.resonance(0.707);

  // Adjust gain into the mixer
  //mixer1.gain(0, BASS_GAIN_OFF);
  //mixer1.gain(1, TREBLE_GAIN_OFF);

  // by default the Teensy 3.1 DAC uses 3.3Vp-p output
  // if your 3.3V power has noise, switching to the
  // internal 1.2V reference can give you a clean signal
  //dac.analogReference(INTERNAL);

  // Bitcrusher
  granular1.begin(granularMemory, GRANULAR_MEMORY_SIZE);
  bitcrusher1.bits(current_CrushBits); //set the crusher to defaults. This will passthrough clean at 16,44100
  bitcrusher1.sampleRate(current_SampleRate); //set the crusher to defaults. This will passthrough clean at 16,44100

  /*bitcrusher param: 
  bitcrush freq 5500 Hz
  bit > 8 
  */

  SerialMillisecondCounter = millis();

  if(DEBUG){
    Serial.println("Finished init");  
  }
  
  //digitalWrite(AMP_ENABLE, HIGH);
}


void loop() {
  if (DEBUG && millis() - SerialMillisecondCounter >= 5000  ) {
    Serial.print("Proc = ");
    Serial.print(AudioProcessorUsage());
    Serial.print(" (");
    Serial.print(AudioProcessorUsageMax());
    Serial.print("),  Mem = ");
    Serial.print(AudioMemoryUsage());
    Serial.print(" (");
    Serial.print(AudioMemoryUsageMax());
    Serial.println(")");
    SerialMillisecondCounter = millis();
    AudioProcessorUsageMaxReset();
    AudioMemoryUsageMaxReset();
  }

  // Update all the button objects
  button0.update();
  button1.update();
  int inter1 = digitalRead(inter);
  float crush = (float)analogRead(pink1)/1023.0;
  float freq = (float)analogRead(pink2)/1023.0 ;

/*  if(tmp< knobA3 ){
    tmp = knobA3;
    Serial.println(tmp);
  }*/

  if(inter1 == LOW){
    ratio=  map_Generic(crush,0,63,0.5,2.0 );
    msec= map_Generic(freq,0,63,0.2,37.0 );

    granular1.beginPitchShift(msec) ;
    digitalWrite(led_inter, HIGH);
  }
 else{
    digitalWrite(led_inter, LOW);
    granular1.stop() ;
  }

  //float knobA2 = (float)analogRead(pink2) / 1023.0;
  //button2.update();

  // Start test sound if it is not playing. This will loop infinitely.


 if (button0.fallingEdge()) {
    //Bitcrusher BitDepth
    //if (current_CrushBits >= 2) { //eachtime you press it, deduct 1 bit from the settings.
    if (current_CrushBits >= 4) {
        current_CrushBits--;
        blink(led_b1,button_time);
    } else {
      current_CrushBits = 16; // if you get down to 1 go back to the top.
      blink(led_b1,button_time);
      delay(button_time);
      blink(led_b1,button_time);
    }

    bitcrusher1.bits(current_CrushBits);
    //bitcrusher1.sampleRate(current_SampleRate);
    if(DEBUG){
      Serial.print("Bitcrusher set to ");
      Serial.print(current_CrushBits);
      Serial.print(" Bit, Samplerate at ");
      Serial.print(current_SampleRate);
      Serial.println("Hz");
    }
  }

  if (button1.fallingEdge()) {
    //Bitcrusher SampleRate // the lowest sensible setting is 345. There is a 128 sample buffer, and this will copy sample 1, to each of the other 127 samples.
    if (current_SampleRate >= 690) { // 345 * 2, so we can do one more divide
      current_SampleRate = current_SampleRate / 2; // half the sample rate each time
      blink(led_b2,button_time);
    } else {
      current_SampleRate=44100; // if you get down to the minimum then go back to the top and start over.
      blink(led_b2,button_time);
      delay(button_time);
      blink(led_b2,button_time);
    }

    //bitcrusher1.bits(current_CrushBits);
    bitcrusher1.sampleRate(current_SampleRate);
    if(DEBUG){
      Serial.print("Bitcrusher set to ");
      Serial.print(current_CrushBits);
      Serial.print(" Bit, Samplerate at ");
      Serial.print(current_SampleRate);
      Serial.println("Hz");
    }
  }

  //microphone function 
  if ( (fps > 24) && peak1.available() ) {
    //from peak example
    fps = 0 ;
    
    
    // State machine
    switch ( state ) {

      // Wait until the mic picks up some sound
      case QUIET:
        if ( peak1.read() > SQUELCH_CUTOFF ) {
          timer = millis();
          state = QUIET_TO_LOUD;
        }
        break;

      // If sound continues, play sound effect
      case QUIET_TO_LOUD:
        if ( peak1.read() <= SQUELCH_CUTOFF ) {
          state = QUIET;
        } else {
          if ( millis() > timer + HYSTERESIS_TIME_ON ) {
            
            if ( DEBUG ) {
              //Serial.println(ratio);
             // Serial.println(knobA3);
              Serial.println("ON");
            }

            // Turn on amp, play sound, turn on mic
            digitalWrite(AMP_ENABLE, HIGH);

            //mixer1.gain(0, BASS_GAIN_ON);
           // mixer1.gain(1, TREBLE_GAIN_ON);

            // Go to next state
            state = LOUD;
          }
        }
        break;

      // Filter mic input and play it through speakers
      case LOUD:
        if ( peak1.read() <= SQUELCH_CUTOFF ) {
          timer = millis();
          state = LOUD_TO_QUIET;
        }
        break;

      // If no sound for a time, play click or burst
      case LOUD_TO_QUIET:
        if ( peak1.read() > SQUELCH_CUTOFF ) {
          state = LOUD;
        } else {
          if ( millis() > timer + HYSTERESIS_TIME_OFF ) {

            if ( DEBUG ) {
              Serial.println("OFF");
            }

            // Turn off mic and amp
            digitalWrite(AMP_ENABLE, LOW);
            //mixer1.gain(0, BASS_GAIN_OFF);
            //mixer1.gain(1, TREBLE_GAIN_OFF);
            state = QUIET;
          }
        }
        break;

      // You really shouldn't get here
      default:
        break;
    }
  }

    //ratio = powf(2.0, knobA2 * 2.0 - 1.0); // 0.5 to 2.0
    //ratio = powf(2.0, knobA3 * 2.0 - 1.0); // 0.5 to 2.0
    //ratio = 1 ;
    //ratio: 0.72
    //msec :37.61
    granular1.setSpeed(ratio);
    //granular1.beginPitchShift(37) ;
    //interactive_menu(&msec,&ratio ) ;
}



/*Pitch shift by continuously sampling grains and playing them at altered speed. The grainLength is specified in milliseconds, up to one third of the memory from begin();*/

void interactive_menu(float *msec, float*ratio ) {
  if (Serial.available() > 0) {
    // read the incoming string:
    String cmd = Serial.readString();
    float cmd_sub ; 
    // prints the received data
    //Serial.print("I received: ");
    //Serial.println(cmd);
    //Serial.println(cmd.substring(1));
    switch (cmd[0]) {
      case 'p' :
        granular1.beginPitchShift(*msec) ;
        Serial.println("start pitch"); 
        break;
      case 's' :
        granular1.stop() ;
        Serial.println("stop pitch"); 
        break;
      case 'm':
          cmd_sub = (cmd.substring(1)).toFloat() ;
      //    Serial.println(cmd_sub);
          *msec = cmd_sub ; 
          Serial.print("msec : ");
          Serial.println(*msec);
           break;
      case 'g':
          
          cmd_sub = (cmd.substring(1)).toFloat() ; 
        //  Serial.println(cmd_sub);
          *ratio = cmd_sub ;
          Serial.print("ratio : ");
          Serial.println(*ratio) ;
          break;
      case '1' :
        /* preset 1 : 9 bit 11025 hZ, ration 0.7, msec 1.00  */
      //ratio range : 0.5 to 2.0   (too much range 0.125 to 8.0)
      //msec : 
      default:
          Serial.print("Bitcrusher set to ");
          Serial.print(current_CrushBits);
          Serial.print(" Bit, Samplerate at ");
          Serial.print(current_SampleRate);
          Serial.println("Hz");

          Serial.print("ratio : ");
          Serial.println(*ratio) ;
          Serial.print("msec : ");
          Serial.println(*msec);
    }
  }

}
