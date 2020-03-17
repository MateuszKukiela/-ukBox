#include "FastLED.h"
//#ifdef __AVR__
//#include <avr/power.h>
//#endif

/** THIS SKETCH HAS ONLY GOT THE ADUIO VISUALIZER MODE USING THE MSGEQ7
    FOR OTHER MODES SEE THE "RGBStripe_Control_WS2812.ino SKETCH        **/

#define USES_POTENTIOMETER                                                     \
  1 // SET THIS TO "1" IF YOU ARE USING POTENTIOMETER FOR BRIGHTNESS CONTROL!
#define POT_DIVIDE A3
#define POT_BRIGHTNESS A2

#define LED_DATA 6
#define NUM_LEDS 200
#define BRIGHTNESS 255
#define DELAY 13

#define MSGEQ_OUT A1
#define STROBE 4
#define RESET 5

#define BLINKING_TOGGLE 3

#define LED_DT 6 // Data pin to connect to the strip.
#define LED_TYPE WS2812
#define COLOR_ORDER GRB

#define MUX_CH_COUNT 16 // Reduce this number if you use less channels
#define PIN_D_MUX_S0 8  // bit 7 of PORTB
#define PIN_D_MUX_S1 9  // bit 6 of PORTB
#define PIN_D_MUX_S2 10 // bit 5 of PORTB
#define PIN_D_MUX_S3 11 // bit 4 of PORTB
#define PIN_A_MUX_SIG 0 // This pin will read the input from the mux.

// Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DATA, NEO_GRB +
// NEO_KHZ800);
int encoderVal = 4;
float last_value = 0;
uint16_t brightness = BRIGHTNESS;
float snake_speed = 0.25;
bool dupa = false;
float move_pixel = 37;
float move_pixel_average = 0;
int divide = 2;
int factor = NUM_LEDS / divide;
bool skip_green = false;
bool blinking = false;
bool spped_variant = false;
bool brightness_variant = false;
int channel = 4;
int counter = 0;
int temp_r = 0;
int temp_g = 0;
int temp_b = 0;
int r = 0;
int g = 0;
int b = 0;
int r1 = 0;
int g1 = 0;
int b1 = 0;
int r2 = 0;
int g2 = 0;
int b2 = 0;
bool dupa_blink = false;
double resultRed = 0;
double resultGreen = 0;
double resultBlue = 0;
float percent = 0;
int gradient = 0;
int spectrumValue[8];
uint8_t mapValue[8];
const int numReadings = 20;
static int colors[3] = {0, 0, 0};
int border = 0;
int LastValue = 0;
int DoubleLastValue = 0;
bool flipped = false;
int Value = 0;
int power = 0;
int last_move_pixel = 0;
float move_pixel_delta = 0;
bool ON_OFF = false;
bool one_color = false;

int readings[numReadings]; // the readings from the analog input
int readIndex = 0;         // the index of the current reading
int total = 0;             // the running total
int average = 0;           // the average

const int pixel_numReadings = 20;
float pixel_readings[pixel_numReadings]; // the readings from the analog input
int pixel_readIndex = 0;                 // the index of the current reading
float pixel_total = 0;                   // the running total
float pixel_average = 0;                 // the average

uint16_t audioBuffer[NUM_LEDS];
int filter = 0;

struct CRGB leds[NUM_LEDS]; // Initialize our LED array.

int max_bright = 255;

bool musicAnalyzerToggle = false;
bool allLedsToggle = false;
int STROBE_DELAY = 0;
bool StrobeAllToggle = false;
bool coolToggle = true;
bool pixelsToggle = false;

bool RED0 = false;
bool RED1 = false;
bool S0 = false;
// ------------------------------------
#define qsubd(x, b)                                                            \
  ((x > b) ? b : 0) // Digital unsigned subtraction macro. if result <0, then =>
                    // 0. Otherwise, take on fixed value.
#define qsuba(x, b)                                                            \
  ((x > b) ? x - b                                                             \
           : 0) // Analog Unsigned subtraction macro. if result <0, then => 0
#define FASTLED_ALLOW_INTERRUPTS 0 // Used for ESP8266.
#include "FastLED.h"               // FastLED library.

#define MIC_PIN 5 // Analog port for microphone
uint8_t squelch =
    7;      // Anything below this is background noise, so we'll make it '0'.
int sample; // Current sample.
float sampleAvg = 0; // Smoothed Average.
float micLev = 0;    // Used to convert returned value to have '0' as minimum.
uint8_t maxVol = 11; // Reasonable value for constant volume for 'peak
                     // detector', as it won't always trigger.
bool samplePeak =
    0; // Boolean flag for peak. Responding routine must reset this flag.
int8_t thisdir = 1;

// Fixed definitions cannot change on the fly.
#define LED_DT 6 // Data pin to connect to the strip.
#define LED_CK 11
#define COLOR_ORDER GRB // It's GRB for WS2812B and BGR for APA102
#define LED_TYPE                                                               \
  WS2812 // What kind of strip are you using (WS2801, WS2812B or APA102)?
#define NUM_LEDS 200 // Number of LED's.

CRGBPalette16 currentPalette(OceanColors_p);
CRGBPalette16 targetPalette(LavaColors_p);
TBlendType currentBlending; // NOBLEND or LINEARBLEND

static int16_t xdist; // A random number for our noise generator.
static int16_t ydist;
uint16_t xscale = 30;    // Wouldn't recommend changing this on the fly, or the
                         // animation will be really blocky.
uint16_t yscale = 30;    // Wouldn't recommend changing this on the fly, or the
                         // animation will be really blocky.
uint8_t maxChanges = 24; // Value for blending between palettes.

int sampleAgc, multAgc;
uint8_t targetAgc = 60;
int avarage = 0;

void setup() {
  pinMode(PIN_D_MUX_S0, OUTPUT);
  pinMode(PIN_D_MUX_S1, OUTPUT);
  pinMode(PIN_D_MUX_S2, OUTPUT);
  pinMode(PIN_D_MUX_S3, OUTPUT);

  LEDS.addLeds<LED_TYPE, LED_DT, COLOR_ORDER>(leds,
                                              NUM_LEDS); // Use this for WS2812B
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.show(); // Display everything.
                  //   strip.setBrightness(BRIGHTNESS);
                  //   strip.begin();
                  //   strip.show();

  Serial.begin(115200);
  Serial.println("START");
  pinMode(MSGEQ_OUT, INPUT);
  pinMode(STROBE, OUTPUT);
  pinMode(RESET, OUTPUT);

  digitalWrite(RESET, LOW);
  digitalWrite(STROBE, HIGH);

  pinMode(MSGEQ_OUT, INPUT);

#if USES_POTENTIOMETER
  pinMode(POT_BRIGHTNESS, INPUT);
#endif

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void getSample() {

  int16_t micIn; // Current sample starts with negative values and large values,
                 // which is why it's 16 bit signed.
  static long peakTime;
  digitalWrite(RESET, HIGH);
  digitalWrite(RESET, LOW);
  avarage = 0;
  // Read all 8 Audio Bands
  for (int i = 0; i < 8; i++) {
    digitalWrite(STROBE, LOW);
    delayMicroseconds(30);
    spectrumValue[i] = analogRead(MSGEQ_OUT);
    spectrumValue[i] = constrain(spectrumValue[i], filter, 1023);
    if (i < 5)
      spectrumValue[i] = int(spectrumValue[i] / 2.5);
    else
      spectrumValue[i] = int(spectrumValue[i] / 2.5);

    digitalWrite(STROBE, HIGH);
    mapValue[i] = map(spectrumValue[i], filter, 1023, 0, 255);
    avarage += mapValue[i];
  }
  avarage = log(avarage) * 100;
  mapValue[channel] = map(spectrumValue[channel], filter, 1023, 0, 255);
  //  Serial.print(mapValue[4]);
  //  Serial.print('\n');
  micIn = 2 * mapValue[channel]; // Poor man's analog Read.
  //   micIn = avarage; // Poor man's analog Read.
  micLev =
      ((micLev * 31) + micIn) /
      32; // Smooth it out over the last 32 samples for automatic centering.
  micIn -= micLev;    // Let's center it to 0 now.
  micIn = abs(micIn); // And get the absolute value of each sample.
  sample =
      (micIn <= squelch)
          ? 0
          : (sample + micIn) /
                2; // Using a ternary operator, the resultant sample is either 0
                   // or it's a bit smoothed out with the last sample.
  sampleAvg = ((sampleAvg * 31) + sample) /
              32; // Smooth it out over the last 32 samples.

  if (sample > (sampleAvg + maxVol) &&
      millis() > (peakTime + 50)) { // Poor man's beat detection by seeing if
                                    // sample > Average + some value.
    samplePeak =
        1; // Then we got a peak, else we don't. Display routines need to reset
           // the samplepeak value in case they miss the trigger.
    peakTime = millis();
  }

} // getSample()

void agcAvg() { // A simple averaging multiplier to automatically adjust sound
                // sensitivity.

  multAgc = (sampleAvg < 1)
                ? targetAgc
                : targetAgc / sampleAvg; // Make the multiplier so that
                                         // sampleAvg * multiplier = setpoint
  sampleAgc = sample * multAgc;
  if (sampleAgc > 255)
    sampleAgc = 255;

} // agcAvg()

void pixels() {

  // Local definitions

  // Persistent local variables
  static uint16_t
      currLED; // Persistent local value to count the current LED location.

  // Temporary local variables

  currLED = (currLED + 1) %
            (NUM_LEDS); // Cycle through all the LED's. By Andrew Tuline.

  //  CRGB newcolour = ColorFromPalette(currentPalette, sin8(sample), 255,
  //  currentBlending);   // Colour of the LED will be based on the sample,
  //  while brightness is based on sampleavg. leds[currLED] = newcolour; //
  //  Direct assignment of pixel colour.

  leds[(millis() % (NUM_LEDS - 1)) + 1] =
      ColorFromPalette(currentPalette, 255, 255, currentBlending);

} // pixels()

void plasma() {

  // Local definitions

  // Persistent local variables
  static int16_t thisphase = 0; // Phase of a cubicwave8.
  static int16_t thatphase = 0; // Phase of the cos8.

  // Temporary local variables
  uint16_t thisbright;
  uint16_t colorIndex;

  thisphase +=
      beatsin8(6, -4, 4); // You can change direction and speed individually.
  thatphase += beatsin8(
      7, -4,
      4); // Two phase values to make a complex pattern. By Andrew Tuline.

  for (int k = 0; k < NUM_LEDS;
       k++) { // For each of the LED's in the strand, set a brightness based on
              // a wave as follows.
    thisbright = cubicwave8((k * 8) + thisphase) / 2;
    thisbright +=
        cos8((k * 10) + thatphase) / 2; // Let's munge the brightness a bit and
                                        // animate it all with the phases.
    colorIndex = thisbright;
    thisbright = qsuba(
        thisbright,
        255 - sampleAvg * 4); // qsuba chops off values below a threshold
                              // defined by sampleAvg. Gives a cool effect.

    //#define qsuba(x, b)  ((x>b)?x-b:0)                            // Unsigned
    // subtraction macro. if result <0, then => 0.

    leds[k] += ColorFromPalette(
        currentPalette, colorIndex, thisbright,
        currentBlending); // Let's now add the foreground colour.
  }

  fadeToBlackBy(leds, NUM_LEDS, 64);

} // plasma()

void cool() {
  while (coolToggle) {
    getControls();
    coolToggle = S0;
    pixelsToggle = RED0;
    bool plasmaToggle = RED1;
    checkBrightness();
    EVERY_N_MILLISECONDS(10) {
      uint8_t maxChanges = 24;
      nblendPaletteTowardPalette(
          currentPalette, targetPalette,
          maxChanges); // AWESOME palette blending capability.
      //    fillnoise8();                                             // Update
      //    the LED array with noise based on sound input

      //    fadeToBlackBy(leds, NUM_LEDS, 32);                         // 8 bit,
      //    1 = slow, 255 = fast
      fadeToBlackBy(leds, NUM_LEDS, 1); // 8 bit, 1 = slow, 255 = fast
    }

    EVERY_N_SECONDS(
        5) { // Change the target palette to a random one every 5 seconds.
      targetPalette = CRGBPalette16(CHSV(random8(), 255, random8(128, 255)),
                                    CHSV(random8(), 255, random8(128, 255)),
                                    CHSV(random8(), 192, random8(128, 255)),
                                    CHSV(random8(), 255, random8(128, 255)));
    }

    getSample(); // Sample the sound.
    agcAvg();
    //  fadeToBlackBy(leds, NUM_LEDS, 32);
    if (pixelsToggle)
      pixels();

    if (plasmaToggle)
      plasma();
    FastLED.show(); // Display everything.
  }
}

void rainbow(int x) {
  static int values[3];
  int _r = 0;
  int _g = 0;
  int _b = 0;
  x = x % 255;
  if (x < 85) {
    _g = 0;
    _r = ((float)x / 85.0f) * 255.0f;
    _b = 255 - _r;
  } else if (x < 170) {
    _g = ((float)(x - 85) / 85.0f) * 255.0f;
    _r = 255 - _g;
    _b = 0;
  } else if (x < 255) {
    _b = ((float)(x - 170) / 85.0f) * 255.0f;
    _g = 255 - _b;
    _r = 0;
  }
  colors[0] = _r;
  colors[1] = _g;
  colors[2] = _b;
}

boolean checkBrightness() {
  // WS2812 takes value between 0-255
  uint16_t bright =
      map(constrain(analogRead(POT_BRIGHTNESS), 0, 1024), 0, 1024, 256, 9);
  if (abs(bright - brightness) > 10) {
    brightness = bright;
    FastLED.setBrightness(brightness);
    return true;
  }
  return false;
}

int readMux(int channel) {
  int controlPin[] = {PIN_D_MUX_S0, PIN_D_MUX_S1, PIN_D_MUX_S2, PIN_D_MUX_S3};

  int muxChannel[16][4] = {
      {0, 0, 0, 0}, // channel 0
      {1, 0, 0, 0}, // channel 1
      {0, 1, 0, 0}, // channel 2
      {1, 1, 0, 0}, // channel 3
      {0, 0, 1, 0}, // channel 4
      {1, 0, 1, 0}, // channel 5
      {0, 1, 1, 0}, // channel 6
      {1, 1, 1, 0}, // channel 7
      {0, 0, 0, 1}, // channel 8
      {1, 0, 0, 1}, // channel 9
      {0, 1, 0, 1}, // channel 10
      {1, 1, 0, 1}, // channel 11
      {0, 0, 1, 1}, // channel 12
      {1, 0, 1, 1}, // channel 13
      {0, 1, 1, 1}, // channel 14
      {1, 1, 1, 1}  // channel 15
  };

  // loop through the 4 sig
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  // read the value at the SIG pin
  int val = analogRead(PIN_A_MUX_SIG);

  // return the value
  return val;
}

void getControls() {
  for (byte i = 0; i < MUX_CH_COUNT; i++) {
    // "val" holds the value for input "i", so you can insert your custom code
    // here.

    // Print the values...
    //   Serial.print(i);
    //   Serial.print(": ");
    //   Serial.print(val);
    //   Serial.print(" | ");
    //   Serial.println("");
    int val = readMux(i);
    if (i == 0) {
      if (val > 900)
        RED0 = true;
      else
        RED0 = false;
    }
    if (i == 1) {
      if (val > 900)
        RED1 = true;
      else
        RED1 = false;
    }
    if (i == 8) {
      if (val > 900)
        musicAnalyzerToggle = true;
      else
        musicAnalyzerToggle = false;
    }
    if (i == 9) {
      if (val > 900)
        ON_OFF = true;
      else
        ON_OFF = false;
    }
    if (i == 10) {
      if (val > 900)
        S0 = true;
      else
        S0 = false;
    }
    if (i == 14) {
      if (val > 900)
        one_color = true;
      else
        one_color = false;
    }
    if (i == 15) {
      if (val > 900)
        blinking = true;
      else
        blinking = false;
    }
  }
}

void allLeds() {
  temp_r = map(constrain(analogRead(A3), 0, 1024), 0, 1024, 254, -1);
  temp_g = map(constrain(analogRead(A4), 0, 1024), 0, 1024, 254, -1);
  temp_b = map(constrain(analogRead(A5), 0, 1024), 0, 1024, 254, -1);
  fill_solid(leds, NUM_LEDS, CRGB(temp_r, temp_g, temp_b));
  FastLED.show();
}

void StrobeAll() {
  STROBE_DELAY = map(constrain(analogRead(A3), 0, 1024), 0, 1024, 0, 50);
  fill_solid(leds, NUM_LEDS, CRGB(temp_r, temp_g, temp_b));
  FastLED.show();
  delay(1);
  fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
  FastLED.show();
  delay(STROBE_DELAY * 10);
}

void musicAnalyzer() {
  while (musicAnalyzerToggle) {
    getControls();
    encoderVal =
        map(constrain(analogRead(POT_DIVIDE), 0, 1024), 0, 1024, 1, 40);
    if (NUM_LEDS % encoderVal == 0 and encoderVal % 2 == 0 or encoderVal == 1)
      divide = encoderVal;
    //    Serial.print(encoderVal);
    factor = NUM_LEDS / divide;

    if (ON_OFF == false) {
      FastLED.setBrightness(0);
      FastLED.show();
    } else {
      FastLED.setBrightness(brightness);
#if USES_POTENTIOMETER
      checkBrightness();
#endif
      // Reset MSGEQ
      digitalWrite(RESET, HIGH);
      digitalWrite(RESET, LOW);
      // Read all 8 Audio Bands
      for (int i = 0; i < 8; i++) {
        digitalWrite(STROBE, LOW);
        delayMicroseconds(30);
        spectrumValue[i] = analogRead(MSGEQ_OUT);
        spectrumValue[i] = constrain(spectrumValue[i], filter, 1023);
        if (i < 5)
          spectrumValue[i] = int(spectrumValue[i] / 2.5);
        else
          spectrumValue[i] = int(spectrumValue[i] / 2.5);

        digitalWrite(STROBE, HIGH);
      }
      mapValue[channel] = map(spectrumValue[channel], filter, 1023, 0, 255);
      if (blinking)
        if (int(mapValue[channel]) < average * 1.1 or
            int(mapValue[channel]) < 10)
          mapValue[channel] = 0;
      // subtract the last reading:
      total = total - readings[readIndex];
      // read from the sensor:
      readings[readIndex] = mapValue[channel];
      // add the reading to the total:
      total = total + readings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;

      // if we're at the end of the array...
      if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
      }

      // calculate the average:
      average = total / numReadings;
      // Shift LED values forward
      for (int k = NUM_LEDS - 1; k > 0; k--) {
        audioBuffer[k] = audioBuffer[k - 1];
      }
      // Uncomment / Comment above to Shift LED values backwards
      // for (int k = 0; k < NUM_LEDS-1; k++) {
      //  audioBuffer[k] = audioBuffer[k + 1];
      //}

      // Load new Audio value to first LED
      // Uses band 0,2,4 (Bass(red), Middle(green), High(blue) Frequency Band)
      // Lowest 8Bit: Blue , Middle Green , Highest Red
      // Use audioBuffer[NUM_LEDS-1] when using LED shift backwards!
      audioBuffer[0] = mapValue[5]; // RED
      audioBuffer[0] = audioBuffer[0] << 16;
      audioBuffer[0] |= ((mapValue[2] / 2) << 8); // GREEN
      audioBuffer[0] |= (mapValue[4] / 4);        // BLUE
      if (int(mapValue[channel]) > average * 0.5 and
          int(mapValue[channel]) > 10) {
        power = int(((mapValue[channel] - average) / 8.f));
        //        power = int(((mapValue[channel] - average) / (average
        //        / 7.f)));
        if(blinking){
            if(power>20){
                power = 20;
            }
        }
        move_pixel += power;
        last_value = mapValue[channel];
      } else if (int(mapValue[channel]) > 6) {
        move_pixel += snake_speed * 0;
        power = 0;
      } else
        power = 0;
      if (spped_variant) {
        if (int(mapValue[channel]) > average * 1 and
            int(mapValue[channel]) > 10)
          counter = counter + int(3 * ((mapValue[channel] + 20) / 20));
      } else if (not spped_variant)
        counter++;
      if (counter > 255)
        counter = 0;
      pixel_total = pixel_total - pixel_readings[readIndex];
      // read from the sensor:
      pixel_readings[readIndex] = move_pixel;
      // add the reading to the total:
      pixel_total = pixel_total + pixel_readings[readIndex];
      // advance to the next position in the array:
      pixel_readIndex = pixel_readIndex + 1;

      // if we're at the end of the array...
      if (pixel_readIndex >= pixel_numReadings) {
        // ...wrap around to the beginning:
        pixel_readIndex = 0;
      }

      // calculate the average:
      pixel_average = pixel_total / pixel_numReadings;
      if (skip_green) {
        if (counter < 128) {
          g = 0;
          temp_r = ((float)(counter) / 128.0f) * 200.0f;
          r = temp_r + 55;
          b = 255 - temp_r;
        } else if (counter < 256) {
          g = 0;
          temp_b = ((float)(counter - 128) / 128.0f) * 200.0f;
          b = temp_b + 55;
          r = 255 - temp_b;
        }
        Serial.print(b);
        Serial.print('\n');
        Serial.print(r);
        Serial.print('\n');
      } else {
        rainbow(counter);
        r1 = colors[0];
        g1 = colors[1];
        b1 = colors[2];
        //    if (blinking ) {
        //      //   if ((int(mapValue[channel]) < average * 1.2 or
        //      //       int(mapValue[channel]) < 10) and dupa_blink) {
        //      if (abs(pixel_average) * 1.1 < move_pixel ) {
        //        r1 = 0;
        //        g1 = 0;
        //        b1 = 0;
        //        // dupa_blink = not dupa_blink;
        //      }
        //    }
        rainbow(counter + 128);
        r2 = colors[0];
        g2 = colors[1];
        b2 = colors[2];
        if (one_color) {
          r2 = 0;
          g2 = 0;
          b2 = 0;
        }
        //    if (blinking ) {
        //      //   if ((int(mapValue[channel]) < average * 1.2 or
        //      //       int(mapValue[channel]) < 10 ) and not dupa_blink) {
        //      if (abs(pixel_average) * 1.1 < move_pixel) {
        //        r2 = 0;
        //        g2 = 0;
        //        b2 = 0;
        //        //             r2 = colors[0];
        //        // g2 = colors[1];
        //        // b2 = colors[2];
        //        // dupa_blink = not dupa_blink;
        //      }
        //    }
      }
      dupa_blink = not dupa_blink;
      if (not blinking and brightness_variant) {
        r = int(r * ((mapValue[channel] + 20) / 10));
        g = int(g * ((mapValue[channel] + 20) / 10));
        b = int(b * ((mapValue[channel] + 20) / 10));
      }

      else if (mapValue[channel] == 0) {
        r = 0;
        g = 0;
        b = 0;
      }
      //   if (move_pixel < 0)
      //     move_pixel = 0;
      move_pixel += 0.5;
      Serial.print(power);
      Serial.print('\n');

      Value = (abs(int(move_pixel)) % int(factor));

      if (abs(LastValue - Value) > 12) {
        dupa = not dupa;
      }

      border = Value;
      //   if (border < 0)
      //     border = 0;
      DoubleLastValue = LastValue;
      LastValue = Value;
      border = Value;
      move_pixel_delta = move_pixel - last_move_pixel;
      last_move_pixel = move_pixel;
      counter++;

      //   if(move_pixel<1)
      //   strip.setBrightness(0);
      //   else
      //   strip.setBrightness(100+move_pixel*40);
      //        Serial.print(power);
      //        Serial.print('\n');
      //   move_pixel=0;
      for (int i = 0; i < NUM_LEDS; i++) {
        if (i == 0) {
          if (dupa == true) {
            temp_r = r2;
            temp_g = g2;
            temp_b = b2;
          } else {
            temp_r = r1;
            temp_g = g1;
            temp_b = b1;
          }
        }
        if (divide == 1) {
          temp_r = r1;
          temp_g = g1;
          temp_b = b1;
        } else {
          for (int j = 1; j < 5; j++) {
            if ((i + j) % factor == int(border)) {
              percent = 1 - (0.2 * j);
              if (dupa == true) {
                temp_r = r2 + percent * (r1 - r2);
                temp_g = g2 + percent * (g1 - g2);
                temp_b = b2 + percent * (b1 - b2);
              } else {
                temp_r = r1 + percent * (r2 - r1);
                temp_g = g1 + percent * (g2 - g1);
                temp_b = b1 + percent * (b2 - b1);
              }
            }
          }
          if ((i) % factor == int(border)) {
            if (dupa == false) {
              temp_r = r2;
              temp_g = g2;
              temp_b = b2;
              dupa = not dupa;
            } else {
              temp_r = r1;
              temp_g = g1;
              temp_b = b1;
              dupa = not dupa;
            }
          }
        }
        leds[i].red = temp_r;
        leds[i].green = temp_g;
        leds[i].blue = temp_b;
      }

      FastLED.show();
      delay(DELAY);
    }
  }
}

void loop() {
  getControls();
  allLedsToggle = RED0;
  coolToggle = S0;
  StrobeAllToggle = RED1;
  if (musicAnalyzerToggle)
    musicAnalyzer();
  if (allLedsToggle)
    allLeds();
  if (StrobeAllToggle)
    StrobeAll();
  if (coolToggle)
    cool();
}
#if USES_POTENTIOMETER
/* Checks if Potentiometer value has changed, sets new Brightness and return
 * true */

#endif
