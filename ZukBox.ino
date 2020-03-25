#include "FastLED.h"

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

#define FRAMES_PER_SECOND 120

// Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DATA, NEO_GRB +
// NEO_KHZ800);
unsigned long time = 0;
unsigned long time_last = 0;
int pattern_time_my = 0;

int pattern = 0;
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
bool rave_mode = false;

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
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

struct CRGB leds[NUM_LEDS]; // Initialize our LED array.

int max_bright = 255;

bool musicAnalyzerToggle = false;
bool allLedsToggle = false;
int STROBE_DELAY = 0;
bool StrobeAllToggle = false;
bool coolToggle = true;
bool pixelsToggle = false;
float gain = 8;

bool RED0 = false;
bool RED1 = false;
bool RED2 = false;
bool RED3 = false;
bool RED4 = false;
bool RED5 = false;
bool RED6 = false;
bool RED7 = false;
bool RED8 = false;
bool S0 = false;
bool S1 = false;
bool S2 = false;
bool S3 = false;
bool S4 = false;
// ------------------------------------
#define qsubd(x, b)                                                            \
  ((x > b) ? b : 0) // Digital unsigned subtraction macro. if result <0, then =>
                    // 0. Otherwise, take on fixed value.
#define qsuba(x, b)                                                            \
  ((x > b) ? x - b                                                             \
           : 0) // Analog Unsigned subtraction macro. if result <0, then => 0
#define FASTLED_ALLOW_INTERRUPTS 0 // Used for ESP8266.

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
  micIn = 2 * mapValue[channel]; // Poor man's analog Read.
  if (channel == 9)
    micIn = avarage; // Poor man's analog Read.
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

  //  CRGB newcolour = ColorFromPalette(currentPalette, sin8(sample), sampleAgc,
  //  currentBlending);   // Colour of the LED will be based on the sample,
  //  while brightness is based on sampleavg. leds[currLED] = newcolour;
  //  // Direct assignment of pixel colour.

  leds[(millis() % (NUM_LEDS - 1)) + 1] =
      ColorFromPalette(currentPalette, sampleAgc, sampleAgc, currentBlending);

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

void jugglep() { // Use the juggle routine, but adjust the timebase based on
                 // sampleAvg for some randomness.

  // Local definitions

  // Persistent local variables
  static int thistime = 20; // Time shifted value keeps changing thus
                            // interrupting the juggle pattern.

  // Temporary local variables

  fadeToBlackBy(leds, NUM_LEDS, 1); // Fade the strand.

  leds[beatsin16(thistime, 0, NUM_LEDS - 1, 0, 0)] += ColorFromPalette(
      currentPalette, millis() / 4, sampleAgc, currentBlending);
  leds[beatsin16(thistime - 3, 0, NUM_LEDS - 1, 0, 0)] += ColorFromPalette(
      currentPalette, millis() / 4, sampleAgc, currentBlending);

  EVERY_N_MILLISECONDS(250) {
    thistime = sampleAvg /
               2; // Change the beat frequency every 250 ms. By Andrew Tuline.
  }

} // jugglep()

void noisewide() {

  // Local definitions
#define GRAVITY 5

  // Persistent local variables
  static uint8_t topLED;
  static int gravityCounter = 0;

  // Temporary local variables
  uint8_t tempsamp = constrain(
      sampleAvg, 0, NUM_LEDS / 2 - 1); // Keep the sample from overflowing.

  fadeToBlackBy(leds, NUM_LEDS, 160);

  for (int i = 0; i < tempsamp; i++) {
    uint8_t index = inoise8(i * sampleAvg + millis(), 5000 + i * sampleAvg * 2);
    leds[NUM_LEDS / 2 - i] =
        ColorFromPalette(currentPalette, index, sampleAvg * 2, currentBlending);
    leds[NUM_LEDS / 2 + i] =
        ColorFromPalette(currentPalette, index, sampleAvg * 2, currentBlending);
  }

  if (tempsamp >= topLED)
    topLED = tempsamp;
  else if (gravityCounter % GRAVITY == 0)
    topLED--;

  if (topLED > 0) {
    leds[NUM_LEDS / 2 - topLED] =
        ColorFromPalette(currentPalette, millis(), 255,
                         LINEARBLEND); // LED falls when the volume goes down.
    leds[topLED + NUM_LEDS / 2] =
        ColorFromPalette(currentPalette, millis(), 255,
                         LINEARBLEND); // LED falls when the volume goes down.
  }

  gravityCounter = (gravityCounter + 1) % GRAVITY;

} // noisewide()

void onesine() {

  // Local definitions

  // Persistent local variable.
  static int thisphase = 0; // Phase change value gets calculated.

  // Temporary local variables
  uint8_t allfreq = 20; // You can change the frequency, thus distance between
                        // bars. Wouldn't recommend changing on the fly.
  uint8_t thiscutoff;   // You can change the cutoff value to display this wave.
                        // Lower value = longer wave.

  thiscutoff = 255 - sampleAgc;

  thisphase +=
      sampleAvg / 6 +
      beatsin16(20, -10, 10); // Move the sine waves along as a function of
                              // sound plus a bit of sine wave.

  for (int k = 0; k < NUM_LEDS;
       k++) { // For each of the LED's in the strand, set a brightness based on
              // a wave as follows:
    int thisbright =
        qsubd(cubicwave8((k * allfreq) + thisphase),
              thiscutoff); // qsub sets a minimum value called thiscutoff. If <
                           // thiscutoff, then bright = 0. Otherwise, bright =
                           // 128 (as defined in qsub)..

    leds[k] = ColorFromPalette(currentPalette, millis() / 2, thisbright,
                               currentBlending); // Let's now add the foreground
                                                 // colour. By Andrew Tuline.
  }

} // onesine()

void sinephase() {

  // Local definitions

  // Persistent local variables

  // Temporary local variables

  for (int i = 0; i < NUM_LEDS; i++) {

    int hue = sampleAvg * 2 + sin8(i * 4 + beatsin16(13, -20, 50));
    int bri = hue;
    bri = bri * bri / 255;
    leds[i] = ColorFromPalette(currentPalette, hue, bri, currentBlending);
  }

} // sinephase()

void fillnoise() { // Another perlin noise based routine.

  // Local definitions
#define xscale 160
#define yscale 160

  // Persistent local variables
  static int16_t xdist; // A random number for our noise generator.
  static int16_t ydist;

  // Temporary local variables

  if (sampleAvg > NUM_LEDS)
    sampleAvg = NUM_LEDS; // Clip the sampleAvg to maximize at NUM_LEDS.

  for (int i = (NUM_LEDS - sampleAvg / 2) / 2;
       i < (NUM_LEDS + sampleAvg / 2) / 2;
       i++) { // The louder the sound, the wider the soundbar.
    uint8_t index = inoise8(
        i * sampleAvg * 8 + xdist,
        ydist + i * sampleAvg * 8); // Get a value from the noise function. I'm
                                    // using both x and y axis.

    leds[i] = ColorFromPalette(
        currentPalette, index, sampleAgc * 3,
        LINEARBLEND); // With that value, look up the 8 bit colour palette value
                      // and assign it to the current LED.
  } // Effect is a NOISE bar the width of sampleAvg. Very fun. By Andrew Tuline.

  xdist += beatsin8(
      5, 0, 3); // Moving forward in the NOISE field, but with a sine motion.
  ydist += beatsin8(
      4, 0, 3); // Moving sideways in the NOISE field, but with a sine motion.

  waveit(); // Move the pixels to the left/right, but not too fast.

  fadeToBlackBy(
      leds + NUM_LEDS / 2 - 1, 2,
      64); // Fade the center, while waveit moves everything out to the edges.

} // fillnoise()

void matrix() { // A 'Matrix' like display using sampleavg for brightness.

  // Local definitions

  // Persistent local variables

  // Temporary local variables

  if (thisdir == 1) {
    leds[0] = ColorFromPalette(currentPalette, millis(), sampleAgc * 100,
                               currentBlending);
  } else {
    leds[NUM_LEDS - 1] = ColorFromPalette(currentPalette, millis(),
                                          sampleAgc * 100, currentBlending);
  }

  if (thisdir == 1) {
    for (int i = NUM_LEDS - 1; i > 0; i--)
      leds[i] = leds[i - 1];
  } else {
    for (int i = 0; i < 10; i++)
      leds[i] = leds[i + 1];
  }
}

void ripple() { // Display ripples triggered by peaks.

  // Local definitions
#define maxsteps 16 // Maximum number of steps.

  // Persistent local variables
  static uint8_t colour;      // Ripple colour is based on samples.
  static uint16_t center = 0; // Center of current ripple.
  static int8_t step = -1;    // Phase of the ripple as it spreads out.

  // Temporary local variables

  if (samplePeak) {
    samplePeak = 0;
    step = -1;
  } // Trigger a new ripple if we have a peak.

  fadeToBlackBy(leds, NUM_LEDS,
                64); // Fade the strand, where 1 = slow, 255 = fast

  switch (step) {

  case -1: // Initialize ripple variables. By Andrew Tuline.
    center = random(NUM_LEDS);
    colour = (sample) % 255; // More peaks/s = higher the hue colour.
    step = 0;
    break;

  case 0:
    leds[center] += ColorFromPalette(
        currentPalette, colour + millis(), 255,
        currentBlending); // Display the first pixel of the ripple.
    step++;
    break;

  case maxsteps: // At the end of the ripples.
    // step = -1;
    break;

  default: // Middle of the ripples.

    leds[(center + step + NUM_LEDS) % NUM_LEDS] += ColorFromPalette(
        currentPalette, colour + millis(), 255 / step * 2,
        currentBlending); // A spreading and fading pattern up the strand.
    leds[(center - step + NUM_LEDS) % NUM_LEDS] += ColorFromPalette(
        currentPalette, colour + millis(), 255 / step * 2,
        currentBlending); // A spreading and fading pattern down the strand.
    step++;               // Next step.
    break;

  } // switch step

} // ripple()

void rainbowpeak() {

  // Local definitions

  // Persistent local variables

  // Temporary local variables
  uint8_t beatA = beatsin8(17, 0, 255); // Starting hue.

  if (samplePeak) { // Trigger a rainbow with a peak.

    samplePeak = 0; // Got a peak, now reset it.

    uint8_t locn = random8(0, NUM_LEDS);
    fill_rainbow(leds + locn, random8(0, (NUM_LEDS - locn)), beatA, 8);
  }

  fadeToBlackBy(leds, NUM_LEDS, 40); // Fade everything. By Andrew Tuline.

} // rainbowpeak()

void myvumeter() { // A vu meter. Grabbed the falling LED from Reko MeriÃ¶.

  // Local definitions
#define GRAVITY 2

  // Persistent local variables
  static uint8_t topLED;
  static int gravityCounter = 0;

  // Temporary local variables
  uint8_t tempsamp = constrain(
      sampleAvg * 5, 0, NUM_LEDS - 1); // Keep the sample from overflowing.

  fadeToBlackBy(leds, NUM_LEDS, 20);

  for (int i = 0; i < tempsamp; i++) {
    uint8_t index =
        inoise8(i * 2 * sampleAvg + millis(), 5000 + i * 2 * sampleAvg);
    leds[i] = ColorFromPalette(currentPalette, index, sampleAvg * 8,
                               currentBlending);
  }

  if (tempsamp >= topLED)
    topLED = tempsamp;
  else if (gravityCounter % GRAVITY == 0)
    topLED--;

  if (topLED > 0) {
    leds[topLED] =
        ColorFromPalette(currentPalette, millis(), 255,
                         LINEARBLEND); // LED falls when the volume goes down.
  }

  gravityCounter = (gravityCounter + 1) % GRAVITY;

} // myvumeter()

void cool() {
  channel = 9;
  while (coolToggle) {
    getControls();
    getGain();

    coolToggle = S0;
    pixelsToggle = RED0;
    bool plasmaToggle = RED1;
    bool rippleToggle = RED2;
    bool jugglepToggle = RED3;
    bool noisewideToogle = RED4;
    bool sinephaseToggle = RED5;
    bool onesineToggle = RED6;
    bool fillnoiseToggle = RED7;
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

    if (rippleToggle)
      ripple();

    if (jugglepToggle)
      jugglep();

    if (noisewideToogle)
      noisewide();

    if (sinephaseToggle)
      sinephase();

    if (onesineToggle)
      onesine();

    if (fillnoiseToggle)
      fillnoise();

    if (RED8)
      rainbowpeak();

    if (S3)
      setChannel();

    FastLED.show(); // Display everything.
  }
}

void rainbow_non_music() {
  // FastLED's built-in rainbow generator
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}

void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV(gHue + random8(64), 200, 255);
}

void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS - 1);
  leds[pos] += CHSV(gHue, 255, 192);
}

void bpm() {
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
  for (int i = 0; i < NUM_LEDS; i++) { // 9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for (int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void animations() {
  while (S2) {
    getControls();
    checkBrightness();
    float gHue_step = map(constrain(analogRead(A3), 0, 1024), 0, 1024, 8, -1);
    EVERY_N_MILLISECONDS(20) {
      gHue += gHue_step;
    } // slowly cycle the "base color" through the rainbow
    while (RED0) {
      getControls();
      pattern_time_my =
          map(constrain(analogRead(A4), 0, 1024), 0, 1024, 100, 0);
      Serial.print(pattern_time_my);
      Serial.print('\n');
      EVERY_N_MILLISECONDS(20) { gHue += 2; }
      if (millis() - time_last > 1000 * pattern_time_my) {
        time_last = millis();
        pattern += 1;
      }
      if (pattern > 3)
        pattern = 0;
      switch (pattern) {
      case 0:
        rainbow_non_music();
        break;
      case 1:
        confetti();
        break;
      case 2:
        sinelon();
        break;
      case 3:
        juggle();
        break;
      }
      FastLED.show();
    }
    if (RED1)
      rainbow_non_music();
    if (RED2)
      confetti();
    if (RED3)
      sinelon();
    if (RED4)
      bpm();
    if (RED5)
      juggle();
    // send the 'leds' array out to the actual LED strip
    FastLED.show();
    // insert a delay to keep the framerate modest
    FastLED.delay(1000 / FRAMES_PER_SECOND);
  }
}

void getGain() {
  gain = map(constrain(analogRead(A4), 0, 1024), 0, 1024, 0, 9);
}

void setChannel() {
  fadeToBlackBy(leds, NUM_LEDS, 64);
  channel = map(constrain(analogRead(A7), 0, 1024), 0, 1024, 9, 0);
  for (byte i = 0; i < 3 * channel; i++) {
    if (i % 3 == 0)
      leds[NUM_LEDS / 2 + i].setRGB(0, 0, 0);
    else
      leds[NUM_LEDS / 2 + i] = CRGB::HotPink;
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
    if (i == 2) {
      if (val > 900)
        RED2 = true;
      else
        RED2 = false;
    }
    if (i == 3) {
      if (val > 900)
        RED3 = true;
      else
        RED3 = false;
    }
    if (i == 4) {
      if (val > 900)
        RED4 = true;
      else
        RED4 = false;
    }
    if (i == 5) {
      if (val > 900)
        RED5 = true;
      else
        RED5 = false;
    }
    if (i == 6) {
      if (val > 900)
        RED6 = true;
      else
        RED6 = false;
    }
    if (i == 7) {
      if (val > 900)
        RED7 = true;
      else
        RED7 = false;
    }
    if (i == 8) {
      if (val > 900)
        RED8 = true;
      else
        RED8 = false;
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
    if (i == 11) {
      if (val > 900)
        S1 = true;
      else
        S1 = false;
    }
    if (i == 12) {
      if (val > 900)
        S2 = true;
      else
        S2 = false;
    }
    if (i == 14) {
      if (val > 900)
        S3 = true;
      else
        S3 = false;
    }
    if (i == 15) {
      if (val > 900)
        S4 = true;
      else
        S4 = false;
    }
  }
}

void allLeds() {
  while (RED0) {
    getControls();
    checkBrightness();
    temp_r = map(constrain(analogRead(A3), 0, 1024), 0, 1024, 254, -1);
    temp_g = map(constrain(analogRead(A4), 0, 1024), 0, 1024, 254, -1);
    temp_b = map(constrain(analogRead(A5), 0, 1024), 0, 1024, 254, -1);
    fill_solid(leds, NUM_LEDS, CRGB(temp_r, temp_g, temp_b));
    FastLED.show();
  }
}

void StrobeAll() {
  while (RED1) {
    getControls();
    checkBrightness();
    STROBE_DELAY = map(constrain(analogRead(A3), 0, 1024), 0, 1024, 0, 50);
    fill_solid(leds, NUM_LEDS, CRGB(temp_r, temp_g, temp_b));
    FastLED.show();
    delay(1);
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
    FastLED.show();
    delay(STROBE_DELAY * 10);
  }
}

void musicAnalyzer() {
  move_pixel = 0;
  move_pixel_average = 0;
  power = 0;
  counter = 0;
  channel = 4;
  divide = 1;
  gain = 8;
  while (musicAnalyzerToggle) {
    musicAnalyzerToggle = S1;
    getControls();
    rave_mode = RED0;
    if (RED3)
      getGain();
    else
      gain = 8;

    encoderVal =
        map(constrain(analogRead(POT_DIVIDE), 0, 1024), 0, 1024, 10, -1);
    if (NUM_LEDS % encoderVal == 0 and encoderVal % 2 == 0 or encoderVal == 1)
      divide = encoderVal;
    factor = NUM_LEDS / divide;

    if (ON_OFF == false) {
      FastLED.setBrightness(0);
      FastLED.show();
    } else {
      FastLED.setBrightness(brightness);

      blinking = RED1;
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
        power = int(((mapValue[channel] - average) / gain));
        //        power = int(((mapValue[channel] - average) / (average
        //        / 7.f)));
        if (blinking) {
          if (power > 20) {
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
      if (not rave_mode)
        checkBrightness();
      else {
        // if (int(mapValue[channel]) < average * 1.1 or
        //     int(mapValue[channel]) < 10)
        if (int(mapValue[channel]) > average * 1.1 and
            int(mapValue[channel]) > 10)
          FastLED.setBrightness(0);
        counter += mapValue[channel] / 10;
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
        one_color = not RED2;
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
      if (move_pixel < 0)
        move_pixel = 0;
      move_pixel += 0.5;

      Value = (abs(int(move_pixel)) % int(factor));

      if (abs(LastValue - Value) > 12) {
        dupa = not dupa;
      }

      border = Value;
      if (border < 0)
        border = 0;
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
      if (S3)
        setChannel();
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
  musicAnalyzerToggle = S1;
  if (musicAnalyzerToggle)
    musicAnalyzer();
  else if (allLedsToggle)
    allLeds();
  else if (StrobeAllToggle)
    StrobeAll();
  else if (coolToggle)
    cool();
  else if (S2)
    animations();
  else {
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
    FastLED.show();
  }
}
#if USES_POTENTIOMETER
/* Checks if Potentiometer value has changed, sets new Brightness and return
 * true */

#endif

void addGlitter(fract8 chanceOfGlitter) { // Let's add some glitter

  if (random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }

} // addGlitter()

void lineit() { // Send the pixels one or the other direction down the line.

  if (thisdir == 0) {
    for (int i = NUM_LEDS - 1; i > 0; i--)
      leds[i] = leds[i - 1];
  } else {
    for (int i = 0; i < NUM_LEDS - 1; i++)
      leds[i] = leds[i + 1];
  }

} // lineit()

void waveit() { // Shifting pixels from the center to the left and right.

  for (int i = NUM_LEDS - 1; i > NUM_LEDS / 2; i--) { // Move to the right.
    leds[i] = leds[i - 1];
  }

  for (int i = 0; i < NUM_LEDS / 2; i++) { // Move to the left.
    leds[i] = leds[i + 1];
  }

} // waveit()
