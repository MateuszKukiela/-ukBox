#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

/** THIS SKETCH HAS ONLY GOT THE ADUIO VISUALIZER MODE USING THE MSGEQ7
    FOR OTHER MODES SEE THE "RGBStripe_Control_WS2812.ino SKETCH        **/

#define USES_POTENTIOMETER 1 // SET THIS TO "1" IF YOU ARE USING POTENTIOMETER FOR BRIGHTNESS CONTROL!
#define POT_BRIGHTNESS A2

#define LED_DATA 6
#define NUM_LEDS 200
#define BRIGHTNESS 255
#define DELAY 13

#define MSGEQ_OUT A1
#define STROBE 4
#define RESET 5

#define ON_OFF 12
#define BLINKING_TOGGLE 13

#define clkPin 11
#define dtPin 10

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DATA, NEO_GRB + NEO_KHZ800);
bool dupa_enkodera = false;
int oldA = HIGH; //set the oldA as HIGH
int oldB = HIGH; //set the oldB as HIGH
int encoderVal = 0;
float last_value = 0;
uint16_t brightness = BRIGHTNESS;
float snake_speed = 0.25;
bool dupa = false;
float move_pixel = 37;
int divide = 2;
int factor = NUM_LEDS / divide;
bool skip_green = false;
bool blinking = false;
bool spped_variant = false;
bool brightness_variant = false ;
int channel = 5;
int counter = 0;
int temp_r = 0;
int temp_g = 0;
int temp_b = 0;
int r = 0;
int g = 0;
int b = 0;
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

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

uint32_t audioBuffer[NUM_LEDS];
int filter = 0;

void setup() {
  pinMode(clkPin, INPUT);
  pinMode(dtPin, INPUT);

  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();

  Serial.begin(2000000);
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
  }
  else if (x < 170) {
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
  //  Serial.print(r);
  //  Serial.print('\n');
  return values;
}

int getEncoderTurn(void)
{
  int result = 0;
  int newA = digitalRead(clkPin);//read the value of clkPin to newA
  int newB = digitalRead(dtPin);//read the value of dtPin to newB
  if (newA != oldA or newB != oldB) //if the value of clkPin or the dtPin has changed
  {
    // something has changed
    if (oldA == HIGH && newA == LOW)
    {
      result = (oldB);
      //      Serial.print("CHUJ");
      //      Serial.print('\n');

    }
  }
  oldA = newA;
  oldB = newB;
  return result;
}


void loop() {
  musicAnalyzer();
}

#if USES_POTENTIOMETER
/* Checks if Potentiometer value has changed, sets new Brightness and return true */
boolean checkBrightness() {
  //WS2812 takes value between 0-255
  uint16_t bright = map(constrain(analogRead(POT_BRIGHTNESS), 0, 1024), 0, 1024, 10, 255);
  if (abs(bright - brightness) > 10) {
    brightness = bright;
    strip.setBrightness(brightness);
    return true;
  }
  return false;
}
#endif

void musicAnalyzer() {
  while (true) {
    int change = getEncoderTurn();//

    if (encoderVal < 1)
      encoderVal = 1;
    if (change > 0) {
      dupa_enkodera = false;
      encoderVal = encoderVal + change;
      if (encoderVal > 1) {
        while (NUM_LEDS % encoderVal != 0 or encoderVal % 2 != 0) {
          encoderVal++;
          if (encoderVal > NUM_LEDS) {
            encoderVal = 1;
            break;
          }

        }
      }
    }
    else if (dupa_enkodera) {
      encoderVal = encoderVal - change;
      while (NUM_LEDS % encoderVal != 0 or encoderVal % 2 != 0) {
        encoderVal--;
        if (encoderVal > NUM_LEDS) {
          encoderVal = 2;
          break;
        }
      }
    }

    divide = encoderVal;
    factor = NUM_LEDS / divide;


    if (digitalRead(ON_OFF) == HIGH) {
      strip.setBrightness(0);
      strip.show();
    }
    else {
      strip.setBrightness(brightness);
#if USES_POTENTIOMETER
      checkBrightness();
#endif
      //Reset MSGEQ
      digitalWrite(RESET, HIGH);
      digitalWrite(RESET, LOW);
      //Read all 8 Audio Bands
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
        if (int(mapValue[channel]) < average * 1.1 or int(mapValue[channel]) < 10)
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
      //Shift LED values forward
      for (int k = NUM_LEDS - 1; k > 0; k--) {
        audioBuffer[k] = audioBuffer[k - 1];
      }
      //Uncomment / Comment above to Shift LED values backwards
      //for (int k = 0; k < NUM_LEDS-1; k++) {
      //  audioBuffer[k] = audioBuffer[k + 1];
      //}

      //Load new Audio value to first LED
      //Uses band 0,2,4 (Bass(red), Middle(green), High(blue) Frequency Band)
      //Lowest 8Bit: Blue , Middle Green , Highest Red
      //Use audioBuffer[NUM_LEDS-1] when using LED shift backwards!
      audioBuffer[0] = mapValue[5]; //RED
      audioBuffer[0] = audioBuffer[0] << 16;
      audioBuffer[0] |= ((mapValue[2] / 2) << 8); //GREEN
      audioBuffer[0] |= (mapValue[4] / 4);       //BLUE
      if (int(mapValue[channel]) > average * 0.5 and int(mapValue[channel]) > 10) {
        power = int(((mapValue[channel] - average) / 8.f));
        move_pixel += power;
        last_value = mapValue[channel];
      }
      else if (int(mapValue[channel]) > 6)
        move_pixel += snake_speed * 0;
      if (spped_variant) {
        if (int(mapValue[channel]) > average * 1 and int(mapValue[channel]) > 10)
          counter = counter + int(3 * ((mapValue[channel] + 20) / 20));
      }
      else if (not spped_variant)
        counter ++;
      if (counter > 255)
        counter = 0;
      if (skip_green) {
        if (counter < 128) {
          g = 0;
          temp_r = ((float)(counter) / 128.0f) * 200.0f;
          r = temp_r + 55;
          b = 255 - temp_r;
        }
        else if (counter < 256) {
          g = 0;
          temp_b = ((float)(counter - 128) / 128.0f) * 200.0f;
          b = temp_b + 55;
          r = 255 - temp_b;
        }
        Serial.print(b);
        Serial.print('\n');
        Serial.print(r);
        Serial.print('\n');
      }
      else {
        rainbow(counter);
        r = colors[0];
        g = colors[1];
        b = colors[2] ;
      }
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
      Value = (abs(int(move_pixel)) % int(factor));

      if (abs(LastValue - Value) > 12) {
        Serial.print(abs(LastValue - Value));
        Serial.print('\n');
        dupa = not dupa;
      }

      border = Value;

      DoubleLastValue = LastValue;
      LastValue = Value;
      border = Value;
      for ( int i = 0; i < NUM_LEDS; i++) {
        if ( i == 0) {
          if (dupa == true) {
            rainbow(counter + 128);
            temp_r = colors[0];
            temp_g = colors[1];
            temp_b = colors[2];
          }
          else {
            rainbow(counter);
            temp_r = colors[0];
            temp_g = colors[1];
            temp_b = colors[2];
          }
        }
        if (divide == 1) {
          temp_r = r;
          temp_g = g;
          temp_b = b;
        }
        else {
//          if ((i - 7) % factor == int(border)) {
//            if (dupa == true)
//              rainbow(counter + 16);
//            else
//              rainbow(counter + 112);
//            temp_r = colors[0];
//            temp_g = colors[1];
//            temp_b = colors[2];
//          }
//          if ((i - 6) % factor == int(border)) {
//            if (dupa == true)
//              rainbow(counter + 32);
//            else
//              rainbow(counter + 96);
//            temp_r = colors[0];
//            temp_g = colors[1];
//            temp_b = colors[2];
//          }
//          if ((i - 5) % factor == int(border)) {
//            if (dupa == true)
//              rainbow(counter + 48);
//            else
//              rainbow(counter + 80);
//            temp_r = colors[0];
//            temp_g = colors[1];
//            temp_b = colors[2];
//          }
//
//          if ((i - 4) % factor == int(border)) {
//            if (dupa == true)
//              rainbow(counter + 64);
//            else
//              rainbow(counter + 64);
//            temp_r = colors[0];
//            temp_g = colors[1];
//            temp_b = colors[2];
//          }
//          if ((i - 3) % factor == int(border)) {
//            if (dupa == true)
//              rainbow(counter + 80);
//            else
//              rainbow(counter + 48);
//            temp_r = colors[0];
//            temp_g = colors[1];
//            temp_b = colors[2];
//          }
//          if ((i - 2) % factor == int(border)) {
//            if (dupa == true)
//              rainbow(counter + 96);
//            else
//              rainbow(counter + 32);
//            temp_r = colors[0];
//            temp_g = colors[1];
//            temp_b = colors[2];
//          }
//          if ((i - 1) % factor == int(border)) {
//            if (dupa == true)
//              rainbow(counter + 112);
//            else
//              rainbow(counter + 16);
//            temp_r = colors[0];
//            temp_g = colors[1];
//            temp_b = colors[2];
//          }
          if ((i) % factor == int(border)) {
            if (dupa == false) {
              rainbow(counter + 128);
              temp_r = colors[0];
              temp_g = colors[1];
              temp_b = colors[2];
              dupa = not dupa;
            }
            else {
              rainbow(counter);
              temp_r = colors[0];
              temp_g = colors[1];
              temp_b = colors[2];
              dupa = not dupa;

            }
          }

        }

        strip.setPixelColor(i, strip.Color(temp_r, temp_g, temp_b));
      }

      strip.show();
      delay(DELAY);
      if (digitalRead(BLINKING_TOGGLE) == HIGH) {
        blinking = true;
      }
      else
        blinking = false;
    }
  }
}
