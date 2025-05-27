#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define THUNDER_LED_PIN 7 // THUNDER LED PIN

#define NUM_THUNDERPIXEL 50 // NeoPixel thunder

#define VERTICAL_LED_PIN_1 6 // VERTICAL LED PIN 1
#define VERTICAL_LED_PIN_2 5 // VERTICAL LED PIN 2
#define VERTICAL_LED_PIN_3 4 // VERTICAL LED PIN 3

#define NUM_VERTICALPIXEL 20 // NeoPixel vertical
#define NUM_VERTICAL_STRIP 3 // Vertical Strip number

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals.
Adafruit_NeoPixel thunder_pixels(NUM_THUNDERPIXEL, THUNDER_LED_PIN, NEO_GRB);

Adafruit_NeoPixel vertical_pixels[] = {Adafruit_NeoPixel(NUM_VERTICALPIXEL, VERTICAL_LED_PIN_1, NEO_GRB), Adafruit_NeoPixel(NUM_VERTICALPIXEL, VERTICAL_LED_PIN_2, NEO_GRB), Adafruit_NeoPixel(NUM_VERTICALPIXEL, VERTICAL_LED_PIN_3, NEO_GRB)};

#define DELAYVAL 20 // Time (in milliseconds) to pause between vertical pixels animation pixels

struct LED_Color
{
  int red;
  int green;
  int blue;
  int brightness;
};

LED_Color led_color;

void setup()
{

  led_color.red = 181;
  led_color.green = 200;
  led_color.blue = 200;
  led_color.brightness = 255;

  thunder_pixels.begin();
  thunder_pixels.setBrightness(255);
  thunder_pixels.show();
  thunder_pixels.clear();

  for (int i = 0; i < thunder_pixels.numPixels(); ++i)
  {
    thunder_pixels.setPixelColor(i, thunder_pixels.Color(led_color.red, led_color.green, led_color.blue));
    thunder_pixels.show();
  }

  for (int i = 0; i < NUM_VERTICAL_STRIP; ++i)
  {
    vertical_pixels[i].begin();
    vertical_pixels[i].setBrightness(255);
    vertical_pixels[i].show();
    vertical_pixels[i].clear();
  }
}

void loop()
{
  thunderEffect();
  verticalEffect();
}

void verticalEffect()
{
  for (int i = 0; i < NUM_VERTICALPIXEL; ++i)
  {
    for (int j = 0; j < NUM_VERTICAL_STRIP; ++j)
    {
      vertical_pixels[j].setPixelColor(i, vertical_pixels[i].Color(led_color.red, led_color.green, led_color.blue));
      vertical_pixels[j].show();
    }

    delay(DELAYVAL);
  }

  for (int i = 0; i < NUM_VERTICALPIXEL; ++i)
  {
    for (int j = 0; j < NUM_VERTICAL_STRIP; ++j)
    {
      vertical_pixels[j].setPixelColor(i, vertical_pixels[i].Color(0, 0, 0));
      vertical_pixels[j].show();
    }

    delay(DELAYVAL);
  }
}

void thunderEffect()
{
  uint32_t color = thunder_pixels.Color(led_color.red, led_color.green, led_color.blue);

  int flashBrightness = 255;

  // number of flashes
  int flashCount = random(5, 15);
  // flash white brightness range - 0-255
  int flashBrightnessMin = 5;
  int flashBrightnessMax = 255;
  // flash duration range - ms
  int flashDurationMin = 5;
  int flashDurationMax = 75;
  // flash off range - ms
  int flashOffsetMin = 0;
  int flashOffsetMax = 75;
  // time to next flash range - ms
  int nextFlashDelayMin = 1;
  int nextFlashDelayMax = 50;

  for (int flash = 0; flash <= flashCount; flash++)
  {
    // add variety to color
    int colorV = getRandomValueOrZero(0, 50);
    color = thunder_pixels.Color(led_color.red + colorV, led_color.green + colorV, led_color.blue + colorV, flashBrightness);

    int rpt = random(4, 6);
    for (int i = 0; i < rpt; ++i)
    {
      int thunder_led_start = getRandomValueOrZero(0, NUM_THUNDERPIXEL);
      int thunder_led_end = getRandomValueOrZero(thunder_led_start, NUM_THUNDERPIXEL);
      thunder_pixels.fill(color, thunder_led_start, thunder_led_end);
      thunder_pixels.show();
      delay(random(flashOffsetMin, flashOffsetMax));
    }

    thunder_pixels.clear();
    thunder_pixels.show();
    delay(random(nextFlashDelayMin, nextFlashDelayMax));
  }
}

int getRandomValueOrZero(int min, int max)
{
  int rnd_val = random(min, max);

  if (rnd_val < 0)
  {
    return 0;
  }

  return rnd_val;
}