// Audiolux 2.0 :: NeoPixel audio-light display.
// Benjamin Shanahan, 21 January 2016.
//
// Uses FFT to analyze frequencies of incoming audio to produce a light show.
//
// This file inverts the SA (spectrum analyzer) output so that the light strips can be installed 
// in the opposite orientation.



// INCLUDE //
#include "FastLED.h"
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>



// VARIABLES //
// LED controller
#define BV_STRIP 0
#define SA_STRIP 1
#define NUM_STRIPS 2
CLEDController *controllers[NUM_STRIPS];
uint8_t global_brightness = 128;
// power
#define POWER_PIN 13
// mode
#define MODE_PIN 7
#define MODE_COUNT 2  // number of modes
#define MODE_HANG 30  // delay before mode can change
#define MODE_SA 0
#define MODE_PEAK 1
int mode = 0;
int mode_change_count = 0;
// bass LED
#define BASS_PIN 2              // fires on heavy bass hit
#define BASS_EXTENSION_PIN 3    // additional pins for new lights 
#define BASS_HIT_THRESHOLD 75   // between 0-255
#define BASS_FREQ_BIN 1         // which frequency bin determines a bass hit
// audio line-in
#define LINE_IN A2
float current_audio_peak;
float max_audio_peak = 0;
// dual-channel spectral analyzer
#define SA_DATA_PIN 4
#define SA_NUM_LEDS 15
#define SA_INPUT_SCALE 110  // arbitrary scale for audio line-in
#define SA_BASE_MAX_VALUE_IN 65  // ceiling for line-in
// bass visualizer
#define BV_DATA_PIN 5
#define BV_NUM_LEDS 60
// FFT calculations
// see http://www.pjrc.com/teensy/gui/?info=AudioAnalyzeFFT256
#define FFT_TYPE 1024                 // update this to match FFT being used
#define FREQ_RES 43                   // selected FFT frequency resolution in HZ
float min_freq = FREQ_RES;
float max_freq = FREQ_RES*(FFT_TYPE/2);
// hanging dot for peak meter display
#define DOT_HANG 10      // time for dot to hang before falling to current peak
int dot_hang_count = 0;  // frame counter for holding peak dot
int dot_fall_count = 0;  // frame counter for falling dot
int dot_peak = 0;        // current peak of dot
// initialize LED arrays
CRGB sa_leds[SA_NUM_LEDS];
CRGB bv_leds[BV_NUM_LEDS];
// creat audio components
AudioInputAnalog adc(LINE_IN);
AudioAnalyzePeak peak;
AudioAnalyzeFFT1024 fft;
AudioConnection patch1(adc, fft);
AudioConnection patch2(adc, peak);
// loop variables
uint8_t hue;
int counter;
float databin;
float loweredge;
float upperedge;
int current_bass;



// FUNCTIONS //
// map linear value to logarithmic scale
// logspace(start frequency, stop frequency, point to compute, number of bins)
double logspace(double start, double stop, int n, int N) {
    return start * pow(stop/start, n/(double)(N-1));
}
// push new color to beginning of bv_leds array
void bv_shift_right(CRGB c) {
  int k = BV_NUM_LEDS - 1;
  while (k > 0) {
    bv_leds[k] = bv_leds[k-1];
    k--;
  }
  bv_leds[0] = c;
}
void bv_shift_left(CRGB c) {
  int k = 0;
  while (k < BV_NUM_LEDS-1) {
    bv_leds[k] = bv_leds[k+1];
    k++;
  }
  bv_leds[BV_NUM_LEDS-1] = c;
}
// wheel function: input a value from 0-255 and get a color back
CRGB wheel(byte pos, int brightness) {
  float b = constrain(float(brightness) / 255.0, 0, 1);
  if(pos < 85) {
    return CRGB(int(pos*3*b), int((255-pos*3)*b), 0);
  } else if(pos < 170) {
    pos -= 85;
    return CRGB(int((255-pos*3)*b), 0, int(pos*3*b));
  } else {
    pos -= 170;
    return CRGB(0, int(pos*3*b), int((255-pos*3)*b));
  }
}
// fire bass visualizer
void bv_fire(int p) {
  // bv_shift_right(wheel(p, p));
  bv_shift_left(wheel(p, p));
}
// draw a line on SA LED strip
void draw_line(int p) {
  for(int i = 0; i < SA_NUM_LEDS; i++) {
    CRGB color = wheel(map(i,0,SA_NUM_LEDS,0,255), 255);
    if (i <= p && p > 0) sa_leds[i] = color;
    else sa_leds[i] = CRGB(0,0,0);
  }
}
// draw colored dot at given pixel
void draw_dot(int p) {
  sa_leds[p] = wheel(map(p,0,SA_NUM_LEDS,0,255), 255);
}



// PROGRAM LOOP //
void setup() {
  AudioMemory(64); // allocate memory
  fft.windowFunction(AudioWindowHanning1024);

  controllers[0] = &FastLED.addLeds<NEOPIXEL,BV_DATA_PIN>(bv_leds,BV_NUM_LEDS);
  controllers[1] = &FastLED.addLeds<NEOPIXEL,SA_DATA_PIN>(sa_leds,SA_NUM_LEDS);

  pinMode(POWER_PIN, OUTPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(BASS_PIN, OUTPUT);
  pinMode(BASS_EXTENSION_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  Serial.begin(9600);
}
void loop() {
  // loop through light modes on button press
  if (digitalRead(MODE_PIN) == HIGH) {  // button is being pressed
    if (mode_change_count <= 0) {
      mode_change_count = MODE_HANG;  // set this first so button inactivates immediately
      if (mode == MODE_COUNT - 1) mode = 0;
      else mode++;
      FastLED.clear(); // clear all strips
    }
  }
  if (mode_change_count > 0) mode_change_count--;

  // current audio peak
  current_audio_peak = peak.read();

  loweredge = 0;
  if (fft.available()) {  // check if fft data is available
    counter = 0;
    current_bass = 0;

    for (int led = 0; led < SA_NUM_LEDS; led++) {
      upperedge = logspace(min_freq, max_freq, led, SA_NUM_LEDS);
      databin = 0;  // bin with all data in range
      
      for (int i = 0; i < FFT_TYPE/2; i += 1) {  // bin into logarithmic bins
        float data = fft.read(i);
        if (i*FREQ_RES >= loweredge && i*FREQ_RES <= upperedge) {
          databin += data;
        } else if (i*FREQ_RES > upperedge) break;
      }
      
      if (databin >= 0.01) {
        int data2int = int(databin * SA_INPUT_SCALE);
        hue = map(data2int, 0, SA_BASE_MAX_VALUE_IN, 0, 255);
        if (led == BASS_FREQ_BIN) current_bass = hue;  // current bass value
      } else hue = 0;
      if (mode == MODE_SA) {  // spectrum analyzer
        if (hue > 0) sa_leds[SA_NUM_LEDS-1-counter] = CHSV(hue, 255, hue);
      }
      counter++;
      loweredge = upperedge;  // update afterwards with upperedge
    }
    controllers[SA_STRIP]->showLeds(global_brightness);  // update strip

    // BASS VISUALIZER
    controllers[BV_STRIP]->showLeds(global_brightness);  // update strip (doesn't work if after the following if/then loop)
    if (current_bass >= BASS_HIT_THRESHOLD) {
      digitalWrite(BASS_PIN, HIGH);
      digitalWrite(BASS_EXTENSION_PIN, HIGH);
      bv_fire(map(current_bass, BASS_HIT_THRESHOLD, 255, 25, 255));
    } else {
      digitalWrite(BASS_PIN, LOW);
      digitalWrite(BASS_EXTENSION_PIN, LOW);
      bv_fire(0);
    }
  }

  // peak meter
  if (mode == MODE_PEAK) {
    int h = map(int(current_audio_peak*255), 0, 255, 0, SA_NUM_LEDS);
    draw_line(h);
  
    // add hanging dot at peak amplitude
    if (h > dot_peak) {  // keep dot on top of amplitude
      dot_peak = h;
      dot_hang_count = 0;  // reset hang count
      dot_fall_count = 0;  // reset fall count
    } else {
      // can we let the dot start to fall?
      if (millis() % 3 == 0) {  // delay this a bit
        if (dot_hang_count >= DOT_HANG) dot_peak--;
        else dot_hang_count++;
      }
    }
    if (dot_peak > 0) draw_dot(dot_peak);  // draw dot at pixel p
  }
  controllers[SA_STRIP]->showLeds(global_brightness);  // update strip
}
