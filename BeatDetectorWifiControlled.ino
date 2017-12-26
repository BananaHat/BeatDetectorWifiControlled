/*
fht_adc.pde
guest openmusiclabs.com 9.5.12
example sketch for testing the fht library.
it takes in data on ADC0 (Analog0) and processes them
with the fht. the data is sent out over the serial
port at 115.2kb.  there is a pure data patch for
visualizing the data.

======================================================


Aid Vllasaliu Modifications:

Simplified the code. Probably a bit slowe code, but for me it does not matter that much.

Added Noise cancellation.
*/

#define DEBUG 1
#define SERIALVU 0

#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // set to 256 point fht
#define PIN 6
#define PIXELS 50

#include "channel.h"
#include <FHT.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS, PIN, NEO_RGB + NEO_KHZ800);

uint8_t A[FHT_N];
uint8_t B[FHT_N];
uint8_t Result[FHT_N];

/////////////////////////////////////////////////////////////////////////////////////////////
// Data Collection
/////////////////////////////////////////////////////////////////////////////////////////////

void GetFHT(int ref_or_signal){
  //cli();  // UDRE interrupt slows this way down on arduino1.0
  for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
    delayMicroseconds(10);
    int in = analogRead(A5);
    in = (in <= 515 && in >= 495) ? 505 : in;
    if (ref_or_signal==0) fht_input[i] = in;
    else
    if (ref_or_signal==1) fht_input[i] = analogRead(A0);
  }
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_log(); // take the output of the fht
  //sei();
}

void collect() {
  //Do FFT on Noise reference input (Analog Input 0) and store in temporary array A
    GetFHT(0);
    for (int i = 0 ; i < FHT_N ; i++) A[i] = fht_log_out[i];
    
    //Do FFT on Audio input (Analog Input 5) and store in temporary array B
    GetFHT(1);
    for (int i = 0 ; i < FHT_N ; i++) B[i] = fht_log_out[i];

    
    //Do noise cancellation
    for (int i = 0 ; i < FHT_N ; i++)
      if ( B[i]>=A[i]) Result[i] = B[i]-A[i]; //if Audio signal is louder than
                                              //noise reference, then do noise cancellation 
    serialWrite(Result, FHT_N/2);
    //Serial.write(255); // send a start byte
    //Serial.write(); // send out the data
}

Chan getChannel(Chan in) {
  in.impulse = checkBeat(in.rangeStart, in.rangeEnd);
  serialPrint(String(in.channel));
  serialPrint("; impulse: ",in.impulse);
  in.avg = in.avgTotal/10;
  if (in.impulse <100) {
    in.impulse = 0;
  }
  if (in.impulse > in.avg*in.thresh) {
    in.beat = true;
    in.skips = 0;
    in.thresh = in.thresh + (in.impulse - in.avg*in.thresh)/100;
    in.hits++;
    if (in.thresh > 1.5) {
      in.thresh = 1.5;
    } 
  }else {
    in.beat = false;
    //if (in.skips == 0) {
    //}
    in.hits = 0;
    in.skips ++;
    in.thresh = in.thresh - in.thresh / 2;
    if (in.thresh < 1.1) {
      in.thresh = 1.1;
    }
  }    
  serialPrint(", Skips: ", in.skips);
  serialPrint(", Hits: ", in.hits);
  serialPrint(", beat: ", in.beat);
  serialPrint(", thresh: ", in.thresh);
  serialPrintln("-----------");
  in.avgTotal = in.avgTotal - in.base[in.avgPos];
  in.avgTotal = in.avgTotal + in.impulse;
  in.base[in.avgPos] = in.impulse;
  in.avgPos++;
  in.avgPos = in.avgPos % 10;
  return in;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Ardunio
/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200); // use the serial port
  //TIMSK0 = 0; // turn off timer0 for lower jitter
  strip.begin();
  strip.setBrightness(100);
  strip.show(); // Initialize all pixels to 'off'
  initServer();
}

void loop() {
  checkConnection();
  unsigned long collectTime = millis();
  collect();
  serialPrintln("collect: ", (unsigned long)millis() - collectTime);
  unsigned long publish = millis();
  updateLeds();
  serialPrintln("publish: ", (unsigned long)millis() - publish);
}

void serialWrite(uint8_t array[], int i) {
  if (SERIALVU) {
    Serial.write(255);
    Serial.write(array, i);
  }
}

void serialPrint(String s) {
  if (DEBUG) {
    Serial.print(s);
  }
}

void serialPrint(String s, int i) {
  if (DEBUG) {
    Serial.print(s);
    Serial.print(i);
  }
}

void serialPrint(String s, float i) {
  if (DEBUG) {
    Serial.print(s);
    Serial.print(i);
  }
}

void serialPrintln(String s, int i) {
  if (DEBUG) {
    Serial.print(s);
    Serial.println(i);
  }
}

void serialPrintln(String s){  
  if (DEBUG) {
    Serial.println(s);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Pixel Basics
/////////////////////////////////////////////////////////////////////////////////////////////
int currentPixel = 0;
int stripR[PIXELS];
int stripB[PIXELS];
int stripG[PIXELS];

Chan r;
Chan g;
Chan b;
void updateLeds() {
  
  r.channel = 'R';
  g.channel = 'G';
  b.channel = 'B';

  r.rangeStart = 0;
  r.rangeEnd = r.rangeStart + 15;
  g.rangeStart = r.rangeEnd + 1;
  g.rangeEnd = g.rangeStart + 25;
  b.rangeStart = g.rangeEnd + 1;
  b.rangeEnd = FHT_N/2;
  
  r = getChannel(r);
  g = getChannel(g);
  b = getChannel(b);
  
  showPattern(makeColor(r, g, b));
}

void dim() {
  for (int i = 0; i < strip.numPixels(); i++) {
    setPixel(i, stripR[i]/1.2, stripG[i]/1.2, stripB[i]/1.2);
  }
}

void setPixel(int pix, int R, int G, int B) {
  strip.setPixelColor(pix, strip.Color(R, G, B));
  stripR[pix] = R;
  stripG[pix] = G;
  stripB[pix] = B;
}
/////////////////////////////////////////////////////////////////////////////////////////////
// Pixel Colors
/////////////////////////////////////////////////////////////////////////////////////////////

int makePrimaries(int i, int to) {
  int p = 0;
  while (i < to) {
    int v = + Result[i];
    p = p + v;
    i++;
  }
  return p;
}

int bind (int i) {
  if ( i > 255) {
    return 255;
  }
  return i;
}


rgb makeColor(Chan r, Chan g, Chan b) {
  rgb color;
  color.r = bind(r.impulse*(r.beat ? 0.05 : 0.2)*10);
  serialPrint("R: ", color.r);
  color.g = bind(g.impulse*(g.beat ? 0.05 : 0.2)*10);
  serialPrint(", G: ", color.g);
  color.b = bind(b.impulse*(b.beat ? 0.05 : 0.2)*10);
  serialPrintln(", B: ", color.b);
  return color;
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Pixel Patterns
///////////////////////////////////////////////////////////////////////////////////////////// 

int hits;
int skips;
long nextRain;
void showPattern(rgb color) {
  hits = r.hits + g.hits + b.hits;
  skips = r.skips + g.skips + b.skips;
  
  if(skips > 100 && hits < 5){
    if(millis() > nextRain) {
      rainbowCycle();
      nextRain = millis() + 20;
    }
    return;
  }
  
  if(r.beat || g.beat || b.beat) {
    serialPrintln(":::::");
  }
  if (skips  > 30 && (r.beat || g.beat || b.beat)) {
    pattern3(color);
    strip.show();
    currentPixel = 0;
    return;
  } else if (hits > skips && (r.beat || g.beat || b.beat)){
    pattern4(color);
  } else if (r.beat || g.beat || b.beat){
    dim();
    pattern1(color, currentPixel);
  }
  
  currentPixel++;
  currentPixel = currentPixel % strip.numPixels();
  strip.show();
}

void pattern2(rgb color, int pix) {
  /*if(skips > 4){
    currentPixel = 1;
  }*/
  setPixel(pix, color.r, color.g, color.b);
}

void pattern3(rgb color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    setPixel(i, color.r, color.g, color.b);
  }
  //setPixel(pix, color.r, color.g, color.b);
}

void pattern4(rgb color) {
  int mid = PIXELS/2;

  setPixel(mid, color.r, color.g, color.b);
  
  for (int i = 0; i < mid; i++) {
    setPixel(i, stripR[i+1], stripG[i+1], stripB[i+1]);
  } 
  
  for (int i = strip.numPixels(); i > mid; i--) {
    setPixel(i, stripR[i-1], stripG[i-1], stripB[i-1]);
  }
  
  for (int i = currentPixel%2; i < strip.numPixels(); i = i + 2) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  
}

uint16_t j = 0;
void rainbowCycle() {
  uint16_t i;
  j++;
  if(j>256) j = 0;
  //for(j=0; j<256; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
}

void pattern1(rgb color, int pix) {
  int mid = PIXELS/2;
  if(skips > 12){
    pix = 0;
    currentPixel = 0;
    skips = 5;
  } else {
    skips = 0;
  }
  setPixel(mid+(pix/2), color.r, color.g, color.b);
  setPixel(mid-(pix/2), color.r, color.g, color.b);
  while (currentPixel < 5) {
    currentPixel++;
    setPixel(mid+(currentPixel/2), color.r, color.g, color.b);
    setPixel(mid-(currentPixel/2), color.r, color.g, color.b);
    
  }
}

int checkBeat(int s, int e) {
  long beat = 0;
  for (int i = s; i < e; i++) {
    beat = beat + Result[i];
  }
  return beat;
}

void ledBrightness(int b) {
  strip.setBrightness(b);
  strip.show();
}


