/*
 * Hermes LED shoes
 * Copyright 2013-2014 RGAM LLC
 *
 */

/* Run parameters: */
#define MAX_BRIGHTNESS 0.65 // Max LED brightness.
#define MIN_BRIGHTNESS 0.3
#define WAIT_FOR_KEYBOARD 0 // Use keyboard to pause/resume program.

/* Neopixel parameters: */
#define LED_COUNT 100
#define DATA_PIN 6

/* Animation parameters: */
// ~15 ms minimum crawl speed for normal mode,
// ~2 ms minimum for superfast hack mode.
#define CRAWL_SPEED_MS 10
// General sensitivity of the animation.
// Raising this raises the vector magnitude needed to reach max (purple),
// and thus lowers sensitivity.
// Eg: 800 = more sensitive, 1600 = less sensitive
#define HERMES_SENSITIVITY 1000.0
// Emulate two strips by starting the crawl in the
// middle of the strip and crawling both ways.
#define ENABLE_SPLIT_STRIP 1
// Center LED, aka LED #0.
#define SPLIT_STRIP_CENTER 1

/* Sleeping parameters: */
#define SLEEP_BRIGHTNESS 0.30
#define SLEEP_CYCLE_MS 5000 // 5 second breathing cycle.
#define SLEEP_WAIT_TIME_MS 5000 // No movement for 5 seconds triggers breathing.
#define SLEEP_SENSITIVITY 25

/* Debug parameters: */
#define PRINT_LOOP_TIME 0

/* Advanced: */
#define ONBOARD_LED_PIN 7 // Pin D7 has an LED connected on FLORA.

///////////////////////////////////////////////////////////////////

// LED imports.
#include <Adafruit_NeoPixel.h>

// Accel imports.
#include <Wire.h>
#include <Adafruit_LSM303_Old.h>

// Our custom data type.
#include "AccelReading.h"


// Pushbutton setup
#define NUMMODES 6
int buttonPin = 10;             // the number of the pushbutton pin
int buttonState;               // the current reading from the input pin
int lastButtonState = HIGH;    // the previous reading from the input pin
int buttonMode = 0;
int buttonColorMode = 0;
long buttonWaitTime = 0;         // the last time the output pin was toggled
long buttonWaitDelay = 500;
int colorIncrement = 10;
bool isColorMode = 0;      
int brightnessIncrease = 30;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, DATA_PIN, NEO_RGB + NEO_KHZ800);

void setup() {
  if (WAIT_FOR_KEYBOARD) {
    Serial.begin(9600);

    // Wait for serial to initalize.
    while (!Serial) { }

    Serial.println("Strike any key to start...");

    // Wait for the next keystroke.
    while (!Serial.available()) { }

    // Clear the serial buffer.
    Serial.read();
  }
  
  // Make input & enable pull-up resistors on switch pins for pushbutton
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH); 
  lastButtonState = digitalRead(buttonPin);
  
  checkSuperfastHack();
  
  colorSetup();
  
  accelSetup();
}

void loop() {
  loopDebug();

  checkButton();
  //Serial.println(buttonState);
  //Serial.println(buttonMode);



  if (isColorMode) {
    colorWipe(color(buttonColorMode, brightnessIncrease), 0); // Red
  } else {
    switch (buttonMode) {
    case 0:
    accelPoll();
    updateLED();
    break;
    case 1:
    rainbowCycle(10, 0, 50);
    break;
    case 2:
    rainbowCycle(30, 50, 100);
    break;
    case 3:
    rainbowCycle(50, 100, 150);
    break;
    case 4:
    rainbowCycle(100, 150, 255);
    break;
    case 5:
    rainbow(10);
    break;
    case 6:
    rainbowCycle(10, 0, 255);
    break;
  }
  }
  

}

bool checkButton(){
  int buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    if ((millis() - buttonWaitTime) < buttonWaitDelay){
      isColorMode = 1;
      buttonColorMode += colorIncrement;
      if (buttonColorMode > 384){
        buttonColorMode = 0;
        brightnessIncrease += 10;
        if (brightnessIncrease > 100){
        brightnessIncrease = 10;
        }
      }
      
      
    } else {
      isColorMode = 0;
      buttonMode += 1;
      if (buttonMode >= NUMMODES) {
        buttonMode = 0;
      }
      isColorMode = 0;
    }
    buttonWaitTime = millis();
    lastButtonState = buttonState;
    return true;
  } else {
    return false;
  }
}


// Debug functions controlled by run/debug parameters.
unsigned long before = 0;
void loopDebug() {
  if (WAIT_FOR_KEYBOARD) {
    pauseOnKeystroke();
  }
  if (PRINT_LOOP_TIME) {
    unsigned long now = millis();
    Serial.println(now - before);
    before = millis();
  }
}

void checkSuperfastHack() {
  #if SUPERFAST_LED_HACK
    #ifdef _COMPILE_TIME_LEDS_
      Serial.println("Using superfast LED hack.");
    #elif
      // Wait for serial to initalize.
      while (!Serial) { }
      Serial.println("WARNING: You need to install the LPD8806Fast library.");
    #endif
  #endif
}

void pauseOnKeystroke() {
  if (Serial.available()) {
    // Clear the serial buffer.
    Serial.read();

    Serial.println("Paused. Strike any key to resume...");

    // Turn all LEDs off.
    showColorOff();

    // Wait for the next keystroke.
    while (!Serial.available()) { }

    // Clear the serial buffer.
    Serial.read();
  }
}

/*
/////// Button

void buttonCheck() {
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    
  }

  if (buttonState == LOW && lastButtonState == LOW) {
    if ((millis() - buttonHoldTime) > buttonHoldDelay) {

      if(mode == 2) {
        mode = 0;
        lastMin = 16;
        lastHour = 16;
        colorWipe(strip.Color(0, 0, 0), 20);
        buttonHoldTime = millis();
      } 
      else {
        mode = mode + 1;
        colorWipe(strip.Color(0, 0, 0), 20);
        buttonHoldTime = millis();
      }
    }
  }
}
*/

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// accel //
///////////

Adafruit_LSM303_Old lsm; // Bridge to accelerometer hardware.
AccelReading accelBuffer[10]; // Buffer for storing the last 10 readings.
int bufferPosition; // Current read position of the buffer.

double calibration; // Baseline for accelerometer data.
unsigned long calibrationLEDTime;
bool calibrationLEDOn;

// For breathing, track the time of the last significant movement.
unsigned long lastSignificantMovementTime;

// Initialization.
void accelSetup() {
  Serial.println("BEGIN");
  
  lsm.begin();
  
  bufferPosition = 0;

  // Initialize the full buffer to zero.
  for (int i = 0; i < bufferSize(); i++) {
    accelBuffer[i].x = 0;
    accelBuffer[i].y = 0;
    accelBuffer[i].z = 0;
  }
  
  calibrate();
}

void calibrate() {
  calibration = 0;
  calibrationLEDTime = 0;
  calibrationLEDOn = false;
  
  showCalibration();

  while (1) {
    // Update onboard LED.
    unsigned long now = millis();
    if (now - calibrationLEDTime > 250) {
      calibrationLEDTime = now;
      calibrationLEDOn = !calibrationLEDOn;
      digitalWrite(ONBOARD_LED_PIN, calibrationLEDOn ? HIGH : LOW);
    }
    
    // Fill the buffer.
    if(!fillBuffer()) {
      delay(10);
      continue;
    }
    
    // Check to see if we're done.
    bool pass = true;
    double avg = 0;
    for (int i = 0; i < bufferSize(); i++) {
      double m = getMagnitude(accelBuffer[i]);
      pass = pass && (abs(m - calibration) < 10);
      avg += m;
    }
    
    if (pass) {
      if (WAIT_FOR_KEYBOARD) {
        Serial.print("Calibration: ");
        Serial.println(calibration);
      }
      break;
    } else {
      avg /= bufferSize();
      calibration = avg;
    }
  }
  
  // Turn the calibration light off.
  digitalWrite(ONBOARD_LED_PIN, LOW);
}

// Gathers data from accelerometer into the buffer. Only writes to the buffer
// if the hardware has gathered data since we last wrote to the buffer.
void accelPoll() {
  // Read new accelerometer data. If there is no new data, return immediately.
  if (!fillBuffer()) {
    return;
  }
  
  /* PRINT DATA: */
  // printBuffer();
  // printDelta();
  // printMagnitude();
  // Serial.println();
}

// Gets the vector for the given reading.
double getVector(AccelReading reading) {
  double normalizedVector = abs(calibration - getMagnitude(reading));
  return normalizedVector;
}

///////////////////////////////////////////////////////////////////

// This may or may not fill the next buffer position. If the accelerometer hasn't
// processed a new reading since the last buffer, this function immediately exits,
// returning false.
// Otherwise, if the accelerometer has read new data, this function advances the
// buffer position, fills the buffer with accelerometer data, and returns true.
bool fillBuffer() {
  // Read from the hardware.
  lsm.read();
  
  AccelReading newReading;
  newReading.x = lsm.accelData.x;
  newReading.y = lsm.accelData.y;
  newReading.z = lsm.accelData.z;
  
  // The accelerometer hasn't processed a new reading since the last buffer.
  // Do nothing and return false.
  if (equalReadings(getCurrentReading(), newReading)) {
    return false;
  }
  
  // The accelerometer has read new data.
  
  // Advance the buffer.
  if (++bufferPosition >= bufferSize()) {
    bufferPosition = 0;
  }

  AccelReading *mutableCurrentReading = &accelBuffer[bufferPosition];
  
  mutableCurrentReading->x = newReading.x;
  mutableCurrentReading->y = newReading.y;
  mutableCurrentReading->z = newReading.z;
  
  return true;
}

///////////////////////////////////////////////////////////////////

// Gets the average difference between the latest buffer and previous buffer.
int getDelta() {
  AccelReading previousReading = getPreviousReading();
  AccelReading currentReading  = getCurrentReading();
  
  int deltaX = abs(abs(currentReading.x) - abs(previousReading.x));
  int deltaY = abs(abs(currentReading.y) - abs(previousReading.y));
  int deltaZ = abs(abs(currentReading.z) - abs(previousReading.z));
  
  return (deltaX + deltaY + deltaZ) / 3;
}

void printDelta() {
  AccelReading previousReading = getPreviousReading();
  AccelReading currentReading  = getCurrentReading();
  
  int deltaX = abs(abs(currentReading.x) - abs(previousReading.x));
  int deltaY = abs(abs(currentReading.y) - abs(previousReading.y));
  int deltaZ = abs(abs(currentReading.z) - abs(previousReading.z));

  Serial.print(deltaX); Serial.print ("\t");
  Serial.print(deltaY); Serial.print ("\t");
  Serial.print(deltaZ); Serial.print ("\t");
  Serial.print(getDelta()); Serial.println();
}

// Gets the vector magnitude for the given reading.
// http://en.wikipedia.org/wiki/Euclidean_vector#Length
double getMagnitude(AccelReading reading) {
  double x = reading.x;
  double y = reading.y;
  double z = reading.z;

  double vector = x * x + y * y + z * z;

  return sqrt(vector);
}

void printMagnitude() {
  Serial.println(getMagnitude(getCurrentReading()));
}

// Prints the latest buffer reading to the screen.
void printBuffer() {
  Serial.print(accelBuffer[bufferPosition].x); Serial.print ("\t");
  Serial.print(accelBuffer[bufferPosition].y); Serial.print ("\t");
  Serial.print(accelBuffer[bufferPosition].z); Serial.println();
}

///////////////////////////////////////////////////////////////////

// Returns the number of items held by the buffer.
int bufferSize() {
  return sizeof(accelBuffer) / sizeof(accelBuffer[0]);
}

AccelReading getCurrentReading() {
  return accelBuffer[bufferPosition];
}

// Gets the previous buffer reading.
AccelReading getPreviousReading() {
  int previous = bufferPosition - 1;
  if (previous < 0) previous = bufferSize() - 1;
  return accelBuffer[previous];
}

// Returns true if two readings are equal.
bool equalReadings(AccelReading a, AccelReading b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// color //
///////////

int COLOR_RANGE = 384;
uint32_t lastColor;
unsigned long lastCrawl;
uint32_t lightArray[LED_COUNT];



void colorSetup() {
  lastColor = 0;
  lastCrawl = 0;
  
  // Turn the strip on.
  strip.begin();
  stripShow();
  
  // Initialize the LED buffer.
  for (int i = 0; i < LED_COUNT; i++) {
    lightArray[i] = 0;
  }
}

void updateLED() {
  // LED color takes a value from 0.0 to 1.0. Calculate scale from the current vector.

  // Largest vector needed to hit max color (1.0).
  double upperBound = HERMES_SENSITIVITY;
  double normalizedVector = abs(calibration - getMagnitude(getCurrentReading()));
  double scale = normalizedVector / upperBound;
  
  uint32_t pixelColor = pixelColorForScale(scale);
  
  // Change LED strip color.
  //showColor(scale);
  
  if (sleep()) {
    breathe();
  } else {
    crawlColor(pixelColor);
  }
}

// "Crawls" the given color along the strip.
// This always sets LED[0] to the given color.
// After CRAWL_SPEED_MS milliseconds,
// we set LED[n + 1] = LED[n] for each LED.
void crawlColor(uint32_t color) {
  
  // Set the head pixel to the new color.
  uint32_t head = lightArray[0];
  lightArray[0] = color;
  
  unsigned long now = millis();
  
  // Shift the array if it's been long enough since last shifting,
  // or if a new color arrives.
  bool shouldUpdate = 
      (now - lastCrawl > CRAWL_SPEED_MS)
      || (color != head);

  if (!shouldUpdate) {
    return;
  }

  lastCrawl = now;
  
  // Shift the array.
  for (int i = LED_COUNT - 1; i > 0; --i) {
    lightArray[i] = lightArray[i - 1];
  }

  if (ENABLE_SPLIT_STRIP) {
    int centerLED = SPLIT_STRIP_CENTER;
    int LEDsPerSide = floor(LED_COUNT / 2);
  
    // Crawl 'low' side (center down)
    uint32_t *pixelColor = lightArray;
    for (int led = centerLED - 1; led >= centerLED - 1 - LEDsPerSide; led--) {
      strip.setPixelColor(constrainBetween(led, 0, LED_COUNT - 1), *pixelColor++);
    }
  
    // Crawl 'high' side (center up)
    pixelColor = lightArray;
    for (int led = centerLED; led < centerLED + LEDsPerSide; led++) {
      strip.setPixelColor(constrainBetween(led, 0, LED_COUNT - 1), *pixelColor++);
    }
  
    stripShow();
    
    return;
  }
  
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, lightArray[i]);
  }
  stripShow();
}

int constrainBetween(int value, int lower, int higher) {
  if (value < lower) {
    value = higher - (lower - value) + 1;
  } else if (value > higher) {
    value = lower + (value - higher) - 1;
  }
  return value;
}

// Sets the strip all one color.
// Scale parameter is a value 0.0 to 1.0,
// representing how far on the rainbow to go.
void showColor(float scale) {
  uint32_t pixelColor = pixelColorForScale(scale);

  if (pixelColor == lastColor) {
    // No change since we last set the pixels; don't bother changing them.
    return;
  }
  lastColor = pixelColor;

  // Serial.print("Show "); Serial.print(scale); Serial.println(c);
  for (int i = 0; i < LED_COUNT; i++) {
   strip.setPixelColor(i, pixelColor);
  }
  stripShow();
}

// Returns a pixel color for use by strip.setPixelColor().
// Automatically adjusts brightness.
// Takes a scale, from 0.0 to 1.0, indicating progression
// through the color rainbow.
uint32_t pixelColorForScale(double scale) {
  float brightness = MAX_BRIGHTNESS * (scale + MIN_BRIGHTNESS);
  int c = COLOR_RANGE * scale; // Intentionally round to an int.

  return color(c, brightness);
}

// Shows the color progression.
void showColorProgression() {
  for (int j = 0; j < 384; j++) {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, color(j, 0.5));
    }
    stripShow();
    delay(1);
  }
  
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
  }
  stripShow();
  delay(1);
}

// Color 1 from 384; brightness 0.0 to 1.0.
uint32_t color(uint16_t color, float brightness)  {
  byte r, g, b;
  int range = color / 128;
  switch (range) {
    case 0: // Red to Yellow (1 to 128)
      r = 127 - color % 128;
      g = color % 128;
      b = 0;
      break;
    case 1: // Yellow to Teal (129 to 256)
      r = 0;
      g = 127 - color % 128;
      b = color % 128;
      break;
    case 2: // Teal to Purple (257 to 384)
      r = color % 128;
      g = 0;
      b = 127 - color % 128;
      break;
  }
  r *= brightness;
  g *= brightness;
  b *= brightness;
  return strip.Color(r, g, b);
}

void showColorOff() {
  colorOff();
  stripShow();
}

void colorOff() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
  }
}

// Show the calibration colors.
void showCalibration() {
  colorOff();

  int mid = LED_COUNT / 2;
  float brightness = 0.3;
  
  // Red
  strip.setPixelColor(mid - 1, strip.Color(127 * brightness, 0, 0));
  // Green
  strip.setPixelColor(mid, strip.Color(0, 127 * brightness, 0));
  // Blue
  strip.setPixelColor(mid + 1, strip.Color(0, 0, 127 * brightness));
  
  stripShow();
}

void stripShow() {
  #if SUPERFAST_LED_HACK
    #ifdef _COMPILE_TIME_LEDS_
      // These settings are for Leonardo (ATmega32u4) with
      // LED pins data=6, clock=12.
      // See CompileTimeLEDs.h for more info.
      strip.showCompileTime<6, 7>(PORTD, PORTD);
    #elif
      // Can't actually use superfast hack, it isn't installed properly.
      strip.show();
    #endif
    return;
  #endif

  strip.show();
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// sleep //
///////////
bool sleeping = false;

bool sleep() {
  unsigned long now = millis();

  // See if this movement is significant, aka enough to wake us from sleep.
  double m = getMagnitude(getCurrentReading());
  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    lastSignificantMovementTime = now;
  }
  
  // Last significant movement time needs to be longer than sleep wait time.
  if (now - lastSignificantMovementTime < SLEEP_WAIT_TIME_MS) {
    // Haven't waited long enough.
    resetBreathe();
    sleeping = false;
    return false;
  }
  
  // Only start sleeping on the sleep period.
  if (!sleeping && (now % SLEEP_CYCLE_MS != 0)) {
    resetBreathe();
    sleeping = false;
    return false;
  }
  
  sleeping = true;

  return true;
}

///////////////////////////////////////////////////////////////////

const uint8_t KEYFRAMES[]  = {
  // Rising
  20, 21, 22, 24, 26, 28, 31, 34, 38, 41, 45, 50, 55, 60, 66, 73, 80, 87, 95,
  103, 112, 121, 131, 141, 151, 161, 172, 182, 192, 202, 211, 220, 228, 236,
  242, 247, 251, 254, 255,

  // Falling
  254, 251, 247, 242, 236, 228, 220, 211, 202, 192, 182, 172, 161, 151, 141,
  131, 121, 112, 103, 95, 87, 80, 73, 66, 60, 55, 50, 45, 41, 38, 34, 31, 28,
  26, 24, 22, 21, 20,
  20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 
};

unsigned long lastBreath = 0.0;
int keyframePointer = 0;

void resetBreathe() {
  keyframePointer = 0;
}

void breathe() {
  int numKeyframes = sizeof(KEYFRAMES) - 1;
  float period = SLEEP_CYCLE_MS / numKeyframes;
  unsigned long now = millis();
  
  if ((now - lastBreath) > period) {
    lastBreath = now;

    for (int i = 0; i < strip.numPixels(); i++) {
      uint8_t color = (SLEEP_BRIGHTNESS * 127 * KEYFRAMES[keyframePointer]) / 256;
      strip.setPixelColor(i, color, 0, 0);
    }
    strip.show();   

    // Increment the keyframe pointer.
    if (++keyframePointer > numKeyframes) {
      // Reset to 0 after the last keyframe.
      keyframePointer = 0;
    }   
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    if (checkButton()){
        break;
      }
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    if (checkButton()){
        break;
      }
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait, int cyclestart, int cycleend) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    if (checkButton()){
        break;
      }
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
      if (checkButton()){
        break;
      }
      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 255; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();
      if (checkButton()){
        break;
      }
      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
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

