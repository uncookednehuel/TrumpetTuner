#include "arduinoFFT.h"

#define CHANNEL A0

// Parameters
const float lowFrequency = 210;
const float highFrequency = 235;
const float magnitudeThreshold = 15;

const uint16_t samples = 512; // This value MUST ALWAYS be a power of 2
const float samplingFrequency = highFrequency * 2; // Hz
unsigned int sampling_period_us;
unsigned long microseconds;

// Math optimization
const float logTwo = log(2);

// Motor pins
const int pwmPin = 6;
const int dir1Pin = 7;
const int dir2Pin = 8;

const int enableContinuousPin = 12;
const int enableDiscretePin = 11;
const int tunePin = 5;
const int manualUpPin = 10;
const int manualDownPin = 9;

float vReal[samples];
float vImag[samples];

/* Create FFT object with weighing factor storage */

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency, true);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  //start serial connection
  Serial.begin(9600);

  // Button inputs
  pinMode(enableContinuousPin, INPUT_PULLUP);
  pinMode(enableDiscretePin, INPUT_PULLUP);
  pinMode(manualUpPin, INPUT_PULLUP);
  pinMode(manualDownPin, INPUT_PULLUP);
  pinMode(tunePin, INPUT_PULLUP);

  // Motor pins
  pinMode(pwmPin, OUTPUT); // PWM pin
  pinMode(dir1Pin, OUTPUT); // Dir 1
  pinMode(dir2Pin, OUTPUT); // Dir 2

  analogWrite(pwmPin, 255);
}

void loop() {
  // Read button inputs
  int enableContinuous = 1 - digitalRead(enableContinuousPin);
  String enableContinuousRead = "enableContinuous(" + String(enableContinuous) + ")";
  int enableDiscrete = 1 - digitalRead(enableDiscretePin);
  int tune = 1 - digitalRead(tunePin);
  int manualUp = 1 - digitalRead(manualUpPin);
  int manualDown = 1 - digitalRead(manualDownPin);

  // Manual up/down buttons
  if (manualUp ^ manualDown == 1) {
    digitalWrite(dir1Pin, manualUp == 1 ? HIGH : LOW);
    digitalWrite(dir2Pin, manualDown == 1 ? HIGH : LOW);
    Serial.println("Setting motor dir to high: " + String(manualUp == 1 ? HIGH : LOW) + " low: " + String(manualDown == 1 ? HIGH : LOW));\
    return;
  }

  // Return if in discrete mode and not tuning
  if(enableDiscrete == 1 && tune == 0) { 
    setDir(0);
    return;
  }

  // Sampling
  microseconds = micros();
  for(int i=0; i<samples; i++) {
      vReal[i] = analogRead(A0);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us) { /* Empty loop*/ }
      microseconds += sampling_period_us;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	// Weigh data
  FFT.compute(FFTDirection::Forward); // Compute FFT
  FFT.complexToMagnitude(); // Compute magnitudes
  float x = findFundemental(vReal, (samples >> 1));

  // Auto tuning
  float dir = calculateCents(x);
  Serial.println(String(x) + " : " + String(dir));

  float absoluteDir = dir > 0 ? dir : -dir;
  if ((absoluteDir > 0.10) && x != 0) {
    setDir((dir * -1) / absoluteDir);
    delay(500);
    setDir(0);
  } else {
    setDir(0);
  }
}

void setDir(int dir) {
  if (dir == -1) {
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    return;
  }
  if (dir == 1) {
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
    return;
  }
  if (dir == 0) {
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, LOW);
    return;
  }
}

// n = 12 * log₂(f/440 Hz)
float calculateCents(float frequency) {
  float fromA4 = 12 * (log(frequency / 440) / logTwo);
  return fromA4 - round(fromA4);
}

// Finds the fundamental frequency by looping through samples and finding the peak that fits through the filter
float findFundemental(float *vData, uint16_t bufferSize) {
  // Pre-calculates the index range to reduce loop iterations
  int firstIndex = frequencyIndex(lowFrequency);
  int lastIndex = frequencyIndex(highFrequency);
  float bestFrequency = 0;
  float bestMagnitude = 0;

  for (uint16_t i = firstIndex; i < lastIndex; i++) {
    // Calculates frequency currently being tested
    float frequency = ((i * 1.0 * samplingFrequency) / samples);
    float magnitude = vData[i];

    //Filters out based on magnitude and frequency
    if (magnitude > bestMagnitude && magnitude > magnitudeThreshold && frequency > lowFrequency && frequency < highFrequency) {
      bestMagnitude = magnitude;
      bestFrequency = frequency;
    }
  }
  return bestFrequency;
}

float frequencyIndex(float frequency) {
  return (frequency * samples) / samplingFrequency;
}
