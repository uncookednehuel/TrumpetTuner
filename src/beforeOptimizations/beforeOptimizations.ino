#include "arduinoFFT.h"

#define CHANNEL A0
const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
const float samplingFrequency = 1000; //Hz
unsigned int sampling_period_us;
unsigned long microseconds;

int pwmPin = 6;
int dir1 = 7;
int dir2 = 8;

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
  pinMode(12, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  // Motor pins
  pinMode(pwmPin, OUTPUT); // PWM pin
  pinMode(dir1, OUTPUT); // Dir 1
  pinMode(dir2, OUTPUT); // Dir 2
}

void loop() {
  // Read button inputs
  int enableContinuous = 1 - digitalRead(12);
  String enableContinuousRead = "enableContinuous(" + String(enableContinuous) + ")";
  int enableDiscrete = 1 - digitalRead(11);
  int tune = 1 - digitalRead(5);
  int manualUp = 1 - digitalRead(10);
  int manualDown = 1 - digitalRead(9);

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

  // Set the direction of the motor by switching the H bridge using the two direction pins
  if (manualUp ^ manualDown == 1) {
    digitalWrite(dir1, manualUp == 1 ? HIGH : LOW);
    digitalWrite(dir2, manualDown == 1 ? HIGH : LOW);
    Serial.println("Setting motor dir to high: " + String(manualUp == 1 ? HIGH : LOW) + " low: " + String(manualDown == 1 ? HIGH : LOW));
  } else {
    // Auto tuning
    float dir = calculateDirection(x);
    Serial.println(x);
    //Serial.println(dir);
    float absoluteDir = dir > 0 ? dir : -dir;
    if ((x > 150) && (absoluteDir > 0.15) && !isnan(x)) {
      int range = 200;
      //int power = constrain((255 - range) + round(range * absoluteDir), 0, 255);
      int power = 255;
      Serial.println("amonmg");
      analogWrite(pwmPin, power);
      if (dir > 0) {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
      } else {
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
      }
    } else {
      // Set motor power to half (0 to 255)
      analogWrite(pwmPin, 255);
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, LOW);
    }
  }
}

// n = 12 * log₂(f/440 Hz)
float calculateDirection(float frequency) {
  float fromA4 = 12 * (log(frequency / 440) / log(2));
  return fromA4 - round(fromA4);
}

float findFundemental(float *vData, uint16_t bufferSize) {
  float bestFrequency = 0;
  float bestMagnitude = 0;
  for (uint16_t i = 0; i < bufferSize; i++) {
    float frequency = ((i * 1.0 * samplingFrequency) / samples); // Frequency it is at
    float magnitude = vData[i];

    if (magnitude > bestMagnitude && magnitude > 50 && frequency > 200 && frequency < 250) {
      bestMagnitude = magnitude;
      bestFrequency = frequency;
    }
  }
  return bestFrequency;
}
