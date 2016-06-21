#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include "Vape.h"
#include <Adafruit_ADS1015.h>
#include <PID_v1.h>


int pioPin = 49;
int csPin = 53;
int button = 7;
int fivePin = 8;
uint8_t adsAddress = 0x49;
int probe = 2;
double KpRaw = 1;
double KiRaw = 4;
double KdRaw = 0;


void setup() {
  Serial.begin(9600);
}

void loop() {
  Executor execute(pioPin, csPin, button, fivePin, adsAddress, probe, KpRaw, KiRaw, KdRaw);
  float userWattage = 10.000;
  float setpoint = userWattage;
  int operatingMode = 1;
  execute.waitForInput(operatingMode, setpoint);
}
