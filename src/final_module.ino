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
double pidSetpoint = 0;

float userWattage = 10.000;
float setpoint = userWattage;
int operatingMode = 1;

bool not_init = true;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Executor execute(pioPin, csPin, fivePin, adsAddress, probe, KpRaw, KiRaw, KdRaw);
  if(not_init) {
    Translator translate(button);
    pidSetpoint = translate.takeInput( operatingMode, setpoint);
    not_init = false;
  }
  execute.waitForInput(pidSetpoint);
}
