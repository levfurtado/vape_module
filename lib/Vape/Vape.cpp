#include "Arduino.h"
#include "Vape.h"
#include "../Adafruit_ADS1X15_master/Adafruit_ADS1015.h"
#include <SPI.h>
#include "../Arduino_PID_Library/PID_v1.h"

Translator::Translator(int button) {
    _button = button;
}

double Translator::takeInput(int pidSetting, float rawSetting) {
  if(pidSetting == 0 ) {
    double setpoint = setVoltage(rawSetting);
    return setpoint;
  }
  else if(pidSetting == 1) {
    double setpoint = setWattage(rawSetting);
    return setpoint;
  }
}

double Translator::setVoltage(float voltage) {
  double setVoltage = voltage * 156;
  return setVoltage;
}

double Translator::setWattage(float wattage) {
  OhmMeter ohmMeter(_button);
  float resistance = ohmMeter.ohmTranslate(ohmMeter.ohmCheck());
  double wattageSetpoint = sqrt(wattage * resistance) * 156;
  Serial.print("setWattage"); Serial.println(wattage);
  Serial.print("Ohms"); Serial.println(resistance);
  Serial.print("wattageSetpoint"); Serial.println(wattageSetpoint);
  Serial.print("root"); Serial.println(sqrt(wattage * resistance));
  return wattageSetpoint;
}

Executor::Executor(int pioPin, int csPin, int fivePin, uint8_t adsAddress, int probe, double KpRaw, double KiRaw, double KdRaw) {
  _pioPin = pioPin;
  _csPin = csPin;
  _fivePin = fivePin;
  _adsAddress = adsAddress;
  _probe = probe;
  _KpRaw = KpRaw;
  _KiRaw = KiRaw;
  _KdRaw = KdRaw;
}

void Executor::waitForInput(double pidSetpoint) {
  PhysicalInput pio(_pioPin);
  DigitalPot digiPot(_csPin);
  FiveFiveFive five(_fivePin);
  VoltageMeter vMeter(_adsAddress);
  double InputRaw = 0;
  double OutputRaw = 0;
  double SetpointRaw = 0;
  PID myPIDRaw(&InputRaw, &OutputRaw, &SetpointRaw, _KpRaw, _KiRaw, _KdRaw, DIRECT);
  myPIDRaw.SetMode(AUTOMATIC);
  myPIDRaw.SetOutputLimits(0, 255);
  myPIDRaw.SetSampleTime(1);

  if(pio.readInput()) {
    Serial.print("after"); Serial.println(pio.readInput());
    // Used in order for raspberry pi monitoring device to get "up to speed"
    delay(500);
    while(pio.readInput()) {
      five.fiveEnable();
      InputRaw = vMeter.voltage(_probe);
      SetpointRaw = pidSetpoint;
      myPIDRaw.Compute();
      int intOutput = OutputRaw;
      digiPot.writeToPot(intOutput);
      // debuging led on arduino
      digitalWrite(13, 1);
    }
    digiPot.killPot();
    five.fiveKill();
    // debuging led on arduino
    digitalWrite(13, 0);
  }
  digiPot.killPot();
  five.fiveKill();
  // debuging led on arduino
  digitalWrite(13, 0);

}

VoltageMeter::VoltageMeter(uint8_t adsAddress) {
  _adsAddress = adsAddress;
  Adafruit_ADS1015 ads(_adsAddress);     //adc
  ads.begin();
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV (default)
}

int16_t VoltageMeter::voltage(int probe) {
  Adafruit_ADS1015 ads(_adsAddress);     //adc
  int16_t voltageRawV = 0;
  voltageRawV = ads.readADC_SingleEnded(probe);
  return voltageRawV;
}

OhmMeter::OhmMeter(int button) {
  pinMode(button, OUTPUT);
  digitalWrite(button,LOW );
  _button = button;
}

int* OhmMeter::ohmCheck() {
  VoltageMeter ohmMeter(0x49);
  int resultsArr[256];
  int thisReading = 0;
  int twoLoops=0;
  for (int thisReading = 0; thisReading < 256; thisReading++) {
    resultsArr[thisReading] = '\0';
  }
  twoLoops = 0;
  int sysVoltage[2];
  bool itsOver = false; // closed V mode not found
  int score[256];
  int16_t ohmRawV = 0;
  int Vline = 0;
  int ohmProbe = 0;
  int voltageProbe = 2;
  digitalWrite(_button,HIGH );
  while (twoLoops < 2) {
    int itty = 0;
    if (itsOver == false){
      if (Vline == 0) {
        ohmRawV = ohmMeter.voltage(ohmProbe);
      }
      else {
        ohmRawV = ohmMeter.voltage(voltageProbe);
      }
    Serial.print("ohmRawV"); Serial.println(ohmRawV);
      while(resultsArr[itty]!= '\0' && itsOver == false) {
        if (resultsArr[itty] == ohmRawV) {
        score[itty] = score[itty] + 1;
          if (score[itty] > 20 ) {
            int mode=0;
            itsOver= true;
            mode = resultsArr[itty];
            sysVoltage[twoLoops] = mode;
            twoLoops++;
            Vline++;
            delay(100);
            Serial.print("------------------------------------------------------------------------winner "); Serial.println(mode);
          }
        break;
        }
        else {
          itty++;
        }
      }
    resultsArr[itty]= ohmRawV;
    itty = 0;
    }
    else {
      for (thisReading = 0; thisReading < 256; thisReading++) {
        resultsArr[thisReading] = '\0';
      }
      for (thisReading = 0; thisReading < 256; thisReading++) {
        score[thisReading] = 0;
      }
      itsOver =false;
    }
  }
  digitalWrite(_button,LOW );
  Serial.print("------------------------------------------------------------------------sys0 "); Serial.println(sysVoltage[0]);
  Serial.print("------------------------------------------------------------------------sys1 "); Serial.println(sysVoltage[1]);
  return sysVoltage;
}

float OhmMeter::ohmTranslate(int* ohmVoltageResults) {
  float attyOhms = 0;
  float attyPercentage = 1.000 - ( (float)ohmVoltageResults[1] / (float)ohmVoltageResults[0] );
  attyOhms = (11.100 / attyPercentage) - 11.100;
  Serial.print("Percentage"); Serial.println(attyPercentage);
  Serial.print("Ohms"); Serial.println(attyOhms);
  return attyOhms;
}

PhysicalInput::PhysicalInput(int pioPin) {
  _pioPin = pioPin;
  pinMode(_pioPin, INPUT);
  digitalWrite(_pioPin, HIGH); //turn pullup resistor on
}

int PhysicalInput::readInput() {
  int pinStatus = digitalRead(_pioPin);
  return pinStatus;
}

DigitalPot::DigitalPot(int csPin) {
  _csPin = csPin;
  pinMode(_csPin, OUTPUT);
  SPI.begin();
  killPot();
}

void DigitalPot::writeToPot(int value) {
  digitalWrite(_csPin, LOW); //select slave
  SPI.transfer(_address);
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH); //de-select slave
}

void DigitalPot::killPot() {
  writeToPot(0);
}

FiveFiveFive::FiveFiveFive(int fivePin) {
  _fivePin = fivePin;
  pinMode(fivePin, OUTPUT);
  fiveKill();
}

void FiveFiveFive::fiveEnable() {
  digitalWrite(_fivePin, 1);
}

void FiveFiveFive::fiveKill() {
  digitalWrite(_fivePin, 0);
}
