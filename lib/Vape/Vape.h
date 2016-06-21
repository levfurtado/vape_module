#ifndef Vape_h
#define Vape_h

#include "Arduino.h"

class Translator {
  public:
    Translator(int button);
    double takeInput(int pidSetting, float rawSetting);
    double setVoltage(float voltage);
    double setWattage(float wattage);
  private:
    int _button;
};

class VoltageMeter {
  public:
    VoltageMeter(uint8_t adsAddress);
    int16_t voltage(int probe);
  private:
    uint8_t _adsAddress;
};

class OhmMeter {
  public:
    OhmMeter(int button);
    int* ohmCheck();
    float ohmTranslate(int* ohmVoltageResults);
  private:
    int _button;
};

class PhysicalInput {
  public:
    PhysicalInput(int pioPin);
    int readInput();
  private:
    int _pioPin;
};

class DigitalPot {
  public:
    DigitalPot(int csPin);
    void writeToPot(int value );
    void killPot();
  private:
    int _address = 0;
    int _csPin;
};

class FiveFiveFive {
  public:
    FiveFiveFive(int fivePin);
    void fiveEnable();
    void fiveKill();
  private:
    int _fivePin;
};

class Executor {
  public:
      Executor(int pioPin, int csPin, int fivePin, uint8_t adsAddress, int probe, double KpRaw, double KiRaw, double KdRaw);
      void waitForInput(double pidSetpoint);
    private:
      int _pioPin;
      int _csPin;
      int _fivePin;
      uint8_t _adsAddress;
      int _probe;
      double _KpRaw;
      double _KiRaw;
      double _KdRaw;
};

#endif
