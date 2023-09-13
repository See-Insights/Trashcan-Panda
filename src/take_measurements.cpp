
//Particle Functions
#include "Particle.h"
#include "device_pinout.h"
#include "MyPersistentData.h"
#include "math.h"
#include "Take_Measurements.h"
#include "Measure_Trash.h"

FuelGauge fuelGauge;                                // Needed to address issue with updates in low battery state

Take_Measurements *Take_Measurements::_instance;

// [static]
Take_Measurements &Take_Measurements::instance() {
    if (!_instance) {
        _instance = new Take_Measurements();
    }
    return *_instance;
}

Take_Measurements::Take_Measurements() {
}

Take_Measurements::~Take_Measurements() {
}

bool Take_Measurements::setup() {
  if (Measure_Trash::instance().setup()) return true;
  else return false;
}

void Take_Measurements::loop() {
    // Put your code to run during the application thread loop here
}

bool Take_Measurements::takeMeasurements() { 

    Measure_Trash::instance().measureHeight();                         // Measure the height of the trash in the trashcan

    current.set_batteryVoltage(fuelGauge.getVCell());                  // Get the battery voltage
    Log.info("Battery voltage is %4.2f",current.get_batteryVoltage());

    if (Particle.connected()) getSignalStrength();

    current.set_internalTempC((analogRead(INTERNAL_TEMP_PIN) * 3.3 / 4096.0 - 0.5) * 100.0);  // 10mV/degC, 0.5V @ 0degC
    Log.info("Internal Temp: %4.2fC",current.get_internalTempC());

    return 1;
}

void Take_Measurements::getSignalStrength() {
  char signalStr[16];
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(signalStr,sizeof(signalStr), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
  Log.info(signalStr);
}

float Take_Measurements::getTemperature(int reading) {                                     // Get temperature and make sure we are not getting a spurrious value

  if ((reading < 0) || (reading > 2048)) {                              // This corresponds to -50 degrees to boiling - outside this range we have an error
    return -255;
  }

  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  return ((voltage - 0.5) * 100.0);                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
}
