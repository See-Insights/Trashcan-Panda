
//Particle Functions
#include "Particle.h"
#include "MyPersistentData.h"
#include "device_pinout.h"
#include "Measure_Trash.h"
#include "SparkFun_VL53L1X.h"
#include "LIS3DH.h"

Measure_Trash *Measure_Trash::_instance;

LIS3DHI2C accel(Wire, 0, INT_PIN);                   // Initialize the accelerometer in i2c mode
SFEVL53L1X distanceSensor;                          // Initialize the TOF sensor - no interrupts
LIS3DHSample sample;                                // Stores latest value from the accelerometer

// [static]
Measure_Trash &Measure_Trash::instance() {
  if (!_instance) {
      _instance = new Measure_Trash();
  }
  return *_instance;
}

Measure_Trash::Measure_Trash() {
}

Measure_Trash::~Measure_Trash() {
}

bool Measure_Trash::setup() {
  bool setupSuccess = true;

  Log.info("Starting the TOF sensor");

  if (distanceSensor.begin() != 0)                                    // Begin returns 0 on a good init
  {
    Log.info("TOF sensor initialization failed - ERROR State");
    setupSuccess = false;
  }
  else Log.info("TOF Sensor initialized");

  // Initialize Accelerometer sensor
	LIS3DHConfig config;
	config.setAccelMode(LIS3DH::RATE_100_HZ);

  if (accel.setup(config)) {
    Log.info("Accelerometer Initialized");
  }
  else {
    Log.info("Accelerometer failed initialization - entering ERROR state");
    setupSuccess = false;
  }

  return setupSuccess;

}

void Measure_Trash::loop() {
    // Put your code to run during the application thread loop here
}

void Measure_Trash::measureHeight() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the device
{
  float lastPercentFull = current.get_percentFull();                   // Going to see if the trashcan was emptied

  // Read the height of the trash in the can
  distanceSensor.sensorOn();                                           // Fire up the sensor

  // focus the detection area
  distanceSensor.stopRanging();
  distanceSensor.clearInterrupt();
  distanceSensor.setROI(8,8,199);                                      // Set the ROI to 8 pixels wide by 8 pixels high centered on the sensor
  delay(1);
  distanceSensor.startRanging();                                       // Write configuration bytes to initiate measurement

  waitFor(distanceSensor.checkForDataReady,10000);

  if (distanceSensor.checkForDataReady()) {
    // Calculate the height of the trash in the can
    current.set_trashHeight(int(distanceSensor.getDistance() * 0.0393701)); //Get the result of the measurement from the sensor
    if (isnan(current.get_trashHeight())) current.set_trashHeight(0);

    Log.info("Data ready and distance is %i inches", current.get_trashHeight());

    // Calculate percent full and log information
    current.set_trashHeight(constrain(current.get_trashHeight(),sysStatus.get_trashFull(),sysStatus.get_trashEmpty()));
    current.set_percentFull(((float)((sysStatus.get_trashEmpty()-sysStatus.get_trashFull()) - (current.get_trashHeight() - sysStatus.get_trashFull()))/(sysStatus.get_trashEmpty()-sysStatus.get_trashFull()))*100);

    // Was trashcan emptied?
    if (current.get_percentFull() < 20.0 && lastPercentFull > 30.0) current.set_trashcanEmptied(true);
    Log.info("Trash height is %i and can is %4.1f%% full and %s emptied", current.get_trashHeight(),current.get_percentFull(), (current.get_trashcanEmptied()) ? "was": "was not");
  }
  else Log.info("TOF Data not ready");

  distanceSensor.clearInterrupt();
  distanceSensor.sensorOff();                                          // Done - turn that puppy off 
  distanceSensor.stopRanging();

  LIS3DHSample sample;

  if (accel.getSample(sample)) {
    int threshold = 10000;
    if (sample.z > threshold) {
      Log.info("Lid rightside up with x:%d, y:%d, z:%d", sample.x, sample.y, sample.z);
      current.set_lidPosition(5);
    }
    else if (sample.z < -1 * threshold) {
      Log.info("Lid upside down x:%d, y:%d, z:%d", sample.x, sample.y, sample.z);
      current.set_lidPosition(6);
    }
    else {
      Log.info("Lid on side x:%d, y:%d, z:%d", sample.x, sample.y, sample.z);
      current.set_lidPosition(1);
    }
  }
  else {
    Serial.println("no sample");
  }
}