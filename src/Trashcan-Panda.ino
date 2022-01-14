/*
* Project Morrisville City - Transcan Monitoring
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:January 11, 2022
*/

/*  This is a refinement on the Boron Connected Counter firmware and incorporates new watchdog and RTC
*   capabilities as laid out in AN0023 - https://github.com/particle-iot/app-notes/tree/master/AN023-Watchdog-Timers
*   This software is designed to work with a laser TOF sensor
*/

/* Alert Code Definitions
* 0 = Normal Operations - No Alert
// device alerts
* 10 = Battery temp too high / low to charge
* 11 = PMIC Reset required
* 12 = Initialization error (likely FRAM)
* 13 = Excessive resets
* 14 = Out of memory
// deviceOS or Firmware alerts
* 20 = Firmware update completed
* 21 = Firmware update timed out
* 22 = Firmware update failed
* 23 = Update attempt limit reached - done for the day
// Connectivity alerts
* 30 = Particle connection timed out but Cellular connection completed
* 31 = Failed to connect to Particle or cellular
// Particle cloud alerts
* 40 = Failed to get Webhook response when connected
*/

//v1 - Adapted from the Boron Connected Counter Code at release v44
//v1.20 - Initial deployment to Morrisville

// Particle Product definitions
PRODUCT_ID(PLATFORM_ID);                            // No longer need to specify - but device needs to be added to product ahead of time.
PRODUCT_VERSION(1);
char currentPointRelease[6] ="1.20";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 1;                    // Increment this number each time the memory map is changed

struct currentStatus_structure {                    // currently 10 bytes long
  int trashHeight;                                  // Current trash height in inches
  float percentFull;                                // How full is the trashcan in percent
  int placeholder;                                  // In period daily count
  unsigned long lastMeasureTime;                    // When did we record our height value
  bool trashcanEmptied;                             // Did the trashcan get emptied
  int temperature;                                  // Current Temperature inside the enclosure
  int alerts;                                       // What is the current alert value - see secret decoder ring at top of comments
  uint16_t maxConnectTime = 0;                      // Longest connect time for the hour
  int minBatteryLevel = 100;                        // Lowest Battery level for the day - not sure this is needed
  uint8_t updateAttempts = 0;                       // Number of attempted updates each day
} current;


// Included Libraries
#include <Wire.h>
#include "AB1805_RK.h"
#include "BackgroundPublishRK.h"
#include "PublishQueuePosixRK.h"
#include "MB85RC256V-FRAM-RK.h"
#include "SparkFun_VL53L1X.h"

// Libraries to move out common functions
#include "time_zone_fn.h"
#include "sys_status.h"
#include "particle_fn.h"

struct systemStatus_structure sysStatus;

// This is the maximum amount of time to allow for connecting to cloud. If this time is
// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
unsigned long connectMaxTimeSec = 10 * 60;   // Timeout for trying to connect to Particle cloud in seconds - reduced to 10 mins
// If updating, we need to delay sleep in order to give the download time to come through before sleeping
const std::chrono::milliseconds firmwareUpdateMaxTime = 10min; // Set at least 5 minutes

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
FuelGauge fuelGauge;                                // Needed to address issue with updates in low battery state


// For monitoring / debugging, you can uncomment the next line
// SerialLogHandler logHandler(LOG_LEVEL_ALL);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE, FIRMWARE_UPDATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait", "Firmware Update"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Battery Conect variables
// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

// Pin Constants - Boron Carrier Board v1.x
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor - on the carrier board - inside the enclosure
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int disableModule = D3;                       // Pin to shut down the device - active low
const int intPin =        D2;                       // Hardware interrupt - poliarity set in the library

// SFEVL53L1X distanceSensor(Wire, disableModule, intPin); // Need to put below the pin definition 
SFEVL53L1X distanceSensor;

// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // Sets a reporting frequency of 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 30000;            // How long will we wait for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
char currentOffsetStr[10];                          // What is our offset from UTC
unsigned long lastReportedTime = 0;                 // Need to keep this separate from time so we know when to report
unsigned long connectionStartTime;                  // Timestamp to keep track of how long it takes to connect

// Program Variables
bool dataInFlight = false;                          // Tracks if we have sent data but not yet received a response
bool firmwareUpdateInProgress = false;              // Helps us track if a firmware update is in progress
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char percentFullStr[12];                             // How full is the trashcan
char lowPowerModeStr[16];                           // In low power mode?
char openTimeStr[8]="NA";                           // Park Open Time
char closeTimeStr[8]="NA";                          // Park close Time
char sensorTypeConfigStr[16];
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write - system object
bool currentStatusWriteNeeded = false;              // Current counts object write needed

// System Health Variables
int outOfMemory = -1;                               // From reference code provided in AN0023 (see above)

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Interrupt Variables
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered
volatile bool userSwitchDetect = false;              // Flag for a user switch press while in connected state

Timer verboseCountsTimer(2*3600*1000, userSwitchISR, true);   // This timer will turn off verbose counts after 2 hours

void setup()                                        // Note: Disconnected Setup()
{
  Wire.begin();                                     // Not sure this is needed

  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  digitalWrite(blueLED,HIGH);                       // Turn on the led so we can see how long the Setup() takes

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event
  System.on(firmware_update, firmwareUpdateHandler);// Registers a handler that will track if we are getting an update
  System.on(out_of_memory, outOfMemoryHandler);     // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

  Particle.variable("TrashHeight", current.trashHeight);                // Define my Particle variables
  Particle.variable("TrashEmpty",sysStatus.trashEmpty);
  Particle.variable("TrashFull",sysStatus.trashFull);
  Particle.variable("HowFull",percentFullStr);
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("OpenTime", openTimeStr);
  Particle.variable("CloseTime",closeTimeStr);
  Particle.variable("Alerts",current.alerts);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextMessage);
  Particle.variable("SensorStatus",sensorTypeConfigStr);

  Particle.function("setTrashEmpty", setTrashEmpty);                  // These are the functions exposed to the mobile app and console
  Particle.function("setTrashFull",setTrashFull);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);

  // Particle and System Set up next
  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly

  // Watchdog Timer and Real Time Clock Initialization
  ab1805.withFOUT(D8).setup();                                         // The carrier board has D8 connected to FOUT for wake interrupts
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);                         // Enable watchdog

  // Take a look at the battery state of charge - good to do this before turning on the cellular modem
  fuelGauge.wakeup();                                                  // Expliciely wake the Feul gauge and give it a half-sec
  delay(500);
  fuelGauge.quickStart();                                              // May help us re-establish a baseline for SoC

  // Next we will load FRAM and check or reset variables to their correct values
  fram.begin();                                                        // Initialize the FRAM module
  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);                            // Load the FRAM memory map version into a variable for comparison
  if (tempVersion != FRAMversionNumber) {                              // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                      // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                          // See if this worked
    if (tempVersion != FRAMversionNumber) {
      state = ERROR_STATE;                                             // Device will not work without FRAM will need to reset
      resetTimeStamp = millis();                                       // Likely close to zero but, for form's sake
      current.alerts = 12;                                             // FRAM is messed up so can't store but will be read in ERROR state
    }
    else loadSystemDefaults();                                         // Out of the box, we need the device to be awake and connected
  }
  else {
    fram.get(FRAM::systemStatusAddr,sysStatus);                        // Loads the System Status array from FRAM
    fram.get(FRAM::currentCountsAddr,current);                         // Loead the current values array from FRAM
  }

  // Now that the system object is loaded - let's make sure the values make sense
  checkSystemValues();                                                // Make sure System values are all in valid range

  Log.info("Starting the TOF sensor");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Log.info("TOF sensor initialization failed - ERROR State");
    state = ERROR_STATE;                                             // Device will not work without sensor so we will need to reset
    resetTimeStamp = millis();                                       // Likely close to zero but, for form's sake
    current.alerts = 12;                                             // FRAM is messed up so can't store but will be read in ERROR state
  }

  // Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    if (sysStatus.resetCount > 3) current.alerts = 13;               // Excessive resets
  }

  // Publish Queue Posix is used exclusively for sending webhooks and update alerts in order to conserve RAM and reduce writes / wear
  PublishQueuePosix::instance().setup();                             // Start the Publish Queie

  // Next we set the timezone and check is we are in daylight savings time
  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits
  isDSTusa() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string


  // Next - check to make sure we are not in an endless update loop
  if (current.updateAttempts >= 3 && current.alerts != 23) {         // Send out alert the first time we are over the limit
    char data[64];
    System.disableUpdates();                                         // We will only try to update three times in a day
    current.alerts = 23;                                             // Set an alert that we have maxed out our updates for the day
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }

  // If  the user is holding the user button - we will load defaults
  if (!digitalRead(userSwitch)) loadSystemDefaults();                  // Make sure the device wakes up and connects - reset to defaults and exit low power mode

  // Strings make it easier to read the system values in the console / mobile app
  makeUpStringMessages();                                              // Updated system settings - refresh the string messages

  // Make sure we have the right power settings
  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  takeMeasurements();                                                  // Populates values so you can read them before the hour
  if (sysStatus.lowBatteryMode) setLowPowerMode("1");                  // If battery is low we need to go to low power state

  if ((Time.hour() >= sysStatus.openTime) && (Time.hour() < sysStatus.closeTime)) { // Park is open let's get ready for the day
    stayAwake = stayAwakeLong;                                         // Keeps Boron awake after reboot - helps with recovery
    if (!sysStatus.lowPowerMode) state = CONNECTING_STATE;             // If we are not in low power mode, we should connect
  }

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;               // IDLE unless otherwise from above code

  systemStatusWriteNeeded = true;                                      // Update FRAM with any changes from setup
  Log.info("Startup complete");
  digitalWrite(blueLED,LOW);                                           // Signal the end of startup
}


void loop()
{
  switch(state) {
  case IDLE_STATE:                                                     // Where we spend most time - note, the order of these conditionals is important
    if (state != oldState) publishStateTransition();
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;         // When in low power mode, we can nap between taps
    if (firmwareUpdateInProgress) state= FIRMWARE_UPDATE;                                                     // This means there is a firemware update on deck
    if (Time.hour() != Time.hour(lastReportedTime)) state = REPORTING_STATE;                                  // We want to report on the hour but not after bedtime
    if ((Time.hour() >= sysStatus.closeTime) || (Time.hour() < sysStatus.openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    break;

  case SLEEPING_STATE: {                                               // This state is triggered once the park closes and runs until it opens - Sensor is off and interrupts disconnected
    if (state != oldState) publishStateTransition();
    if (Particle.connected() || !Cellular.isOff()) disconnectFromParticle();  // Disconnect cleanly from Particle
    stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                                // Head back to the idle state after we sleep
    ab1805.stopWDT();                                                  // No watchdogs interrupting our slumber
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary) + 1;  // Adding one second to reduce prospect of round tripping to IDLE
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device continues operations from here
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
    stayAwakeTimeStamp = millis();
    if (result.wakeupPin() == userSwitch) {                            // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
      setLowPowerMode("0");                                            // We are waking the device for a reason
      Log.info("Resetting opening hours");
      sysStatus.openTime = 0;                                          // This is for the edge case where the clock is not set and the device won't connect as it thinks it is off hours
      sysStatus.closeTime = 24;                                        // This only resets if the device beleives it is off-hours
      stayAwakeTimeStamp = millis();
      stayAwake = stayAwakeLong;
      systemStatusWriteNeeded = true;
    }
    else if (Time.hour() < sysStatus.closeTime && Time.hour() >= sysStatus.openTime) { // We might wake up and find it is opening time.  Park is open let's get ready for the day
      stayAwake = stayAwakeLong;                                       // Keeps Boron awake after deep sleep - may not be needed
    }
    } break;

  case NAPPING_STATE: {                                                // This state puts the device in low power mode quickly - napping supports the sensor activity and interrupts
    if (state != oldState) publishStateTransition();
    if (sensorDetect)  break;                                          // Don't nap until we are done with event - exits back to main loop but stays in napping state
    if (Particle.connected() || !Cellular.isOff()) disconnectFromParticle();           // Disconnect cleanly from Particle and power down the modem
    stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                                // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    ab1805.stopWDT();                                                  // If we are sleeping, we will miss petting the watchdog
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
    stayAwakeTimeStamp = millis();
    if (result.wakeupPin() == userSwitch) setLowPowerMode("0");        // The user woke the device and we need to make sure it stays awake
    } break;

  case CONNECTING_STATE:{                                              // Will connect - or not and head back to the Idle state
    static State retainedOldState;                                     // Keep track for where to go next (depends on whether we were called from Reporting)
    static unsigned long connectionStartTimeStamp;                     // Time in Millis that helps us know how long it took to connect

    if (state != oldState) {                                           // Non-blocking function - these are first time items
      retainedOldState = oldState;                                     // Keep track for where to go next
      sysStatus.lastConnectionDuration = 0;                            // Will exit with 0 if we do not connect or are connected or the connection time if we do
      publishStateTransition();

      // Let's make sure we need to connect
      if (Particle.connected()) {
        Log.info("Connecting state but already connected");
        stayAwake = stayAwakeLong;                                     // Keeps device awake after reboot - helps with recovery
        stayAwakeTimeStamp = millis();
        (retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE;
        break;
      }

      // If we are in a low battery state - we are not going to connect unless we are over-riding with user switch (active low)
      if (sysStatus.lowBatteryMode && digitalRead(userSwitch)) {
        Log.info("Connecting state but low battery mode");
        state = IDLE_STATE;
        break;
      }
      // If we are in low power mode, we may bail if battery is too low and we need to reduce reporting frequency
      if (sysStatus.lowPowerMode && digitalRead(userSwitch)) {         // Low power mode and user switch not pressed
        if (sysStatus.stateOfCharge <= 50 && (Time.hour() % 4)) {      // If the battery level is <50%, only connect every fourth hour
          Log.info("Connecting but <50%% charge - four hour schedule");
          state = IDLE_STATE;                                          // Will send us to connecting state - and it will send us back here
          break;
        }                                                              // Leave this state and go connect - will return only if we are successful in connecting
        else if (sysStatus.stateOfCharge <= 65 && (Time.hour() % 2)) { // If the battery level is 50% -  65%, only connect every other hour
          Log.info("Connecting but 50-65%% charge - two hour schedule");
          state = IDLE_STATE;                                          // Will send us to connecting state - and it will send us back here
          break;                                                       // Leave this state and go connect - will return only if we are successful in connecting
        }
      }
      // OK, let's do this thing!
      connectionStartTimeStamp = millis();                             // Have to use millis as the clock will get reset on connect
      Cellular.on();                                                   // Needed until they fix this: https://github.com/particle-iot/device-os/issues/1631
      Particle.connect();                                              // Told the Particle to connect, now we need to wait
    }

    sysStatus.lastConnectionDuration = int((millis() - connectionStartTimeStamp)/1000);

    if (Particle.connected()) {
      if (!sysStatus.clockSet ) {                                      // Set the clock once a day
        sysStatus.clockSet = true;
        Particle.syncTime();                                           // Set the clock each day
        waitFor(Particle.syncTimeDone,30000);                          // Wait for up to 30 seconds for the SyncTime to complete
      }
      sysStatus.lastConnection = Time.now();                           // This is the last time we attempted to connect
      stayAwake = stayAwakeLong;                                       // Keeps device awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
      recordConnectionDetails();                                       // Record outcome of connection attempt
      attachInterrupt(userSwitch, userSwitchISR,FALLING);              // Attach interrupt for the user switch to enable verbose counts
      (retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE;
    }
    else if (sysStatus.lastConnectionDuration > connectMaxTimeSec) {
      recordConnectionDetails();                                       // Record outcome of connection attempt
      Log.info("cloud connection unsuccessful");
      disconnectFromParticle();                                        // Make sure the modem is turned off
      if (sysStatus.solarPowerMode) setLowPowerMode("1");              // If we cannot connect, there is no point to stayng out of low power mode
      if ((Time.now() - sysStatus.lastConnection) > 6 * 3600L) {       // Only sends to ERROR_STATE if it has been over six hours - this ties to reporting and low battery state
        state = ERROR_STATE;
        resetTimeStamp = millis();
        break;
      }
      else state = IDLE_STATE;
    }
    } break;

  case REPORTING_STATE:
    if (state != oldState) publishStateTransition();
    lastReportedTime = Time.now();                                    // We are only going to report once each hour from the IDLE state.  We may or may not connect to Particle
    takeMeasurements();                                               // Take Measurements here for reporting
    if (Time.hour() == sysStatus.openTime) dailyCleanup();            // Once a day, clean house and publish to Google Sheets
    sendEvent();                                                      // Publish hourly but not at opening time as there is nothing to publish
    if (Time.hour() % 6 == 0 || Time.now() - sysStatus.lastConnection > 6 * 3600L)  {  // To save power, we will only connect three times a day at 6am, noon and 6pm
      state = CONNECTING_STATE;                                       // Go to the connecting state
    }
    else state = IDLE_STATE;                                          // Go back to IDLE


    break;

  case RESP_WAIT_STATE: {
    static unsigned long webhookTimeStamp = 0;                        // Webhook time stamp
    if (state != oldState) {
      webhookTimeStamp = millis();                                    // We are connected and we have published, head to the response wait state
      dataInFlight = true;                                            // set the data inflight flag
      publishStateTransition();
    }
    if (!dataInFlight)  {                                             // Response received --> back to IDLE state
      state = IDLE_STATE;
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      current.alerts = 40;                                            // Raise the missed webhook flag
      currentStatusWriteNeeded = true;
      if (Time.now() - sysStatus.lastHookResponse > 3 * 3600L) {      // Failed to get a webhook response for over three hours
        resetTimeStamp = millis();
        state = ERROR_STATE;                                          // Response timed out
      }
      else state = IDLE_STATE;
    }
    } break;

  case ERROR_STATE: {                                                  // New and improved - now looks at the alert codes
    if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
    if (millis() > resetTimeStamp + resetWait) {                       // This simply gives us some time to catch the device if it is in a reset loop                           
      char errorStr[64];                                             // Let's publish to let folks know what is going on
      snprintf(errorStr, sizeof(errorStr),"Resetting device with alert code %i",current.alerts);
      if (Particle.connected()) Particle.publish("ERROR_STATE", errorStr, PRIVATE);
      Log.info(errorStr);
      delay(2000);

      if (Particle.connected() || !Cellular.isOff()) disconnectFromParticle();  // Disconnect cleanly from Particle and power down the modem

      switch (current.alerts) {                                        // All of these events will reset the device
        case 12:                                                       // This is an initialization error - likely FRAM - need to power cycle to clear
          ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 30 ... 31:                                                // Device failed to connect too many times
          sysStatus.lastConnection = Time.now();                       // Make sure we don't do this very often
          fram.put(FRAM::systemStatusAddr,sysStatus);                  // Unless a FRAM error sent us here - store alerts value
          delay(100);                                                  // Time to write to FRAM
          System.reset();
          break;

        case 13:                                                       // Excessive resets of the device - time to power cycle
          sysStatus.resetCount = 0;                                    // Reset so we don't do this too often
          fram.put(FRAM::systemStatusAddr,sysStatus);                  // Won't get back to the main loop
          delay (100);
          ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 14:                                                       // This is an out of memory error
          System.reset();
          break;

        case 40:                                                       // This is for failed webhook responses over three hours
          System.reset();
          break;

        default:                                                        // Make sure that, no matter what - we do not get stuck here
          System.reset();
          break;
      }
    }
    } break;

  case FIRMWARE_UPDATE: {
      static unsigned long stateTime;
      char data[64];

      if (state != oldState) {
        stateTime = millis();                                          // When did we start the firmware update?
        Log.info("In the firmware update state");
        publishStateTransition();
      }
      if (!firmwareUpdateInProgress) {                                 // Done with the update
          Log.info("firmware update completed");
          state = IDLE_STATE;
      }
      else
      if (millis() - stateTime >= firmwareUpdateMaxTime.count()) {     // Ran out of time
          Log.info("firmware update timed out");
          current.alerts = 21;                                          // Record alert for timeout
          snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
          PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
          current.updateAttempts++;                                    // Increment the update attempt counter
          state = IDLE_STATE;
      }
    } break;
  }
  // Take care of housekeeping items here

  ab1805.loop();                                                       // Keeps the RTC synchronized with the Boron's clock

  PublishQueuePosix::instance().loop();                                // Check to see if we need to tend to the message queue

  if (systemStatusWriteNeeded) {                                       // These flags get set when a value is changed
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentStatusWriteNeeded) {
    fram.put(FRAM::currentCountsAddr,current);
    currentStatusWriteNeeded = false;
  }

  if (outOfMemory >= 0) {                                              // In this function we are going to reset the system if there is an out of memory error
    current.alerts = 14;                                               // Out of memory alert
    resetTimeStamp = millis();
    state = ERROR_STATE;
  }
  // End of housekeeping - end of main loop
}

void  recordConnectionDetails()  {                                     // Whether the connection was successful or not, we will collect and publish metrics
  char data[64];

  if (sysStatus.lastConnectionDuration > connectMaxTimeSec+1) sysStatus.lastConnectionDuration = 0;
  else if (sysStatus.lastConnectionDuration > current.maxConnectTime) current.maxConnectTime = sysStatus.lastConnectionDuration; // Keep track of longest each day

  if (Cellular.ready()) getSignalStrength();                           // Test signal strength if the cellular modem is on and ready

  snprintf(data, sizeof(data),"Connected in %i secs",sysStatus.lastConnectionDuration);                   // Make up connection string and publish
  Log.info(data);

  if (Particle.connected()) {
    Log.info("Cloud connection successful");
    if (sysStatus.verboseMode) Particle.publish("Cellular",data,PRIVATE);
  }
  else if (Cellular.ready()) {                                        // We want to take note of this as it implies an issue with the Particle back-end
    Log.info("Connected to cellular but not Particle");
    current.alerts = 30;                                              // Record alert for timeout on Particle but connected to cellular
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
  }
  else {
    Log.info("Failed to connect");
    current.alerts = 31;                                              // Record alert for timeout on cellular
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
  }

  systemStatusWriteNeeded = true;
  currentStatusWriteNeeded = true;
}

/**
 * @brief This functions sends the current data payload to Ubidots using a Webhook
 *
 * @details This idea is that this is called regardless of connected status.  We want to send regardless and connect if we can later
 * The time stamp is the time of the last count or the beginning of the hour if there is a zero hourly count for that period
 *
 *
 */
void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"height\":%i, \"percentfull\":%4.2f, \"trashcanemptied\":%i, \"battery\":%i,\"key1\":\"%s\",\"temp\":%i, \"resets\":%i, \"alerts\":%i,\"connecttime\":%i,\"timestamp\":%lu000}",current.trashHeight, current.percentFull, current.trashcanEmptied, sysStatus.stateOfCharge, batteryContext[sysStatus.batteryState], current.temperature, sysStatus.resetCount, current.alerts, sysStatus.lastConnectionDuration, Time.now());
  PublishQueuePosix::instance().publish("Ubidots-Measurement-Hook-v1", data, PRIVATE | WITH_ACK);
  Log.info("Ubidots Webhook: %s", data);                              // For monitoring via serial
  current.alerts = 0;                                                 // Reset the alert after publish
  current.trashcanEmptied = false;                                    // Reset this flag after reporting
}


void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  if (sysStatus.verboseMode && Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("Ubidots Hook", responseString, PRIVATE);
  }
}

/**
 * @brief The Firmware update handler tracks changes in the firmware update status
 *
 * @details This handler is subscribed to in setup with System.on event and sets the firmwareUpdateinProgress flag that
 * will trigger a state transition to the Firmware update state.  As some events are only see in this handler, failure
 * and success success codes are assigned here and the time out code in the main loop state.
 *
 * @param event  - Firmware update
 * @param param - Specific firmware update state
 */

void firmwareUpdateHandler(system_event_t event, int param) {
  switch(param) {
    char data[64];                                                     // Store the date in this character array - not global

    case firmware_update_begin:
      firmwareUpdateInProgress = true;
      break;
    case firmware_update_complete:
      firmwareUpdateInProgress = false;
      current.alerts = 20;                                             // Record a successful attempt
      snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
      PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
      current.updateAttempts = 0;                                      // Zero the update attempts counter
      break;
    case firmware_update_failed:
      firmwareUpdateInProgress = false;
      current.alerts = 22;                                             // Record a failed attempt
      snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
      PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publlish queue
      current.updateAttempts++;                                        // Increment the update attempts counter
      break;
  }
  currentStatusWriteNeeded = true;
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{ 

  float lastPercentFull = current.percentFull;                         // Going to see if the trashcan was emptied

  // Read the height of the trash in the can
  distanceSensor.sensorOn();                                           // Fire up the sensor
  distanceSensor.startRanging();                                       // Write configuration bytes to initiate measurement
  distanceSensor.clearInterrupt();

  while (!distanceSensor.checkForDataReady()) delay(1);
  current.trashHeight = int(distanceSensor.getDistance() * 0.0393701); //Get the result of the measurement from the sensor
  Log.info("Current reading %i inches",current.trashHeight);
  distanceSensor.clearInterrupt();

  distanceSensor.sensorOff();                                          // Done - turn that puppy off 
  distanceSensor.stopRanging();

  current.trashHeight = constrain(current.trashHeight,sysStatus.trashFull,sysStatus.trashEmpty);
  current.percentFull = ((float)((sysStatus.trashEmpty-sysStatus.trashFull) - (current.trashHeight - sysStatus.trashFull))/(sysStatus.trashEmpty-sysStatus.trashFull))*100;
  if (current.percentFull < 20.0 && lastPercentFull > 30.0) current.trashcanEmptied = true;
  snprintf(percentFullStr,sizeof(percentFullStr),"%4.1f%% Full",current.percentFull);
  Log.info("Trashcan is %s and %s emptied", percentFullStr, (current.trashcanEmptied) ? "was": "was not");

  getTemperature();                                                    // Get Temperature at startup as well

  sysStatus.batteryState = System.batteryState();                      // Call before isItSafeToCharge() as it may overwrite the context

  isItSafeToCharge();                                                  // See if it is safe to charge

  if (sysStatus.lowPowerMode) {                                        // Need to take these steps if we are sleeping
    fuelGauge.quickStart();                                            // May help us re-establish a baseline for SoC
    delay(500);
  }

  sysStatus.stateOfCharge = int(fuelGauge.getSoC());                   // Assign to system value

  if (sysStatus.stateOfCharge < 65 && sysStatus.batteryState == 1) {
    System.setPowerConfiguration(SystemPowerConfiguration());          // Reset the PMIC
    current.alerts = 11;                                               // Keep track of this
  }

  if (sysStatus.stateOfCharge < current.minBatteryLevel) {
    current.minBatteryLevel = sysStatus.stateOfCharge;                 // Keep track of lowest value for the day
    currentStatusWriteNeeded = true;
  }

  if (sysStatus.stateOfCharge < 30) {
    sysStatus.lowBatteryMode = true;                                   // Check to see if we are in low battery territory
    if (!sysStatus.lowPowerMode) setLowPowerMode("1");                 // Should be there already but just in case...
  }
  else sysStatus.lowBatteryMode = false;                               // We have sufficient to continue operations

  systemStatusWriteNeeded = true;
  currentStatusWriteNeeded = true;
}

bool isItSafeToCharge()                                                // Returns a true or false if the battery is in a safe charging range.
{
  PMIC pmic(true);
  if (current.temperature < 36 || current.temperature > 100 )  {       // Reference: https://batteryuniversity.com/learn/article/charging_at_high_and_low_temperatures (32 to 113 but with safety)
    pmic.disableCharging();                                            // It is too cold or too hot to safely charge the battery
    sysStatus.batteryState = 1;                                        // Overwrites the values from the batteryState API to reflect that we are "Not Charging"
    current.alerts = 10;                                                // Set a value of 1 indicating that it is not safe to charge due to high / low temps
    return false;
  }
  else {
    pmic.enableCharging();                                             // It is safe to charge the battery
    if (current.alerts == 10) current.alerts = 0;                      // Reset the alerts flag if we previously had disabled charging
    return true;
  }
}

void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature() {                                                // Get temperature and make sure we are not getting a spurrious value

  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  if (reading < 400) {                                                // This ocrresponds to 0 degrees - less than this and we should take another reading to be sure
    delay(50);
    reading = analogRead(tmp36Pin);
  }
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  currentStatusWriteNeeded=true;
  return current.temperature;
}


// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}

void userSwitchISR() {
  userSwitchDetect = true;                                            // The the flag for the user switch interrupt
}


// Power Management function
int setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());            // To restore the default configuration
  if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(900)                                    // Set maximum current the power source can provide (applies only when powered through VIN)
        .powerSourceMinVoltage(5080)                                   // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(900)                                     // Set battery charge current
        .batteryChargeVoltage(4208)                                    // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST);  // For the cases where the device is powered through VIN
                                                                       // but the USB cable is connected to a USB host, this feature flag
                                                                       // enforces the voltage/current limits specified in the configuration
                                                                       // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf);                      // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA
        .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
        .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf);                     // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
}


void loadSystemDefaults() {                                           // Default settings for the device - connected, not-low power and always on
  if (Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Loading System Defaults", PRIVATE);
  }
  Log.info("Loading system defaults");
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.lowBatteryMode = false;
  if (digitalRead(userSwitch)) setLowPowerMode("1");                  // Low power mode or not depending on user switch
  else setLowPowerMode("0");

  sysStatus.timezone = -5;                                            // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.openTime = 6;
  sysStatus.closeTime = 22;                                           // New standard with v20
  sysStatus.solarPowerMode = true;
  sysStatus.lastConnectionDuration = 0;                               // New measure
  fram.put(FRAM::systemStatusAddr,sysStatus);                         // Write it now since this is a big deal and I don't want values over written
}

 /**
  * @brief This function checks to make sure all values that we pull from FRAM are in bounds
  *
  * @details As new devices are comissioned or the sysStatus structure is changed, we need to make sure that values are
  * in bounds so they do not cause unpredectable execution.
  *
  */
void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range
  if (sysStatus.sensorType > 1) {                                   // Values are 0 for Pressure and 1 for PIR
    sysStatus.sensorType = 0;
    strncpy(sensorTypeConfigStr,"Pressure Sensor",sizeof(sensorTypeConfigStr));
  }
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) sysStatus.openTime = 0;
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;
  if (sysStatus.lastConnectionDuration > connectMaxTimeSec) sysStatus.lastConnectionDuration = 0;

  if (current.maxConnectTime > connectMaxTimeSec) {
    current.maxConnectTime = 0;
    currentStatusWriteNeeded = true;
  }

  // None for lastHookResponse
  systemStatusWriteNeeded = true;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

 /**
  * @brief Simple Function to construct the strings that make the console easier to read
  *
  * @details Looks at all the system setting values and creates the appropriate strings.  Note that this
  * is a little inefficient but it cleans up a fair bit of code.
  *
  */
void makeUpStringMessages() {

  if (sysStatus.openTime == 0 && sysStatus.closeTime == 24) {                                 // Special case for 24 hour operations
    snprintf(openTimeStr, sizeof(openTimeStr), "NA");
    snprintf(closeTimeStr, sizeof(closeTimeStr), "NA");
  }
  else {
    snprintf(openTimeStr, sizeof(openTimeStr), "%i:00", sysStatus.openTime);                  // Open and Close Times
    snprintf(closeTimeStr, sizeof(closeTimeStr), "%i:00", sysStatus.closeTime);
  }

  if (sysStatus.lowPowerMode) strncpy(lowPowerModeStr,"Low Power", sizeof(lowPowerModeStr));  // Low Power Mode Strings
  else strncpy(lowPowerModeStr,"Not Low Power", sizeof(lowPowerModeStr));

  return;
}

bool disconnectFromParticle()                                          // Ensures we disconnect cleanly from Particle
                                                                       // Updated based on this thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181
{
  Log.info("In the disconnect from Particle function");
  Particle.disconnect();
  waitFor(Particle.disconnected, 15000);                               // make sure before turning off the cellular modem
  Cellular.disconnect();                                               // Disconnect from the cellular network
  Cellular.off();                                                      // Turn off the cellular modem
  waitFor(Cellular.isOff, 30000);                                      // As per TAN004: https://support.particle.io/hc/en-us/articles/1260802113569-TAN004-Power-off-Recommendations-for-SARA-R410M-Equipped-Devices
  systemStatusWriteNeeded = true;
  detachInterrupt(userSwitch);                                         // Stop watching the userSwitch as we will no longer be connected
  return true;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the device
{
  if (command == "1")
  {
    if (Particle.connected()) Particle.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
    delay(2000);
    ab1805.deepPowerDown(10);                                         // Power cycles the Boron and carrier board
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

/**
 * @brief Resets all counts to start a new day.
 *
 * @details Once run, it will reset all daily-specific counts and trigger an update in FRAM.
 */
void resetEverything() {                                              // The device is waking up in a new day or is a new install
  char data[64];
  current.lastMeasureTime = Time.now();                                 // Set the time context to the new day
  current.maxConnectTime = 0;                                         // Reset values for this time period
  current.minBatteryLevel = 100;
  currentStatusWriteNeeded = true;
  if (current.alerts == 23 || current.updateAttempts >=3) {           // We had tried to update enough times that we disabled updates for the day - resetting
    System.enableUpdates();
    current.alerts = 0;
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }
  current.updateAttempts = 0;                                         // Reset the update attempts counter for the day
  currentStatusWriteNeeded=true;                                      // Make sure that the values are updated in FRAM

  sysStatus.resetCount = 0;                                           // Reset the reset count as well
  systemStatusWriteNeeded = true;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig();                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig();                                                // Change the power management settings
    if (Particle.connected()) Particle.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

/**
 * @brief Turns on/off verbose mode.
 *
 * @details Extracts the integer command. Turns on verbose mode if the command is "1" and turns
 * off verbose mode if the command is "0".
 *
 * @param command A string with the integer command indicating to turn on or off verbose mode.
 * Only values of "0" or "1" are accepted. Values outside this range will cause the function
 * to return 0 to indicate an invalid entry.
 *
 * @return 1 if successful, 0 if invalid command
 */
int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

/**
 * @brief Returns a string describing the battery state.
 *
 * @return String describing battery state.
 */
String batteryContextMessage() {
  return batteryContext[sysStatus.batteryState];
}

/**
 * @brief Sets the closing time of the facility.
 *
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                             // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                     // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  makeUpStringMessages();                                              // Updated system settings - refresh the string messages
  systemStatusWriteNeeded = true;                                      // Need to store to FRAM back in the main loop
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
    Particle.publish("Time",data, PRIVATE);
  }
  return 1;
}

/**
 * @brief Sets the closing time of the facility.
 *
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  makeUpStringMessages();                                           // Updated system settings - refresh the string messages
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}

/**
 * @brief Sets the the minimum height for the sensor - the trashcan full height (in inches)
 *
 * @details When we install the sensor, we will need to capture a value that represents a full trashcan
 *
 * @param command A string for the new daily count.
 * Inputs outside of "0" - "100" will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setTrashFull(String command)
{
  char * pEND;
  char data[256];
  int tempValue = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempValue < 0) || (tempValue > 100)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.trashFull = tempValue;
  systemStatusWriteNeeded = true;                                // Store the new value in FRAM
  snprintf(data, sizeof(data), "Trashcan Full set to  %i inches",sysStatus.trashFull);
  if (Particle.connected()) Particle.publish("Setup",data, PRIVATE);
  return 1;
}

/**
 * @brief Sets the the maximum height for the sensor - the trashcan empty height (in inches)
 *
 * @details When we install the sensor, we will need to capture a value that represents a full trashcan
 *
 * @param command A string for the new daily count.
 * Inputs outside of "0" - "100" will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setTrashEmpty(String command)
{
  char * pEND;
  char data[256];
  int tempValue = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempValue < 0) || (tempValue > 100)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.trashEmpty = tempValue;
  systemStatusWriteNeeded = true;                                // Store the new value in FRAM
  snprintf(data, sizeof(data), "Trashcan Empty set to  %i inches",sysStatus.trashEmpty);
  if (Particle.connected()) Particle.publish("Setup",data, PRIVATE);
  return 1;
}

/**
 * @brief Toggles the device into low power mode based on the input command.
 *
 * @details If the command is "1", sets the device into low power mode. If the command is "0",
 * sets the device into normal mode. Fails if neither of these are the inputs.
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * A "1" indicates low power mode, a "0" indicates normal mode. Inputs that are neither of these commands
 * will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    sysStatus.lowPowerMode = true;
    makeUpStringMessages();                                           // Updated system settings - refresh the string messages
    Log.info("Set Low Power Mode");
    if (Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode",lowPowerModeStr, PRIVATE);
    }
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    sysStatus.lowPowerMode = false;
    makeUpStringMessages();                                           // Updated system settings - refresh the string messages
    Log.info("Cleared Low Power Mode");
    if (!Particle.connected()) {                                 // In case we are not connected, we will do so now.
      state = CONNECTING_STATE;                                       // Will connect - if connection fails, will need to reset device
    }
    else {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode",lowPowerModeStr, PRIVATE);
    }
  }
  systemStatusWriteNeeded = true;
  return 1;
}


/**
 * @brief Publishes a state transition to the Log Handler and to the Particle monitoring system.
 *
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if (sysStatus.verboseMode) {
    if (Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("State Transition",stateTransitionString, PRIVATE);
    }
  }
  Log.info(stateTransitionString);
}


/**
 * @brief Cleanup function that is run at the beginning of the day.
 *
 * @details May or may not be in connected state.  Syncs time with remote service and sets low power mode.
 * Called from Reporting State ONLY. Cleans house at the beginning of a new day.
 */
void dailyCleanup() {
  if (Particle.connected()) Particle.publish("Daily Cleanup","Running", PRIVATE);   // Make sure this is being run
  Log.info("Running Daily Cleanup");
  sysStatus.verboseMode = false;                                       // Saves bandwidth - keep extra chatter off
  sysStatus.clockSet = false;                                          // Once a day we need to reset the clock
  isDSTusa() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here - once a day

  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 65) {     // If Solar or if the battery is being discharged
    setLowPowerMode("1");
  }

  resetEverything();                                                   // If so, we need to Zero the counts for the new day

  systemStatusWriteNeeded = true;
}
