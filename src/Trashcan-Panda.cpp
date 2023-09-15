#include "Particle.h"
/*
* Project Morrisville City - Transcan Monitoring
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:January 11, 2022
*/

/*  This is a refinement on the Boron Connected Counter firmware and incorporates new watchdog and RTC
*   capabilities as laid out in AN0023 - https://github.com/particle-iot/app-notes/tree/master/AN023-Watchdog-Timers
*   This software is designed to work with a laser TOF sensor
*   Concept of operations:
*   This is a device that needs to last a year on a single battery - a 3.9V / 16aH Lithium Thionyl Chloride primary cell 
*   It will wake hourly from open time to close time and take measurements of trash height, lid orientation and system data
*   Three times a day (need to validate) at 6am, noon and 6pm - it will send the collected data to Particle via webhook and on to Ubidots
*   Items yet to be worked on:
*     - Using the TOF sensors' ability to "point" and "focus" to get a more accurate reading
*     - Taking adiditional steps to ensure the device does not waste time connecting if there is an outage
*     - Working out how to manage power on the sensor board (shutdown via i2c commands and the power line) - currently power on resets device
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
//v1.20 - Initial deployment to Morrisville, 6 hour wakeup, no more sensor config vairable. 
//v1.30 - New version to accomodate LIS3DH sensor and the new pinouts on the Trashcan Panda sensor board
//v1.31 - Added position change interrupt for detecting the trascan lid being moved, current structure and Webhook updated with position
//v1.40 - Updated with new production sensors (i2c address change) and moved to non-interrupt model - This is the first release
//v2.00 - Going to reduce the number of SPADs in use to 8 to focus measurement area - updated to deviceOS@4.1.0
//v3.00 - Updating for bad sensor behaviour, adding a waitFor on the serial connection, hard coding the full and empty values
//v4.00 - Implementing classes - adopted from Connected Counter Next
//v4.01 - Minor fixes - still reports hourly
//v4.02 - Found an error with AlertHandler for code 31 - Device will now power cycle if it failes to connect for 2+ hours

// Need to update code - time initializion is a mess
// Need to update code - need to add a check for the battery voltage and if it is too low, we need to go into low power mode


// Included Libraries
#include "AB1805_RK.h"
#include "BackgroundPublishRK.h"
#include "PublishQueuePosixRK.h"
#include "LocalTimeRK.h"

// Libraries to move out common functions (lower case = not Singleton)
#include "device_pinout.h"
#include "Alert_Handling.h"
#include "MyPersistentData.h"
#include "Particle_Functions.h"
#include "take_measurements.h"

#define FIRMWARE_RELEASE 4.01						            // Will update this and report with stats
PRODUCT_VERSION(4);									                // For now, we are putting nodes and gateways in the same product group - need to deconflict #

// Prototype Functions
void publishStateTransition(void);                  // Keeps track of state machine changes - for debugging
void userSwitchISR();                               // interrupt service routime for the user switch
void sensorISR(); 
void countSignalTimerISR();							            // Keeps the Blue LED on
void UbidotsHandler(const char *event, const char *data);
bool isParkOpen(bool verbose);						          // Simple function returns whether park is open or not
void dailyCleanup();								                // Reset each morning
void softDelay(uint32_t t);			                    // Extern function for safe delay()

// System Health Variables
int outOfMemory = -1;                               // From reference code provided in AN0023 (see above)

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Prototypes and System Mode calls
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
void outOfMemoryHandler(system_event_t event, int param);
extern LocalTimeConvert conv;								        // For determining if the park should be opened or closed - need local time
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library

// Program Variables
volatile bool userSwitchDectected = false;		
volatile bool sensorDetect = false;					        // Flag for sensor interrupt
bool dataInFlight = false;                          // Flag for whether we are waiting for a response from the webhook

Timer countSignalTimer(1000, countSignalTimerISR, true);      // This is how we will ensure the BlueLED stays on long enough for folks to see it.

// Timing variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // Sets a reporting frequency of 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000UL;        // In lowPowerMode, how long to stay awake every hour
const unsigned long stayAwakeShort = 1000UL;		  	// In lowPowerMode, how long to stay awake when not reporting
const unsigned long webhookWait = 45000UL;          // How long will we wait for a WebHook response
const unsigned long resetWait = 30000UL;            // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0UL;             // Timestamps for our timing variables..
unsigned long stayAwake = stayAwakeLong;            // Stores the time we need to wait before napping


void setup()                                        // Note: Disconnected Setup()
{
  // Make sure you match the same Wire interface in the constructor to LIS3DHI2C to this!
	Wire.setSpeed(CLOCK_SPEED_100KHZ);

 	char responseTopic[125];
	String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
	deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
	Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event
	System.on(out_of_memory, outOfMemoryHandler);     // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

	waitFor(Serial.isConnected, 10000);               // Wait for serial to connect - for debugging
	softDelay(2000);								  // For serial monitoring - can delete

	Particle_Functions::instance().setup();			// Initialize Particle Functions and Variables

  	initializePinModes();                           // Sets the pinModes

	digitalWrite(BLUE_LED, HIGH);					// Turn on the Blue LED for setup

  	initializePowerCfg();                           // Sets the power configuration for solar

	sysStatus.setup();								// Initialize persistent storage
	sysStatus.set_firmwareRelease(FIRMWARE_RELEASE);
	current.setup();
	current.set_alertCode(0);						// Clear any alert codes

  	PublishQueuePosix::instance().setup();          // Start the Publish Queue
	PublishQueuePosix::instance().withFileQueueSize(200);

	// Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  	if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    	sysStatus.set_resetCount(sysStatus.get_resetCount() + 1);
    	if (sysStatus.get_resetCount() > 3) current.set_alertCode(13);                 // Excessive resets 
  	}

    ab1805.withFOUT(D8).setup();                	// Initialize AB1805 RTC   
	if (!ab1805.detectChip()) current.set_alertCode(12);
    ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);	// Enable watchdog

	// Setup local time and set the publishing schedule
	LocalTime::instance().withConfig(LocalTimePosixTimezone("EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"));			// East coast of the US
	conv.withCurrentTime().convert();  	

	System.on(out_of_memory, outOfMemoryHandler);   // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

	if (!Take_Measurements::instance().setup()) current.set_alertCode(12);			// Initialize the sensor and take measurements

  	Take_Measurements::instance().takeMeasurements();   // Populates values so you can read them before the hour

	if (!digitalRead(BUTTON_PIN)) {						// The user will press this button at startup to reset settings
		Log.info("User button at startup - setting defaults");
		state = CONNECTING_STATE;
		sysStatus.initialize();                  	// Make sure the device wakes up and connects - reset to defaults and exit low power mode
	}

	if (!Time.isValid()) {
		Log.info("Time is invalid -  %s so connecting", Time.timeStr().c_str());
		state = CONNECTING_STATE;
	}
	else {
		Log.info("LocalTime initialized, time is %s and RTC %s set", conv.format("%I:%M:%S%p").c_str(), (ab1805.isRTCSet()) ? "is" : "is not");
		if (Time.day(sysStatus.get_lastConnection()) != Time.day()) {
			Log.info("New day, resetting counts");
			dailyCleanup();
		}
	} 

	attachInterrupt(BUTTON_PIN,userSwitchISR,FALLING); // We may need to monitor the user switch to change behaviours / modes
	attachInterrupt(INT_PIN,sensorISR,RISING);      // We need to monitor the sensor for activity


	if (state == INITIALIZATION_STATE) {
		if(sysStatus.get_lowPowerMode()) {
			state = IDLE_STATE;               	// Go to the IDLE state unless changed above
		}
		else {
			state = CONNECTING_STATE;          	// Go to the CONNECTING state unless changed above
		}
	}

	conv.withTime(sysStatus.get_lastConnection()).convert();	// Want to know the last time we connected in local time
  	Log.info("Startup complete with last connect %s in %s", conv.format("%I:%M:%S%p").c_str(), (sysStatus.get_lowPowerMode()) ? "low power mode" : "normal mode");
	conv.withCurrentTime().convert();
  	digitalWrite(BLUE_LED,LOW);                                          	// Signal the end of startup

	isParkOpen(true);

	Alert_Handling::instance().setup();
}


void loop()
{
  switch(state) {
		case IDLE_STATE: {													// Unlike most sketches - nodes spend most time in sleep and only transit IDLE once or twice each period
			if (state != oldState) publishStateTransition();
			if (sysStatus.get_lowPowerMode() && (millis() - stayAwakeTimeStamp) > stayAwake) state = SLEEPING_STATE;         // When in low power mode, we can nap between taps
			if (isParkOpen(false) && Time.hour() != Time.hour(sysStatus.get_lastReport())) state = REPORTING_STATE;                                  // We want to report on the hour but not after bedtime
		} break;

		case SLEEPING_STATE: {
			if (state != oldState) publishStateTransition();              	// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
	    	if (sensorDetect || countSignalTimer.isActive())  break;           // Don't nap until we are done with event - exits back to main loop but stays in napping state
			if (Particle.connected() || !Cellular.isOff()) {
				if (!Particle_Functions::instance().disconnectFromParticle()) {                                 // Disconnect cleanly from Particle and power down the modem
					current.set_alertCode(15);
					break;
				}
			}
			if (!isParkOpen(true)) digitalWrite(ENABLE_PIN,HIGH);
			else digitalWrite(ENABLE_PIN,LOW);
			stayAwake = stayAwakeShort;                                       // Keeps device awake for just a second - when we are not reporting
			int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary) + 1;;	// Figure out how long to sleep 		
			config.mode(SystemSleepMode::ULTRA_LOW_POWER)
				.gpio(BUTTON_PIN,CHANGE)
				.gpio(INT_PIN,RISING)
				.duration(wakeInSeconds * 1000L);
			ab1805.stopWDT();  												   // No watchdogs interrupting our slumber
			SystemSleepResult result = System.sleep(config);              	// Put the device to sleep device continues operations from here
			ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
			if (result.wakeupPin() == BUTTON_PIN) {                         // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
				Log.info("Woke with user button - Resetting hours and going to connect");
				sysStatus.set_lowPowerMode(false);
				sysStatus.set_closeTime(24);
				sysStatus.set_openTime(0);
				stayAwake = stayAwakeLong;
				stayAwakeTimeStamp = millis();
				state = CONNECTING_STATE;
			}
			else if (result.wakeupPin() == INT_PIN) {
				Log.info("Woke with sensor - counting");
				state = IDLE_STATE;
			}
			else {															// In this state the device was awoken for hourly reporting
				softDelay(2000);											// Gives the device a couple seconds to get the battery reading
				Log.info("Time to wake up at %s with %li free memory", Time.format((Time.now()+wakeInSeconds), "%T").c_str(), System.freeMemory());
				if (isParkOpen(true)) stayAwake = stayAwakeLong;                 // Keeps device awake after reboot - helps with recovery
				state = IDLE_STATE;
			}
		} break;

		case REPORTING_STATE: {
			if (state != oldState) publishStateTransition();
			sysStatus.set_lastReport(Time.now());                              // We are only going to report once each hour from the IDLE state.  We may or may not connect to Particle
			Take_Measurements::instance().takeMeasurements();                  // Take Measurements here for reporting
			if (Time.day(sysStatus.get_lastConnection()) != conv.getLocalTimeYMD().getDay()) {
				dailyCleanup();
				Log.info("New Day - Resetting everything");
			}
			Particle_Functions::instance().sendEvent();                        // Publish hourly but not at opening time as there is nothing to publish
			state = CONNECTING_STATE;                                          // Default behaviour would be to connect and send report to Ubidots

			// Let's see if we need to connect 
			if (Particle.connected()) {                                        // We are already connected go to response wait
				stayAwakeTimeStamp = millis();
				state = RESP_WAIT_STATE;
			}
			// If we are in a low battery state - we are not going to connect unless we are over-riding with user switch (active low)
			else if (sysStatus.get_lowBatteryMode() && digitalRead(BUTTON_PIN)) {
				Log.info("Not connecting - low battery mode");
				state = IDLE_STATE;
			}
			// If we are in low power mode, we may bail if battery is too low and we need to reduce reporting frequency
			else if (sysStatus.get_lowPowerMode() && digitalRead(BUTTON_PIN)) {      // Low power mode and user switch not pressed
				// Code ususally goes here around state of charge - but Pandas don'y use LiPO
				Log.info("Connecting");
			}
		} break;

  		case RESP_WAIT_STATE: {
    		static unsigned long webhookTimeStamp = 0;                         // Webhook time stamp
    		if (state != oldState) {
      			webhookTimeStamp = millis();                                     // We are connected and we have published, head to the response wait state
      			dataInFlight = true;                                             // set the data inflight flag
      			publishStateTransition();
    		}

    		if (!dataInFlight)  {                                              // Response received --> back to IDLE state
				stayAwakeTimeStamp = millis();
      			state = IDLE_STATE;
    		}
    		else if (millis() - webhookTimeStamp > webhookWait) {              // If it takes too long - will need to reset
				current.set_alertCode(40);
			}
  		} break;

  		case CONNECTING_STATE:{                                              // Will connect - or not and head back to the Idle state - We are using a 3,5, 7 minute back-off approach as recommended by Particle
			static State retainedOldState;                                   // Keep track for where to go next (depends on whether we were called from Reporting)
			static unsigned long connectionStartTimeStamp;                   // Time in Millis that helps us know how long it took to connect
			char data[64];                                                   // Holder for message strings

			if (state != oldState) {                                         // Non-blocking function - these are first time items
				retainedOldState = oldState;                                     // Keep track for where to go next
				sysStatus.set_lastConnectionDuration(0);                         // Will exit with 0 if we do not connect or are already connected.  If we need to connect, this will record connection time.
				publishStateTransition();
				connectionStartTimeStamp = millis();                             // Have to use millis as the clock may get reset on connect
				Particle.connect();                                              // Tells Particle to connect, now we need to wait
			}

			sysStatus.set_lastConnectionDuration(int((millis() - connectionStartTimeStamp)/1000));

			if (Particle.connected()) {
				sysStatus.set_lastConnection(Time.now());                    // This is the last time we last connected
				stayAwakeTimeStamp = millis();                               // Start the stay awake timer now
				Take_Measurements::instance().getSignalStrength();           // Test signal strength since the cellular modem is on and ready
				snprintf(data, sizeof(data),"Connected in %i secs",sysStatus.get_lastConnectionDuration());  // Make up connection string and publish
				Log.info(data);
				if (sysStatus.get_verboseMode()) Particle.publish("Cellular",data,PRIVATE);
				(retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE; // so, if we are connecting to report - next step is response wait - otherwise IDLE
			}
			else if (sysStatus.get_lastConnectionDuration() > 600) { 		// What happens if we do not connect - non-zero alert code will send us to the Error state
				Log.info("Failed to connect in 10 minutes");
				if (Cellular.ready()) current.set_alertCode(30);
				else current.set_alertCode(31);
				sysStatus.set_lowPowerMode(true);						    // If we are not connected after 10 minutes, we are going to go to low power mode
			}
		} break;

		case ERROR_STATE: {													// Where we go if things are not quite right
			static int alertResponse = 0;
			static unsigned long resetTimer = millis();

			if (state != oldState) {
				publishStateTransition();                					// We will apply the back-offs before sending to ERROR state - so if we are here we will take action
				alertResponse = Alert_Handling::instance().alertResolution();
				Log.info("Alert Response: %i so %s",alertResponse, (alertResponse == 0) ? "No action" : (alertResponse == 1) ? "Connecting" : (alertResponse == 2) ? "Reset" : (alertResponse == 3) ? "Power Down" : "Unknown");
				resetTimer = millis();
			}

			if (alertResponse >= 2 && millis()-resetTimer < 30000L) break;
			else Log.info("Delay is up - executing");

			switch (alertResponse) {
				case 0:
					Log.info("No Action - Going to Idle");
					state = IDLE_STATE;					// Least severity - no additional action required
					break;
				case 1:
					Log.info("Need to report - connecting");
					state = CONNECTING_STATE;			// Issue that needs to be reported - could be in a reset loop
					break;
				case 2:
					Log.info("Resetting");
					delay(1000);						// Give the system a second to get the message out
					System.reset();						// device needs to be reset
					break;
				case 3: 
					Log.info("Powering down");
					delay(1000);						// Give the system a second to get the message out
					ab1805.deepPowerDown();				// Power off the device for 30 seconds
					break;
				default:								// Ensure we do not get trapped in the ERROR State
					System.reset();
					break;
			}
		} break;
  }
  // Take care of housekeeping items here
	ab1805.loop();                                  	// Keeps the RTC synchronized with the Boron's clock

	// Housekeeping for each transit of the main loop
	current.loop();
	sysStatus.loop();

	PublishQueuePosix::instance().loop();               // Check to see if we need to tend to the message queue
	Alert_Handling::instance().loop();	

	if (outOfMemory >= 0) {                         	// In this function we are going to reset the system if there is an out of memory error
	  current.set_alertCode(14);
  	}

	if (current.get_alertCode() > 0) state = ERROR_STATE;

	if (sensorDetect) {									// If the sensor has been triggered, we need to record the count
		sensorDetect = false;

	}		

	if (userSwitchDectected) {							// If the user switch has been pressed, we need to reset the device
		userSwitchDectected = false;
		digitalWrite(ENABLE_PIN, !digitalRead(ENABLE_PIN));						// Toggle the enable pin
		Log.info("User switch pressed and Enable pin is now %s", (digitalRead(ENABLE_PIN)) ? "HIGH" : "LOW");
		delay(1000);	// Give the system a second to get the message out
	}
  // End of housekeeping - end of main loop
}
/**
 * @brief Publishes a state transition to the Log Handler and to the Particle monitoring system.
 *
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
	char stateTransitionString[256];
	if (state == IDLE_STATE) {
		if (!Time.isValid()) snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s with invalid time", stateNames[oldState],stateNames[state]);
		else snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
	}
	else snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
	oldState = state;
	Log.info(stateTransitionString);
}

// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}

void userSwitchISR() {
  	userSwitchDectected = true;                                          	// The the flag for the user switch interrupt
}

void sensorISR() {
	sensorDetect = true;													// Set the flag for the sensor interrupt
}

void countSignalTimerISR() {
  digitalWrite(BLUE_LED,LOW);
}

bool isParkOpen(bool verbose) {
	conv.withCurrentTime().convert();
	if (verbose) {
	  Log.info("Local hour is %i and the park is %s", conv.getLocalTimeHMS().hour, (conv.getLocalTimeHMS().hour < sysStatus.get_openTime() || conv.getLocalTimeHMS().hour > sysStatus.get_closeTime()) ? "closed" : "open");
	}
	if (conv.getLocalTimeHMS().hour < sysStatus.get_openTime() || conv.getLocalTimeHMS().hour > sysStatus.get_closeTime()) return false;
	else return true;
}


void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
	dataInFlight = false;											 // We have received a response - so we can send another
    sysStatus.set_lastHookResponse(Time.now());                          // Record the last successful Webhook Response
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  if (sysStatus.get_verboseMode() && Particle.connected()) {
    Particle.publish("Ubidots Hook", responseString, PRIVATE);
  }
  Log.info(responseString);
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
  sysStatus.set_verboseMode(false);                                       			// Saves bandwidth - keep extra chatter off
  sysStatus.set_lowPowerMode(true);
  current.resetEverything();                                                   		// If so, we need to Zero the counts for the new day
}

/**
 * @brief soft delay let's us process Particle functions and service the sensor interrupts while pausing
 * 
 * @details takes a single unsigned long input in millis
 * 
 */
inline void softDelay(uint32_t t) {
  for (uint32_t ms = millis(); millis() - ms < t; Particle.process());  //  safer than a delay()
}