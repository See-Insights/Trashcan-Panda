#include "Particle.h"
#include "device_pinout.h"
#include "Take_Measurements.h"
#include "MyPersistentData.h"
#include "Particle_Functions.h"
#include "JsonParserGeneratorRK.h"
#include "PublishQueuePosixRK.h"
#include "LocalTimeRK.h"

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));


// For monitoring / debugging, you can uncomment the next line
// Serial1LogHandler logHandler1(LOG_LEVEL_INFO);
SerialLogHandler logHandler(LOG_LEVEL_INFO);     // Easier to see the program flow
LocalTimeConvert conv;								// For determining if the park should be opened or closed - need local time

// Battery Conect variables
// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

Particle_Functions *Particle_Functions::_instance;

// [static]
Particle_Functions &Particle_Functions::instance() {
    if (!_instance) {
        _instance = new Particle_Functions();
    }
    return *_instance;
}

Particle_Functions::Particle_Functions() {
}

Particle_Functions::~Particle_Functions() {
}

void Particle_Functions::setup() {
    Log.info("Initializing Particle functions and variables");     // Note: Don't have to be connected but these functions need to in first 30 seconds
    Particle.function("Commands", &Particle_Functions::jsonFunctionParser, this);

    // Setup local time and set the publishing schedule
	  LocalTime::instance().withConfig(LocalTimePosixTimezone("EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"));			// East coast of the US
	  conv.withCurrentTime().convert();  	
}


void Particle_Functions::loop() {
    // Put your code to run during the application thread loop here
}

int Particle_Functions::jsonFunctionParser(String command) {
    // const char * const commandString = "{\"cmd\":[{\"var\":\"hourly\",\"fn\":\"reset\"},{\"var\":1,\"fn\":\"lowpowermode\"},{\"var\":\"daily\",\"fn\":\"report\"}]}";
    // String to put into Uber command window {"cmd":[{"node":1,"var":"hourly","fn":"reset"},{"node":0,"var":1,"fn":"lowpowermode"},{"node":2,"var":"daily","fn":"report"}]}

	String variable;
	String function;
  char * pEND;
  char messaging[64]=" ";
  bool success = true;

	JsonParserStatic<1024, 80> jp;	// Global parser that supports up to 256 bytes of data and 20 tokens

  Log.info(command.c_str());

	jp.clear();
	jp.addString(command);
	if (!jp.parse()) {
		Log.info("Parsing failed - check syntax");
    Particle.publish("cmd", "Parsing failed - check syntax",PRIVATE);
		return 0;
	}

	const JsonParserGeneratorRK::jsmntok_t *cmdArrayContainer;			// Token for the outer array
	jp.getValueTokenByKey(jp.getOuterObject(), "cmd", cmdArrayContainer);
	const JsonParserGeneratorRK::jsmntok_t *cmdObjectContainer;			// Token for the objects in the array (I beleive)

	for (int i=0; i<10; i++) {												// Iterate through the array looking for a match
		cmdObjectContainer = jp.getTokenByIndex(cmdArrayContainer, i);
		if(cmdObjectContainer == NULL) {
      if (i == 0) return 0;                                       // No valid entries
			else break;								                    // Ran out of entries 
		} 
		jp.getValueByKey(cmdObjectContainer, "var", variable);
		jp.getValueByKey(cmdObjectContainer, "fn", function);

    // In this section we will parse and execute the commands from the console or JSON - assumes connection to Particle
    // ****************  Note: currently there is no valudiation on the nodeNumbers ***************************
    if (function == "restart") {
      // Format - function - restart, variable - either "soft" or "hard"
      // Test - {"cmd":[{"var":"soft","fn":"restart"}]}
      if (variable == "soft") {
        snprintf(messaging, sizeof(messaging),"Soft reset in 30 seconds");
        current.set_alertCode(2);
      }
      else if (variable == "hard") {
        snprintf(messaging,sizeof(messaging),"Hard reset in 30 seconds");
        current.set_alertCode(3);
      }
      else snprintf(messaging,sizeof(messaging),"Invalid: soft or hard");
    }

    // Report on status
    else if (function == "status") {
      char data[128];
      // Format - function - status, variables - short, long
      // Test - {"cmd":[{"var":"short", "fn":"status"}]}
      Take_Measurements::instance().takeMeasurements();
      snprintf(data, sizeof(data),"Height: %d\" and %4.2f%% full.  Lid is %s and battery is %4.2fV",current.get_trashHeight(), current.get_percentFull(), (current
    .get_lidPosition() == 1) ? "on its side" : (current.get_lidPosition() == 5) ? "right side up" : "upside down", current.get_batteryVoltage());
      Log.info(data);
      Particle.publish("status",data,PRIVATE);
      if (variable == "long") {
        conv.withCurrentTime().convert();  	
        snprintf(data,sizeof(data),"Time: %s, open: %d, close: %d, mode %s, release %4.2f", conv.format("%I:%M:%S%p").c_str(), sysStatus.get_openTime(), sysStatus.get_closeTime(), (sysStatus.get_lowPowerMode()) ? "low power":"not low power", sysStatus.get_firmwareRelease());
        Log.info(data);
        Particle.publish("status",data,PRIVATE);
      }
    }

    // Command to send data
    else if (function == "send") {
      // Format - function - send, variables - NA
      // Test - {"cmd":[{"var":"","fn":"send"}]}
      Take_Measurements::instance().takeMeasurements();
      Particle_Functions::sendEvent();
    }

    // Stay Connected
    else if (function == "stay") {
      // Format - function - rpt, variables - true or false
      // Test - {"cmd":[{"var":"true","fn":"stay"}]}
      if (variable == "true") {
        snprintf(messaging,sizeof(messaging),"Going to keep the device online");
        sysStatus.set_lowPowerMode(false);
      }
      else {
        snprintf(messaging,sizeof(messaging),"Going back to normal connectivity");
        sysStatus.set_lowPowerMode(true);
      }
    }

    // Setting Open and close hours
    else if (function == "open") {
      // Format - function - open, node - 0, variables - 0-12 open hour
      // Test - {"cmd":[{"var":"6","fn":"open"}]}
      int tempValue = strtol(variable,&pEND,10);                       // Looks for the first integer and interprets it
      if ((tempValue >= 0) && (tempValue <= 12)) {
        snprintf(messaging,sizeof(messaging),"Setting opening hour to %d:00", tempValue);
        sysStatus.set_openTime(tempValue);
      }
      else {
        snprintf(messaging,sizeof(messaging),"Open hour - must be 0-12");
        success = false;                                                       // Make sure it falls in a valid range or send a "fail" result
      }
    }

    else if (function == "close") {
      // Format - function - close, node - 0, variables - 13-24 open hour
      // Test - {"cmd":[{"var":"21","fn":"close"}]}
      int tempValue = strtol(variable,&pEND,10);                       // Looks for the first integer and interprets it
      if ((tempValue >= 13 ) && (tempValue <= 24)) {
        snprintf(messaging,sizeof(messaging),"Setting closing hour to %d:00", tempValue);
        sysStatus.set_closeTime(tempValue);
      }
      else {
        snprintf(messaging,sizeof(messaging),"Close hour - must be 13-24");
        success = false;                                                       // Make sure it falls in a valid range or send a "fail" result
      }
    }
    
    // Note - currently the full and empty values are set in the code - not exposed to the console
    /*
    else if (function == "setFull") {
      // Format - function - setFull, variables - 0-100
      // Test - {"cmd":[{"var":"100","fn":"setFull"}]}
      int tempValue = strtol(variable,&pEND,10);                       // Looks for the first integer and interprets it
      if ((tempValue < 0) || (tempValue > 100)) return 0;   // Make sure it falls in a valid range or send a "fail" result
      snprintf(messaging, sizeof(messaging), "Trashcan Full set to  %i inches",tempValue);
      sysStatus.set_trashFull(tempValue);
    }

    else if (function == "setEmpty")  {
      // Format - function - setEmpty, variables - 0-100
      // Test - {"cmd":[{"var":"100","fn":"setEmpty"}]}
      int tempValue = strtol(variable,&pEND,10);                       // Looks for the first integer and interprets it
      if ((tempValue < 0) || (tempValue > 100)) return 0;   // Make sure it falls in a valid range or send a "fail" result
      snprintf(messaging, sizeof(messaging), "Trashcan Empty set to  %i inches",tempValue);
      sysStatus.set_trashFull(tempValue);

    }
    */

    // What if none of these functions are recognized
    else {
      snprintf(messaging,sizeof(messaging),"%s is not a valid command", function.c_str());
      success = false;
    }

    if (!(strncmp(messaging," ",1) == 0)) {
      Log.info(messaging);
      if (Particle.connected()) Particle.publish("cmd",messaging,PRIVATE);
    }

	}
	return success;
}

/**
 * @brief This functions sends the current data payload to Ubidots using a Webhook
 *
 * @details This idea is that this is called regardless of connected status.  We want to send regardless and connect if we can later
 * The time stamp is the time of the last count or the beginning of the hour if there is a zero hourly count for that period
 *
 *
 */
void Particle_Functions::sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  unsigned long timeStampValue;                                       // Going to start sending timestamps - and will modify for midnight to fix reporting issue
  timeStampValue = Time.now()-(Time.minute()*60L+Time.second()+1L);   // Set the timestamp as the last second of the previous hour

  snprintf(data, sizeof(data), "{\"height\":%i, \"percentfull\":%4.2f, \"trashcanemptied\":%d, \"lidposition\":%i, \"battery\":%4.2f, \"temp\":%4.2f, \"resets\":%i, \"alerts\":%i,\"connecttime\":%i,\"timestamp\":%lu000}",current.get_trashHeight(), current.get_percentFull(), current.get_trashcanEmptied(), current.get_lidPosition(), current.get_batteryVoltage(), current.get_internalTempC(), sysStatus.get_resetCount(), current.get_alertCode(), sysStatus.get_lastConnectionDuration(), timeStampValue);
  PublishQueuePosix::instance().publish("Ubidots-Measurement-Hook-v1", data, PRIVATE | WITH_ACK);
  Log.info("Ubidots Webhook: %s", data);                              // For monitoring via serial
  current.set_alertCode(0);                                           // Reset the alert after publish
}


bool Particle_Functions::disconnectFromParticle() {                    // Ensures we disconnect cleanly from Particle
                                                                       // Updated based on this thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181                                                                      		// Updated based on this thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181
  time_t startTime = Time.now();
  Log.info("In the disconnect from Particle function");
  Particle.disconnect();                                               		// Disconnect from Particle
  waitForNot(Particle.connected, 15000);                               		// Up to a 15 second delay() 
  Particle.process();
  if (Particle.connected()) {                      							// As this disconnect from Particle thing can be a·syn·chro·nous, we need to take an extra step to wait, 
    Log.info("Failed to disconnect from Particle");
    return(false);
  }
  else Log.info("Disconnected from Particle in %i seconds", (int)(Time.now() - startTime));
  // Then we need to disconnect from Cellular and power down the cellular modem
  startTime = Time.now();
  Cellular.disconnect();                                               // Disconnect from the cellular network
  Cellular.off();                                                      // Turn off the cellular modem
  waitFor(Cellular.isOff, 30000);                                      // As per TAN004: https://support.particle.io/hc/en-us/articles/1260802113569-TAN004-Power-off-Recommendations-for-SARA-R410M-Equipped-Devices
  Particle.process();
  if (Cellular.isOn()) {                                               // At this point, if cellular is not off, we have a problem
    Log.info("Failed to turn off the Cellular modem");
    return(false);                                                     // Let the calling function know that we were not able to turn off the cellular modem
  }
  else {
    Log.info("Turned off the cellular modem in %i seconds", (int)(Time.now() - startTime));
    return true;
  }
}

bool Particle_Functions::meterParticlePublish() {
  static unsigned long lastPublish = 0;  

  if (millis() - lastPublish >= 1000) return 1;

  return 0;
}