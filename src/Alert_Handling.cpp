
//Particle Functions
#include "Particle.h"
#include "PublishQueuePosixRK.h"
#include "MyPersistentData.h"
#include "Alert_Handling.h"

Alert_Handling *Alert_Handling::_instance;

// [static]
Alert_Handling &Alert_Handling::instance() {
  if (!_instance) {
      _instance = new Alert_Handling();
  } 
  return *_instance;
}

Alert_Handling::Alert_Handling() {
}

Alert_Handling::~Alert_Handling() {
}

void Alert_Handling::setup() {
}

void Alert_Handling::loop() {
    // Put your code to run during the application thread loop here
}

int Alert_Handling::alertResolution() { 
  char data[64];                                                   // Let's publish to let folks know what is going on
  int resolutionCode = 0;                                          // Default to no resolution
  
  if (current.get_alertCode() > 10) {
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.get_alertCode(), Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
    Log.info(data);
  }

  switch (current.get_alertCode()) {                              // The resolution of the alert will depend on the value and the history
    case 0:
      resolutionCode = 0;                                         // Default - no action will be taken
      break;

    case 2:                                                       // This is a manual request for a soft reset
      resolutionCode = 2;                                         // This will generate a soft reset
      break;  

    case 3:                                                       // This is a manal request for a poower cycle reset
      resolutionCode = 3;                                         // This will generate a hard reset
      break;

    case 10:                                                      // Too hot / too cold to charge
      Log.info("Too hot / cold to charge");
      resolutionCode = 0;                                         // Default - no action will be taken
      break;

    case 12:                                                       // This is an initialization error - likely FRAM - need to power cycle to clear
      if (Time.now() - sysStatus.get_lastConnection() > 3600L) {
        Log.info("Initialization error - reporting");
        sysStatus.set_lastConnection(Time.now());                  // Keep us from looping here
        resolutionCode = 1;                                        // This will generate a report
      }
      Log.info("Initialization error  - power cycle");
      resolutionCode = 3;                                         // This will generate a hard reset
      break;

    case 13:                                                       // Excessive resets of the device - time to power cycle
      sysStatus.set_resetCount(0);                                 // Reset so we don't do this too often
      if (Time.now() - sysStatus.get_lastConnection() > 3600L) {
        sysStatus.set_lastConnection(Time.now());                  // Keep us from looping here
        sysStatus.set_resetCount(0);                                 // Reset so we don't do this too often        return 1;      
        resolutionCode = 1;                                        // This will generate a report
      }
      resolutionCode = 3;                                         // This will generate a hard reset
      break;

    case 14:                                                       // This is an out of memory error
      resolutionCode = 2;                                         // This will generate a soft reset
      break;

    case 15:                                                       // This is a modem or cellular power down error - need to power cycle to clear
      resolutionCode = 3;                                         // This will generate a hard reset
      break;

    case 30:                                                       // We connected to cellular but not to Particle
      resolutionCode = 2;                                         // This will generate a soft reset
      break;

    case 31:                                                       // Device failed to connect - bailing for this hour or resetting depending
      if (Time.now() - sysStatus.get_lastConnection() > 2 * 3600L) {     // If we fail to connect for a couple hours in a row, let's power cycle the device
        sysStatus.set_lastConnection(Time.now());                  // Make sure we don't do this very often
        resolutionCode = 3;                                         // This will generate a hard reset
      }
      resolutionCode = 0;                                         // Default - no action will be taken
      Log.info("Failed to connect - try again next hour");
      break;

    case 32:                                                       // We are still trying to connect but need to reset first as it is not happening quickly enough
      resolutionCode = 2;                                         // This will generate a soft reset
      break;

    case 40:                                                       // This is for failed webhook responses over three hours
      if (Time.now() - sysStatus.get_lastHookResponse() > 3 * 3600L) {      // Failed to get a webhook response for over three hours
        resolutionCode = 2;                                         // This will generate a soft reset
      }
      resolutionCode = 0;                                         // Default - no action will be taken
      Log.info("Failed to get Webhook response - try again next hour");
      break;

    default:                                                       // Make sure that, no matter what - we do not get stuck here
      resolutionCode = 0;                                         // Default - no action will be taken
      break;
  }
  current.set_alertCode(0);                                         // Need to reset - Alert resolved
  return resolutionCode;
} 


