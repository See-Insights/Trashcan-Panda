/*
 * @file Alert_Handling.h
 * @author Chip McClelland (chip@seeinisghts.com)
 * @brief This function will take alert codes and generate the appropriate response
 * 
 * @version 0.3
 * @date 2022-08-27
 * 
 */

/* Alert Code Definitions
* 0 = Normal Operations - No Alert
// device alerts
* 10 = Battery temp too high / low to charge
* 11 = PMIC Reset required
* 12 = Initialization error (likely FRAM)
* 13 = Excessive resets
* 14 = Out of memory
* 15 = Particle disconnect or Modem Power Down Failure
// deviceOS or Firmware alerts
* 20 = Firmware update completed - deleted
* 21 = Firmware update timed out - deleted
* 22 = Firmware update failed - deleted
* 23 = Update attempt limit reached - deleted
// Connectivity alerts
* 30 = Particle connection timed out but Cellular connection completed
* 31 = Failed to connect to Particle or cellular - skipping to next hour
* 32 = Failed to connect quickly - resetting to keep trying
// Particle cloud alerts
* 40 = Failed to get Webhook response when connected
*/

#ifndef __ALERT_HANDLING_H
#define __ALERT_HANDLING_H

#include "Particle.h"


/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * Alert_Handling::instance().setup();
 * 
 * From global application loop you must call:
 * Alert_Handling::instance().loop();
 */
class Alert_Handling {
public:
    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use Alert_Handling::instance() to instantiate the singleton.
     */
    static Alert_Handling &instance();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use Alert_Handling::instance().setup();
     */
    void setup();

    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use Alert_Handling::instance().loop();
     */
    void loop();

    /**
     * @brief Looks at the current alert code and generates the right response
     * 
     * @details Alert handling may require differnt results
     * 
     */
    int alertResolution();                               // See if it is safe to charge based on the temperature

protected:
    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use Alert_Handling::instance() to instantiate the singleton.
     */
    Alert_Handling();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~Alert_Handling();

    /**
     * This class is a singleton and cannot be copied
     */
    Alert_Handling(const Alert_Handling&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    Alert_Handling& operator=(const Alert_Handling&) = delete;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static Alert_Handling *_instance;

};
#endif  /* __Alert_Handling_H */
