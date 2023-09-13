/*
 * @file take_measurements.h
 * @author Chip McClelland (chip@seeinisghts.com)
 * @brief This function will take measurements at intervals defined int he sleep_helper_config file.  
 * The libraries and functions needed will depend the spectifics of the device and sensors
 * 
 * @version 0.2
 * @date 2022-12-23
 * 
 */

#ifndef __TAKE_MEASUREMENTS_H
#define __TAKE_MEASUREMENTS_H

#include "Particle.h"

/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * Take_Measurements::instance().setup();
 * 
 * From global application loop you must call:
 * Take_Measurements::instance().loop();
 */
class Take_Measurements {
public:
    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use Take_Measurements::instance() to instantiate the singleton.
     */
    static Take_Measurements &instance();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use Take_Measurements::instance().setup();
     */
    bool setup();

    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use Take_Measurements::instance().loop();
     */
    void loop();

    /**
     * @brief This code collects basic data from the default sensors - TMP-36 (inside temp), battery charge level and signal strength
     * 
     * @details Uses an analog input and the appropriate scaling
     * 
     * @returns Returns true if succesful and puts the data into the current object
     * 
     */
    bool takeMeasurements();                               // Function that calls the needed functions in turn

    /**
     * @brief tmp36TemperatureC
     * 
     * @details generic code that convers the analog value of the TMP-36 sensors to degrees c - Assume 3.3V Power
     * 
     * @returns Returns the temperature in C as a float value
     * 
     */
    float tmp36TemperatureC (int adcValue);                // Temperature from the tmp36 - inside the enclosure

    /**
     * @brief Get the Signal Strength values and make up a string for use in the console
     * 
     * @details Provides data on the signal strength and quality
     * 
     */
    void getSignalStrength();

    /**
     * @brief Get the Temperature value from the TMP-36 sensor
     * 
     * @details This function will get the temperature from the TMP-36 sensor and make sure that it is a valid value
     * 
    */
    float getTemperature(int reading);                               // Get temperature and make sure we are not getting a spurrious value


protected:
    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use Take_Measurements::instance() to instantiate the singleton.
     */
    Take_Measurements();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~Take_Measurements();

    /**
     * This class is a singleton and cannot be copied
     */
    Take_Measurements(const Take_Measurements&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    Take_Measurements& operator=(const Take_Measurements&) = delete;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static Take_Measurements *_instance;

};
#endif  /* __TAKE_MEASUREMENTS_H */
