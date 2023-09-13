/*
 * @file Measure_Trash.h
 * @author Chip McClelland (chip@seeinisghts.com)
 * @brief This function will manage the record counts for the device
 * 
 * @version 0.1
 * @date 2022-09-11
 * 
 */

#ifndef __MEASURE_TRASH_H
#define __MEASURE_TRASH_H

#include "Particle.h"

/**
 * This class is a singleton; you do not create one as a global, on the stack, or with new.
 * 
 * From global application setup you must call:
 * Measure_Trash::instance().setup();
 * 
 * From global application loop you must call:
 * Measure_Trash::instance().loop();
 */
class Measure_Trash {
public:
    /**
     * @brief Gets the singleton instance of this class, allocating it if necessary
     * 
     * Use Measure_Trash::instance() to instantiate the singleton.
     */
    static Measure_Trash &instance();

    /**
     * @brief Perform setup operations; call this from global application setup()
     * 
     * You typically use Measure_Trash::instance().setup();
     */
    bool setup();

    /**
     * @brief Perform application loop operations; call this from global application loop()
     * 
     * You typically use Measure_Trash::instance().loop();
     */
    void loop();

    /**
     * @brief This function is called once a hardware interrupt is triggered by the device's sensor
     * 
     * @details The sensor may change based on the settings in sysSettings but the overall concept of operations
     * is the same regardless.  The sensor will trigger an interrupt, which will set a flag. In the main loop
     * that flag will call this function which will determine if this event should "count" as a visitor.
     * 
     */
    void measureHeight();                               // Determine height of trash in the trashcan

protected:
    /**
     * @brief The constructor is protected because the class is a singleton
     * 
     * Use Measure_Trash::instance() to instantiate the singleton.
     */
    Measure_Trash();

    /**
     * @brief The destructor is protected because the class is a singleton and cannot be deleted
     */
    virtual ~Measure_Trash();

    /**
     * This class is a singleton and cannot be copied
     */
    Measure_Trash(const Measure_Trash&) = delete;

    /**
     * This class is a singleton and cannot be copied
     */
    Measure_Trash& operator=(const Measure_Trash&) = delete;

    /**
     * @brief Singleton instance of this class
     * 
     * The object pointer to this class is stored here. It's NULL at system boot.
     */
    static Measure_Trash *_instance;

};
#endif  /* __Measure_Trash_H */
