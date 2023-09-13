/**
 * @file   device_pinout.h
 * @author Chip McClelland
 * @date   7-5-2022
 * @brief  File containing the pinout documentation and initializations
 * */

#ifndef DEVICE_PINOUT_H
#define DEVICE_PINOUT_H

#include "Particle.h"

// Pin definitions (changed from example code)
extern const pin_t BUTTON_PIN;
extern const pin_t BLUE_LED;
extern const pin_t WAKEUP_PIN;   
extern const pin_t INTERNAL_TEMP_PIN;

// Specific to the sensor
extern const pin_t INT_PIN;
extern const pin_t ENABLE_PIN;

bool initializePinModes();
bool initializePowerCfg();

#endif