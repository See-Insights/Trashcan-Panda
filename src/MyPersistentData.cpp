#include "Particle.h"
#include "MB85RC256V-FRAM-RK.h"
#include "StorageHelperRK.h"
#include "MyPersistentData.h"


MB85RC64 fram(Wire, 0);

// *******************  SysStatus Storage Object **********************
//
// ********************************************************************

sysStatusData *sysStatusData::_instance;

// [static]
sysStatusData &sysStatusData::instance() {
    if (!_instance) {
        _instance = new sysStatusData();
    }
    return *_instance;
}

sysStatusData::sysStatusData() : StorageHelperRK::PersistentDataFRAM(::fram, 0, &sysData.sysHeader, sizeof(SysData), SYS_DATA_MAGIC, SYS_DATA_VERSION) {

};

sysStatusData::~sysStatusData() {
}

void sysStatusData::setup() {
    fram.begin();
    sysStatus
    //  .withLogData(true)
        .withSaveDelayMs(100)
        .load();

    // Log.info("sizeof(SysData): %u", sizeof(SysData));
}

void sysStatusData::loop() {
    sysStatus.flush(false);
}

bool sysStatusData::validate(size_t dataSize) {
    bool valid = PersistentDataFRAM::validate(dataSize);
    if (valid) {

        if (sysStatus.get_openTime() < 0 || sysStatus.get_openTime() > 12) {
            Log.info("data not valid open time =%d", sysStatus.get_openTime());
            valid = false;
        }
        else if (sysStatus.get_lastConnection() < 0 || sysStatus.get_lastConnectionDuration() > 900) {
            Log.info("data not valid last connection duration =%d", sysStatus.get_lastConnectionDuration());
            valid = false;
        }
        else if (sysStatus.get_trashEmpty() != 38) {
            Log.info("Data not valid trash empty = %d", sysStatus.get_trashEmpty());
            valid = false;
        }
        else if (sysStatus.get_trashFull() != 9) {
            Log.info("Data not valid trash full = %d", sysStatus.get_trashFull());
            valid = false;
        }
    }
    Log.info("sysStatus data is %s",(valid) ? "valid": "not valid");
    return valid;
}

void sysStatusData::initialize() {
    PersistentDataFRAM::initialize();

    const char message[26] = "Loading System Defaults";
    Log.info(message);
    if (Particle.connected()) Particle.publish("Mode",message, PRIVATE);
    sysStatus.set_trashFull(9);
    sysStatus.set_trashEmpty(38);
    sysStatus.set_verboseMode(true);
    sysStatus.set_lowBatteryMode(false);
    sysStatus.set_lowPowerMode(false);          // This should be changed to true once we have tested
    sysStatus.set_openTime(0);
    sysStatus.set_closeTime(24);                                           // New standard with v20
    sysStatus.set_lastConnectionDuration(0);                               // New measure

    // If you manually update fields here, be sure to update the hash
    updateHash();
}

uint8_t sysStatusData::get_structuresVersion() const {
    return getValue<uint8_t>(offsetof(SysData, structuresVersion));
}

void sysStatusData::set_structuresVersion(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, structuresVersion), value);
}

bool sysStatusData::get_verboseMode() const {
    return getValue<bool>(offsetof(SysData,verboseMode));
}

void sysStatusData::set_verboseMode(bool value) {
    setValue<bool>(offsetof(SysData, verboseMode), value);
}

bool sysStatusData::get_lowPowerMode() const  {
    return getValue<bool>(offsetof(SysData,lowPowerMode ));
}
void sysStatusData::set_lowPowerMode(bool value) {
    setValue<bool>(offsetof(SysData, lowPowerMode), value);
}

bool sysStatusData::get_lowBatteryMode() const  {
    return getValue<bool>(offsetof(SysData, lowBatteryMode));
}
void sysStatusData::set_lowBatteryMode(bool value) {
    setValue<bool>(offsetof(SysData, lowBatteryMode), value);
}

uint8_t sysStatusData::get_resetCount() const  {
    return getValue<uint8_t>(offsetof(SysData,resetCount));
}
void sysStatusData::set_resetCount(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, resetCount), value);
}

uint8_t sysStatusData::get_openTime() const  {
    return getValue<uint8_t>(offsetof(SysData,openTime));
}
void sysStatusData::set_openTime(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, openTime), value);
}

uint8_t sysStatusData::get_closeTime() const  {
    return getValue<uint8_t>(offsetof(SysData,closeTime));
}
void sysStatusData::set_closeTime(uint8_t value) {
    setValue<uint8_t>(offsetof(SysData, closeTime), value);
}

time_t sysStatusData::get_lastReport() const  {
    return getValue<time_t>(offsetof(SysData,lastReport));
}
void sysStatusData::set_lastReport(time_t value) {
    setValue<time_t>(offsetof(SysData, lastReport), value);
}

time_t sysStatusData::get_lastConnection() const  {
    return getValue<time_t>(offsetof(SysData,lastConnection));
}
void sysStatusData::set_lastConnection(time_t value) {
    setValue<time_t>(offsetof(SysData, lastConnection), value);
}

uint16_t sysStatusData::get_lastConnectionDuration() const  {
    return getValue<uint16_t>(offsetof(SysData,lastConnectionDuration));
}
void sysStatusData::set_lastConnectionDuration(uint16_t value) {
    setValue<uint16_t>(offsetof(SysData, lastConnectionDuration), value);
}

time_t sysStatusData::get_lastHookResponse() const  {
    return getValue<time_t>(offsetof(SysData,lastHookResponse));
}
void sysStatusData::set_lastHookResponse(time_t value) {
    setValue<time_t>(offsetof(SysData, lastHookResponse), value);
}

float sysStatusData::get_firmwareRelease() const {
    return getValue<float>(offsetof(SysData,firmwareRelease));
}

void sysStatusData::set_firmwareRelease(float value) {
    setValue<float>(offsetof(SysData, firmwareRelease),value);
}

int sysStatusData::get_trashFull() const {
    return getValue<int>(offsetof(SysData, trashFull));
}

void sysStatusData::set_trashFull(int value) {
    setValue<int>(offsetof(SysData,trashFull), value);
}

int sysStatusData::get_trashEmpty() const {
    return getValue<int>(offsetof(SysData, trashEmpty));
}   

void sysStatusData::set_trashEmpty(int value) {
    setValue<int>(offsetof(SysData, trashEmpty), value);
}   

// *****************  Current Status Storage Object *******************
// 
// ********************************************************************

currentStatusData *currentStatusData::_instance;

// [static]
currentStatusData &currentStatusData::instance() {
    if (!_instance) {
        _instance = new currentStatusData();
    }
    return *_instance;
}

currentStatusData::currentStatusData() : StorageHelperRK::PersistentDataFRAM(::fram, 100, &currentData.currentHeader, sizeof(CurrentData), CURRENT_DATA_MAGIC, CURRENT_DATA_VERSION) {
};

currentStatusData::~currentStatusData() {
}

void currentStatusData::setup() {
    fram.begin();
    current
    //    .withLogData(true)
        .withSaveDelayMs(250)
        .load();
}

void currentStatusData::loop() {
    current.flush(false);
}

void currentStatusData::resetEverything() {                             // The device is waking up in a new day or is a new install
  sysStatus.set_resetCount(0);                                          // Reset the reset count as well
  current.set_alertCode(0);
}


bool currentStatusData::validate(size_t dataSize) {
    PersistentDataFRAM::validate(dataSize);

    if (current.get_trashHeight() < sysStatus.get_trashEmpty() || current.get_trashHeight() > sysStatus.get_trashFull()) return false;
    else if (current.get_percentFull() < 0 || current.get_percentFull() > 100) return false;
    else if (current.get_lastMeasureTime() < 0) return false;
    else if (current.get_internalTempC() < -40 || current.get_internalTempC() > 85) return false;
    else if (current.get_lidPosition() < 0 || current.get_lidPosition() > 100) return false;
    else if (current.get_alertCode() < 0 || current.get_alertCode() >256) return false;
    else return true;

};                      // This needs to be fleshed out

void currentStatusData::initialize() {
    PersistentDataFRAM::initialize();

    Log.info("Current Data Initialized");

    currentStatusData::resetEverything();

    // If you manually update fields here, be sure to update the hash
    updateHash();
}

int currentStatusData::get_trashHeight() const {
    return getValue<int>(offsetof(CurrentData, trashHeight));
}

void currentStatusData::set_trashHeight(int value) {
    setValue<int>(offsetof(CurrentData,trashHeight), value);
}

float currentStatusData::get_percentFull() const {
    return getValue<float>(offsetof(CurrentData, percentFull));
}

void currentStatusData::set_percentFull(float value) {
    setValue<float>(offsetof(CurrentData,percentFull),value);
}

time_t currentStatusData::get_lastMeasureTime() const {
    return getValue<time_t>(offsetof(CurrentData, lastMeasureTime));
}

void currentStatusData::set_lastMeasureTime(time_t value) {
    setValue<time_t>(offsetof(CurrentData, lastMeasureTime), value);
}


bool currentStatusData::get_trashcanEmptied() const {
    return getValue<bool>(offsetof(CurrentData, trashcanEmptied));
}   

void currentStatusData::set_trashcanEmptied(bool value) {
    setValue<bool>(offsetof(CurrentData, trashcanEmptied), value);
}

float currentStatusData::get_internalTempC() const {
    return getValue<float>(offsetof(CurrentData, internalTempC));
}

void currentStatusData::set_internalTempC(float value) {
    setValue<float>(offsetof(CurrentData, internalTempC), value);
}

uint8_t currentStatusData::get_lidPosition() const  {
    return getValue<uint8_t>(offsetof(CurrentData, lidPosition));
}
void currentStatusData::set_lidPosition(uint8_t value) {
    setValue<uint8_t>(offsetof(CurrentData, lidPosition), value);
}

int8_t currentStatusData::get_alertCode() const {
    return getValue<int8_t>(offsetof(CurrentData, alertCode));
}

void currentStatusData::set_alertCode(int8_t value) {
    setValue<int8_t>(offsetof(CurrentData, alertCode), value);
}

float currentStatusData::get_batteryVoltage() const  {
    return getValue<float>(offsetof(CurrentData,batteryVoltage));
}
void currentStatusData::set_batteryVoltage(float value) {
    setValue<float>(offsetof(CurrentData, batteryVoltage), value);
}

