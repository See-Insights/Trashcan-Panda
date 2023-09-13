// These must be included before StorageHelperRK.h!
#include "SdFat.h"

#include "StorageHelperRK.h"


SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler(LOG_LEVEL_TRACE);

const int SD_CHIP_SELECT = A2;

SdFat sdCard;


class MyPersistentData : public StorageHelperRK::PersistentDataFileSystem {
public:
	class MyData {
	public:
		// This structure must always begin with the header (16 bytes)
		StorageHelperRK::PersistentDataBase::SavedDataHeader header;
		// Your fields go here. Once you've added a field you cannot add fields
		// (except at the end), insert fields, remove fields, change size of a field.
		// Doing so will cause the data to be corrupted!
		// You may want to keep a version number in your data.
		int test1;
		bool test2;
		double test3;
		char test4[10];
		// OK to add more fields here 
	};

	static const uint32_t DATA_MAGIC = 0x20a99e73;
	static const uint16_t DATA_VERSION = 1;

	MyPersistentData() : StorageHelperRK::PersistentDataFileSystem(new StorageHelperRK::FileSystemSdFat(sdCard), "test4.dat", &myData.header, sizeof(MyData), DATA_MAGIC, DATA_VERSION) {
    };

	int getValue_test1() const {
		return getValue<int>(offsetof(MyData, test1));
	}

	void setValue_test1(int value) {
		setValue<int>(offsetof(MyData, test1), value);
	}

	bool getValue_test2() const {
		return getValue<bool>(offsetof(MyData, test2));
	}

	void setValue_test2(bool value) {
		setValue<bool>(offsetof(MyData, test2), value);
	}

	double getValue_test3() const {
		return getValue<double>(offsetof(MyData, test3));
	}

	void setValue_test3(double value) {
		setValue<double>(offsetof(MyData, test3), value);
	}

	String getValue_test4() const {
		String result;
		getValueString(offsetof(MyData, test4), sizeof(MyData::test4), result);
		return result;
	}
	bool setValue_test4(const char *str) {
		return setValueString(offsetof(MyData, test4), sizeof(MyData::test4), str);
	}

    void logData(const char *msg) {
        Log.info("%s: %d, %d, %lf, %s", msg, myData.test1, (int)myData.test2, myData.test3, myData.test4);
    }


	MyData myData;
};




void setup() {
    // Optional: Enable to make it easier to see debug USB serial messages at startup
    waitFor(Serial.isConnected, 10000);
    delay(2000);

	if (sdCard.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
		Log.info("sd card initialized");
	}
	else {
		Log.info("failed to initialize sd card");
	}

	Particle.connect();


}

void loop() {

    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= 10000) {
        lastCheck = millis();

        MyPersistentData data;

        data.load();

        data.logData("after loading");

        data.setValue_test1(data.getValue_test1() + 1);
        data.setValue_test2(!data.getValue_test2());
        data.setValue_test3(data.getValue_test3() - 0.1);
        data.setValue_test4("testing!"); 

        data.logData("after update");

        data.flush(true);
    }  
}



