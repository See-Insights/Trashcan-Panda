#include "Particle.h"
#include "StorageHelperRK.h"


void readTestData(const char *filename, char *&data, size_t &size) {

	FILE *fd = fopen(filename, "r");
	if (!fd) {
		printf("failed to open %s\n", filename);
		return;
	}

	fseek(fd, 0, SEEK_END);
	size = ftell(fd);
	fseek(fd, 0, SEEK_SET);

	data = (char *) malloc(size + 1);
	fread(data, 1, size, fd);
	data[size] = 0;

	fclose(fd);
}

char *readTestData(const char *filename) {
	char *data = 0;
	size_t size;

	readTestData(filename, data, size);

	return data;
}

#define assertInt(msg, got, expected) _assertInt(msg, got, expected, __LINE__)
void _assertInt(const char *msg, int got, int expected, int line) {
	if (expected != got) {
		printf("assertion failed %s line %d\n", msg, line);
		printf("expected: %d\n", expected);
		printf("     got: %d\n", got);
		assert(false);
	}
}

#define assertDouble(msg, got, expected, margin) _assertDouble(msg, got, expected, margin, __LINE__)
void _assertDouble(const char *msg, double got, double expected, double margin, int line) {
	if ((expected < (got - margin)) || (expected > (got + margin))) {
		printf("assertion failed %s line %d\n", msg, line);
		printf("expected: %lf\n", expected);
		printf("     got: %lf\n", got);
		assert(false);
	}
}


#define assertStr(msg, got, expected) _assertStr(msg, got, expected, __LINE__)
void _assertStr(const char *msg, const char *got, const char *expected, int line) {
	if (strcmp(expected, got) != 0) {
		printf("assertion failed %s line %d\n", msg, line);
		printf("expected: %s\n", expected);
		printf("     got: %s\n", got);
		assert(false);
	}
}

#define assertFile(msg, got, expected) _assertFile(msg, got, expected, __LINE__)
void _assertFile(const char *msg, const char *gotPath, const char *expectedPath, int line) {
	char *gotData, *expectedData;
	size_t gotSize, expectedSize;

	readTestData(gotPath, gotData, gotSize);
	readTestData(expectedPath, expectedData, expectedSize);

	if (gotSize != expectedSize) {
		printf("assertion failed %s line %d\n", msg, line);
		printf("expected size: %lu for %s\n", expectedSize, expectedPath);
		printf("     got size: %lu for %s\n", gotSize, gotPath);
		assert(false);
	}
	for(size_t ii = 0; ii < gotSize; ii++) {
		if (gotData[ii] != expectedData[ii]) {
			printf("assertion failed %s line %d\n", msg, line);
			printf("expected data: %02x index %lu for %s\n", expectedData[ii], ii, expectedPath);
			printf("     got data: %02x index %lu for %s\n", gotData[ii], ii, gotPath);
			assert(false);
		}
	}

	free(gotData);
	free(expectedData);
}

const char *persistentDataPath = "./temp02.dat";


class MyPersistentData : public StorageHelperRK::PersistentDataFile {
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

	MyPersistentData() : PersistentDataFile(persistentDataPath, &myData.header, sizeof(MyData), DATA_MAGIC, DATA_VERSION) {};

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


	MyData myData;
};



class MyPersistentData2 : public StorageHelperRK::PersistentDataFile {
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
		uint32_t test5;
		// OK to add more fields here 
	};

	static const uint32_t DATA_MAGIC = 0x20a99e73;
	static const uint16_t DATA_VERSION = 1;

	MyPersistentData2() : PersistentDataFile(persistentDataPath, &myData.header, sizeof(MyData), DATA_MAGIC, DATA_VERSION) {};

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

	uint32_t getValue_test5() const {
		return getValue<uint32_t>(offsetof(MyData, test5));
	}

	void setValue_test5(uint32_t value) {
		setValue<uint32_t>(offsetof(MyData, test5), value);
	}

	MyData myData;
};


void customPersistentDataTest() {
	unlink(persistentDataPath);

	MyPersistentData data;
	bool bResult;
	String s;

	data.load();
	data.save();

	data.setValue_test1(0x55aa55aa);
	assertInt("", data.getValue_test1(), 0x55aa55aa);

	assertInt("", data.getValue_test2(), false);
	data.setValue_test2(true);
	assertInt("", data.getValue_test2(), true);


	assertDouble("", data.getValue_test3(), 0.0, 0.001);
	data.setValue_test3(9999999.12345);
	assertDouble("", data.getValue_test3(), 9999999.12345, 0.001);

	s = data.getValue_test4();
	assertStr("", s, "");
	data.setValue_test4("testing!"); 
	s = data.getValue_test4();
	assertStr("", s, "testing!");

	data.setValue_test4("testing1!"); 
	s = data.getValue_test4();
	assertStr("", s, "testing1!");

	bResult = data.setValue_test4("testing12!"); 
	assertInt("", bResult, false);
	s = data.getValue_test4();
	assertStr("", s, "testing1!");

	data.save();


	MyPersistentData data2;
	data2.load();

	assertInt("", data2.getValue_test1(), 0x55aa55aa);
	assertInt("", data2.getValue_test2(), true);
	assertDouble("", data2.getValue_test3(), 9999999.12345, 0.001);
	assertStr("", data2.getValue_test4(), "testing1!");

	// Simulate a new version that adds a new field without changing the version or magic
	MyPersistentData2 data2b;
	data2b.load();

	assertInt("", data2b.getValue_test1(), 0x55aa55aa);
	assertInt("", data2b.getValue_test2(), true);
	assertDouble("", data2b.getValue_test3(), 9999999.12345, 0.001);
	assertStr("", data2b.getValue_test4(), "testing1!");
	assertInt("", data2b.getValue_test5(), 0);

	data2b.setValue_test5(12345);
	assertInt("", data2b.getValue_test5(), 12345);

	data2b.save();

	MyPersistentData2 data2c;
	data2c.load();

	assertInt("", data2c.getValue_test5(), 12345);

	unlink(persistentDataPath);

}


class RetainedDataTest : public StorageHelperRK::PersistentDataBase {
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

	static const uint32_t DATA_MAGIC = 0xd971e39b;
	static const uint16_t DATA_VERSION = 1;

	RetainedDataTest(StorageHelperRK::PersistentDataBase::SavedDataHeader *header) : StorageHelperRK::PersistentDataBase(header, sizeof(MyData), DATA_MAGIC, DATA_VERSION) {};

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

};

void customRetainedDataTest() {

	RetainedDataTest::MyData retainedData; // Simulating retained data
	memset(&retainedData, 0, sizeof(retainedData));


	RetainedDataTest data(&retainedData.header);
	bool bResult;
	String s;

	data.load();

	data.setValue_test1(0x55aa55aa);
	assertInt("", data.getValue_test1(), 0x55aa55aa);

	assertInt("", data.getValue_test2(), false);
	data.setValue_test2(true);
	assertInt("", data.getValue_test2(), true);


	assertDouble("", data.getValue_test3(), 0.0, 0.001);
	data.setValue_test3(9999999.12345);
	assertDouble("", data.getValue_test3(), 9999999.12345, 0.001);

	s = data.getValue_test4();
	assertStr("", s, "");
	data.setValue_test4("testing!"); 
	s = data.getValue_test4();
	assertStr("", s, "testing!");

	data.setValue_test4("testing1!"); 
	s = data.getValue_test4();
	assertStr("", s, "testing1!");

	bResult = data.setValue_test4("testing12!"); 
	assertInt("", bResult, false);
	s = data.getValue_test4();
	assertStr("", s, "testing1!");


	RetainedDataTest data2(&retainedData.header);
	data2.load();

	assertInt("", data2.getValue_test1(), 0x55aa55aa);
	assertInt("", data2.getValue_test2(), true);
	assertDouble("", data2.getValue_test3(), 9999999.12345, 0.001);
	assertStr("", data2.getValue_test4(), "testing1!");

}



int main(int argc, char *argv[]) {
	customPersistentDataTest();
	customRetainedDataTest();
	return 0;
}
