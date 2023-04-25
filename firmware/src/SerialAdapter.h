#include "HardwareSerial.h"

class SerialAdapter : HardwareSerial {
private:
    Stream* streamIn;
    Stream* streamOut;

public:
	SerialAdapter(Stream* streamIn, Stream* streamOut): HardwareSerial(Serial1) {
		this->streamIn = streamIn;
		this->streamOut = streamOut;
	}

	size_t write(uint8_t byte) override;

	void begin(unsigned long baud);

	int read() override;


	int available() override;


	void flush() override;


	int peek() override;


	size_t write(const uint8_t* buffer, size_t size) override;


	int availableForWrite() override;

};