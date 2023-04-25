#include "SerialAdapter.h"
#include "DebugPrint.h"

size_t SerialAdapter::write(uint8_t byte)
{
	return streamOut->write(byte);
}

size_t SerialAdapter::write(const uint8_t* buffer, size_t size)
{
	return streamOut->write(buffer, size);
}

int SerialAdapter::availableForWrite()
{
	return streamOut->availableForWrite();
}

int SerialAdapter::read()
{
	return streamIn->read();
}

int SerialAdapter::available()
{
	return streamIn->available();
}

void SerialAdapter::flush()
{
	return streamOut->flush();
}

void SerialAdapter::begin(unsigned long buad) {
	return;
}

int SerialAdapter::peek()
{
	return streamIn->peek();
}

