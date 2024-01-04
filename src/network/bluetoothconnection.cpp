/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#if defined(XIAO_NRF52840)

#include "bluefruit.h"

#include "bluetoothconnection.h"
#include "GlobalVars.h"
#include "logging/Logger.h"
#include "packets.h"

#define TIMEOUT 3000UL

template <typename T>
unsigned char* convert_to_chars(T src, unsigned char* target) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	un.v = src;
	for (size_t i = 0; i < sizeof(T); i++) {
		target[i] = un.c[sizeof(T) - i - 1];
	}
	return target;
}

template <typename T>
T convert_chars(unsigned char* const src) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	for (size_t i = 0; i < sizeof(T); i++) {
		un.c[i] = src[sizeof(T) - i - 1];
	}
	return un.v;
}

namespace SlimeVR {
namespace Network {

#define MUST_TRANSFER_BOOL(b) \
	if (!b)                   \
		return false;

#define MUST(b) \
	if (!b)     \
		return;

bool BluetoothConnection::beginPacket() {
    return true;
}

bool BluetoothConnection::endPacket() {
    return true;
}

bool BluetoothConnection::beginBundle() {
    return true;
}

bool BluetoothConnection::endBundle() {
    return true;
}

size_t BluetoothConnection::write(const uint8_t *buffer, size_t size) {
    return 0;
}

size_t BluetoothConnection::write(uint8_t byte) {
    return 0;
}

bool BluetoothConnection::sendFloat(float f) {
    return true;
}

bool BluetoothConnection::sendByte(uint8_t c) { return true; }

bool BluetoothConnection::sendShort(uint16_t i) {
    return true;
}

bool BluetoothConnection::sendInt(uint32_t i) {
    return true;
}

bool BluetoothConnection::sendLong(uint64_t l) {
    return true;
}

bool BluetoothConnection::sendBytes(const uint8_t* c, size_t length) {
    return true;
}

bool BluetoothConnection::sendPacketNumber() {
    return true;
}

bool BluetoothConnection::sendShortString(const char* str) {
    return true;
}

bool BluetoothConnection::sendPacketType(uint8_t type) {
    return true;
}

bool BluetoothConnection::sendLongString(const char* str) {
    return true;
}

int BluetoothConnection::getWriteError() { return 0; }

// PACKET_HEARTBEAT 0
void BluetoothConnection::sendHeartbeat() {

}

// PACKET_ACCEL 4
void BluetoothConnection::sendSensorAcceleration(uint8_t sensorId, Vector3 vector) {

}

// PACKET_BATTERY_LEVEL 12
void BluetoothConnection::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {

}

// PACKET_TAP 13
void BluetoothConnection::sendSensorTap(uint8_t sensorId, uint8_t value) {

}

// PACKET_ERROR 14
void BluetoothConnection::sendSensorError(uint8_t sensorId, uint8_t error) {

}

// PACKET_SENSOR_INFO 15
void BluetoothConnection::sendSensorInfo(Sensor* sensor) {

}

// PACKET_ROTATION_DATA 17
void BluetoothConnection::sendRotationData(uint8_t sensorId,
		Quat* const quaternion,
		uint8_t dataType,
		uint8_t accuracyInfo){

}

// PACKET_MAGNETOMETER_ACCURACY 18
void BluetoothConnection::sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) {

}

// PACKET_SIGNAL_STRENGTH 19
void BluetoothConnection::sendSignalStrength(uint8_t signalStrength) {

}

// PACKET_TEMPERATURE 20
void BluetoothConnection::sendTemperature(uint8_t sensorId, float temperature) {

}

// PACKET_FEATURE_FLAGS 22
void BluetoothConnection::sendFeatureFlags() {

}

void BluetoothConnection::sendTrackerDiscovery() {

}

#if ENABLE_INSPECTION
void BluetoothConnection::sendInspectionRawIMUData(
	uint8_t sensorId,
	int16_t rX,
	int16_t rY,
	int16_t rZ,
	uint8_t rA,
	int16_t aX,
	int16_t aY,
	int16_t aZ,
	uint8_t aA,
	int16_t mX,
	int16_t mY,
	int16_t mZ,
	uint8_t mA
) {

}

void BluetoothConnection::sendInspectionRawIMUData(
	uint8_t sensorId,
	float rX,
	float rY,
	float rZ,
	uint8_t rA,
	float aX,
	float aY,
	float aZ,
	uint8_t aA,
	float mX,
	float mY,
	float mZ,
	uint8_t mA
) {

}
#endif

void BluetoothConnection::returnLastPacket(int len) {

}

void BluetoothConnection::updateSensorState(std::vector<Sensor *> & sensors) {

}

void BluetoothConnection::maybeRequestFeatureFlags() {	

}

void BluetoothConnection::searchForServer() {

}

void BluetoothConnection::reset() {

}

void BluetoothConnection::update() {

}

void BluetoothConnection::sendTest() {
	uint8_t test = 90;
	BleNetwork::indicate(&test, sizeof(test));
}

}  // namespace Network
}  // namespace SlimeVR

#endif