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

typedef struct {
	uint8_t buf[512];
	uint16_t pos;
} ble_buf;

#define DEFINE_BUF(buffer) \
	ble_buf buffer = {0};
#define ADD_TO_BUF(buffer, data) \
	memcpy(&buffer.buf[buffer.pos], &data, sizeof(data)); \
	buffer.pos += sizeof(data);
#define ADD_U8_TO_BUF(buffer, data) \
	buffer.buf[buffer.pos] = data; \
	buffer.pos += 1;
#define ADD_ARR_TO_BUF(buffer, data) \
	memcpy(&buffer.buf[buffer.pos], data, sizeof(data)); \
	buffer.pos += sizeof(data);
#define ADD_FLOAT_TO_BUF(buffer, data) \
	convert_to_chars(data, &buffer.buf[buffer.pos]); \
	buffer.pos += sizeof(data);


#define TIMEOUT 3000UL

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
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_HEARTBEAT);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_ACCEL 4
void BluetoothConnection::sendSensorAcceleration(uint8_t sensorId, Vector3 vector) {
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_ACCEL);
	ADD_FLOAT_TO_BUF(buffer, vector.x);
	ADD_FLOAT_TO_BUF(buffer, vector.y);
	ADD_FLOAT_TO_BUF(buffer, vector.z);
	ADD_TO_BUF(buffer, sensorId);

	for(int i = 0; i < buffer.pos; i++)
	{
		m_Logger.warn("%u", buffer.buf[i]);
	}

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_BATTERY_LEVEL 12
void BluetoothConnection::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {
	DEFINE_BUF(buffer);
	
	ADD_U8_TO_BUF(buffer, PACKET_BATTERY_LEVEL);
	ADD_TO_BUF(buffer, batteryVoltage);
	ADD_TO_BUF(buffer, batteryPercentage);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_TAP 13
void BluetoothConnection::sendSensorTap(uint8_t sensorId, uint8_t value) {
	DEFINE_BUF(buffer);
	
	ADD_U8_TO_BUF(buffer, PACKET_TAP);
	ADD_TO_BUF(buffer, sensorId);
	ADD_TO_BUF(buffer, value);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_ERROR 14
void BluetoothConnection::sendSensorError(uint8_t sensorId, uint8_t error) {
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_ERROR);
	ADD_TO_BUF(buffer, sensorId);
	ADD_TO_BUF(buffer, error);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_SENSOR_INFO 15
void BluetoothConnection::sendSensorInfo(Sensor* sensor) {
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_SENSOR_INFO);
	ADD_U8_TO_BUF(buffer, sensor->getSensorId());
	ADD_U8_TO_BUF(buffer, (uint8_t)sensor->getSensorState());
	ADD_U8_TO_BUF(buffer, sensor->getSensorType());

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_ROTATION_DATA 17
void BluetoothConnection::sendRotationData(uint8_t sensorId,
		Quat* const quaternion,
		uint8_t dataType,
		uint8_t accuracyInfo){
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_ROTATION_DATA);
	ADD_TO_BUF(buffer, sensorId);
	ADD_TO_BUF(buffer, dataType);
	ADD_FLOAT_TO_BUF(buffer, quaternion->x);
	ADD_FLOAT_TO_BUF(buffer, quaternion->y);
	ADD_FLOAT_TO_BUF(buffer, quaternion->z);
	ADD_FLOAT_TO_BUF(buffer, quaternion->w);
	ADD_TO_BUF(buffer, accuracyInfo);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_MAGNETOMETER_ACCURACY 18
void BluetoothConnection::sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) {
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_MAGNETOMETER_ACCURACY);
	ADD_TO_BUF(buffer, sensorId);
	ADD_TO_BUF(buffer, accuracyInfo);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_SIGNAL_STRENGTH 19
void BluetoothConnection::sendSignalStrength(uint8_t signalStrength) {
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_SIGNAL_STRENGTH);
	ADD_U8_TO_BUF(buffer, 255);
	ADD_TO_BUF(buffer, signalStrength);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_TEMPERATURE 20
void BluetoothConnection::sendTemperature(uint8_t sensorId, float temperature) {
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_TEMPERATURE);
	ADD_TO_BUF(buffer, sensorId);
	ADD_TO_BUF(buffer, temperature);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

// PACKET_FEATURE_FLAGS 22
void BluetoothConnection::sendFeatureFlags() {
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_FEATURE_FLAGS);
	// ADD_TO_BUF(buffer, FirmwareFeatures::flags.data());

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

void BluetoothConnection::sendTrackerDiscovery() {
	uint8_t mac[6];
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_HANDSHAKE);
	ADD_U8_TO_BUF(buffer, (uint32_t)(0));
	ADD_U8_TO_BUF(buffer, (uint32_t)(BOARD));
	ADD_U8_TO_BUF(buffer, (uint32_t)(IMU));
	ADD_U8_TO_BUF(buffer, (uint32_t)(HARDWARE_MCU));
	ADD_U8_TO_BUF(buffer, (uint32_t)(0));
	ADD_U8_TO_BUF(buffer, (uint32_t)(0));
	ADD_U8_TO_BUF(buffer, (uint32_t)(0));
	ADD_U8_TO_BUF(buffer, (uint32_t)(FIRMWARE_BUILD_NUMBER));
	ADD_TO_BUF(buffer, FIRMWARE_VERSION);
	ADD_TO_BUF(buffer, mac);

	BleNetwork::indicate(buffer.buf, buffer.pos);
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
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_INSPECTION);
	ADD_U8_TO_BUF(buffer, PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA);
	ADD_TO_BUF(buffer, sensorId);
	ADD_U8_TO_BUF(buffer, PACKET_INSPECTION_DATATYPE_INT);
	ADD_TO_BUF(buffer, rX);
	ADD_TO_BUF(buffer, rY);
	ADD_TO_BUF(buffer, rZ);
	ADD_TO_BUF(buffer, rA);
	ADD_TO_BUF(buffer, aX);
	ADD_TO_BUF(buffer, aY);
	ADD_TO_BUF(buffer, aZ);
	ADD_TO_BUF(buffer, aA);
	ADD_TO_BUF(buffer, mX);
	ADD_TO_BUF(buffer, mY);
	ADD_TO_BUF(buffer, mZ);
	ADD_TO_BUF(buffer, mA);

	BleNetwork::indicate(buffer.buf, buffer.pos);
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
	DEFINE_BUF(buffer);

	ADD_U8_TO_BUF(buffer, PACKET_INSPECTION);
	ADD_U8_TO_BUF(buffer, PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA);
	ADD_TO_BUF(buffer, sensorId);
	ADD_U8_TO_BUF(buffer, PACKET_INSPECTION_DATATYPE_FLOAT);
	ADD_FLOAT_TO_BUF(buffer, rX);
	ADD_FLOAT_TO_BUF(buffer, rY);
	ADD_FLOAT_TO_BUF(buffer, rZ);
	ADD_FLOAT_TO_BUF(buffer, rA);
	ADD_FLOAT_TO_BUF(buffer, aX);
	ADD_FLOAT_TO_BUF(buffer, aY);
	ADD_FLOAT_TO_BUF(buffer, aZ);
	ADD_FLOAT_TO_BUF(buffer, aA);
	ADD_FLOAT_TO_BUF(buffer, mX);
	ADD_FLOAT_TO_BUF(buffer, mY);
	ADD_FLOAT_TO_BUF(buffer, mZ);
	ADD_FLOAT_TO_BUF(buffer, mA);

	BleNetwork::indicate(buffer.buf, buffer.pos);
}
#endif

void BluetoothConnection::returnLastPacket(int len) {
	DEFINE_BUF(buffer);

	// ADD_TO_BUF()

	BleNetwork::indicate(buffer.buf, buffer.pos);
}

void BluetoothConnection::updateSensorState(std::vector<Sensor *> & sensors) {
	m_LastSensorInfoPacketTimestamp = millis();

	for (int i = 0; i < (int)sensors.size(); i++) {
		if (m_AckedSensorState[i] != sensors[i]->getSensorState()) {
			sendSensorInfo(sensors[i]);
		}
	}
}

void BluetoothConnection::maybeRequestFeatureFlags() {	
	if (m_ServerFeatures.isAvailable() || m_FeatureFlagsRequestAttempts >= 15) {
		return;
	}

	if (millis() - m_FeatureFlagsRequestTimestamp < 500) {
		return;
	}

	sendFeatureFlags();
	m_FeatureFlagsRequestTimestamp = millis();
	m_FeatureFlagsRequestAttempts++;
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