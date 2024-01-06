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

#include "Iconnection.h"

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

bool IConnection::beginPacket() {
    return true;
}

bool IConnection::endPacket() {
    return true;
}

bool IConnection::beginBundle() {
	MUST_TRANSFER_BOOL(m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT));
	MUST_TRANSFER_BOOL(m_Connected);
	MUST_TRANSFER_BOOL(!m_IsBundle);
	MUST_TRANSFER_BOOL(beginPacket());

	m_IsBundle = true;
	m_BundlePacketInnerCount = 0;
	return true;
}

bool IConnection::endBundle() {
	MUST_TRANSFER_BOOL(m_IsBundle);

	m_IsBundle = false;
	
	MUST_TRANSFER_BOOL((m_BundlePacketInnerCount > 0));

	return endPacket();
}

size_t IConnection::write(const uint8_t *buffer, size_t size) {
    return 0;
}

size_t IConnection::write(uint8_t byte) {
	return write(&byte, 1);
}

bool IConnection::sendFloat(float f) {
	convert_to_chars(f, m_Buf);

	return write(m_Buf, sizeof(f)) != 0;
}

bool IConnection::sendByte(uint8_t c) { return write(&c, 1) != 0; }

bool IConnection::sendShort(uint16_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool IConnection::sendInt(uint32_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool IConnection::sendLong(uint64_t l) {
	convert_to_chars(l, m_Buf);

	return write(m_Buf, sizeof(l)) != 0;
}

bool IConnection::sendBytes(const uint8_t* c, size_t length) {
	return write(c, length) != 0;
}

bool IConnection::sendPacketNumber() {
	if (m_IsBundle) {
		return true;
	}

	uint64_t pn = m_PacketNumber++;

	return sendLong(pn);
}

bool IConnection::sendShortString(const char* str) {
	uint8_t size = strlen(str);

	MUST_TRANSFER_BOOL(sendByte(size));
	MUST_TRANSFER_BOOL(sendBytes((const uint8_t*)str, size));

	return true;
}

bool IConnection::sendPacketType(uint8_t type) {
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));

	return sendByte(type);
}

bool IConnection::sendLongString(const char* str) {
	int size = strlen(str);

	MUST_TRANSFER_BOOL(sendInt(size));

	return sendBytes((const uint8_t*)str, size);
}

int IConnection::getWriteError() { return 0; }

// PACKET_HEARTBEAT 0
void IConnection::sendHeartbeat() {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HEARTBEAT));
	MUST(sendPacketNumber());

	MUST(endPacket());
}

// PACKET_ACCEL 4
void IConnection::sendSensorAcceleration(uint8_t sensorId, Vector3 vector) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ACCEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(vector.x));
	MUST(sendFloat(vector.y));
	MUST(sendFloat(vector.z));
	MUST(sendByte(sensorId));

	MUST(endPacket());
}

// PACKET_BATTERY_LEVEL 12
void IConnection::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_BATTERY_LEVEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(batteryVoltage));
	MUST(sendFloat(batteryPercentage));

	MUST(endPacket());
}

// PACKET_TAP 13
void IConnection::sendSensorTap(uint8_t sensorId, uint8_t value) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TAP));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(value));

	MUST(endPacket());
}

// PACKET_ERROR 14
void IConnection::sendSensorError(uint8_t sensorId, uint8_t error) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ERROR));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(error));

	MUST(endPacket());
}

// PACKET_SENSOR_INFO 15
void IConnection::sendSensorInfo(Sensor* sensor) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SENSOR_INFO));
	MUST(sendPacketNumber());
	MUST(sendByte(sensor->getSensorId()));
	MUST(sendByte((uint8_t)sensor->getSensorState()));
	MUST(sendByte(sensor->getSensorType()));

	MUST(endPacket());
}

// PACKET_ROTATION_DATA 17
void IConnection::sendRotationData(
	uint8_t sensorId,
	Quat* const quaternion,
	uint8_t dataType,
	uint8_t accuracyInfo
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ROTATION_DATA));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(dataType));
	MUST(sendFloat(quaternion->x));
	MUST(sendFloat(quaternion->y));
	MUST(sendFloat(quaternion->z));
	MUST(sendFloat(quaternion->w));
	MUST(sendByte(accuracyInfo));

	MUST(endPacket());
}

// PACKET_MAGNETOMETER_ACCURACY 18
void IConnection::sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_MAGNETOMETER_ACCURACY));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(accuracyInfo));

	MUST(endPacket());
}

// PACKET_SIGNAL_STRENGTH 19
void IConnection::sendSignalStrength(uint8_t signalStrength) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SIGNAL_STRENGTH));
	MUST(sendPacketNumber());
	MUST(sendByte(255));
	MUST(sendByte(signalStrength));

	MUST(endPacket());
}

// PACKET_TEMPERATURE 20
void IConnection::sendTemperature(uint8_t sensorId, float temperature) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TEMPERATURE));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(temperature));

	MUST(endPacket());
}

// PACKET_FEATURE_FLAGS 22
void IConnection::sendFeatureFlags() {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_FEATURE_FLAGS));
	MUST(sendPacketNumber());
	MUST(write(FirmwareFeatures::flags.data(), FirmwareFeatures::flags.size()));

	MUST(endPacket());
}

void IConnection::sendTrackerDiscovery() {

}

#if ENABLE_INSPECTION
void IConnection::sendInspectionRawIMUData(
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
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_INT));

	MUST(sendInt(rX));
	MUST(sendInt(rY));
	MUST(sendInt(rZ));
	MUST(sendByte(rA));

	MUST(sendInt(aX));
	MUST(sendInt(aY));
	MUST(sendInt(aZ));
	MUST(sendByte(aA));

	MUST(sendInt(mX));
	MUST(sendInt(mY));
	MUST(sendInt(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
}

void IConnection::sendInspectionRawIMUData(
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
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_FLOAT));

	MUST(sendFloat(rX));
	MUST(sendFloat(rY));
	MUST(sendFloat(rZ));
	MUST(sendByte(rA));

	MUST(sendFloat(aX));
	MUST(sendFloat(aY));
	MUST(sendFloat(aZ));
	MUST(sendByte(aA));

	MUST(sendFloat(mX));
	MUST(sendFloat(mY));
	MUST(sendFloat(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
}
#endif

void IConnection::returnLastPacket(int len) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendBytes(m_Packet, len));

	MUST(endPacket());
}

void IConnection::updateSensorState(std::vector<Sensor *> & sensors) {
	if (millis() - m_LastSensorInfoPacketTimestamp <= 1000) {
		return;
	}

	m_LastSensorInfoPacketTimestamp = millis();

	for (int i = 0; i < (int)sensors.size(); i++) {
		if (m_AckedSensorState[i] != sensors[i]->getSensorState()) {
			sendSensorInfo(sensors[i]);
		}
	}
}

void IConnection::maybeRequestFeatureFlags() {	
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

void IConnection::searchForServer() {

}

void IConnection::reset() {

}

void IConnection::update() {

}

}  // namespace Network
}  // namespace SlimeVR