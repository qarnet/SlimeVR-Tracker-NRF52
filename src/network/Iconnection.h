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
#ifndef SLIMEVR_NETWORK_ICONNECTION_H_
#define SLIMEVR_NETWORK_ICONNECTION_H_

#include <Arduino.h>

#include "globals.h"
#include "quat.h"
#include "sensors/sensor.h"
#include "featureflags.h"

namespace SlimeVR {
namespace Network {

class IConnection {
public:
	virtual void searchForServer() = 0;
	virtual void update() = 0;
	virtual void reset() = 0;

	bool isConnected() const { return m_Connected; };

	// PACKET_ACCEL 4
	void sendSensorAcceleration(uint8_t sensorId, Vector3 vector);

	// PACKET_BATTERY_LEVEL 12
	void sendBatteryLevel(float batteryVoltage, float batteryPercentage);

	// PACKET_TAP 13
	void sendSensorTap(uint8_t sensorId, uint8_t value);

	// PACKET_ERROR 14
	void sendSensorError(uint8_t sensorId, uint8_t error);

	// PACKET_ROTATION_DATA 17
	void sendRotationData(uint8_t sensorId,
		Quat* const quaternion,
		uint8_t dataType,
		uint8_t accuracyInfo);

	// PACKET_MAGNETOMETER_ACCURACY 18
	void sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo);

	// PACKET_SIGNAL_STRENGTH 19
	void sendSignalStrength(uint8_t signalStrength);

	// PACKET_TEMPERATURE 20
	void sendTemperature(uint8_t sensorId, float temperature);

	// PACKET_FEATURE_FLAGS 22
	void sendFeatureFlags();

#if ENABLE_INSPECTION
	void sendInspectionRawIMUData();
	void sendInspectionRawIMUData();
#endif

	const ServerFeatures& getServerFeatureFlags();

	virtual bool beginBundle();
	bool endBundle();

	protected:
	virtual bool beginPacket() = 0;
	virtual bool endPacket() = 0;
	virtual size_t write(const uint8_t *buffer, size_t size) = 0;
	virtual int getWriteError() = 0;
	// PACKET_HANDSHAKE 3
	virtual void sendTrackerDiscovery() = 0;

	void updateSensorState(std::vector<Sensor *> & sensors);
	void maybeRequestFeatureFlags();
	
	virtual size_t write(uint8_t byte);

	bool sendPacketType(uint8_t type);
	bool sendPacketNumber();
	bool sendFloat(float f);
	bool sendByte(uint8_t c);
	bool sendShort(uint16_t i);
	bool sendInt(uint32_t i);
	bool sendLong(uint64_t l);
	bool sendBytes(const uint8_t* c, size_t length);
	bool sendShortString(const char* str);
	bool sendLongString(const char* str);

	void returnLastPacket(int len);

	// PACKET_HEARTBEAT 0
	void sendHeartbeat();

	// PACKET_SENSOR_INFO 15
	void sendSensorInfo(Sensor* sensor);

	bool m_Connected = false;
	SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("UDPConnection");

	unsigned char m_Packet[128];  // buffer for incoming packets
	uint64_t m_PacketNumber = 0;

	unsigned long m_LastConnectionAttemptTimestamp;
	unsigned long m_LastPacketTimestamp;

	SensorStatus m_AckedSensorState[MAX_IMU_COUNT] = {SensorStatus::SENSOR_OFFLINE};
	unsigned long m_LastSensorInfoPacketTimestamp = 0;

	uint8_t m_FeatureFlagsRequestAttempts = 0;
	unsigned long m_FeatureFlagsRequestTimestamp = millis();
	ServerFeatures m_ServerFeatures{};

	bool m_IsBundle = false;
	uint16_t m_BundlePacketPosition = 0;
	uint16_t m_BundlePacketInnerCount = 0;

	unsigned char m_Buf[8];
};

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_ICONNECTION_H_
