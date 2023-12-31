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
	virtual bool isConnected() const = 0;

	// PACKET_ACCEL 4
	virtual void sendSensorAcceleration(uint8_t sensorId, Vector3 vector) = 0;

	// PACKET_BATTERY_LEVEL 12
	virtual void sendBatteryLevel(float batteryVoltage, float batteryPercentage) = 0;

	// PACKET_TAP 13
	virtual void sendSensorTap(uint8_t sensorId, uint8_t value) = 0;

	// PACKET_ERROR 14
	virtual void sendSensorError(uint8_t sensorId, uint8_t error) = 0;

	// PACKET_ROTATION_DATA 17
	virtual void sendRotationData(uint8_t sensorId,
		Quat* const quaternion,
		uint8_t dataType,
		uint8_t accuracyInfo) = 0;

	// PACKET_MAGNETOMETER_ACCURACY 18
	virtual void sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) = 0;

	// PACKET_SIGNAL_STRENGTH 19
	virtual void sendSignalStrength(uint8_t signalStrength) = 0;

	// PACKET_TEMPERATURE 20
	virtual void sendTemperature(uint8_t sensorId, float temperature) = 0;

	// PACKET_FEATURE_FLAGS 22
	virtual void sendFeatureFlags() = 0;

#if ENABLE_INSPECTION
	virtual void sendInspectionRawIMUData() = 0;
	virtual void sendInspectionRawIMUData() = 0;
#endif

	virtual const ServerFeatures& getServerFeatureFlags() = 0;

	virtual bool beginBundle() = 0;
	virtual bool endBundle() = 0;
};

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_ICONNECTION_H_
