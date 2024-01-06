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
	if (m_IsBundle) {
		m_BundlePacketPosition = 0;
		return true;
	}

	this->ble_buf.clear();
	this->ble_buf.reserve(128);

    return true;
}

bool BluetoothConnection::endPacket() {
	if (m_IsBundle) {
		uint32_t innerPacketSize = m_BundlePacketPosition;

		MUST_TRANSFER_BOOL((innerPacketSize > 0));

		m_IsBundle = false;
		
		if (m_BundlePacketInnerCount == 0) {
			sendPacketType(PACKET_BUNDLE);
			sendPacketNumber();
		}
		sendShort(innerPacketSize);
		sendBytes(m_Packet, innerPacketSize);

		m_BundlePacketInnerCount++;
		m_IsBundle = true;
		return true;
	}
	BleNetwork::indicate(this->ble_buf.data(), this->ble_buf.size());
    return true;
}

bool BluetoothConnection::beginBundle() {
	// MUST_TRANSFER_BOOL(m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT));
	MUST_TRANSFER_BOOL(m_Connected);
	MUST_TRANSFER_BOOL(!m_IsBundle);
	MUST_TRANSFER_BOOL(beginPacket());

	m_IsBundle = true;
	m_BundlePacketInnerCount = 0;

    return true;
}

size_t BluetoothConnection::write(const uint8_t *buffer, size_t size) {
	if (m_IsBundle) {
		if (m_BundlePacketPosition + size > sizeof(m_Packet)) {
			return 0;
		}
		memcpy(m_Packet + m_BundlePacketPosition, buffer, size);
		m_BundlePacketPosition += size;
		return size;
	}

	this->ble_buf.insert(this->ble_buf.end(), buffer, buffer + size);
    return 1;
}

// size_t BluetoothConnection::write(uint8_t byte) {
// 	this->ble_buf.insert(this->ble_buf.end(), byte);
//     return 1;
// }

int BluetoothConnection::getWriteError() { return 0; }

void BluetoothConnection::sendTrackerDiscovery() {
	MUST(!m_Connected);

	uint8_t mac[6];
	// WiFi.macAddress(mac);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HANDSHAKE));
	// Packet number is always 0 for handshake
	MUST(sendLong(0));
	MUST(sendInt(BOARD));
	// This is kept for backwards compatibility,
	// but the latest SlimeVR server will not initialize trackers
	// with firmware build > 8 until it recieves a sensor info packet
	MUST(sendInt(IMU));
	MUST(sendInt(HARDWARE_MCU));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(FIRMWARE_BUILD_NUMBER));
	MUST(sendShortString(FIRMWARE_VERSION));
	// MAC address string
	MUST(sendBytes(mac, 6));

	MUST(endPacket());
}

void BluetoothConnection::searchForServer() {
	while (true) {
		if (!BleNetwork::isPacketAvailable()) {
			break;
		}

		// receive incoming UDP packets
		int len = BleNetwork::getPacket(m_Packet, sizeof(m_Packet));

#ifdef DEBUG_NETWORK
		// m_Logger.trace(
		// 	"Received %d bytes from %s, port %d",
		// 	packetSize,
		// 	m_UDP.remoteIP().toString().c_str(),
		// 	m_UDP.remotePort()
		// );
		// m_Logger.traceArray("UDP packet contents: ", m_Packet, len);
#endif
		// Handshake is different, it has 3 in the first byte, not the 4th, and data
		// starts right after
		if (m_Packet[0] == PACKET_HANDSHAKE) {
		// if(BleNetwork::isSubscribed()) { // TODO: (qarnet) Evaluate this as alternative handshake
			if (strncmp((char*)m_Packet + 1, "Hey OVR =D 5", 12) != 0) {
				m_Logger.error("Received invalid handshake packet");
				continue;
			}

			m_LastPacketTimestamp = millis();
			m_Connected = true;
			
			m_FeatureFlagsRequestAttempts = 0;
			m_ServerFeatures = ServerFeatures { };

			statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, false);
			ledManager.off();

			// m_Logger.debug(
			// 	"Handshake successful, server is %s:%d",
			// 	// m_UDP.remoteIP().toString().c_str(),
			// 	// m_UDP.remotePort()
			// );

			break;
		}
	}

	auto now = millis();

	// This makes the LED blink for 20ms every second
	if (m_LastConnectionAttemptTimestamp + 1000 < now) {
		m_LastConnectionAttemptTimestamp = now;
		m_Logger.info("Searching for the server on the local network...");
		BluetoothConnection::sendTrackerDiscovery();
		ledManager.on();
	} else if (m_LastConnectionAttemptTimestamp + 20 < now) {
		ledManager.off();
	}
}

void BluetoothConnection::reset() {
	m_Connected = false;
	std::fill(m_AckedSensorState, m_AckedSensorState+MAX_IMU_COUNT, SensorStatus::SENSOR_OFFLINE);

	// m_UDP.begin(m_ServerPort);

	statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);
}

void BluetoothConnection::update() {
	std::vector<Sensor *> & sensors = sensorManager.getSensors();

	updateSensorState(sensors);
	maybeRequestFeatureFlags();

	if (!m_Connected) {
		searchForServer();
		return;
	}

	// if (m_LastPacketTimestamp + TIMEOUT < millis()) {
	// 	statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);

	// 	m_Connected = false;
	// 	std::fill(m_AckedSensorState, m_AckedSensorState+MAX_IMU_COUNT, SensorStatus::SENSOR_OFFLINE);
	// 	m_Logger.warn("Connection to server timed out");

	// 	return;
	// }

	if (!BleNetwork::isPacketAvailable()) {
		return;
	}

	m_LastPacketTimestamp = millis();

	int len = BleNetwork::getPacket(m_Packet, sizeof(m_Packet));

#ifdef DEBUG_NETWORK
	// m_Logger.trace(
	// 	"Received %d bytes from %s, port %d",
	// 	packetSize,
	// 	m_UDP.remoteIP().toString().c_str(),
	// 	m_UDP.remotePort()
	// );
	// m_Logger.traceArray("UDP packet contents: ", m_Packet, len);
#else
	// (void)packetSize;
#endif

	switch (convert_chars<int>(m_Packet)) {
		case PACKET_RECEIVE_HEARTBEAT:
			sendHeartbeat();
			break;

		case PACKET_RECEIVE_VIBRATE:
			break;

		case PACKET_RECEIVE_HANDSHAKE:
			// Assume handshake successful
			m_Logger.warn("Handshake received again, ignoring");
			break;

		case PACKET_RECEIVE_COMMAND:
			break;

		case PACKET_CONFIG:
			break;

		case PACKET_PING_PONG:
			returnLastPacket(len);
			break;

		case PACKET_SENSOR_INFO:
			if (len < 6) {
				m_Logger.warn("Wrong sensor info packet");
				break;
			}

			for (int i = 0; i < (int)sensors.size(); i++) {
				if (m_Packet[4] == sensors[i]->getSensorId()) {
					m_AckedSensorState[i] = (SensorStatus)m_Packet[5];
					break;
				}
			}

			break;

		case PACKET_FEATURE_FLAGS:
		{
			// Packet type (4) + Packet number (8) + flags (len - 12)
			if (len < 13) {
				m_Logger.warn("Invalid feature flags packet: too short");
				break;
			}

			bool hadFlags = m_ServerFeatures.isAvailable();
			
			uint32_t flagsLength = len - 12;
			m_ServerFeatures = ServerFeatures::from(&m_Packet[12], flagsLength);

			if (!hadFlags) {
				#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
					if (m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
						m_Logger.debug("Server supports packet bundling");
					}
				#endif
			}

			break;
		}
	}
}

}  // namespace Network
}  // namespace SlimeVR

#endif