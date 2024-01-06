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
#ifndef SLIMEVR_NETWORK_CONNECTION_H_
#define SLIMEVR_NETWORK_CONNECTION_H_

#if defined(XIAO_NRF52840) //TODO (qarnet): Find a better way to guard this?

#include <Arduino.h>

#include "Iconnection.h"
#include "globals.h"
#include "quat.h"
#include "sensors/sensor.h"
#include "featureflags.h"
#include "bluetoothhandler.h"

namespace SlimeVR {
namespace Network {

class BluetoothConnection : public IConnection {
public:
	void searchForServer() override;
	void update() override;
	void reset() override;
	bool beginBundle() override;
private:
	bool beginPacket() override;
	bool endPacket() override;
	size_t write(const uint8_t *buffer, size_t size) override;
	size_t write(uint8_t byte) override;
	int getWriteError() override;
	// PACKET_HANDSHAKE 3
	void sendTrackerDiscovery() override;

	std::vector<uint8_t> ble_buf;
};

}  // namespace Network
}  // namespace SlimeVR

#endif
#endif  // SLIMEVR_NETWORK_BLUETOOTHCONNECTION_H_
