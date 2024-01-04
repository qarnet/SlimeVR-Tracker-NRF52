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

#include <bluefruit.h>

#include "bluetoothmanager.h"
#include "GlobalVars.h"

#define ADV_TIMEOUT   0 // seconds. Set this higher to automatically stop advertising after a time

namespace SlimeVR {
namespace Network {

void adv_stop_callback(void)
{
	Serial.println("Advertising time passed, advertising will now stop.");
}

void BluetoothManager::startAdv()
	{
	// Advertising packet
	Bluefruit.Advertising.clearData();
	Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
	Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);

	Bluefruit.ScanResponse.addName();

	/* Start Advertising
	* - Enable auto advertising if disconnected
	* - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
	* - Timeout for fast mode is 30 seconds
	* - Start(timeout) with timeout = 0 will advertise forever (until connected)
	* 
	* For recommended advertising interval
	* https://developer.apple.com/library/content/qa/qa1931/_index.html
	*/
	Bluefruit.Advertising.setStopCallback(adv_stop_callback);
	Bluefruit.Advertising.restartOnDisconnect(true);
	Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
	Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
	Bluefruit.Advertising.start(ADV_TIMEOUT);      // Stop advertising entirely after ADV_TIMEOUT seconds 
}

void BluetoothManager::setup() {
	Bluefruit.begin();
	Bluefruit.setName("SlimeVR");

	this->startAdv();
}

void BluetoothManager::update() {

	auto wasConnected = m_IsConnected;

	if (!m_IsConnected) {
		return;
	}

	if (!wasConnected) {
		// WiFi was reconnected, rediscover the server and reconnect
		networkConnection->reset();
	}

	networkConnection->update();
}

}  // namespace Network
}  // namespace SlimeVR

#endif