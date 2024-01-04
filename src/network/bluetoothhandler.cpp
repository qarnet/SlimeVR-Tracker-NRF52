/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#include "bluetoothhandler.h"
#include "globals.h"
#include "logging/Logger.h"
#include "GlobalVars.h"

#define ADV_TIMEOUT   0 // seconds. Set this higher to automatically stop advertising after a time

unsigned long lastBleReportTime = 0;
unsigned long bleConnectionTimeout = millis();
bool isBleConnected = false;
uint8_t bleState = SLIME_BLE_NOT_SETUP;
bool hadBle = false;
unsigned long last_rssi_sample = 0;

volatile bool subscribed = false;

// TODO: Cleanup with proper classes
SlimeVR::Logging::Logger bleHandlerLogger("BleHandler");

const uint8_t UUID128_SRV_CONN[16] =
{
	0x08, 0x26, 0xf0, 0x44, 0x14, 0xbb, 0x4e, 0x01,
	0xa8, 0xcf, 0xd7, 0x4b, 0xfc, 0xba, 0x7a, 0x67
};

const uint8_t UUID128_CHR_CONN[16] =
{
	0x9d, 0x7c, 0xae, 0xaa, 0x18, 0x81, 0x4b, 0x14,
	0x9f, 0xca, 0xda, 0xd1, 0x9d, 0xaa, 0xd1, 0x6f
};

BLEService connection_service = BLEService(UUID128_SRV_CONN);
BLECharacteristic connection_characteristic = BLECharacteristic(UUID128_CHR_CONN);

void adv_stop_callback(void)
{
	Serial.println("Advertising time passed, advertising will now stop.");
}

void connect_callback(uint16_t conn_handle)
{
	// Get the reference to current connection
	BLEConnection* connection = Bluefruit.Connection(conn_handle);

	char central_name[32] = { 0 };
	connection->getPeerName(central_name, sizeof(central_name));

	Serial.print("Connected to ");
	Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
	(void) conn_handle;
	(void) reason;

	Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
	Serial.println("Advertising!");

	subscribed = false;
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == connection_characteristic.uuid) {
        if (chr->indicateEnabled(conn_hdl)) {
            Serial.println("Temperature Measurement 'Indicate' enabled");
			subscribed = true;
        } else {
            Serial.println("Temperature Measurement 'Indicate' disabled");
			subscribed = false;
        }
    }
}

void setupConn(void)
{
	// Configure the Health Thermometer service
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.health_thermometer.xml
	// Supported Characteristics:
	// Name                         UUID    Requirement Properties
	// ---------------------------- ------  ----------- ----------
	// Temperature Measurement      0x2A1C  Mandatory   Indicate
	//
	// Temperature Type             0x2A1D  Optional    Read                  <-- Not used here
	// Intermediate Temperature     0x2A1E  Optional    Read, Notify          <-- Not used here
	// Measurement Interval         0x2A21  Optional    Read, Write, Indicate <-- Not used here
	connection_service.begin();

	// Note: You must call .begin() on the BLEService before calling .begin() on
	// any characteristic(s) within that service definition.. Calling .begin() on
	// a BLECharacteristic will cause it to be added to the last BLEService that
	// was 'begin()'ed!

	// Configure the Temperature Measurement characteristic
	// See:https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.temperature_measurement.xml
	// Properties = Indicte
	// Min Len    = 6
	// Max Len    = 6
	//    B0      = UINT8  - Flag (MANDATORY)
	//      b3:7  = Reserved
	//      b2    = Temperature Type Flag (0 = Not present, 1 = Present)
	//      b1    = Timestamp Flag (0 = Not present, 1 = Present)
	//      b0    = Unit Flag (0 = Celsius, 1 = Fahrenheit)
	//    B4:1    = FLOAT  - IEEE-11073 32-bit FLOAT measurement value
	//    B5      = Temperature Type
	connection_characteristic.setProperties(CHR_PROPS_INDICATE | CHR_PROPS_NOTIFY);
	connection_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	// connection_characteristic.setFixedLen(6);
	connection_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
	connection_characteristic.begin();
	uint8_t htmdata[6] = { 0b00000101, 0, 0 ,0 ,0, 2 }; // Set the characteristic to use Fahrenheit, with type (body) but no timestamp field
	connection_characteristic.write(htmdata, sizeof(htmdata));                    // Use .write for init data

	// Temperature Type Value
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.temperature_type.xml
	//    B0      = UINT8 - Temperature Type
	//      0     = Reserved
	//      1     = Armpit
	//      2     = Body (general)
	//      3     = Ear (usually ear lobe)
	//      4     = Finger
	//      5     = Gastro-intestinal Tract
	//      6     = Mouth
	//      7     = Rectum
	//      8     = Toe
	//      9     = Tympanum (ear drum)
	//     10:255 = Reserved
}

void startAdv()
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

bool BleNetwork::indicate(const void *data, uint16_t length)
{
	connection_characteristic.indicate(data, length);
}

void reportBleError() {
    if(lastBleReportTime + 1000 < millis()) {
        lastBleReportTime = millis();
        Serial.print(".");
    }
}

void setStaticIPIfDefined() {
    #ifdef WIFI_USE_STATICIP
    const IPAddress ip(WIFI_STATIC_IP);
    const IPAddress gateway(WIFI_STATIC_GATEWAY);
    const IPAddress subnet(WIFI_STATIC_SUBNET);
    Ble.config(ip, gateway, subnet);
    #endif
}

bool BleNetwork::isConnected() {
    return isBleConnected;
}

void BleNetwork::setBleCredentials(const char * SSID, const char * pass) {
    // stopProvisioning();
    setStaticIPIfDefined();
    // Ble.begin(SSID, pass);
    // Reset state, will get back into provisioning if can't connect
    hadBle = false;
    bleState = SLIME_BLE_SERVER_CRED_ATTEMPT;
    bleConnectionTimeout = millis();
}

// IPAddress BleNetwork::getAddress() {
//     return Ble.localIP();
// }

void BleNetwork::setUp() {
    Bluefruit.begin();

	Bluefruit.Periph.setConnectCallback(connect_callback);
	Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

	Bluefruit.setName("SlimeVR");

	setupConn();

	startAdv();

	while(!subscribed)
	{
		delay(10);
	}

	Serial.println("Done");
    bleHandlerLogger.info("Setting up Ble");
    // Ble.hostname("SlimeVR FBT Tracker");
    // bleHandlerLogger.info("Loaded credentials for SSID %s and pass length %d", Ble.SSID().c_str(), Ble.psk().length());
    setStaticIPIfDefined();
    // wl_status_t status = Ble.begin(); // Should connect to last used access point, see https://arduino-esp8266.readthedocs.io/en/latest/esp8266ble/station-class.html#begin
    // bleHandlerLogger.debug("Status: %d", status);
    bleState = SLIME_BLE_SAVED_ATTEMPT;
    bleConnectionTimeout = millis();

#if POWERSAVING_MODE == POWER_SAVING_NONE
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE || POWERSAVING_MODE == POWER_SAVING_MAXIMUM
#endif
}

void onConnected() {
    // BleNetwork::stopProvisioning();
    statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, false);
    isBleConnected = true;
    hadBle = true;
    // bleHandlerLogger.info("Connected successfully to SSID '%s', ip address %s", Ble.SSID().c_str(), Ble.localIP().toString().c_str());
}

uint8_t BleNetwork::getBleState() {
    return bleState;
}

void BleNetwork::upkeep() {
    // upkeepProvisioning();
    if(true) { // BLE Status
        if(isBleConnected) {
            bleHandlerLogger.warn("Connection to Ble lost, reconnecting...");
            isBleConnected = false;
        }
        statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);
        reportBleError();
        if(bleConnectionTimeout + 11000 < millis()) {
            switch(bleState) {
                case SLIME_BLE_NOT_SETUP: // Wasn't set up
                return;
                case SLIME_BLE_SAVED_ATTEMPT: // Couldn't connect with first set of credentials
                    bleState = SLIME_BLE_SAVED_G_ATTEMPT;
                return;
                case SLIME_BLE_SAVED_G_ATTEMPT: // Couldn't connect with first set of credentials with PHY Mode G
                    bleState = SLIME_BLE_HARDCODE_ATTEMPT;
                return;
                case SLIME_BLE_HARDCODE_ATTEMPT: // Couldn't connect with second set of credentials
                    bleState = SLIME_BLE_HARDCODE_G_ATTEMPT;
                return;
                case SLIME_BLE_SERVER_CRED_ATTEMPT: // Couldn't connect with server-sent credentials.
                return;
                case SLIME_BLE_HARDCODE_G_ATTEMPT: // Couldn't connect with second set of credentials with PHY Mode G.
                case SLIME_BLE_SERVER_CRED_G_ATTEMPT: // Or if couldn't connect with server-sent credentials
                return;
            }
        }
        return;
    }
    if(!isBleConnected) {
        onConnected();
        return;
    } else {
        if(millis() - last_rssi_sample >= 2000) {
            last_rssi_sample = millis();
            // uint8_t signalStrength = Ble.RSSI();
            // networkConnection->sendSignalStrength(signalStrength);
        }
    }
    return;
}

#endif