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
#ifndef SLIMEVR_BLUETOOTH_HANDLER_H_
#define SLIMEVR_BLUETOOTH_HANDLER_H_

#if defined(XIAO_NRF52840)

#include <Arduino.h>

namespace BleNetwork {
    bool isConnected();
    void setUp();
    void upkeep();
    void setBleCredentials(const char * SSID, const char * pass);
    // IPAddress getAddress();
    uint8_t getBleState();
    bool indicate(const void *data, uint16_t length);
}

/** Wifi Reconnection Statuses **/
typedef enum {
    SLIME_BLE_NOT_SETUP = 0,
    SLIME_BLE_SAVED_ATTEMPT,
    SLIME_BLE_SAVED_G_ATTEMPT,
    SLIME_BLE_HARDCODE_ATTEMPT,
    SLIME_BLE_HARDCODE_G_ATTEMPT,
    SLIME_BLE_SERVER_CRED_ATTEMPT,
    SLIME_BLE_SERVER_CRED_G_ATTEMPT
} ble_reconnection_statuses;

#endif
#endif // SLIMEVR_WIFI_H_
