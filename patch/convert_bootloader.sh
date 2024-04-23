#!/bin/sh

adafruit-nrfutil dfu serial -p /dev/ttyACM0 -b 115200 -pkg arcade_feather_nrf52840_express_bootloader-0.8.3_s140_6.1.1.zip -t 1200
