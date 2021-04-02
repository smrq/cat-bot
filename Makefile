BOARD_TAG = teensy31
BOARD_SUB = 96
ARDUINO_LIBS = ServoEx EEPROM
USER_LIB_PATH += lib
MONITOR_PORT = /dev/cu.usb*
include /usr/local/opt/arduino-mk/Teensy.mk

cleanusbserial:
	networksetup -listallnetworkservices | grep "USB Serial" | while read line; do networksetup -deletepppoeservice "$$line"; done