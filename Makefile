BOARD_TAG:nano=
MONITOR_PORT:=/dev/ttyUSB0
#ARDUINO_LIBS:=Servo EnableInterrupt

#ARCHITECTURE:=avr
ARDUINO_VERSION:=1.6.7
MONITOR_BAUDRATE:=115200
AVRDUDE_ARD_BAUDRATE:=57600
MCU:=atmega328p
F_CPU:=16000000
#CORE:=arduino
#VARIANT:=standard
#
ARDUINO_DIR:=$(HOME)/dev/arduino/Arduino/build/linux/work
CXXFLAGS+=-D__STDC_LIMIT_MACROS

#include /usr/share/arduino/Arduino.mk
include $(HOME)/dev/arduino/Arduino-Makefile/Arduino.mk
