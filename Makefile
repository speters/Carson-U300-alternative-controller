#BOARD:=mega2560
BOARD:=nano328
BOARD_TAG:=nano328
#pro5v328
MONITOR_PORT:=/dev/ttyUSB0
CXXFLAGS+=-D__STDC_LIMIT_MACROS

# from: http://ed.am/dev/make/arduino-mk
include /usr/share/arduino/Arduino.mk
