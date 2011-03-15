set(ARDUINO_PROTOCOL "stk500v1")
set(ARDUINO_BOARD "atmega328p")
set(ARDUINO_MCU "m328p")
set(ARDUINO_FCPU "16000000")
set(ARDUINO_UPLOAD_SPEED "115200")
set(ARDUINO_PORT "/dev/ttyACM0")

include(${CMAKE_SOURCE_DIR}/cmake_scripts/arduino.cmake)
