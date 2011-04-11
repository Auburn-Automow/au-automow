set(ARDUINO_PROTOCOL "arduino")
set(ARDUINO_BOARD "atmega328p")
set(ARDUINO_MCU "m328p")
set(ARDUINO_FCPU "16000000")
set(ARDUINO_UPLOAD_SPEED "57600")
set(ARDUINO_PORT "/dev/ttyUSB2")

include(${CMAKE_SOURCE_DIR}/cmake_scripts/arduino.cmake)
