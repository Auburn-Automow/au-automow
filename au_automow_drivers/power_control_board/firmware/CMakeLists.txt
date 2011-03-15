cmake_minimum_required(VERSION 2.6)

#set your project name
Project(PowerControlBoard)
set(ARDUINO_CORE_DIR  arduino)

#Include the arduino board specific cmake file
include(${PROJECT_SOURCE_DIR}/cmake_scripts/arduino_diecimila.cmake)
include_directories(${PROJECT_SOURCE_DIR})


file(GLOB AVR_ROS_SRC
    "avr_ros/*.cpp"
)


#Add of your own avr c++ files to this list
set(PROJECT_SRC
    main.cpp
    ${AVR_ROS_SRC}
    ${ARDUINO_SOURCE_FILES}
)

add_executable(firmware ${PROJECT_SRC})

#to compile, use make
#to program arduino on /dev/ttyUSB0, do make flash
#to check the program size use make size
