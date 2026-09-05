cmake_minimum_required(VERSION 3.22)
project(imu_test)

set(SRC
	imu.cpp	
)

set(HEADER

)

add_executable(imu_test ${SRC} ${HEADER})

find_library(WIRINGPI_LIB NAMES wiringPi REQUIRED)

target_link_libraries(imu_test PRIVATE ${WIRINGPI_LIB})
