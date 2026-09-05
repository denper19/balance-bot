cmake_minimum_required(VERSION 3.22)
project(bot)

set(SRC
	main.cpp
	bot.cpp
)

set(HEADER
	bot.h
)

add_executable(bot ${SRC} ${HEADER})

find_library(WIRINGPI_LIB NAMES wiringPi REQUIRED)

target_link_libraries(bot PRIVATE ${WIRINGPI_LIB})
