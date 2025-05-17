cmake_minimum_required(VERSION 3.30)
project(quadcopter_flight_control C)

set(CMAKE_C_STANDARD 11)

add_executable(quadcopter_flight_control quadcopter_control.c)
