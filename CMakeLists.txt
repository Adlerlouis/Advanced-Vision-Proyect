cmake_minimum_required(VERSION 2.8)
project( Prueba )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( Prueba Prueba.cpp)
target_link_libraries(Prueba wiringPi)
target_link_libraries( Prueba ${OpenCV_LIBS} )

