cmake_minimum_required(VERSION 3.16)
project(ethercat2 VERSION 0.1.0 LANGUAGES C CXX)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
 

add_executable(main src/main.cpp src/utils.cpp)

target_include_directories(main PRIVATE $ENV{HOME}/RPi-dev/staging-aarch64-rpi3/include)

target_link_directories(main PRIVATE $ENV{HOME}/RPi-dev/staging-aarch64-rpi3/lib)
target_link_libraries(main soem)
# install(TARGETS ethercat2
#         RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})