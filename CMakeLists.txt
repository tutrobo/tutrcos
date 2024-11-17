cmake_minimum_required(VERSION 3.22)

project(tutrcos LANGUAGES CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_sources(${PROJECT_NAME} INTERFACE
  src/core/thread.cpp
  src/module/microros.cpp
  src/peripheral/can.cpp
  src/peripheral/fdcan.cpp
  src/peripheral/gpio.cpp
  src/peripheral/uart.cpp
  cobs-c/cobs.c
)

target_include_directories(${PROJECT_NAME} INTERFACE
  include
  cobs-c
)

target_link_libraries(${PROJECT_NAME} INTERFACE "-u _printf_float")

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_OUTPUT_LANGUAGE Japanese)
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
  doxygen_add_docs(docs
    README.md
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()
