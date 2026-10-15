# The firmware of a one-axis tuushin device on an ESP32, ESP32-S3 or
# ESP32-C3 (ESP-IDF 5.x): the servo loop in firmware/core on the board's
# PWM, encoder and host link. See README.md.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tuushin_device)
