# -----------------------------------------------------------------------------------
# Copyright 2025, Gilles Zunino
# -----------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.30)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(MINIMAL_BUILD ON)
project(esp32-smaterbutton)
