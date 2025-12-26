#!/bin/bash

g++ -std=c++17 -O2 -Wall -Wextra raw_still.cpp -o rawstill.bin -DBUILD_TEST_MAIN $(pkg-config --cflags --libs libcamera) -pthread
