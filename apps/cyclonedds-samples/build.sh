#!/bin/bash

rm -rf ./build
mkdir -p build

cmake  . -B build
cmake --build build