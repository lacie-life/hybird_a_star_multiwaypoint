#!/bin/bash

rm -rf build
rm -rf cm
mkdir cm
cd cm
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
cd ..
./build/testresult

#python3 main.py
