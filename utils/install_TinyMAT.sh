#!/bin/bash
git clone git@github.com:jkriege2/TinyMAT.git
cd TinyMAT
mkdir build
cd build
cmake ..
cmake --build . --config "Debug"
sudo cmake --build . --config "Debug" --target install
cd ../../
rm -rf TinyMAT