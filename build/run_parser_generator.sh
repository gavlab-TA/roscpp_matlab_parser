#!/bin/bash
cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build .
make
./bag_analyzer
source ../output/bag_reader_ws/install/setup.bash
mkdir ../msg_data/
./message_analyzer
./parser_generator
#./matlab_parser_generator