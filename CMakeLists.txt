cmake_minimum_required(VERSION 3.8)
project(parser_writer)

include_directories(include)

add_executable(bag_analyzer src/bag_analyzer.cpp)

add_executable(message_analyzer src/message_analyzer.cpp)
#add_executable(parser_writer src/parser_writer.cpp)

add_executable(parser_generator src/parser_generator.cpp src/main.cpp)