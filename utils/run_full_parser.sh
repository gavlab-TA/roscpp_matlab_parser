./run_parser_generator.sh
cd ../output/bag_reader_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run bag_reader bag_reader