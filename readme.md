# ROS2 MATLAB Bag Parser Generator
This package generates a C++ parser for writing data from ROS2 Bags to .mat files for use in MATLAB. The generator is able to operate given the ROS2 bag to be parsed and a list of custom message packages and paths to each. 

## Configure the Generator
---
There are 3 files that need to be configured in order to run the generator. All three are located in the config/ directory. 

1. path.txt

    Give the absolute path to the bagfile with the bagfile name. 
        
        example: /home/user/Documents/<bagfile>
    Note: the path needs to be given to the ROS bag directory containing both the .db3 and metadata file, not the .db3 file itself.
    
    **Important: ensure that the path given in path.txt does not include a trailing '/'.**

2. dependencies.txt
    
    List the dependencies for the parser. These include rclcpp, TinyMAT, and ros2bag_cpp, along with all message packages for the messages in the bag (including native messages like std_msgs, geometry_msgs, etc.).

3. dependency_paths.txt

    List the absolute paths to custom dependencies. This is not needed for messages and packages installed with ROS, but is needed for TinyMAT and any custom message packages.

        example: /home/user/stable/TinyMAT

**In addition to these 3 config files, if the libraries for TinyMAT are not installed on the computer, you need to run the install bash script located in utils.**

```
./utils/install_TinyMAT.sh
```
## Run Parser Generator
---
There is a bash script for building and running the parser generator located in utils. The bash script will create a directory called "output" where it will create a workspace ready for the parser to be built and run from. This workspace contains links to all custom message packages and is where the generator recieves information on the message structures contained in the bag. **The following bash script needs to run from inside the utils/ directory.**

```
cd utils/
./run_parser_generator.sh
```

## Build and Run Parser
---
The generated parser package will be located in the src/ directory of the workspace that was built in the output/ directory of this package. It is possible to move or copy this package to any location and build the parser as part of a ROS2 workspace as long as the path absolute path to the bag does not change. The path to the bag can be ammended by editing line 5 of src/bag_reader.cpp if moving the bag is required after generation. It is recommended to build the parser package (called bag_reader) as part of the workspace that was generated in output/ (release version compiling is recommended).
```
cd output/bag_reader_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
After building the workspace, the parser can be ran using the following commands.
```
source install/setup.bash
ros2 run bag_reader bag_reader
```
The .mat files that are generated will be placed in the same directory as the original bagfile.

## Array Handling in MATLAB
---
Any fields from the ROS bag that are, or are part of arrays have had their names changed to message_name_field_array. The array in matlab is a 1-D array of 2-D data. The first element of each set of data is the size of the array that was present in that message. A representation of 3 messages containing 2 item arrays each is shown below:
            
            [2 1 2 2 3 4 2 5 6] = [1 2], [3 4], [5 6]
A more irregular example is shown here:

            [3 1 2 3 2 4 5 4 6 7 8 9] = [1 2 3], [4 5], [6 7 8 9]

---
## Current Development on this Package
---
- [x] The package currently successfully parses all numerical values from bag files 

- [x] Any empty topics are removed from the data to speed up process and cleanup output.

- [ ] Only 1-D arrays are currently supported

- [ ] String fields are currently not supported

- [ ] Automation scripts can be improved to allow for options to use existing workspaces for the message data instead of needed to build a local workspace
