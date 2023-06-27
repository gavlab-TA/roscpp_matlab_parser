#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>

std::vector<std::string> split(std::string s, char delim)
{
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (getline(ss, item, delim))
    {
        result.push_back(item);
    }

    return result;
}

std::string camelToSnake(std::string str)
{

    // Empty String
    std::string result = "";

    // Append first character(in lower case)
    // to result string
    char c = tolower(str[0]);
    result += (char(c));

    // Traverse the string from
    // ist index to last index
    for (int i = 1; i < str.length(); i++)
    {

        char ch = str[i];

        // Check if the character is upper case
        // then append '_' and such character
        // (in lower case) to result string
        if (isupper(ch))
        {
            result = result + '_';
            result += char(tolower(ch));
        }

        // If the character is lower case then
        // add such character into result string
        else
        {
            result = result + ch;
        }
    }

    // return the result
    return result;
}

bool checkAllUpper(std::string input)
{
    bool all_upper = true;
    for (int i = 0; i < input.length(); i++)
    {
        if (islower(input[i]))
        {
            all_upper = false;
            return all_upper;
        }
    }

    return all_upper;
}

std::string formatMessageTypes(std::string message_type)
{
    std::vector<std::string> data = split(message_type, '/');
    if (checkAllUpper(data.at(2)))
    {
        for (int i = 0; i < data.at(2).length(); i++)
        {
            data.at(2)[i] = tolower(data.at(2)[i]);
        }
    }

    std::string message_name = data.at(2);

    for (int i = 0; i < data.at(2).length() - 1; i++)
    {
        if (isupper(message_name[i]) && isupper(message_name[i + 1]))
        {
            message_name[i] = tolower(message_name[i]);
        }
    }
    message_name[0] = tolower(message_name[0]);
    message_name = camelToSnake(message_name);
    return data.at(0) + "/" + data.at(1) + "/" + message_name;
}

void setupPackageDepends(std::string depends_path_filename, std::string source_path)
{
    std::string temp;
    std::ifstream file;
    std::string command;

    int system_status = system("rm -r ../output/");

    command = "mkdir -p " + source_path;
    system_status = system(command.c_str());
    if (system_status != 0)
    {
        std::cout << "Error creating src file" << std::endl;
    }

    file.open(depends_path_filename);
    while (!file.eof())
    {
        getline(file, temp);
        if (temp != "")
        {
            command = "ln -s " + temp + " " + source_path;
            system_status = system(command.c_str());
            if (system_status != 0)
            {
                std::cout << "Error linking Dependencies" << std::endl;
            }
        }
    }
}

void buildWorkspace(std::string package_name)
{
    std::string command = "cd ../output/" + package_name + "_ws && colcon build";
    int system_status = system(command.c_str());
    if (system_status != 0)
    {
        std::cout << "Error running build" << std::endl;
    }

    system_status = system("pwd");
    if (system_status != 0)
    {
        std::cout << "Error reporting location";
    }
}

bool checkForString(std::vector<std::string> data, std::string str)
{
    for (int i = 0; i < data.size(); i++)
    {
        if (data.at(i) == str)
        {
            return true;
        }
    }

    return false;
}

int main()
{

    int system_status;

    std::ifstream path_file;
    path_file.open("../config/path.txt");
    std::string bag_file_path;
    getline(path_file, bag_file_path);
    path_file.close();

    std::cout << bag_file_path << std::endl;
    system_status = system("touch ../config/bag_info.txt");
    if (system_status != 0)
    {
        std::cout << "Error creating bag_info.txt" << std::endl;
    }

    std::string analyze_command = "ros2 bag info " + bag_file_path + " > ../config/bag_info.txt";
    system_status = system(analyze_command.c_str());
    if (system_status != 0)
    {
        std::cout << "Error pulling bag data" << std::endl;
    }

    std::string temp;
    std::ifstream info_file;
    info_file.open("../config/bag_info.txt");

    std::ofstream topic_data_file;
    std::string topic_data_filename = "../config/topic_data.txt";
    std::string create_data_file_command = "touch " + topic_data_filename;
    system_status = system(create_data_file_command.c_str());
    if (system_status != 0)
    {
        std::cout << "Error creating topic_data.txt" << std::endl;
    }

    std::ofstream topic_name_file;
    std::string topic_name_filename = "../config/topic_names.txt";
    std::string create_topic_name_file_commmand = "touch " + topic_name_filename;
    system_status = system(create_topic_name_file_commmand.c_str());
    if (system_status != 0)
    {
        std::cout << "Error creating topic_names.txt" << std::endl;
    }

    std::ofstream message_type_file;
    std::string message_type_filename = "../config/message_types.txt";
    std::string create_message_type_file_command = "touch " + message_type_filename;
    system_status = system(create_message_type_file_command.c_str());
    if (system_status != 0)
    {
        std::cout << "Error creating message_types.txt" << std::endl;
    }

    std::ofstream message_name_file;
    std::string message_name_filename = "../config/message_names.txt";
    std::string create_message_names_file_command = "touch " + message_name_filename;
    system_status = system(create_message_names_file_command.c_str());
    if (system_status != 0)
    {
        std::cout << "Error creating message_names.txt" << std::endl;
    }

    topic_data_file.open(topic_data_filename);
    topic_name_file.open(topic_name_filename);
    message_type_file.open(message_type_filename);
    message_name_file.open(message_name_filename);

    getline(info_file, temp);
    getline(info_file, temp);
    getline(info_file, temp);
    getline(info_file, temp);
    getline(info_file, temp);
    getline(info_file, temp);
    getline(info_file, temp);
    getline(info_file, temp);

    std::vector<std::string> line_data;
    std::vector<std::string> topic_names;
    std::vector<std::string> message_types;
    std::vector<std::string> message_names;
    std::vector<std::string> topic_data;

    std::string raw_message_name;

    std::string start_parse_prefix = "Topic information";
    std::string prefix_check;
    bool ready = false;

    while (!info_file.eof())
    {
        getline(info_file, temp);

        if (!ready)
        {
            prefix_check = temp.substr(0, start_parse_prefix.length());
            if (prefix_check == start_parse_prefix)
            {
                ready = true;
            }
        }
        else
        {
            temp.erase(0, 18);
            if (temp.length() != 0)
            {
                line_data.clear();
                line_data = split(temp, '|');
                std::string count = line_data.at(2).erase(0, 8);
                if (count.at(0) != '0')
                {

                    std::string name = line_data.at(0).erase(0, 8);
                    name.erase(name.length() - 1, name.length());
                    topic_names.push_back(name);
                    std::string type = line_data.at(1).erase(0, 7);
                    type.erase(type.length() - 1, type.length());
                    bool save_message = !checkForString(message_names, type);
                    raw_message_name = type;
                    if (save_message)
                    {
                        message_names.push_back(type);
                    }
                    type = formatMessageTypes(type);
                    if (save_message)
                    {
                        message_types.push_back(type);
                    }
                    topic_data.push_back(name + "#" + type + "#" + raw_message_name);
                }
            }
        }
    }

    for (int i = 0; i < topic_data.size(); i++)
    {
        topic_name_file << topic_names.at(i) + "\n";
        topic_data_file << topic_data.at(i) + "\n";
    }

    for (int i = 0; i < message_names.size(); i++)
    {
        message_type_file << message_types.at(i) + "\n";
        message_name_file << message_names.at(i) + "\n";
    }

    info_file.close();
    topic_name_file.close();
    topic_data_file.close();
    message_type_file.close();
    message_name_file.close();

    std::string depends_path_filename = "../config/dependency_paths.txt";
    std::string package_name = "bag_reader";
    std::string source_path = "../output/" + package_name + "_ws/src/";

    setupPackageDepends(depends_path_filename, source_path);
    buildWorkspace(package_name);

    return 0;
}