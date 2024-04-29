#pragma once

#include<iostream>
#include<string>
#include<map>

#include <ariac_msgs/msg/part.hpp>

// #include <

namespace utils{
    std::map<std::string, float> OFFSETS{
        {"tray", 0.2},
        {"drop", 0.002},
        {"pick", 0.003},
        {"part", 0.2},
    };
        
    std::map<int, double> PART_HEIGHTS
        = { { ariac_msgs::msg::Part::BATTERY, 0.04 },
            { ariac_msgs::msg::Part::PUMP, 0.12 },
            { ariac_msgs::msg::Part::REGULATOR, 0.07 },
            { ariac_msgs::msg::Part::SENSOR,0.07}};
    // enum PICKPLACE()
}