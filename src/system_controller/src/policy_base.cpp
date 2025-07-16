#include "../include/policy_base.hpp"
#include <sstream>

namespace system_controller {

std::string Policy1::getCommand() {
    static int counter = 0;
    counter++;
    
    std::stringstream ss;
    ss << "move_forward:speed=" << (0.5 + (counter % 5) * 0.1);
    return ss.str();
}

std::string Policy2::getCommand() {
    static int counter = 0;
    counter++;
    
    std::stringstream ss;
    if (counter % 3 == 0) {
        ss << "turn_right:angle=45";
    } else if (counter % 3 == 1) {
        ss << "move_forward:speed=0.8";
    } else {
        ss << "turn_left:angle=30";
    }
    return ss.str();
}

std::string Policy3::getCommand() {
    static int counter = 0;
    counter++;
    
    std::stringstream ss;
    switch (counter % 4) {
        case 0: ss << "hover:altitude=5.0"; break;
        case 1: ss << "move_to_waypoint:x=10,y=20"; break;
        case 2: ss << "scan_area:radius=50"; break;
        case 3: ss << "return_to_base"; break;
    }
    return ss.str();
}

} // namespace system_controller 