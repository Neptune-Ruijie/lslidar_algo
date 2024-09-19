#include "node_ship_gauge.hpp"
#include "std_msgs/msg/string.h"
#include <iostream>
#include "rclcpp/rclcpp.hpp"

using namespace ship_gauge;
// volatile sig_atomic_t flag = 1;

// static void my_handler([[maybe_unused]] int sig) {
//     // std::cout << "Signal received, ending process." << std::endl;
//     flag = 0;
// }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // signal(SIGINT, my_handler);
    auto node = std::make_shared<ship_gauge::NodeShipGauge>("ship_gauge");
    // if (!node->initialize()) {
    //     // LS_ERROR << "cannot initialize lslidar driver." << LS_END;
    //     return 0;
    // }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
