#include "fast_lio_sam_sc_qn.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FastLioSamScQn>(rclcpp::NodeOptions());
    node->initializeSubscribers();
    // Use multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();

    return 0;
}
