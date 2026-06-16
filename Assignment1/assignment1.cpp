#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

// We create a class that inherits from the standard ROS 2 Node
class AudibotStraightLine : public rclcpp::Node 
{
public:
    // Constructor: This runs once when the object is created
    AudibotStraightLine() : Node("audibot_straight_line") 
    {
        // Initialize publishers using template arguments <Type>
        gear_pub_ = this->create_publisher<std_msgs::msg::UInt32>("/audibot/gear_cmd", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float64>("/audibot/steering_cmd", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/audibot/speed_cmd", 10);

        // 10 Hz wall timer (100ms interval) linked to our callback function
        timer_ = this->create_wall_timer(100ms, std::bind(&AudibotStraightLine::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Audibot straight line C++ node initialized. Publishing at 10Hz...");
    }

private:
    // This function runs automatically every 100ms
    void timer_callback() 
    {
        // 1. Gear Command (0 = Drive)
        auto gear_msg = std_msgs::msg::UInt32();
        gear_msg.data = 0;

        // 2. Steering Command (0.0 = Dead center)
        auto steering_msg = std_msgs::msg::Float64();
        steering_msg.data = 0.0;

        // 3. Speed Command (Target velocity: 5.0 m/s)
        auto speed_msg = std_msgs::msg::Float64();
        speed_msg.data = 5.0;

        // Publish all three messages
        gear_pub_->publish(gear_msg);
        steering_pub_->publish(steering_msg);
        speed_pub_->publish(speed_msg);
    }

    // Class Member Variables (equivalent to Python's self.variable)
    // SharedPtr is ROS 2's way of managing memory safely without raw pointers
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr gear_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// The absolute entry point of the program
int main(int argc, char * argv[]) 
{
    // Boot up the ROS 2 C++ client communications
    rclcpp::init(argc, argv);
    
    // Create the node object and keep it alive (spinning)
    rclcpp::spin(std::make_shared<AudibotStraightLine>());
    
    // Clean up when the program is killed (Ctrl+C)
    rclcpp::shutdown();
    return 0;
}