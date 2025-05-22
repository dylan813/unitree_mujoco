#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "motor_crc.h"
#include "unitree_go/msg/wireless_controller.hpp"
#include <algorithm>

// Create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node
{
public:
    low_level_cmd_sender() : Node("low_level_cmd_sender")
    {
        // the cmd_puber is set to subscribe "/lowcmd" topic
        cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        // The timer is set to 200Hz, and bind to low_level_cmd_sender::timer_callback function
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&low_level_cmd_sender::timer_callback, this));

        // Initialize lowcmd
        init_cmd();

        // Subscribe to joystick controller
        wireless_controller_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10, std::bind(&low_level_cmd_sender::wireless_controller_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /wirelesscontroller for joystick input.");        
    }

private:
    void timer_callback()
    {
        runing_time += dt;
        if (runing_time < 3.0)
        {
            // Stand up in first 3 second

            // Total time for standing up or standing down is about 1.2s
            phase = tanh(runing_time / 1.2);
            for (int i = 0; i < 12; i++)
            {
                low_cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                low_cmd.motor_cmd[i].dq = 0;
                low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
                low_cmd.motor_cmd[i].kd = 3.5;
                low_cmd.motor_cmd[i].tau = 0;
            }
        }
        else
        {
            phase = tanh((runing_time - 3.0) / 1.2);
            for (int i = 0; i < 12; i++)
            {
                low_cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
                low_cmd.motor_cmd[i].dq = 0;
                low_cmd.motor_cmd[i].kp = 50;
                low_cmd.motor_cmd[i].kd = 3.5;
                low_cmd.motor_cmd[i].tau = 0;
            }

            const double max_wheel_speed = 15.0; // rad/s
            const double turning_sensitivity = 10.0;

            double forward_speed = joystick_ly_ * max_wheel_speed;
            double turning_speed = joystick_rx_ * max_wheel_speed * turning_sensitivity;

            double right_wheel_speed = forward_speed - turning_speed;
            double left_wheel_speed = forward_speed + turning_speed;

            right_wheel_speed = std::max(-max_wheel_speed, std::min(right_wheel_speed, max_wheel_speed));
            left_wheel_speed = std::max(-max_wheel_speed, std::min(left_wheel_speed, max_wheel_speed));

            // Right wheels: 12 (FR), 14 (BR)
            // Left wheels:  13 (FL), 15 (BL)
            const int right_wheel_motor_indices[] = {12, 14};
            const int left_wheel_motor_indices[] = {13, 15};

            for (int wheel_idx : right_wheel_motor_indices)
            {
                low_cmd.motor_cmd[wheel_idx].mode = 0x01;    // Active mode
                low_cmd.motor_cmd[wheel_idx].q = PosStopF; // Position target
                low_cmd.motor_cmd[wheel_idx].dq = right_wheel_speed;
                low_cmd.motor_cmd[wheel_idx].kp = 0.0;     // No P-gain for velocity control
                low_cmd.motor_cmd[wheel_idx].kd = 3.0;     // D-gain for velocity damping
                low_cmd.motor_cmd[wheel_idx].tau = 0.0;    // Torque command
            }

            for (int wheel_idx : left_wheel_motor_indices)
            {
                low_cmd.motor_cmd[wheel_idx].mode = 0x01;    // Active mode
                low_cmd.motor_cmd[wheel_idx].q = PosStopF; // Position target
                low_cmd.motor_cmd[wheel_idx].dq = left_wheel_speed;
                low_cmd.motor_cmd[wheel_idx].kp = 0.0;     // No P-gain for velocity control
                low_cmd.motor_cmd[wheel_idx].kd = 3.0;     // D-gain for velocity damping
                low_cmd.motor_cmd[wheel_idx].tau = 0.0;    // Torque command
            }
        }

        get_crc(low_cmd);            // Check motor cmd crc
        cmd_puber->publish(low_cmd); // Publish lowcmd message
    }

    void wireless_controller_callback(const unitree_go::msg::WirelessController::SharedPtr msg)
    {
        joystick_ly_ = msg->ly; 
        joystick_rx_ = msg->rx;
        RCLCPP_INFO(this->get_logger(), "LY = %f, RX = %f", joystick_ly_, joystick_rx_);
    }

    void init_cmd()
    {

        for (int i = 0; i < 20; i++)
        {
            low_cmd.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
            low_cmd.motor_cmd[i].q = PosStopF;
            low_cmd.motor_cmd[i].kp = 0;
            low_cmd.motor_cmd[i].dq = VelStopF;
            low_cmd.motor_cmd[i].kd = 0;
            low_cmd.motor_cmd[i].tau = 0;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;                             // ROS2 timer
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber; // ROS2 Publisher
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_sub_;

    unitree_go::msg::LowCmd low_cmd;
    double joystick_ly_ = 0.0;
    double joystick_rx_ = 0.0;
    
    double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                                     0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
                                       1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;
};

int main(int argc, char **argv)
{   
    std::cout << "Press enter to start";
    std::cin.get();
    
    rclcpp::init(argc, argv);                             // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_;                  // Create a timer callback object to send cmd in time intervals
    auto node = std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and make share with low_level_cmd_sender class
    rclcpp::spin(node);                                   // Run ROS2 node
    rclcpp::shutdown();                                   // Exit
    return 0;
}
