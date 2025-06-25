#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "motor_crc.h"
#include "unitree_go/msg/wireless_controller.hpp"
#include <algorithm>
#include <vector>

class low_level_cmd_sender : public rclcpp::Node
{
public:
    low_level_cmd_sender() : Node("low_level_cmd_sender")
    {
        cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&low_level_cmd_sender::timer_callback, this));
        init_cmd();

        wireless_controller_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10, std::bind(&low_level_cmd_sender::wireless_controller_callback, this, std::placeholders::_1));

        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10, std::bind(&low_level_cmd_sender::lowstate_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /wirelesscontroller and /lowstate.");
    }

private:
    enum class RobotState {
        STAND_DOWN,
        TRANSITION_UP,
        STAND_UP,
        TRANSITION_DOWN
    };

    RobotState current_state = RobotState::STAND_DOWN;

    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        for (int i = 0; i < 20; ++i)
        {
            actual_joint_positions[i] = msg->motor_state[i].q;
            actual_joint_velocities[i] = msg->motor_state[i].dq;
        }
    }

    void timer_callback()
    {
        switch (current_state) {
            case RobotState::STAND_DOWN:
                if (stand_up) {
                    current_state = RobotState::TRANSITION_UP;
                    runing_time = 0.0;
                }
                break;

            case RobotState::TRANSITION_UP:
                if (runing_time < 1.2) {
                    runing_time += dt;
                    phase = runing_time / 1.2;
                    for (int i = 0; i < 12; i++) {
                        low_cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                        low_cmd.motor_cmd[i].dq = 0;
                        low_cmd.motor_cmd[i].kp = phase * 50 + (1 - phase) * 20;
                        low_cmd.motor_cmd[i].kd = 3.5;
                        low_cmd.motor_cmd[i].tau = 0;
                    }
                } else {
                    current_state = RobotState::STAND_UP;
                }
                break;

            case RobotState::STAND_UP:
                if (!stand_up) {
                    current_state = RobotState::TRANSITION_DOWN;
                    runing_time = 0.0;
                } else {
                    //gains for hip motors (0, 3, 6, 9)
                    float kp_hip = 80.0;
                    float ki_hip = 0.0;
                    float kd_hip = 12.0;
                    
                    //gains for thigh motors (1, 4, 7, 10)
                    float kp_thigh = 70.0;
                    float ki_thigh = 0.0;
                    float kd_thigh = 8.0;
                    
                    //gains for calf motors (2, 5, 8, 11)
                    float kp_calf = 45.0;
                    float ki_calf = 0.0;
                    float kd_calf = 4.0;

                    for (int i = 0; i < 12; i++) {
                        float desired_q = stand_up_joint_pos[i];
                        float actual_q = actual_joint_positions[i];
                        float error = desired_q - actual_q;
                        float d_error = (error - prev_error[i]) / dt;

                        integral_error[i] += error * dt;
                        
                        float kp, ki, kd;
                        if (i == 0 || i == 3 || i == 6 || i == 9) {
                            kp = kp_hip;
                            ki = ki_hip;
                            kd = kd_hip;
                        } else if (i == 1 || i == 4 || i == 7 || i == 10) {
                            kp = kp_thigh;
                            ki = ki_thigh;
                            kd = kd_thigh;
                        } else {
                            kp = kp_calf;
                            ki = ki_calf;
                            kd = kd_calf;
                        }
                        
                        float tau = kp * error + ki * integral_error[i] + kd * d_error;

                        low_cmd.motor_cmd[i].mode = 0x01;
                        low_cmd.motor_cmd[i].q = PosStopF;
                        low_cmd.motor_cmd[i].dq = VelStopF;
                        low_cmd.motor_cmd[i].kp = 0;
                        low_cmd.motor_cmd[i].kd = 0;
                        low_cmd.motor_cmd[i].tau = tau;

                        prev_error[i] = error;
                    }

                    const double max_wheel_speed = 15.0; // rad/s
                    double forward_speed = joystick_ly_ * max_wheel_speed;
                    double turning_speed = joystick_rx_ * max_wheel_speed;
                    double right_wheel_speed = forward_speed - turning_speed;
                    double left_wheel_speed = forward_speed + turning_speed;
                    right_wheel_speed = std::clamp(right_wheel_speed, -max_wheel_speed, max_wheel_speed);
                    left_wheel_speed = std::clamp(left_wheel_speed, -max_wheel_speed, max_wheel_speed);

                    const int right_wheel_motor_indices[] = {12, 14};
                    const int left_wheel_motor_indices[] = {13, 15};

                    for (int wheel_idx : right_wheel_motor_indices) {
                        low_cmd.motor_cmd[wheel_idx].mode = 0x01;
                        low_cmd.motor_cmd[wheel_idx].q = PosStopF;
                        low_cmd.motor_cmd[wheel_idx].dq = right_wheel_speed;
                        low_cmd.motor_cmd[wheel_idx].kp = 0;
                        low_cmd.motor_cmd[wheel_idx].kd = 0.5;
                        low_cmd.motor_cmd[wheel_idx].tau = 0;
                    }

                    for (int wheel_idx : left_wheel_motor_indices) {
                        low_cmd.motor_cmd[wheel_idx].mode = 0x01;
                        low_cmd.motor_cmd[wheel_idx].q = PosStopF;
                        low_cmd.motor_cmd[wheel_idx].dq = left_wheel_speed;
                        low_cmd.motor_cmd[wheel_idx].kp = 0;
                        low_cmd.motor_cmd[wheel_idx].kd = 0.5;
                        low_cmd.motor_cmd[wheel_idx].tau = 0;
                    }
                }
                break;

            case RobotState::TRANSITION_DOWN:
                if (runing_time < 2.4) {
                    runing_time += dt;
                    phase = runing_time / 2.4;
                    for (int i = 0; i < 12; i++) {
                        low_cmd.motor_cmd[i].q = (1 - phase) * stand_up_joint_pos[i] + phase * stand_down_joint_pos[i];
                        low_cmd.motor_cmd[i].dq = 0;
                        low_cmd.motor_cmd[i].kp = (1 - phase) * 50 + phase * 20;
                        low_cmd.motor_cmd[i].kd = 3.5;
                        low_cmd.motor_cmd[i].tau = 0;
                    }
                } else {
                    current_state = RobotState::STAND_DOWN;
                }
                break;
        }

        get_crc(low_cmd);
        cmd_puber->publish(low_cmd);
    }

    void wireless_controller_callback(const unitree_go::msg::WirelessController::SharedPtr msg)
    {
        joystick_ly_ = msg->ly; 
        joystick_rx_ = msg->rx;
        if (msg->keys & (1 << 8)) stand_up = true;
        if (msg->keys & (1 << 9)) stand_up = false;
        RCLCPP_INFO(this->get_logger(), "LY = %f, RX = %f, A = %d, B = %d", joystick_ly_, joystick_rx_, (msg->keys & (1 << 8)) != 0, (msg->keys & (1 << 9)) != 0);
    }

    void init_cmd()
    {
        for (int i = 0; i < 20; i++) {
            low_cmd.motor_cmd[i].mode = 0x01;
            low_cmd.motor_cmd[i].q = PosStopF;
            low_cmd.motor_cmd[i].kp = 0;
            low_cmd.motor_cmd[i].dq = VelStopF;
            low_cmd.motor_cmd[i].kd = 0;
            low_cmd.motor_cmd[i].tau = 0;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_sub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;

    unitree_go::msg::LowCmd low_cmd;
    std::vector<float> actual_joint_positions = std::vector<float>(20, 0.0f);
    std::vector<float> actual_joint_velocities = std::vector<float>(20, 0.0f);
    std::vector<float> integral_error = std::vector<float>(20, 0.0f);
    std::vector<float> prev_error = std::vector<float>(20, 0.0f);

    double joystick_ly_ = 0.0;
    double joystick_rx_ = 0.0;

    // positions for go2 (no wheels)
    // double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    //                                  0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    // double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    //                                    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double stand_up_joint_pos[12] = {0.01, 0.5, -1.21763, -0.01, 0.5, -1.21763,
                                     0.01, 0.9, -1.21763, -0.01, 0.9, -1.21763};
    double stand_down_joint_pos[12] = {0.01, 0.9, -2.44375, -0.01, 0.9, -2.44375,
                                       0.01, 0.9, -2.44375, -0.01, 0.9, -2.44375};
    bool stand_up = false;
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;
};

int main(int argc, char **argv)
{   
    std::cout << "Press enter to start";
    std::cin.get();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<low_level_cmd_sender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
