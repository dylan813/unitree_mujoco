#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "motor_crc.h"
#include "unitree_go/msg/wireless_controller.hpp"
#include <algorithm>
#include <vector>
#include <chrono>

class low_level_cmd_sender : public rclcpp::Node
{
public:
    // Command structure for sequence control
    enum class PostureCmd {
        NONE,        // No posture change
        STAND_UP,    // Stand up
        CROUCH,      // Crouch down
        STAND_DOWN   // Stand down
    };
    
    struct Command {
        double forward_speed;    // -1.0 to 1.0 (joystick_ly_)
        double turn_speed;       // -1.0 to 1.0 (joystick_rx_)
        PostureCmd posture_cmd;  // Posture command
        double duration;         // duration in seconds
        std::string description; // description of the command
        
        Command(double fwd, double turn, PostureCmd posture, double dur, const std::string& desc)
            : forward_speed(fwd), turn_speed(turn), posture_cmd(posture), duration(dur), description(desc) {}
    };

    low_level_cmd_sender() : Node("low_level_cmd_sender")
    {
        cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&low_level_cmd_sender::timer_callback, this));
        init_cmd();

        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10, std::bind(&low_level_cmd_sender::lowstate_callback, this, std::placeholders::_1));

        // Command sequence (instead of sending joystick commands)
        initialize_command_sequence();
        
        start_time_ = std::chrono::high_resolution_clock::now();
        current_command_index_ = 0;
        command_start_time_ = std::chrono::high_resolution_clock::now();

        if (!command_sequence_.empty()) {
            const Command& first_cmd = command_sequence_[0];
            apply_posture_command(first_cmd.posture_cmd);
            RCLCPP_INFO(this->get_logger(), "Starting with command 1: %s", first_cmd.description.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Initialized with command sequence mode. Total commands: %zu", command_sequence_.size());
    }

private:
    enum class RobotState {
        STAND_DOWN,
        TRANSITION_UP,
        STAND_UP,
        TRANSITION_TO_CROUCH,
        CROUCH,
        TRANSITION_DOWN
    };

    RobotState current_state = RobotState::STAND_DOWN;
    bool transitioning_to_crouch_from_stand_down = false;
    bool transitioning_down_from_crouch = false;

    void apply_posture_command(PostureCmd posture_cmd)
    {
        switch (posture_cmd) {
            case PostureCmd::STAND_UP:
                if (current_state == RobotState::STAND_DOWN || current_state == RobotState::CROUCH) {
                    current_state = RobotState::TRANSITION_UP;
                    runing_time = 0.0;
                    stand_up = true;
                    RCLCPP_INFO(this->get_logger(), "Command: Stand up");
                }
                break;
            case PostureCmd::CROUCH:
                if (current_state == RobotState::STAND_UP || current_state == RobotState::STAND_DOWN) {
                    if (current_state == RobotState::STAND_UP) {
                        current_state = RobotState::TRANSITION_TO_CROUCH;
                        transitioning_to_crouch_from_stand_down = false;
                    } else {
                        current_state = RobotState::TRANSITION_TO_CROUCH;
                        transitioning_to_crouch_from_stand_down = true;
                    }
                    runing_time = 0.0;
                    stand_up = true;
                    RCLCPP_INFO(this->get_logger(), "Command: Crouch");
                }
                break;
            case PostureCmd::STAND_DOWN:
                if (current_state == RobotState::STAND_UP || current_state == RobotState::CROUCH) {
                    transitioning_down_from_crouch = (current_state == RobotState::CROUCH);
                    current_state = RobotState::TRANSITION_DOWN;
                    runing_time = 0.0;
                    stand_up = false;
                    RCLCPP_INFO(this->get_logger(), "Command: Stand down");
                }
                break;
            case PostureCmd::NONE:
                break;
        }
    }

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
        // Safety timeout: check total elapsed time since start
        auto now = std::chrono::high_resolution_clock::now();
        double total_elapsed = std::chrono::duration<double>(now - start_time_).count();
        if (total_elapsed > max_runtime_seconds_) {
            RCLCPP_ERROR(this->get_logger(), "Safety timeout reached (%.1fs). Stopping robot!", total_elapsed);
            rclcpp::shutdown();
            return;
        }
        
        // Update current command from sequence
        update_current_command();
        
        // Get current joystick values from command sequence
        if (current_command_index_ < command_sequence_.size()) {
            const Command& current_cmd = command_sequence_[current_command_index_];
            joystick_ly_ = current_cmd.forward_speed;
            joystick_rx_ = current_cmd.turn_speed;
        } else {
            // Sequence completed, stop robot
            joystick_ly_ = 0.0;
            joystick_rx_ = 0.0;
        }

        switch (current_state) {
            case RobotState::STAND_DOWN:
                break;

            case RobotState::TRANSITION_UP:
                if (runing_time < 1.2) {
                    runing_time += dt;
                    phase = runing_time / 1.2;
                    for (int i = 0; i < 12; i++) {
                        low_cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                        low_cmd.motor_cmd[i].dq = 0;
                        low_cmd.motor_cmd[i].kp = phase * 30 + (1 - phase) * 15;  // ramp up to standing stiffness
                        low_cmd.motor_cmd[i].kd = 9.0;  // high damping throughout transition
                        low_cmd.motor_cmd[i].tau = 0;
                    }
                } else {
                    if (transitioning_to_crouch_from_stand_down) {
                        current_state = RobotState::TRANSITION_TO_CROUCH;
                        transitioning_to_crouch_from_stand_down = false;
                        runing_time = 0.0;
                    } else {
                        current_state = RobotState::STAND_UP;
                    }
                }
                break;

            case RobotState::STAND_UP:
                if (!stand_up) {
                    current_state = RobotState::TRANSITION_DOWN;
                    runing_time = 0.0;
                } else {
                    // Very low gains for smooth, chatter-free operation - rely heavily on feedforward
                    const float hip_kp = 15.0f;   // very low position stiffness
                    const float hip_kd = 4.0f;    // very low damping
                    const float hip_tau = 3.0f;   // increased feedforward
                    
                    const float thigh_kp = 15.0f; // very low position stiffness
                    const float thigh_kd = 4.0f;  // very low damping
                    const float thigh_tau = 6.0f; // increased feedforward - supports weight
                    
                    const float calf_kp  = 15.0f; // very low position stiffness
                    const float calf_kd  = 4.0f;  // very low damping
                    const float calf_tau  = 5.0f; // increased feedforward

                    for (int i = 0; i < 12; i++) {
                        float kp = 0.0f, kd = 0.0f, tau = 0.0f;
                        int mod = i % 3;
                        if (mod == 0) {
                            kp = hip_kp;   kd = hip_kd;   tau = hip_tau;
                        } else if (mod == 1) {
                            kp = thigh_kp; kd = thigh_kd; tau = thigh_tau;
                        } else {
                            kp = calf_kp;  kd = calf_kd;  tau = calf_tau;
                        }

                        low_cmd.motor_cmd[i].mode = 0x01;
                        low_cmd.motor_cmd[i].q  = stand_up_joint_pos[i];
                        low_cmd.motor_cmd[i].dq = 0;
                        low_cmd.motor_cmd[i].kp = kp;
                        low_cmd.motor_cmd[i].kd = kd;
                        low_cmd.motor_cmd[i].tau = tau;
                    }

                    const double max_wheel_speed = 50.0; // rad/s - high speed for wheels
                    double forward_speed = joystick_ly_ * max_wheel_speed;
                    double turning_speed = joystick_rx_ * max_wheel_speed;
                    double right_wheel_speed = forward_speed - turning_speed;
                    double left_wheel_speed = forward_speed + turning_speed;
                    right_wheel_speed = std::max(-max_wheel_speed, std::min(right_wheel_speed, max_wheel_speed));
                    left_wheel_speed = std::max(-max_wheel_speed, std::min(left_wheel_speed, max_wheel_speed));

                    const int right_wheel_motor_indices[] = {12, 14};
                    const int left_wheel_motor_indices[] = {13, 15};
                    
                    // Very gentle velocity control with torque limits to prevent tipping
                    const double wheel_kd = 1.0;  // Reduced damping for gentler response
                    const double feedforward_torque_gain = 0.2;  // Reduced feedforward
                    const double max_wheel_torque = 15.0;  // Limit max torque to prevent tipping
                    
                    for (int wheel_idx : right_wheel_motor_indices) {
                        double feedforward = feedforward_torque_gain * right_wheel_speed;
                        double velocity_error = right_wheel_speed - actual_joint_velocities[wheel_idx];
                        double feedback_torque = wheel_kd * velocity_error;
                        double total_torque = feedforward + feedback_torque;
                        
                        // Clamp torque to safe limits
                        total_torque = std::max(-max_wheel_torque, std::min(total_torque, max_wheel_torque));
                        
                        low_cmd.motor_cmd[wheel_idx].mode = 0x01;  // torque mode
                        low_cmd.motor_cmd[wheel_idx].q = PosStopF;  // disable position control
                        low_cmd.motor_cmd[wheel_idx].dq = VelStopF;  // disable internal velocity control
                        low_cmd.motor_cmd[wheel_idx].kp = 0;  // no position gain
                        low_cmd.motor_cmd[wheel_idx].kd = 0;  // no internal damping
                        low_cmd.motor_cmd[wheel_idx].tau = total_torque;  // limited torque command
                    }

                    for (int wheel_idx : left_wheel_motor_indices) {
                        double feedforward = feedforward_torque_gain * left_wheel_speed;
                        double velocity_error = left_wheel_speed - actual_joint_velocities[wheel_idx];
                        double feedback_torque = wheel_kd * velocity_error;
                        double total_torque = feedforward + feedback_torque;
                        
                        // Clamp torque to safe limits
                        total_torque = std::max(-max_wheel_torque, std::min(total_torque, max_wheel_torque));
                        
                        low_cmd.motor_cmd[wheel_idx].mode = 0x01;  // torque mode
                        low_cmd.motor_cmd[wheel_idx].q = PosStopF;  // disable position control
                        low_cmd.motor_cmd[wheel_idx].dq = VelStopF;  // disable internal velocity control
                        low_cmd.motor_cmd[wheel_idx].kp = 0;  // no position gain
                        low_cmd.motor_cmd[wheel_idx].kd = 0;  // no internal damping
                        low_cmd.motor_cmd[wheel_idx].tau = total_torque;  // limited torque command
                    }
                    
                    // Debug: log what we're sending to wheels
                    static int log_counter = 0;
                    if (log_counter++ % 50 == 0) {  // Log every 50 cycles (~0.1 seconds) for debugging
                        double r_ff = feedforward_torque_gain * right_wheel_speed;
                        double l_ff = feedforward_torque_gain * left_wheel_speed;
                        double r_vel_error = right_wheel_speed - actual_joint_velocities[12];
                        double l_vel_error = left_wheel_speed - actual_joint_velocities[13];
                        RCLCPP_INFO(this->get_logger(), "WHEELS: joy_ly=%.2f, joy_rx=%.2f -> R_des=%.1f, L_des=%.1f rad/s", 
                                    joystick_ly_, joystick_rx_, right_wheel_speed, left_wheel_speed);
                        RCLCPP_INFO(this->get_logger(), "Motor 12 (R): FF=%.1f Nm, vel_err=%.1f, actual=%.1f rad/s", 
                                    r_ff, r_vel_error, actual_joint_velocities[12]);
                        RCLCPP_INFO(this->get_logger(), "Motor 13 (L): FF=%.1f Nm, vel_err=%.1f, actual=%.1f rad/s", 
                                    l_ff, l_vel_error, actual_joint_velocities[13]);
                    }
                }
                break;

            case RobotState::TRANSITION_TO_CROUCH: {
                // Use longer transition time for safety, especially from STAND_DOWN
                double transition_time = transitioning_to_crouch_from_stand_down ? 4.0 : 2.5;
                if (runing_time < transition_time) {
                    runing_time += dt;
                    double raw_phase = runing_time / transition_time;
                    double phase = raw_phase * raw_phase * (3.0 - 2.0 * raw_phase); // Smooth step function
                    
                    for (int i = 0; i < 12; i++) {
                        double* start_pos = transitioning_to_crouch_from_stand_down ? stand_down_joint_pos : stand_up_joint_pos;
                        double start_kp = transitioning_to_crouch_from_stand_down ? 15.0 : 30.0;  // transition from stand or stand_down
                        
                        low_cmd.motor_cmd[i].q = (1 - phase) * start_pos[i] + phase * crouch_joint_pos[i];
                        low_cmd.motor_cmd[i].dq = 0;
                        double end_kp = transitioning_to_crouch_from_stand_down ? 20.0 : 20.0;  // crouch needs moderate stiffness
                        low_cmd.motor_cmd[i].kp = (1 - phase) * start_kp + phase * end_kp;
                        low_cmd.motor_cmd[i].kd = 9.0;  // high damping for stability
                        low_cmd.motor_cmd[i].tau = 0;
                    }
                } else {
                    current_state = RobotState::CROUCH;
                    transitioning_to_crouch_from_stand_down = false; // Reset flag
                }
                break;
            }

            case RobotState::CROUCH: {
                //hip motor (0,3,6,9) gains for crouch - moderate stiffness for low posture
                const float crouch_hip_kp = 20.0f;   // moderate stiffness for crouch position
                const float crouch_hip_kd = 9.0f;    // high damping for stability
                const float crouch_hip_tau = 0.0f;   // pure PD control (no feedforward)
                //thigh motor (1,4,7,10) gains for crouch - moderate stiffness for low posture
                const float crouch_thigh_kp = 20.0f; // moderate stiffness for crouch position
                const float crouch_thigh_kd = 9.0f;  // high damping for stability
                const float crouch_thigh_tau = 0.0f; // pure PD control (no feedforward)
                //calf motor (2,5,8,11) gains for crouch - moderate stiffness for low posture
                const float crouch_calf_kp  = 20.0f; // moderate stiffness for crouch position
                const float crouch_calf_kd  = 9.0f;  // high damping for stability
                const float crouch_calf_tau  = 0.0f; // pure PD control (no feedforward)

                for (int i = 0; i < 12; i++) {
                    float kp = 0.0f, kd = 0.0f, tau = 0.0f;
                    int mod = i % 3;
                    if (mod == 0) {
                        kp = crouch_hip_kp;   kd = crouch_hip_kd;   tau = crouch_hip_tau;
                    } else if (mod == 1) {
                        kp = crouch_thigh_kp; kd = crouch_thigh_kd; tau = crouch_thigh_tau;
                    } else {
                        kp = crouch_calf_kp;  kd = crouch_calf_kd;  tau = crouch_calf_tau;
                    }

                    low_cmd.motor_cmd[i].mode = 0x01;
                    low_cmd.motor_cmd[i].q  = crouch_joint_pos[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = kp;
                    low_cmd.motor_cmd[i].kd = kd;
                    low_cmd.motor_cmd[i].tau = tau;
                }

                // Wheel control for crouch state
                const double max_wheel_speed = 50.0; // rad/s - high speed for wheels
                double forward_speed = joystick_ly_ * max_wheel_speed;
                double turning_speed = joystick_rx_ * max_wheel_speed;
                double right_wheel_speed = forward_speed - turning_speed;
                double left_wheel_speed = forward_speed + turning_speed;
                right_wheel_speed = std::max(-max_wheel_speed, std::min(right_wheel_speed, max_wheel_speed));
                left_wheel_speed = std::max(-max_wheel_speed, std::min(left_wheel_speed, max_wheel_speed));

                const int right_wheel_motor_indices[] = {12, 14};
                const int left_wheel_motor_indices[] = {13, 15};
                
                // Very gentle velocity control with torque limits to prevent tipping
                const double wheel_kd = 1.0;  // Reduced damping for gentler response
                const double feedforward_torque_gain = 0.2;  // Reduced feedforward
                const double max_wheel_torque = 15.0;  // Limit max torque to prevent tipping
                
                for (int wheel_idx : right_wheel_motor_indices) {
                    double feedforward = feedforward_torque_gain * right_wheel_speed;
                    double velocity_error = right_wheel_speed - actual_joint_velocities[wheel_idx];
                    double feedback_torque = wheel_kd * velocity_error;
                    double total_torque = feedforward + feedback_torque;
                    
                    // Clamp torque to safe limits
                    total_torque = std::max(-max_wheel_torque, std::min(total_torque, max_wheel_torque));
                    
                    low_cmd.motor_cmd[wheel_idx].mode = 0x01;  // torque mode
                    low_cmd.motor_cmd[wheel_idx].q = PosStopF;  // disable position control
                    low_cmd.motor_cmd[wheel_idx].dq = VelStopF;  // disable internal velocity control
                    low_cmd.motor_cmd[wheel_idx].kp = 0;  // no position gain
                    low_cmd.motor_cmd[wheel_idx].kd = 0;  // no internal damping
                    low_cmd.motor_cmd[wheel_idx].tau = total_torque;  // limited torque command
                }

                for (int wheel_idx : left_wheel_motor_indices) {
                    double feedforward = feedforward_torque_gain * left_wheel_speed;
                    double velocity_error = left_wheel_speed - actual_joint_velocities[wheel_idx];
                    double feedback_torque = wheel_kd * velocity_error;
                    double total_torque = feedforward + feedback_torque;
                    
                    // Clamp torque to safe limits
                    total_torque = std::max(-max_wheel_torque, std::min(total_torque, max_wheel_torque));
                    
                    low_cmd.motor_cmd[wheel_idx].mode = 0x01;  // torque mode
                    low_cmd.motor_cmd[wheel_idx].q = PosStopF;  // disable position control
                    low_cmd.motor_cmd[wheel_idx].dq = VelStopF;  // disable internal velocity control
                    low_cmd.motor_cmd[wheel_idx].kp = 0;  // no position gain
                    low_cmd.motor_cmd[wheel_idx].kd = 0;  // no internal damping
                    low_cmd.motor_cmd[wheel_idx].tau = total_torque;  // limited torque command
                }
                break;
            }

            case RobotState::TRANSITION_DOWN:
                if (runing_time < 2.4) {
                    runing_time += dt;
                    phase = runing_time / 2.4;
                    for (int i = 0; i < 12; i++) {
                        // Choose starting position based on where we're transitioning from
                        double* start_pos = transitioning_down_from_crouch ? crouch_joint_pos : stand_up_joint_pos;
                        double start_kp = transitioning_down_from_crouch ? 20.0 : 30.0; // transition from crouch or stand
                        
                        low_cmd.motor_cmd[i].q = (1 - phase) * start_pos[i] + phase * stand_down_joint_pos[i];
                        low_cmd.motor_cmd[i].dq = 0;
                        low_cmd.motor_cmd[i].kp = (1 - phase) * start_kp + phase * 15;  // ramp down to stand_down stiffness
                        low_cmd.motor_cmd[i].kd = 9.0;  // high damping for stability
                        low_cmd.motor_cmd[i].tau = 0;
                    }
                } else {
                    current_state = RobotState::STAND_DOWN;
                    transitioning_down_from_crouch = false; // Reset flag
                }
                break;
        }

        get_crc(low_cmd);
        cmd_puber->publish(low_cmd);
    }

    void initialize_command_sequence()
    {
        // Define your command sequence here
        // Format: Command(forward_speed, turn_speed, posture_cmd, duration_seconds, "description")
        // forward_speed: -1.0 (backward) to 1.0 (forward), 0.0 = stop
        // turn_speed: -1.0 (left) to 1.0 (right), 0.0 = straight
        // posture_cmd: PostureCmd::STAND_UP, PostureCmd::CROUCH, PostureCmd::STAND_DOWN, PostureCmd::NONE
        // duration: how long to execute this command in seconds
        
        command_sequence_.clear();
        
        // Stand up first so robot can move
        command_sequence_.push_back(Command(0.0, 0.0, PostureCmd::STAND_UP, 2.0, "Stand up"));
        
        // Repeat snake pattern multiple times
        const int num_snake_repetitions = 8;  // How many times to repeat the entire snake pattern
        const int num_turn_cycles = 5; // single turn cycle includes right and left turn
        const double turn_speed = 0.1; // max turn speed
        const double turn_duration = 0.8; // duration for each turn in seconds
        
        for (int rep = 0; rep < num_snake_repetitions; rep++) {
            // Initial turn for this snake sequence
            std::string init_desc = "Snake rep " + std::to_string(rep + 1) + "/" + std::to_string(num_snake_repetitions) + " - initial turn";
            command_sequence_.push_back(Command(0.1, -0.1, PostureCmd::NONE, 0.4, init_desc));
            
            // Alternating snake pattern
            for (int i = 0; i < num_turn_cycles; i++) {
                // turn right
                std::string right_desc = "Rep " + std::to_string(rep + 1) + " - Snake right (" + std::to_string(i + 1) + "/" + std::to_string(num_turn_cycles) + ")";
                command_sequence_.push_back(Command(0.1, turn_speed, PostureCmd::NONE, turn_duration, right_desc));
                
                // turn left
                std::string left_desc = "Rep " + std::to_string(rep + 1) + " - Snake left (" + std::to_string(i + 1) + "/" + std::to_string(num_turn_cycles) + ")";
                command_sequence_.push_back(Command(0.1, -turn_speed, PostureCmd::NONE, turn_duration, left_desc));
            }
            
            // Final turn for this snake sequence
            std::string final_desc = "Snake rep " + std::to_string(rep + 1) + "/" + std::to_string(num_snake_repetitions) + " - final turn";
            command_sequence_.push_back(Command(0.1, 0.1, PostureCmd::NONE, 0.4, final_desc));

            command_sequence_.push_back(Command(0.0, 0.0, PostureCmd::NONE, 3.0, "Stop while standing"));
        }
        
        RCLCPP_INFO(this->get_logger(), "Command sequence initialized with %zu commands", command_sequence_.size());
    }

    void update_current_command()
    {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - command_start_time_).count();
        
        // Check if current command duration has elapsed
        if (current_command_index_ < command_sequence_.size() && 
            elapsed >= command_sequence_[current_command_index_].duration) {
            
            // Move to next command
            current_command_index_++;
            command_start_time_ = now;
            
            if (current_command_index_ < command_sequence_.size()) {
                const Command& cmd = command_sequence_[current_command_index_];
                RCLCPP_INFO(this->get_logger(), "Executing command %d: %s (%.1fs)", 
                           current_command_index_ + 1, cmd.description.c_str(), cmd.duration);
                
                // Apply posture command immediately when command changes
                apply_posture_command(cmd.posture_cmd);
                
                // Log current movement commands
                if (cmd.forward_speed != 0.0 || cmd.turn_speed != 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Movement: forward=%.2f, turn=%.2f", cmd.forward_speed, cmd.turn_speed);
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Command sequence completed! Stopping robot...");
                // Stop the node to ensure robot doesn't continue running
                rclcpp::shutdown();
            }
        }
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
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;

    unitree_go::msg::LowCmd low_cmd;
    std::vector<float> actual_joint_positions = std::vector<float>(20, 0.0f);
    std::vector<float> actual_joint_velocities = std::vector<float>(20, 0.0f);
    std::vector<float> integral_error = std::vector<float>(20, 0.0f);
    std::vector<float> prev_error = std::vector<float>(20, 0.0f);

    // Command sequence system
    std::vector<Command> command_sequence_;
    size_t current_command_index_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point command_start_time_;
    const double max_runtime_seconds_ = 240.0;  // Safety timeout: stop after 60 seconds

    // Current joystick values (now controlled by command sequence)
    double joystick_ly_ = 0.0;
    double joystick_rx_ = 0.0;

    // positions for go2 (no wheels)
    // double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    //                                  0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    // double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
    //                                    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double stand_up_joint_pos[12] = {-0.005, 0.5, -1.21763, 0.005, 0.5, -1.21763,
                                     -0.005, 0.9, -1.21763, 0.005, 0.9, -1.21763};
    double stand_down_joint_pos[12] = {-0.005, 0.9, -2.44375, 0.005, 0.9, -2.44375,
                                       -0.005, 0.9, -2.44375, 0.005, 0.9, -2.44375};
    double crouch_joint_pos[12] = {-0.005, 1.3, -2.1, 0.005, 1.3, -2.1,
                                    -0.005, 1.45, -2.1, 0.005, 1.45, -2.1};
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
