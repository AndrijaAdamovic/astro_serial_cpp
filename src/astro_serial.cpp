#include <libserial/SerialPort.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace LibSerial;
using namespace std::chrono_literals;

class AstroSerialNode : public rclcpp::Node
{
public:
  AstroSerialNode() : Node("minimal_publisher"), uart_thread_running_(true)
  {
    // my_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // pub_options_.callback_group = my_callback_group_;

    try
    {
      serial_conn_.Open("/dev/ttyACM0");
      serial_conn_.SetBaudRate(BaudRate::BAUD_57600);

      uart_thread_ = std::thread(&AstroSerialNode::read_joint_states, this);
      RCLCPP_INFO(this->get_logger(), "UART Port opened succesfully");
    }
    catch (const OpenFailed &)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the UART port");
    }

    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    current_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/current", 10);
    publisher_timer_ = this->create_wall_timer(20ms, std::bind(&AstroSerialNode::timer_callback, this));
  
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&AstroSerialNode::cmd_vel_callback, this, std::placeholders::_1));
  }

  ~AstroSerialNode()
  {
    uart_thread_running_.store(false);
    if (uart_thread_.joinable())
    {
      uart_thread_.join();
    }

    if (serial_conn_.IsOpen())
    {
      serial_conn_.Close();
    }

    RCLCPP_INFO(this->get_logger(), "ASTRO Serial Node is being destroyed");
  }

private:
  void publish_joint_states()
  {
    auto joint_states_msg = sensor_msgs::msg::JointState();
    auto current_msg = std_msgs::msg::Float32();

    joint_states_msg.header.frame_id = "base_link";
    joint_states_msg.header.stamp.sec = rclcpp::Clock{}.now().seconds();
    joint_states_msg.header.stamp.nanosec = rclcpp::Clock{}.now().nanoseconds();

    joint_states_msg.name = {"left_wheel_joint", "right_wheel_joint"};

    joint_states_msg.position = {std::stod(raw_joint_states.at(0)), std::stod(raw_joint_states.at(1))};
    joint_states_msg.velocity = {std::stod(raw_joint_states.at(2)), std::stod(raw_joint_states.at(3))};
    joint_states_msg.effort = {std::stod(raw_joint_states.at(4)), std::stod(raw_joint_states.at(5))};

    joint_states_publisher_->publish(joint_states_msg);

    current_msg.data = std::stof(raw_joint_states.at(6));

    current_publisher_->publish(current_msg);
  }

  void timer_callback()
  {
    publish_joint_states();
  }

  void read_joint_states()
  {
    while (uart_thread_running_.load())
    {
      if (serial_conn_.IsOpen() && serial_conn_.IsDataAvailable())
      {
        try
        {
          std::string raw_joint_states_string;
          serial_conn_.ReadLine(raw_joint_states_string, '\n', 100);

          std::vector<std::string> temp_joint_states = split_string(raw_joint_states_string, ' ');
          if (temp_joint_states.size() == 7)
          {
            raw_joint_states = temp_joint_states;
          }
        }
        catch (const ReadTimeout &)
        {
          RCLCPP_WARN(this->get_logger(), "UART Read Timeout");
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Error reading from UART: %s", e.what());
        }
      }
    }
  }

  std::vector<std::string> split_string(const std::string &str, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);

    while (std::getline(tokenStream, token, delimiter))
    {
      tokens.push_back(token);
    }

    return tokens;
  }

  double clamp_symetric(double value, double max_abs)
  {
    if(value > max_abs)
    {
      value = max_abs;
    }
    else if (value < -max_abs)
    {
      value = -max_abs;
    }

    return value;
  }

  std::vector<double> diff_drive_inv_kinematics(geometry_msgs::msg::Twist twist_msg)
  {
    std::vector<double> w = {0, 0};
    w.at(0) = clamp_symetric((twist_msg.linear.x / wheelRadius) - 0.5 * twist_msg.angular.z * (wheelSeparation / wheelRadius), maxWheelAngularVelocity);
    w.at(1) = clamp_symetric((twist_msg.linear.x / wheelRadius) + 0.5 * twist_msg.angular.z * (wheelSeparation / wheelRadius), maxWheelAngularVelocity);
  
    return w;
  }

  void send_velocity(std::vector<double> w)
  {
    try
    {
      std::string velcity_command = "v " + std::to_string(w.at(0)) + " " + std::to_string(w.at(1)) + "\n";
      serial_conn_.Write(velcity_command);
    }
    catch(const NotOpen&)
    {
      RCLCPP_ERROR(this->get_logger(), "UART port is not opened");
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error writing to UART: %s", e.what());
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    std::vector<double> w = diff_drive_inv_kinematics(msg);
    send_velocity(w);
  }
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::PublisherOptions pub_options_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  const double wheelRadius = 0.036;
  const double wheelSeparation = 0.304;
  const double maxWheelAngularVelocity = 20.9;

  SerialPort serial_conn_;

  std::vector<std::string> raw_joint_states;

  std::thread uart_thread_;
  std::atomic<bool> uart_thread_running_;
};

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<AstroSerialNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok())
  {
    executor.spin_some();
  }
  rclcpp::shutdown();

  return 0;
}
