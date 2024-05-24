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
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace LibSerial;
using namespace std::chrono_literals;

class AstroSerialNode : public rclcpp::Node
{
public:
  AstroSerialNode() : Node("astro_serial_node"), uart_thread_running_(true)
  {
    this->declare_parameter("serial_port", "/dev/ttyACM0");

    auto baud_rate_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    baud_rate_param_desc.description = "Choose between the following baud rates:\n \t1200\n \t1800\n \t2400\n \t4800\n \t9600\n \t19200\n \t38400\n \t57600 (default)\n \t15200\n \t230400";

    this->declare_parameter("baud_rate", 57600, baud_rate_param_desc);
    this->declare_parameter("wheel_radius", 0.036);
    this->declare_parameter("wheel_separation", 0.304);
    this->declare_parameter("max_wheel_angular_velocity", 20.9);
    this->declare_parameter("odom_joint_states_pub_freq", 50.0);
    this->declare_parameter("publish_odom", true);
    this->declare_parameter("publish_odom_tf", true);

    try
    {
      serial_conn_.Open(this->get_parameter("serial_port").as_string());
      serial_conn_.SetBaudRate(convert_baud_rate(this->get_parameter("baud_rate").as_int()));

      uart_thread_ = std::thread(&AstroSerialNode::read_joint_states, this);
      RCLCPP_INFO(this->get_logger(), "UART Port opened succesfully");
    }
    catch (const OpenFailed &)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to open the UART port");
      rclcpp::shutdown();
    }

    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    current_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/absoulte_current", 10);

    if (this->get_parameter("publish_odom").as_bool())
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (this->get_parameter("publish_odom_tf").as_bool() && this->get_parameter("publish_odom").as_bool())
      odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_joint_states_timer_ = this->create_wall_timer(std::chrono::duration<double>((double)(1 / this->get_parameter("odom_joint_states_pub_freq").as_double())), std::bind(&AstroSerialNode::odom_joint_states_publish_callback, this));

    reset_odom_srv_ = this->create_service<std_srvs::srv::Trigger>("reset_odometry", std::bind(&AstroSerialNode::reset_odometry, this, std::placeholders::_1, std::placeholders::_2));

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&AstroSerialNode::cmd_vel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Started astro_serial node!");
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
  BaudRate convert_baud_rate(int baud_rate)
  {
    switch (baud_rate)
    {
    case 1200:
      return BaudRate::BAUD_1200;
    case 1800:
      return BaudRate::BAUD_1800;
    case 2400:
      return BaudRate::BAUD_2400;
    case 4800:
      return BaudRate::BAUD_4800;
    case 9600:
      return BaudRate::BAUD_9600;
    case 19200:
      return BaudRate::BAUD_19200;
    case 38400:
      return BaudRate::BAUD_38400;
    case 57600:
      return BaudRate::BAUD_57600;
    case 115200:
      return BaudRate::BAUD_115200;
    case 230400:
      return BaudRate::BAUD_230400;
    default:
      RCLCPP_WARN(this->get_logger(), "Error! Baud rate %d not supported, use standard baud rates! Default to 57600", baud_rate);

      return BaudRate::BAUD_57600;
    }
  }

  void publish_joint_states()
  {
    auto joint_states_msg = sensor_msgs::msg::JointState();
    auto current_msg = std_msgs::msg::Float32();

    joint_states_msg.header.frame_id = "base_link";
    joint_states_msg.header.stamp = this->get_clock()->now();

    joint_states_msg.name = {"left_wheel_joint", "right_wheel_joint"};

    joint_states_msg.position = {std::stod(raw_joint_states.at(0)), std::stod(raw_joint_states.at(1))};
    joint_states_msg.velocity = {std::stod(raw_joint_states.at(2)), std::stod(raw_joint_states.at(3))};
    joint_states_msg.effort = {std::stod(raw_joint_states.at(4)), std::stod(raw_joint_states.at(5))};

    joint_states_publisher_->publish(joint_states_msg);

    current_msg.data = std::stof(raw_joint_states.at(6));

    current_publisher_->publish(current_msg);
  }

  void publish_odometry(bool reset = false)
  {
    odom_current_time_ = this->get_clock()->now().nanoseconds() / 1e9;

    double wL = std::stod(raw_joint_states.at(2));
    double wR = std::stod(raw_joint_states.at(3));

    double velX = (this->get_parameter("wheel_radius").as_double() / 2) * (wL + wR);
    double velY = 0;
    double velTh = (this->get_parameter("wheel_radius").as_double() / this->get_parameter("wheel_separation").as_double()) * (wR - wL);

    double dt = reset ? 0 : odom_current_time_ - odom_last_time_;
    double delta_x = (velX * cos(odom_th_)) * dt;
    double delta_y = (velY * sin(odom_th_)) * dt;
    double delta_th = velTh * dt;

    odom_x_ += delta_x;
    odom_y_ += delta_y;
    odom_th_ += delta_th;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_th_);
    q.normalize();

    odom_msg_.header.stamp = this->get_clock()->now();
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";

    odom_msg_.pose.pose.position.x = odom_x_;
    odom_msg_.pose.pose.position.y = odom_y_;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation = tf2::toMsg(q);

    odom_msg_.twist.twist.linear.x = velX;
    odom_msg_.twist.twist.linear.y = velY;
    odom_msg_.twist.twist.angular.z = velTh;

    odom_publisher_->publish(odom_msg_);

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = this->get_clock()->now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = odom_x_;
    odom_trans.transform.translation.y = odom_y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf2::toMsg(q);

    if ((this->get_parameter("publish_odom_tf").as_bool()))
      odom_tf_broadcaster_->sendTransform(odom_trans);

    odom_last_time_ = odom_current_time_;
  }

  void reset_odometry(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Incoming request: reset_odometry");

    response->success = true;
    response->message = "Odometry reset successful!";

    try
    {
      RCLCPP_INFO(this->get_logger(), "Resetting odometry");
      odom_x_ = 0;
      odom_y_ = 0;
      odom_th_ = 0;
      publish_odometry(true);
    }
    catch (std::exception &e)
    {
      response->success = false;
      response->message = "Reset odometry service failed";
    }

    RCLCPP_INFO(this->get_logger(), "Response: odometry was reset");
  }

  void odom_joint_states_publish_callback()
  {
    publish_joint_states();
    if (this->get_parameter("publish_odom").as_bool())
      publish_odometry();
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
    if (value > max_abs)
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

    double wheelRadius = this->get_parameter("wheel_radius").as_double();
    double wheelSeparation = this->get_parameter("wheel_separation").as_double();
    double maxWheelAngularVelocity = this->get_parameter("max_wheel_angular_velocity").as_double();

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
    catch (const NotOpen &)
    {
      RCLCPP_ERROR(this->get_logger(), "UART port is not opened");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error writing to UART: %s", e.what());
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
  {
    std::vector<double> w = diff_drive_inv_kinematics(msg);
    send_velocity(w);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher_;
  rclcpp::TimerBase::SharedPtr odom_joint_states_timer_;
  rclcpp::PublisherOptions pub_options_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

  SerialPort serial_conn_;

  std::vector<std::string> raw_joint_states;

  nav_msgs::msg::Odometry odom_msg_;
  double odom_current_time_;
  double odom_last_time_;
  double odom_x_, odom_y_, odom_th_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_srv_;

  std::thread uart_thread_;
  std::atomic<bool> uart_thread_running_;
};

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<AstroSerialNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}