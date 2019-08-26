#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class Teleop
{
public:
  Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_x_, linear_y_, angular_, deadman_axis_;
  double lx_scale_, ly_scale_, a_scale_, deadzone_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

Teleop::Teleop():
  ph_("~"),
  linear_x_(1),
  linear_y_(0),
  angular_(3),
  deadman_axis_(4),
  lx_scale_(0.8),
  ly_scale_(0.8),
  a_scale_(1.5488),
  deadzone_(0.1)
{
  ph_.param("axis_linear_x", linear_x_, linear_x_);
  ph_.param("axis_linear_y", linear_y_, linear_y_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear_x", lx_scale_, lx_scale_);
  ph_.param("scale_linear_y", ly_scale_, ly_scale_);
  ph_.param("deadzone_scale", deadzone_, deadzone_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Teleop::publish, this));
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  if (joy->axes[angular_] > deadzone_){
    vel.angular.z = a_scale_ * (joy->axes[angular_] - deadzone_);
  }
  else if (joy->axes[angular_] < -deadzone_){
    vel.angular.z = a_scale_ * (joy->axes[angular_] + deadzone_);
  }
  else {
    vel.angular.z = 0;
  }
  if (joy->axes[linear_x_] > deadzone_){
    vel.linear.x = lx_scale_ * (joy->axes[linear_x_] - deadzone_);
  }
  else if (joy->axes[linear_x_] < -deadzone_){
    vel.linear.x = lx_scale_ * (joy->axes[linear_x_] + deadzone_);
  }
  else {
    vel.linear.x = 0;
  }
  if (joy->axes[linear_y_] > deadzone_){
    vel.linear.y = ly_scale_ * (joy->axes[linear_y_] - deadzone_);
  }
  else if (joy->axes[linear_y_] < -deadzone_){
    vel.linear.y = ly_scale_ * (joy->axes[linear_y_] + deadzone_);
  }
  else {
    vel.linear.y = 0;
  }
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
}

void Teleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vrep_holonomic_teleop_joy");
  Teleop teleop;

  ros::spin();
}
