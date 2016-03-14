/**
Copyright 2016 Lucas Walter
*/

#include <opencv_apps/Point2D.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
extern "C" {
#include <xdo.h>
}

class XdoRos
{
public:
  XdoRos();
protected:
  ros::NodeHandle nh_;
  ros::Subscriber point_sub_;
  void mousePosCallback(const opencv_apps::Point2D::ConstPtr& msg);
  ros::Subscriber mouse_rel_sub_;
  void mouseRelCallback(const opencv_apps::Point2D::ConstPtr& msg);
  ros::Subscriber string_sub_;
  void stringCallback(const std_msgs::String::ConstPtr& msg);
  ros::Subscriber keys_down_sub_;
  void keysDownCallback(const std_msgs::String::ConstPtr& msg);
  ros::Subscriber keys_up_sub_;
  void keysUpCallback(const std_msgs::String::ConstPtr& msg);
  xdo_t* xdo_;
  Window window_;
};

XdoRos::XdoRos()
{
  xdo_ = xdo_new(NULL);
  window_ = CURRENTWINDOW;
  // TODO(lucasw) check xdo for success?
  point_sub_ = nh_.subscribe("mouse_pos", 1, &XdoRos::mousePosCallback, this);
  mouse_rel_sub_ = nh_.subscribe("mouse_rel", 1, &XdoRos::mouseRelCallback, this);
  string_sub_ = nh_.subscribe("string", 1, &XdoRos::stringCallback, this);
  keys_down_sub_ = nh_.subscribe("keys_down", 1, &XdoRos::keysDownCallback, this);
  keys_up_sub_ = nh_.subscribe("keys_up", 1, &XdoRos::keysUpCallback, this);
}

void XdoRos::mousePosCallback(const opencv_apps::Point2D::ConstPtr& msg)
{
  xdo_move_mouse(xdo_, msg->x, msg->y, 0);
}

void XdoRos::mouseRelCallback(const opencv_apps::Point2D::ConstPtr& msg)
{
  xdo_move_mouse_relative(xdo_, msg->x, msg->y);
}

void XdoRos::stringCallback(const std_msgs::String::ConstPtr& msg)
{
  xdo_enter_text_window(xdo_, window_, msg->data.c_str(), 12000);
}

void XdoRos::keysDownCallback(const std_msgs::String::ConstPtr& msg)
{
  xdo_send_keysequence_window_down(xdo_, window_, msg->data.c_str(), 12000);
}

void XdoRos::keysUpCallback(const std_msgs::String::ConstPtr& msg)
{
  xdo_send_keysequence_window_up(xdo_, window_, msg->data.c_str(), 12000);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xdo_ros");
  XdoRos xdo_ros;
  ros::spin();
}
