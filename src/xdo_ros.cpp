/**
Copyright 2016 Lucas Walter
*/

#include <opencv_apps/Point2D.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xdo_ros/Mouse.h>

extern "C"
{
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
  ros::Publisher point_pub_;
  ros::Timer timer_;
  void update(const ros::TimerEvent& e);
  xdo_t* xdo_;
  Window window_;
};

XdoRos::XdoRos()
{
  xdo_ = xdo_new(NULL);
  window_ = CURRENTWINDOW;
  // TODO(lucasw) check xdo for success?
  point_sub_ = nh_.subscribe("set_mouse_pos", 1, &XdoRos::mousePosCallback, this);
  mouse_rel_sub_ = nh_.subscribe("set_mouse_rel", 1, &XdoRos::mouseRelCallback, this);
  string_sub_ = nh_.subscribe("set_string", 1, &XdoRos::stringCallback, this);
  keys_down_sub_ = nh_.subscribe("set_keys_down", 1, &XdoRos::keysDownCallback, this);
  keys_up_sub_ = nh_.subscribe("set_keys_up", 1, &XdoRos::keysUpCallback, this);
  point_pub_ = nh_.advertise<xdo_ros::Mouse>("mouse", 3);
  // TODO(lucasw) make duration parameter
  timer_ = nh_.createTimer(ros::Duration(0.05), &XdoRos::update, this);
}

void XdoRos::update(const ros::TimerEvent& e)
{
  // TODO(lucasw) instead of lots of messages at a high update rate, could
  // sample the mouse at a higher rate internally but then publish an array
  // of position that would be the path since the last update.
  int x, y, screen_num;
  Window window;
  xdo_get_mouse_location2(xdo_, &x, &y, &screen_num, &window);
  xdo_ros::Mouse mouse;
  mouse.pos.x = x;
  mouse.pos.y = y;
  mouse.screen_num = screen_num;
  mouse.header.stamp = ros::Time::now();
  unsigned char* name;
  int size;
  int type;
  xdo_get_window_name(xdo_, window, &name, &size, &type);
  mouse.window_name = reinterpret_cast<const char*>(name);
  point_pub_.publish(mouse);
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
