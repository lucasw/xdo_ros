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
  ros::Subscriber string_sub_;
  void mousePosCallback(const opencv_apps::Point2D::ConstPtr& msg);
  void stringCallback(const std_msgs::String::ConstPtr& msg);
  xdo_t* xdo_;
  Window window_;
};

XdoRos::XdoRos()
{
  xdo_ = xdo_new(NULL);
  window_ = CURRENTWINDOW;
  // TODO(lucasw) check xdo for success?
  point_sub_ = nh_.subscribe("mouse_pos", 1, &XdoRos::mousePosCallback, this);
  string_sub_ = nh_.subscribe("string", 1, &XdoRos::stringCallback, this);
}

void XdoRos::mousePosCallback(const opencv_apps::Point2D::ConstPtr& msg)
{
  xdo_move_mouse(xdo_, msg->x, msg->y, 0);
}

void XdoRos::stringCallback(const std_msgs::String::ConstPtr& msg)
{
  xdo_enter_text_window(xdo_, window_, msg->data.c_str(), 12000);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xdo_ros");
  XdoRos xdo_ros;
  ros::spin();
}
