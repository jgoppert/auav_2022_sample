#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// #include <auto_obj_tracking/BoundingBox.msg>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
int count = 0;
int last_sec = 0;
geometry_msgs::PoseStamped cur_drone_pos;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("Called this image %d times.", count++);
  
  int sec = ros::Time::now().toSec();
  
  if (last_sec != sec) {
    last_sec++;
    ROS_DEBUG("%d currently producing %d images per second.\n", count);    
    count = 0;
  }
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  
  }

}
void positionCallback(const geometry_msgs::Point& msg) {
  // Do some math to get the actual pos
  // geometry_msgs::pos_msg rover_pos;
  // mavros_pub.publish(pos_msg);
  return;
};

void updateLocalPosCallback(const geometry_msgs::PoseStamped& msg) {
  // Update the drone's position
  cur_drone_pos = msg;
  return;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  // image_transport::ImageTransport it(nh);
  // image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  // ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/uncompressed_camera", 1);
  
  // ros::Subscriber pos_sub = nh.subscribe("/relative_pos", 1, positionCallback);
  // ros::Publisher pub = nh.advertise<std_msgs::String>("/bounding_box", 1);
  ros::Subscriber pos_sub = nh.subscribe("/relative_rover_pos", 1, positionCallback);
  ros::Subscriber mavros_pos_sub = nh.subscribe("/mavros/local_position/pose", 1, updateLocalPosCallback); // geometry_msg::PoseStamped
  // ros::Publisher mavros_pub = nh.advertise<geometry_msg::PoseStamped>("/mavros/setpoint_position/local");
  ros::spin();
  cv::destroyWindow("view");
}