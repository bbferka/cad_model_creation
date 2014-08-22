
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h> //hydro
#include <sensor_msgs/PointCloud2.h> //hydro
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse_yml.h>
#include <camera_calibration_parsers/parse.h>
#include <string>
#include <iostream>
#include <camera_info_manager/camera_info_manager.h>

#define NAME_COLOR "_color.png"

ros::Publisher pub;
bool get_info =false;
void cloud_cb (const sensor_msgs::CameraInfo  info)
{
    std::cout<<"get info!"<<std::endl;
    const std::string out_file="params2.yml";
    const std::string cam_name="xtion";
    camera_calibration_parsers::writeCalibrationYml(out_file,cam_name,info);
    std::cout<<"write info to params.yml file"<<std::endl;
    get_info=true;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/rgb/camera_info", 1, cloud_cb);

while(!get_info){
  ros::spinOnce();
}
  //End Program
  return (0);
}
