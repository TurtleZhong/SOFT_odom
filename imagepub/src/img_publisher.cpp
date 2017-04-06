/** The node `img_publisher` reads the parameters from the parameter server, namely the frequency of publishing data to a topic and source location of an image file. It then publishes the image file to a ros topic `img_pub` using cv_bridge and image_transport in ROS.
*
* PACKAGE: imagepub
* Contributor: Mayank Mittal
**/

#include "ros/ros.h"
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <stdio.h>

using namespace std;

//main function
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "img_publisher"); 

  // creating nodehandle
  ros::NodeHandle nh;

  // fetching parameters for the node
  int frequency;
  string file;

  nh.param<int>("img_publisher/frequency", frequency, 5);

  if (argc>1){
    file = argv[1];
  }
  else {
      nh.getParam("img_publisher/file", file);
  } 

  // defining topic publisher using image_transport
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("img_pub", frequency);

  //read input from the argument passed to the file
  cv::Mat image = cv::imread(file, CV_LOAD_IMAGE_COLOR);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(frequency);
  
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}