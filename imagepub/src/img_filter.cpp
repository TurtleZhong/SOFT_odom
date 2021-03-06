/** The node `img_filter` subscribes to the ros topic `img_pub` and reads the parameter from the parameter server, namely the frequency of publishing data to a topic. On the basis of the paramters read through dynamic reconfiguration in ROS, it adjusts the contrast and brighntess of the image. The final image is piblished on to a ros topic `img_filt` using cv_bridge and image_transport.
*
* PACKAGE: imagepub
* Contributor: Mayank Mittal
**/

#include "ros/ros.h"
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <stdio.h>

// dynamic reconfigure header files
#include <dynamic_reconfigure/server.h>
#include <imagepub/imgFilterConfig.h>

using namespace std;

//typedef to ease out datatype name
typedef dynamic_reconfigure::Server<imagepub::imgFilterConfig> Server;

// global variable
cv::Mat image;
cv::Mat new_image;
double contrast;
int brightness;

// initalizing publisher and subscriber to topic
image_transport::Publisher pub;
image_transport::Subscriber sub;

// callback function for dynamic reconfigure 
void callback(imagepub::imgFilterConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Filter Parameters, Contrast: %f, Brightness %d", config.contrast, config.brightness);
  contrast = config.contrast;
  brightness = config.brightness;
}

// image processing task new_image = contrast*image + brightness
cv::Mat adjustImage(cv::Mat image) {
  cv::Mat new_image = cv::Mat::zeros( image.size(), image.type() );
  for( int y = 0; y < image.rows; y++ ) {
    for( int x = 0; x < image.cols; x++ ) { 
      for( int c = 0; c < 3; c++ ) {
      new_image.cv::Mat::at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>( contrast*( image.cv::Mat::at<cv::Vec3b>(y,x)[c] ) + brightness );
      }
    }
  }
  return new_image;
}

// callback function for subscriber to topic img_pub
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // perform contrast and brightness adjustment
    new_image = adjustImage(image);
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", new_image).toImageMsg();
    pub.publish(msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

// main function
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "image_publisher");

  // creating node handle
  ros::NodeHandle nh;

  // get paramters from rosparam server
  int frequency;
  nh.param<int>("img_publisher/frequency", frequency, 5);
  
  // enabling dynamic reconfiguration server
  Server server;
  Server::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  image_transport::ImageTransport it(nh);
  // defining topic subscriber using image transport
  sub = it.subscribe("img_pub", frequency, imageCallback);
  // defining topic publisher using image_transport
  pub = it.advertise("img_filt", frequency);

  ros::spin();

  return 0;
}