/** The node `harris_corners` extracts Harris corners from the image subscribed to by the ros topic `img_pub`. The final image is piblished on to a ros topic `harris_features` using cv_bridge and image_transport.
*
* PACKAGE: imagepub
* Contributor: Mayank Mittal
**/

#include "ros/ros.h"
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <sstream>
#include <stdio.h>

// dynamic reconfigure header files
#include <dynamic_reconfigure/server.h>
#include <harris/harrisParamConfig.h>

using namespace std;

//typedef to ease out datatype name
typedef dynamic_reconfigure::Server<harris::harrisParamConfig> Server;

//topics name for the node
string subName = "img_pub";
string pubName = "harris_features";

//frequency at which data to publish/subscrobe from topic
int frequency = 15;

// initalizing publisher and subscriber to topic
image_transport::Publisher pub;
image_transport::Subscriber sub;

class harrisCornerDetector {
  private:
    cv::Mat image;
    int template_size;
    double threshold;

  public:
    void configCallback(harris::harrisParamConfig &, uint32_t);
    cv::Mat convertToGray(cv::Mat );
    cv::Mat findHarrisFeatures(cv::Mat );
    void publishMessage();
    void imageCallback(const sensor_msgs::ImageConstPtr &); 
};

// callback function for dynamic reconfigure 
void harrisCornerDetector::configCallback(harris::harrisParamConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Harris Parameters, Template_Size: %d, Threshold %f", config.template_size, config.threshold);
  template_size = config.template_size;
  threshold = config.threshold;
}

//convert input image to grayscale
cv::Mat harrisCornerDetector::convertToGray(cv::Mat image){
  if (image.cv::Mat::channels() != 1) {
    cv::Mat gray;
        
    // convert RGB image to gray
    cvtColor(image, gray, CV_BGR2GRAY);

    return gray;
  }
  return image;
}

//find the Harris features in the input image using OpenCV library
cv::Mat harrisCornerDetector::findHarrisFeatures(cv::Mat gray){
  //The input image must be grayscale

  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros( gray.size(), CV_32FC1 );

  // Detector parameters
  int blockSize = template_size;
  int apertureSize = 3;
  double k = 0.04;  //Harris detector free parameter (typical value: 0.04)

  float strength = threshold*255;

  // Detecting corners
  cv::cornerHarris( gray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

  // Normalizing
  cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
  cv::convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ ) {
    for( int i = 0; i < dst_norm.cols; i++ ) {
      if( (int) dst_norm.at<float>(j,i) > (int) strength ) {
        cv::circle( dst_norm_scaled, cv::Point( i, j ), 5,  cv::Scalar(0), 2, 8, 0 );
      }    
    }
  }

  return dst_norm_scaled;
}

void harrisCornerDetector::publishMessage() {
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
   pub.publish(msg);
}

void harrisCornerDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
      image = cv_bridge::toCvCopy(msg, "bgr8")->image;
      image = harrisCornerDetector::convertToGray(image);
      image = findHarrisFeatures(image);
      publishMessage();
  }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


//main function
int main(int argc, char **argv) {

  ros::init(argc, argv, "harris_corners");

  // creating node handle
  ros::NodeHandle nh;

  // get paramters from rosparam server
  int template_size;
  double threshold;
  //nh.param<int>("harris_corners/template_size", template_size, 5);
  //nh.param<double>("harris_corners/threshold", threshold, 0.1);

  image_transport::ImageTransport it(nh);

  harrisCornerDetector H;
  //  H.setHarrisParam(template_size, threshold);

  // enabling dynamic reconfiguration server
  Server server;
  Server::CallbackType f;
  f = boost::bind(& harrisCornerDetector::configCallback, &H, _1, _2);
  server.setCallback(f);

  // defining topic subscriber using image transport
  sub = it.subscribe(subName, frequency, &harrisCornerDetector::imageCallback, &H);
  // defining topic publisher using image_transport
  pub = it.advertise(pubName, frequency);

  ros::spin();
  
  return 0;
}