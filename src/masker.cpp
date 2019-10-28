// some ros includes
#include <ros/package.h>
#include <ros/ros.h>

// some std includes
#include <stdio.h>
#include <stdlib.h>

// some opencv includes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rospix/Image.h>

// Image message type is defined by the rospix node
#include <rospix/Image.h>

#include <mrs_lib/ParamLoader.h>

using namespace std;
using namespace cv;

// subscriber for input timepix images
ros::Subscriber image_subscriber;

// publisher for output images
ros::Publisher image_publisher;

// parameters set from config file
bool   filter_broken_pixels;
double filter_sigma;

Eigen::MatrixXd masked_pixels;

cv::Mat image_out = cv::Mat::zeros(256, 256, CV_16UC1);

// is called every time new image comes in
void imageCallback(const rospix::ImageConstPtr& image_in) {

  ROS_INFO("[%s]: got image", ros::this_node::getName().c_str());

  // prepare a message for publishing
  rospix::Image outputImage = *image_in;

  // iterate over all pixels if the image
  for (int i = 0; i < masked_pixels.rows(); i++) {

    outputImage.image[masked_pixels(i, 0) * 256 + masked_pixels(i, 1)] = 0;
  }

  image_publisher.publish(outputImage);
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "masker");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // SUBSCRIBERS
  image_subscriber = nh_.subscribe("image_in", 1, &imageCallback, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  image_publisher = nh_.advertise<rospix::Image>("image_out", 1);

  mrs_lib::ParamLoader param_loader(nh_, "Masker");

  masked_pixels = param_loader.load_matrix_dynamic2("masked_pixels", -1, 2);

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("Could not load all parameters!");
    ros::shutdown();
  }

  ROS_INFO("Starting masker republisher for %s", image_subscriber.getTopic().c_str());

  // needed for stuff to work
  ros::spin();

  return 0;
}
