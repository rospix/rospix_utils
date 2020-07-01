// some ros includes
#include <ros/package.h>
#include <ros/ros.h>

// some std includes
#include <stdio.h>
#include <stdlib.h>

// some opencv includes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <rad_msgs/TimepixImage.h>
#include <rad_msgs/ClusterList.h>

using namespace std;
using namespace cv;

// subscriber for input timepix images
ros::Subscriber image_subscriber;
ros::Subscriber cluster_list_subscriber;

// publisher for output images
ros::Publisher image_publisher;

// parameters set from config file
bool   filter_broken_pixels;
double filter_sigma;

cv::Mat image_out = cv::Mat::zeros(256, 256, CV_16UC1);

// is called every time new image comes in
void imageCallback(const rad_msgs::TimepixImageConstPtr& image_in) {

  ROS_INFO("[%s]: got image", ros::this_node::getName().c_str());

  // iterate over all pixels if the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      long temp_val = image_out.at<uint16_t>(i, j) + image_in->image[i * 256 + j];

      if (temp_val >= 65535) {
        temp_val = 65535;
      }

      image_out.at<uint16_t>(i, j) = temp_val;

    }
  }

  // prepare a message for publishing
  rad_msgs::TimepixImage outputImage;

  // iterate over all pixels if the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      outputImage.image[i * 256 + j] = image_out.at<uint16_t>(i, j);

    }
  }

  image_publisher.publish(outputImage);
}

// is called every time new image comes in
void clusterListCallback(const rad_msgs::ClusterListConstPtr& cluster_list) {

  ROS_INFO("[%s]: got clusters", ros::this_node::getName().c_str());

  // iterate over all clusters
  for (size_t i = 0; i < cluster_list->clusters.size(); i++) {

    rad_msgs::Cluster cluster = cluster_list->clusters[i];

    for (size_t j = 0; j < cluster.pixels.size(); j++) {

      int x = cluster.pixels[j].x;
      int y = cluster.pixels[j].y;

      long temp_val = image_out.at<uint16_t>(x, y) + cluster.pixels[j].energy;

      if (temp_val >= 65535) {
        temp_val = 65535;
      }

      image_out.at<uint16_t>(x, y) = temp_val;

    }
  }

  // prepare a message for publishing
  rad_msgs::TimepixImage outputImage;

  // iterate over all pixels if the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      outputImage.image[i * 256 + j] = image_out.at<uint16_t>(i, j);

    }
  }

  image_publisher.publish(outputImage);
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "integrator");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // SUBSCRIBERS
  image_subscriber = nh_.subscribe("image_in", 1, &imageCallback, ros::TransportHints().tcpNoDelay());
  cluster_list_subscriber = nh_.subscribe("cluster_list_in", 1, &clusterListCallback, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  image_publisher = nh_.advertise<rad_msgs::TimepixImage>("image_out", 1);

  ROS_INFO("Starting integrator republisher for %s", image_subscriber.getTopic().c_str());

  // needed for stuff to work
  ros::spin();

  return 0;
}
