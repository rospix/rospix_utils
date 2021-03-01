/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <rad_msgs/TimepixImage.h>

#include <mrs_lib/param_loader.h>

//}

namespace rospix_utils
{

namespace normalizer
{

/* class Normalizer //{ */

class Normalizer : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  // subscriber for input timepix images
  ros::Subscriber image_subscriber_;

  // publisher for output images
  image_transport::Publisher image_publisher_;

  // parameters set from config file
  bool   filter_broken_pixels;
  double filter_sigma;
  bool   invert_ = false;

  void imageCallback(const rad_msgs::TimepixImageConstPtr& image_in);
};

//}

/* imageCallback() //{ */

void Normalizer::imageCallback(const rad_msgs::TimepixImageConstPtr& image_in) {

  ROS_INFO("[%s]: got image", ros::this_node::getName().c_str());

  // prepare new output image
  cv::Mat image_out = cv::Mat::zeros(256, 256, CV_16UC1);

  // set some stuff
  int    max = -1;
  int    min = 65535;
  int    im_value;
  double mean    = 0;
  int    meanidx = 1;

  // iterate over all pixels if the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      // copy the value for later use
      im_value = image_in->image[i * 256 + j];

      // calculating mean value of the image
      if (im_value > 0) {

        if (mean == 0) {
          mean = im_value;
        } else {
          // just a clever formula for cumulative mean
          mean = mean + (im_value - mean) / (meanidx++ + 1);
        }
      }

      // finding image maximum and minimum value
      if (im_value > max)
        max = im_value;
      if (im_value < min)
        min = im_value;
    }
  }

  // calculate standard deviation of pixel values
  double std    = 0;
  int    stdidx = 1;

  // again, iterate over all pixels of the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      // again, copy the value for later use
      im_value = image_in->image[i * 256 + j];

      // calculate standard deviation, again as a cumulative mean deviation from the mean value
      if (im_value > 0) {

        if (std == 0) {
          std = pow(mean - im_value, 2);
        } else {
          // just cumulative mean
          std = std + (sqrt(pow(mean - im_value, 2)) - std) / (stdidx++ + 1);
        }
      }
    }
  }

  // alter the new maximum, if we want to filter out outliers
  if (filter_broken_pixels) {

    // calculate a new maximum value based on the mean and standard deviation
    max = mean + std * filter_sigma;
  }

  // again, go over all pixels of the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      // again, copy the value for later use
      im_value = image_in->image[i * 256 + j];

      if (im_value <= max)
        // normalize the pixel value... basically stretches the histogram to match full 16bits of the image
        image_out.at<uint16_t>(i, j) = (im_value - min) * ((65535 - 0) / (max - min + 1)) + 0;
      else
        image_out.at<uint16_t>(i, j) = 65535;
    }
  }

  if (invert_) {

    for (int i = 0; i < 256; i++) {
      for (int j = 0; j < 256; j++) {

        // normalize the pixel value... basically stretches the histogram to match full 16bits of the image
        image_out.at<uint16_t>(i, j) = 65535 - image_out.at<uint16_t>(i, j);
      }
    }
  }

  // prepare a message for publishing
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", image_out).toImageMsg();

  // publish the message
  image_publisher_.publish(msg);
}

//}

/* onInit() //{ */

void Normalizer::onInit() {

  ros::NodeHandle nh("~");

  mrs_lib::ParamLoader param_loader(nh, "Normalizer");

  param_loader.loadParam("filter_broken_pixels", filter_broken_pixels);
  param_loader.loadParam("filter_sigma", filter_sigma);
  param_loader.loadParam("invert", invert_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("Could not load all parameters!");
    ros::shutdown();
  }

  // SUBSCRIBERS
  image_subscriber_ = nh.subscribe("image_in", 1, &Normalizer::imageCallback, this, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  image_transport::ImageTransport it(nh);
  image_publisher_ = it.advertise("image_out", 1);

  ROS_INFO("Starting normalizing republisher for %s", image_subscriber_.getTopic().c_str());

  is_initialized_ = true;
}

//}

}  // namespace normalizer

}  // namespace rospix_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rospix_utils::normalizer::Normalizer, nodelet::Nodelet);
