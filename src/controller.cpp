// some ros includes
#include <ros/package.h>
#include <ros/ros.h>

// some std includes
#include <stdio.h>
#include <stdlib.h>

// neccessary to read the image from rospix
#include <rad_msgs/TimepixImage.h>
#include <rad_msgs/RospixExposure.h>
#include <rad_msgs/SetDouble.h>

// messages for setting exposure time

// message for publishing exposure
#include <std_msgs/Float64.h>

// thread and mutex for periodic publishing of exposure time
#include <mutex>
#include <thread>

using namespace std;

// subscriber for camera images
ros::Subscriber image_subscriber;

// publisher for the exposure time... this is just for showing how publishers work
ros::Publisher exposure_time_publisher;

// service clients
ros::ServiceClient service_exposure_time;
ros::ServiceClient service_do_exposure;

// parameters loaded from config file
int    desired_pixel_count_;
double max_exposure_;
double min_exposure_;
double max_exposure_step_;

// gain of proportional feedback to control the xposure
double proportial_gain_;

// store last time we got an image
ros::Time last_time_image;

// have we got any image yet?
bool got_image = false;

// stores the desired exposure time
double     exposure;
std::mutex mutex_exposure;  // to lock it up between the threads

// store value of previous exposure, to chech in time whether the reposnse came
double previous_exposure = 1;

// thread
std::thread publisher_thread;

// thread function
void publisherThread(void) {

  ros::Rate d(10);  // set rate to 10H

  while (ros::ok()) {

    // publish the exposure time
    std_msgs::Float64 newMessage;

    // protect the shared variable
    mutex_exposure.lock();
    { newMessage.data = exposure; }
    mutex_exposure.unlock();

    // publish the message
    exposure_time_publisher.publish(newMessage);

    d.sleep();
  }
}

// is called every time new image comes in
void imageCallback(const rad_msgs::TimepixImageConstPtr& msg) {

  got_image       = true;
  last_time_image = ros::Time::now();

  // count the number of active pixels in the image

  int pixel_count = 0;

  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      if (msg->image[i * 256 + j] > 0) {
        pixel_count++;
      }
    }
  }

  // we are gonna touch a shared variable, protect it by mutex
  mutex_exposure.lock();
  {

    double error = double(desired_pixel_count_) - double(pixel_count);

    previous_exposure      = exposure;
    double exposure_update = error * proportial_gain_;

    // saturate the exposure update
    if (exposure_update > max_exposure_step_)
      exposure_update = max_exposure_step_;
    else if (exposure_update < -max_exposure_step_)
      exposure_update = -max_exposure_step_;

    // update the exposure time
    exposure += exposure_update;

    // saturate the exposure time to the bounds
    if (exposure > max_exposure_)
      exposure = max_exposure_;
    else if (exposure < min_exposure_)
      exposure = min_exposure_;

    // change the exposure time
    rad_msgs::SetDouble newMessage;
    newMessage.request.value = exposure;

    // call the service
    if (service_exposure_time.call(newMessage)) {
      ROS_INFO("Changed exposure to %2.3f s, pixels=%d", exposure, pixel_count);
    } else {
      ROS_ERROR("Failed to set the exposure.");
    }
  }
  mutex_exposure.unlock();
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // load parameters
  // the defaults parameters should not be zero
  // but I find it easier to notice that I am loading them wrong if the defaults are nonsense
  nh_.param("desired_pixel_count", desired_pixel_count_, 0);
  nh_.param("max_exposure", max_exposure_, 0.0);
  nh_.param("min_exposure", min_exposure_, 0.0);
  nh_.param("init_exposure", exposure, 0.0);
  nh_.param("proportional_gain", proportial_gain_, 0.0);
  nh_.param("max_exposure_step", max_exposure_step_, 0.0);

  // tell us we have started
  ROS_INFO("Exposure controller started with parameters: deisred_pixel_count=%d, max_exposure=%2.2f, min_exposure=%2.2f, init_exposure=%2.2f.",
           desired_pixel_count_, max_exposure_, min_exposure_, exposure);

  // SUBSCRIBERS
  image_subscriber = nh_.subscribe("image_in", 1, &imageCallback, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  exposure_time_publisher = nh_.advertise<std_msgs::Float64>("exposure_time", 1);

  // SERVICES
  service_do_exposure   = nh_.serviceClient<rad_msgs::RospixExposure>("do_exposure");  // we choose some general topic name and later remap it in launch file
  service_exposure_time = nh_.serviceClient<rad_msgs::SetDouble>("set_exposure");

  // create the publisher thread
  publisher_thread = std::thread(&publisherThread);

  // initialize times
  last_time_image = ros::Time::now();

  // check whether the sensor is alive
  ros::Rate r(10);  // 10 times per second
  while (ros::ok()) {

    // we are gonna touch a shared variable, protect it with mutex
    mutex_exposure.lock();
    {
      // if we have not got an imave for more then exposure+1 seconds
      if (!got_image || ((ros::Time::now() - last_time_image).toSec() > (previous_exposure + 1))) {

        // reinitiate the exposures
        rad_msgs::RospixExposure newMessage;
        newMessage.request.exposure_time = exposure;

        if (service_do_exposure.call(newMessage)) {
          ROS_INFO_THROTTLE(1, "Initiated exposures with exposure time %2.2f", exposure);
        } else {
          ROS_ERROR_THROTTLE(1, "Error initiating exposures, is ROSpix running?");
        }
      }
    }
    mutex_exposure.unlock();

    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
