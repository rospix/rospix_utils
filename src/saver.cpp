// some ros includes
#include <ros/package.h>
#include <ros/ros.h>

// some std includes
#include <stdio.h>
#include <stdlib.h>

// Image message type is defined by the rospix node
#include <rospix/Image.h>
#include <std_msgs/Float64.h>

using namespace std;

// subscriber for camera images
ros::Subscriber image_subscriber;
ros::Subscriber temperature_0_subscriber, temperature_3_subscriber;

// stores the name of the directory to save files
string directory;

double temperature0, temperature3;

void temp0CallBack(const std_msgs::Float64::ConstPtr& msg) {
  temperature0 = msg->data;
}

void temp3CallBack(const std_msgs::Float64::ConstPtr& msg) {
  temperature3 = msg->data;
}

// is called every time new image comes in
void imageCallback(const rospix::ImageConstPtr& msg) {

  double date = ros::Time::now().toSec();

  // create a filename based on current time, precision on miliseconds should be enough
  char filename[40];
  sprintf(filename, "%.3f.image.txt", date);

  // add the directory name
  string path = string(directory + string(filename));

  // tell us about saving the image
  ROS_INFO("Saving image to %s", path.c_str());

  // open the file
  FILE* f = fopen(path.c_str(), "w");

  if (f == NULL) {
    ROS_ERROR("Cannot open the file %s for writing.", path.c_str());
  } else {

    // iterate over all pixels of the image
    for (int i = 0; i < 256; i++) {
      for (int j = 0; j < 256; j++) {

        // print the value
        fprintf(f, "%d ", msg->image[i * 256 + j]);
      }

      fprintf(f, "\n");
    }

    // probably not neccessary, but to be sure...
    fflush(f);

    fclose(f);
  }

  // metadata file name
  sprintf(filename, "%.3f.metadata.txt", date);
  path = string(directory + string(filename));

  // open metadata file
  f = fopen(path.c_str(), "w");

  if (f == NULL) {
    ROS_ERROR("Cannot open the file %s for writing.", path.c_str());
  } else {

    // print all metadata
    fprintf(f, "time: %f\n", msg->stamp.toSec());
    fprintf(f, "exposure_time: %f\n", msg->exposure_time);
    fprintf(f, "mode: %s\n", msg->mode == 0 ? "MPX" : "TOT");
    fprintf(f, "threshold: %d\n", msg->threshold);
    fprintf(f, "bias: %f\n", msg->bias);
    fprintf(f, "interface: %s\n", msg->interface.c_str());
    fprintf(f, "clock: %d\n", msg->clock);
    fprintf(f, "temperature_0: %.2f\n", temperature0);
    fprintf(f, "temperature-3: %.2f\n", temperature3);

    // probably not neccessary, but to be sure...
    fflush(f);

    fclose(f);
  }
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "saver");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // load parameters from config file (launch file)
  nh_.param("directory", directory, string());

  // SUBSCRIBERS
  image_subscriber         = nh_.subscribe("image_in", 1, &imageCallback, ros::TransportHints().tcpNoDelay());
  temperature_0_subscriber = nh_.subscribe("value0", 1, &temp0CallBack, ros::TransportHints().tcpNoDelay());
  temperature_3_subscriber = nh_.subscribe("value3", 1, &temp3CallBack, ros::TransportHints().tcpNoDelay());


  // needed to make stuff work
  ros::spin();

  return 0;
}
