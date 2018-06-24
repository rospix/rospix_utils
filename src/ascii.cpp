// some ros includes
#include <ros/ros.h>
#include <ros/package.h>

// some std includes
#include <stdlib.h>
#include <stdio.h>

// neccessary to read the image from rospix
#include <rospix/Image.h>

using namespace std;

// subscriber for camera images
ros::Subscriber image_subscriber;

// overallocate, just to be sure.. we have 2GB of ram, this will not hurt
int8_t output[65536];

// size * bin_size = 256
int size; // size of the output image
int bin_size; // size of the bin 

// is called every time new image comes in
void imageCallback(const rospix::ImageConstPtr& msg) {

  // clear the output array
  memset(output, 0, size * size * sizeof(uint8_t));

  // have we found an active pixel within the bin?
  bool found = false;

  // iterate over all binns 
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {

      found = false;

      // find some nonzero pixel in the particular bin of the image
      for (int k = i*bin_size; k < i*bin_size + bin_size; k++) {
        for (int l = j*bin_size; l < j*bin_size + bin_size; l++) {

          // we have an active pixel
          if (msg->image[k*256 + l] > 0) {

            // turn on the bin
            output[i*size + j] = 1;
            found = true;
            break; // leave the cycle
          }
        }
        if (found) { // leave that too, not needed to go further in the bin
          break;
        }
      }
    }
  }

  // output metadata
  ROS_INFO("Image parameters: time=%f, mode=%s, exposure=%2.2f, bias=%2.2f, threshold=%d, interface=%s, clock=%d", msg->stamp.toSec(), ((msg->mode == 0) ? "MPX" : "TOT"), msg->exposure_time, msg->bias, msg->threshold, msg->interface.c_str(), msg->clock);

  // print top of the image frame
  for (int i = 0; i < size+2; i++) {
    printf("-");
  }

  printf("\n");

  // for all rows of the ascii image
  for (int i = 0; i < size; i++) {

    printf("|"); // frame left

    // for all columns of the row of the acii image
    for (int j = 0; j < size; j++) {
      printf("%c", (output[i*size + j] == 0) ? ' ' : 'x');     
    }

    printf("|"); // frame right
    printf("\n");
  }  

  // print the frame bottom
  for (int i = 0; i < size+2; i++) {
    printf("-");
  }
 
  printf("\n");
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "ascii");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // load parameters from config file
  nh_.param("size", size, 16);
  bin_size = 256 / size;

  // SUBSCRIBERS
  image_subscriber = nh_.subscribe("image_in", 1, &imageCallback, ros::TransportHints().tcpNoDelay());

  // needed to do stuff automatically
  ros::spin();

  return 0;
}
