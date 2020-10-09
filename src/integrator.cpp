/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <rad_msgs/TimepixImage.h>
#include <rad_msgs/ClusterList.h>

#include <dynamic_reconfigure/server.h>
#include <rospix_utils/integratorConfig.h>

#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>

//}

namespace utils
{

namespace integrator
{

/* class Integrator //{ */

class Integrator : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  double _publisher_rate_;

  ros::Subscriber image_subscriber_;
  ros::Subscriber cluster_list_subscriber_;

  ros::Publisher image_publisher_;

  ros::Timer timer_cleanup_;
  ros::Timer timer_publish_;

  // parameters set from config file
  bool   filter_broken_pixels;
  double filter_sigma;

  std::mutex mutex_image_out_;
  cv::Mat    image_out_ = cv::Mat::zeros(256, 256, CV_16UC1);

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef rospix_utils::integratorConfig           DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(rospix_utils::integratorConfig& config, uint32_t level);
  DrsConfig_t                                      drs_params_;
  std::mutex                                       mutex_drs_params_;

  // | ------------------------ callbacks ----------------------- |

  void imageCallback(const rad_msgs::TimepixImageConstPtr& image_in);
  void clusterListCallback(const rad_msgs::ClusterListConstPtr& cluster_list);

  // | ------------------------- timers ------------------------- |

  void timerCleanup([[maybe_unused]] const ros::TimerEvent& te);
  void timerPublish([[maybe_unused]] const ros::TimerEvent& te);

  // | --------------------- cluster memory --------------------- |

  std::vector<rad_msgs::ClusterList> cluster_lists_;
  std::mutex                         mutex_cluster_lists_;

  // | ------------------------ routines ------------------------ |

  void plotClusters();
};

//}

/* onInit() //{ */

void Integrator::onInit() {

  ros::NodeHandle nh("~");

  // | ------------------- load ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh, "Integrator");

  param_loader.loadParam("publisher_rate", _publisher_rate_);

  param_loader.loadParam("cleanup/enabled", drs_params_.cleanup_enabled);
  param_loader.loadParam("cleanup/duration", drs_params_.cleanup_duration);

  param_loader.loadParam("cluster_timeout/enabled", drs_params_.cluster_timeout_enabled);
  param_loader.loadParam("cluster_timeout/timeout", drs_params_.cluster_timeout);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Integrator]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ------------------- dynamic reconfigure ------------------ |

  drs_.reset(new Drs_t(mutex_drs_, nh));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&Integrator::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ---------------------- subscribers  ---------------------- |

  image_subscriber_        = nh.subscribe("image_in", 1, &Integrator::imageCallback, this, ros::TransportHints().tcpNoDelay());
  cluster_list_subscriber_ = nh.subscribe("cluster_list_in", 1, &Integrator::clusterListCallback, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  image_publisher_ = nh.advertise<rad_msgs::TimepixImage>("image_out", 1);

  // | ------------------------- timers ------------------------- |

  timer_cleanup_ = nh.createTimer(ros::Duration(drs_params_.cleanup_duration), &Integrator::timerCleanup, this, false, drs_params_.cleanup_enabled);
  timer_publish_ = nh.createTimer(ros::Duration(1.0 / _publisher_rate_), &Integrator::timerPublish, this);

  ROS_INFO("Starting integrator republisher for %s", image_subscriber_.getTopic().c_str());

  is_initialized_ = true;
}

//}

// | ------------------------ callbacks ----------------------- |

/* imageCallback() //{ */

void Integrator::imageCallback(const rad_msgs::TimepixImageConstPtr& image_in) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_image_out_);

  ROS_INFO("[%s]: got image", ros::this_node::getName().c_str());

  // iterate over all pixels if the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      long temp_val = image_out_.at<uint16_t>(i, j) + image_in->image[i * 256 + j];

      if (temp_val >= 65535) {
        temp_val = 65535;
      }

      image_out_.at<uint16_t>(i, j) = temp_val;
    }
  }
}

//}

/* clusterListCallback() //{ */

void Integrator::clusterListCallback(const rad_msgs::ClusterListConstPtr& cluster_list) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_image_out_);

  ROS_INFO("[%s]: got clusters", ros::this_node::getName().c_str());

  // iterate over all clusters
  cluster_lists_.push_back(*cluster_list);
}

//}

/* callbackDrs() //{ */

void Integrator::callbackDrs(rospix_utils::integratorConfig& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_params_ = config;
  }

  timer_cleanup_.stop();
  timer_cleanup_.setPeriod(ros::Duration(config.cleanup_duration), true);

  if (config.cleanup_enabled) {
    timer_cleanup_.start();
  }

  ROS_INFO("[Integrator]: DRS params updated");
}

//}

// | ------------------------- timers ------------------------- |

/* timerCleanup() //{ */

void Integrator::timerCleanup([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  std::scoped_lock lock(mutex_image_out_);

  ROS_INFO("[Integrator]: cleaning up the image");

  // iterate over all pixels if the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      image_out_.at<uint16_t>(i, j) = 0;
    }
  }
}

//}

/* timerPublish() //{ */

void Integrator::timerPublish([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  std::scoped_lock lock(mutex_image_out_);

  ROS_INFO("[Integrator]: publishing");

  plotClusters();

  // prepare a message for publishing
  rad_msgs::TimepixImage outputImage;

  // iterate over all pixels if the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {
      outputImage.image[i * 256 + j] = image_out_.at<uint16_t>(i, j);
    }
  }

  image_publisher_.publish(outputImage);
}

//}

// | ------------------------ routines ------------------------ |

/* plotClusters() //{ */

void Integrator::plotClusters() {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  // clear the image
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {
      image_out_.at<uint16_t>(i, j) = 0;
    }
  }

  // iterate over all clusters lists
  for (auto it = cluster_lists_.begin(); it != cluster_lists_.end();) {

    if (drs_params.cluster_timeout && (ros::Time::now() - it->header.stamp).toSec() > drs_params.cluster_timeout) {
      it = cluster_lists_.erase(it);
      ROS_INFO("[Integrator]: removing old clusters");
      continue;
    }

    for (size_t i = 0; i < it->clusters.size(); i++) {

      rad_msgs::Cluster cluster = it->clusters[i];

      for (size_t j = 0; j < cluster.pixels.size(); j++) {

        int x = cluster.pixels[j].x;
        int y = cluster.pixels[j].y;

        long temp_val = image_out_.at<uint16_t>(x, y) + cluster.pixels[j].energy;

        if (temp_val >= 65535) {
          temp_val = 65535;
        }

        image_out_.at<uint16_t>(x, y) = temp_val;
      }
    }

    it++;
  }
}

//}

}  // namespace integrator

}  // namespace utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(utils::integrator::Integrator, nodelet::Nodelet);
