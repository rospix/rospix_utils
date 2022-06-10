/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <rad_msgs/ClusterList.h>

#include <std_msgs/Float64.h>

#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>

//}

/* defines //{ */

#define DETECTOR_SIZE 0.014

//}

namespace rospix_utils
{

namespace dosimeter
{

/* class Dosimeter //{ */

class Dosimeter : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  double _publisher_rate_;
  double _time_window_;
  double _material_density_;
  double _detector_thickness_;

  bool   _filter_enabled_;
  double _filter_k_;

  double detector_mass_;

  ros::Subscriber subscriber_cluster_list_;
  ros::Publisher  publisher_dose_;

  double filtered_output_ = 0;

  ros::Timer timer_publish_;

  // | ------------------------ callbacks ----------------------- |

  void clusterListCallback(const rad_msgs::ClusterListConstPtr& cluster_list);

  // | ------------------------- timers ------------------------- |

  void timerPublish([[maybe_unused]] const ros::TimerEvent& te);

  // | --------------------- cluster memory --------------------- |

  std::vector<rad_msgs::ClusterList> cluster_lists_;
  std::mutex                         mutex_cluster_lists_;

  // | ------------------------ routines ------------------------ |

  double energykEv2J(const double in);
};

//}

/* onInit() //{ */

void Dosimeter::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh, ros::this_node::getName());

  param_loader.loadParam("publisher_rate", _publisher_rate_);

  param_loader.loadParam("time_window", _time_window_);
  param_loader.loadParam("material_density", _material_density_);
  param_loader.loadParam("detector_thickness", _detector_thickness_);

  param_loader.loadParam("filter/enabled", _filter_enabled_);
  param_loader.loadParam("filter/k", _filter_k_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Dosimeter]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------- calculate the detector mass -------------- |

  detector_mass_ = _material_density_ * (_detector_thickness_ * DETECTOR_SIZE * DETECTOR_SIZE);
  ROS_INFO("[Dosimeter]: detector mass %.4f", detector_mass_);

  // | ----------------------- publishers ----------------------- |

  publisher_dose_ = nh.advertise<std_msgs::Float64>("dose_out", 1);

  // | ---------------------- subscribers  ---------------------- |

  subscriber_cluster_list_ = nh.subscribe("cluster_list_in", 1, &Dosimeter::clusterListCallback, this, ros::TransportHints().tcpNoDelay());

  // | ------------------------- timers ------------------------- |

  timer_publish_ = nh.createTimer(ros::Duration(1.0 / _publisher_rate_), &Dosimeter::timerPublish, this);

  is_initialized_ = true;

  ROS_INFO("[Dosimeter]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* clusterListCallback() //{ */

void Dosimeter::clusterListCallback(const rad_msgs::ClusterListConstPtr& cluster_list) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[%s]: getting clusters", ros::this_node::getName().c_str());

  // iterate over all clusters
  cluster_lists_.push_back(*cluster_list);
}

//}

// | ------------------------- timers ------------------------- |

/* timerPublish() //{ */

void Dosimeter::timerPublish([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  // | ------------------- remove old clusters ------------------ |

  for (auto it = cluster_lists_.begin(); it != cluster_lists_.end();) {

    if ((ros::Time::now() - it->header.stamp).toSec() > _time_window_) {
      it = cluster_lists_.erase(it);
      continue;
    }

    it++;
  }

  // | ------------- calculate the total energy dose ------------ |

  double total_dose = 0;

  for (auto it = cluster_lists_.begin(); it != cluster_lists_.end(); it++) {

    for (auto it2 = it->clusters.begin(); it2 != it->clusters.end(); it2++) {

      total_dose += energykEv2J(it2->energy);
    }
  }

  // -> J/kg
  total_dose = total_dose / detector_mass_;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [J/kg]", total_dose);

  // -> Gy per second
  total_dose = total_dose / _time_window_;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [Gy/s]", total_dose);

  // -> uGy per second
  double total_dose_mgy = total_dose * 1000.0;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [uGy/s]", total_dose);

  // -> uGy per second
  total_dose = total_dose * 1e6;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [uGy/s]", total_dose);

  // -> uGy per minute
  total_dose = 60.0 * total_dose;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [uGy/min]", total_dose);

  // -> mGy per year
  total_dose_mgy = 60 * 60 * 24 * 365 * total_dose_mgy;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [mGy/year]", total_dose_mgy);

  if (_filter_enabled_) {
    filtered_output_ = _filter_k_ * filtered_output_ + (1.0 - _filter_k_) * total_dose_mgy;
  } else {
    filtered_output_ = total_dose_mgy;
  }

  std_msgs::Float64 msg;
  msg.data = filtered_output_;

  publisher_dose_.publish(msg);
}

//}

// | ------------------------ routines ------------------------ |

/* energykEv2J() //{ */

double Dosimeter::energykEv2J(const double in) {

  return (1000.0 * in) / (6.242e18);
}

//}

}  // namespace dosimeter

}  // namespace rospix_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rospix_utils::dosimeter::Dosimeter, nodelet::Nodelet);
