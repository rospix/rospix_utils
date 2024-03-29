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

  ros::Publisher publisher_dose_ngy_s_;
  ros::Publisher publisher_dose_mgy_y_;

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

  publisher_dose_ngy_s_ = nh.advertise<std_msgs::Float64>("dose_ngy_s_out", 1);
  publisher_dose_mgy_y_ = nh.advertise<std_msgs::Float64>("dose_out", 1);

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

  // -> mGy per second
  double total_dose_mgy_s = total_dose * 1000.0;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [mGy/s]", total_dose);

  // -> nGy per second
  double total_dose_ngy_s = total_dose * 1e9;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [nGy/s]", total_dose_ngy_s);

  // -> nGy per minute
  total_dose = 60.0 * total_dose_ngy_s;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [nGy/min]", total_dose);

  // -> mGy per year
  double total_dose_mgy_y = 60 * 60 * 24 * 365 * total_dose_mgy_s;
  ROS_INFO_THROTTLE(1.0, "[Dosimeter]: %f [mGy/year]", total_dose_mgy_y);

  if (_filter_enabled_) {
    filtered_output_ = _filter_k_ * filtered_output_ + (1.0 - _filter_k_) * total_dose_mgy_y;
  } else {
    filtered_output_ = total_dose_mgy_y;
  }

  std_msgs::Float64 msg_mgy_y;
  msg_mgy_y.data = filtered_output_;

  std_msgs::Float64 msg_ngy_s;
  msg_ngy_s.data = total_dose_ngy_s;

  publisher_dose_ngy_s_.publish(msg_ngy_s);
  publisher_dose_mgy_y_.publish(msg_mgy_y);
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
