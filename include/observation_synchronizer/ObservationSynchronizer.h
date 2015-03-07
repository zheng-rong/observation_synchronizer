/*
observation_synchronizer: Synchronizes multiple data types with time-stamp sorting
Copyright (C) 2013  Nathan Michael

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef OBSERVATION_SYNCHRONIZER_H
#define OBSERVATION_SYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <scan_utils/TrimmedLaserScan.h>
#include <tof_utils/TrimmedTOF.h>
#include <quadrotor_msgs/AltitudeObservation.h>
#include <quadrotor_msgs/HeightDelta.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <quadrotor_msgs/LocalOdometry.h>

class ObservationSynchronizer
{
public:
  ObservationSynchronizer();
  ~ObservationSynchronizer();

  typedef enum {
    IMU,
    ATTITUDE,
    ODOM,
    LASER,
    TRIMMEDLASER,
    TWIST,
    HEIGHT,
    ICP,
    DH,
    TOF,
    TRIMMEDTOF,
    IMAGE,
    VO
  } sensor_types_t;

  void sortPendingMessages();
  bool getNextPendingMessage(sensor_types_t& type, unsigned int& index);
  bool nextPendingMessageExists();

  void clearPendingMessages();

  template<class T>
  struct Tagged
  {
    typename T::ConstPtr msg;
    std::string tag;
    typedef boost::shared_ptr< Tagged<T> > Ptr;
    typedef boost::shared_ptr< const Tagged<T> > ConstPtr;
    Tagged(const typename T::ConstPtr m, const std::string& t) : msg(m), tag(t) {}
  };

  typedef std::vector< Tagged<sensor_msgs::LaserScan>::ConstPtr > scan_queue;
  typedef std::vector< Tagged<sensor_msgs::Imu>::ConstPtr > imu_queue;
  typedef std::vector< Tagged<sensor_msgs::Imu>::ConstPtr > attitude_queue;
  typedef std::vector< Tagged<nav_msgs::Odometry>::ConstPtr > odom_queue;
  typedef std::vector< Tagged<geometry_msgs::PoseWithCovarianceStamped>::ConstPtr > icp_queue;
  typedef std::vector< Tagged<quadrotor_msgs::LocalOdometry>::ConstPtr > vo_queue;
  typedef std::vector< Tagged<scan_utils::TrimmedLaserScan>::ConstPtr > trimmed_scan_queue;
  typedef std::vector< Tagged<quadrotor_msgs::AltitudeObservation>::ConstPtr > height_queue;
  typedef std::vector< Tagged<geometry_msgs::TwistWithCovarianceStamped>::ConstPtr > twist_queue;
  typedef std::vector< Tagged<sensor_msgs::PointCloud2>::ConstPtr > tof_queue;
  typedef std::vector< Tagged<quadrotor_msgs::HeightDelta>::ConstPtr > dh_queue;
  typedef std::vector< Tagged<tof_utils::TrimmedTOF>::ConstPtr > trimmed_tof_queue;
  typedef std::vector< Tagged<sensor_msgs::Image>::ConstPtr > image_queue;

  const scan_queue& getPendingLaserScanMessages();
  const trimmed_scan_queue& getPendingTrimmedLaserScanMessages();
  const imu_queue& getPendingImuMessages();
  const attitude_queue& getPendingAttitudeMessages();
  const odom_queue& getPendingOdometryMessages();
  const icp_queue& getPendingIcpMessages();
  const vo_queue& getPendingVoMessages();
  const height_queue& getPendingHeightMessages();
  const dh_queue& getPendingDhMessages();
  const twist_queue& getPendingTwistMessages();
  const tof_queue& getPendingTOFMessages();
  const trimmed_tof_queue& getPendingTrimmedTOFMessages();
  const image_queue& getPendingImageMessages();

  const Tagged<nav_msgs::Odometry>::ConstPtr& getPendingOdometryMessage(unsigned int index);
  const Tagged<geometry_msgs::PoseWithCovarianceStamped>::ConstPtr& getPendingIcpMessage(unsigned int index);
  const Tagged<quadrotor_msgs::LocalOdometry>::ConstPtr& getPendingVoMessage(unsigned int index);
  const Tagged<sensor_msgs::Imu>::ConstPtr& getPendingImuMessage(unsigned int index);
  const Tagged<sensor_msgs::Imu>::ConstPtr& getPendingAttitudeMessage(unsigned int index);
  const Tagged<sensor_msgs::LaserScan>::ConstPtr&
  getPendingLaserScanMessage(unsigned int index);
  const Tagged<scan_utils::TrimmedLaserScan>::ConstPtr&
  getPendingTrimmedLaserScanMessage(unsigned int index);
  const Tagged<quadrotor_msgs::AltitudeObservation>::ConstPtr&
  getPendingHeightMessage(unsigned int index);
  const Tagged<quadrotor_msgs::HeightDelta>::ConstPtr&
  getPendingDhMessage(unsigned int index);
  const Tagged<geometry_msgs::TwistWithCovarianceStamped>::ConstPtr&
  getPendingTwistMessage(unsigned int index);
  const Tagged<sensor_msgs::PointCloud2>::ConstPtr& getPendingTOFMessage(unsigned int index);
  const Tagged<tof_utils::TrimmedTOF>::ConstPtr& getPendingTrimmedTOFMessage(unsigned int index);
  const Tagged<sensor_msgs::Image>::ConstPtr& getPendingImageMessage(unsigned int index);


  void addOdometryMessage(const nav_msgs::Odometry::ConstPtr& msg,
                          const std::string& tag = std::string());
  void addIcpMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                     const std::string& tag = std::string());
  void addVoMessage(const quadrotor_msgs::LocalOdometry::ConstPtr& msg,
                     const std::string& tag = std::string());
  void addLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& msg,
                           const std::string& tag = std::string());
  void addTrimmedLaserScanMessage(const scan_utils::TrimmedLaserScan::ConstPtr& msg,
                                  const std::string& tag = std::string());
  void addImuMessage(const sensor_msgs::Imu::ConstPtr& msg,
                     const std::string& tag = std::string());
  void addAttitudeMessage(const sensor_msgs::Imu::ConstPtr& msg,
                          const std::string& tag = std::string());
  void addHeightMessage(const quadrotor_msgs::AltitudeObservation::ConstPtr& msg,
                        const std::string& tag = std::string());
  void addDhMessage(const quadrotor_msgs::HeightDelta::ConstPtr& msg,
                        const std::string& tag = std::string());
  void addTwistMessage(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg,
                        const std::string& tag = std::string());
  void addTOFMessage(const sensor_msgs::PointCloud2::ConstPtr& msg,
                     const std::string& tag = std::string());
  void addTrimmedTOFMessage(const tof_utils::TrimmedTOF::ConstPtr& msg,
                            const std::string& tag = std::string());
  void addImageMessage(const sensor_msgs::Image::ConstPtr& msg,
                       const std::string& tag = std::string());

  static std::string getTypeString(const sensor_types_t& t)
  {
    switch (t)
      {
      case LASER:
        return std::string("LASER");
      case IMU:
        return std::string("IMU");
      case ATTITUDE:
        return std::string("ATTITUDE");
      case ODOM:
        return std::string("ODOM");
      case ICP:
        return std::string("ICP");
      case VO:
        return std::string("VO");
      case TRIMMEDLASER:
        return std::string("TRIMMEDLASER");
      case HEIGHT:
        return std::string("HEIGHT");
      case DH:
        return std::string("DH");
      case TWIST:
        return std::string("TWIST");
      case TOF:
        return std::string("TOF");
      case TRIMMEDTOF:
        return std::string("TRIMMEDTOF");
      case IMAGE:
        return std::string("IMAGE");
      }
  }

  struct TimedObservations
  {
    double time;
    double epsilon;
    std::map< sensor_types_t, std::vector<double> > sensor_times;
    std::vector< std::pair<sensor_types_t, unsigned int> > observations;
    TimedObservations(double t, double eps) : time(t), epsilon(eps) { }
    bool inSet(double t)
    {
      return fabs(time - t) < epsilon ? true : false;
    }
    void addObservation(sensor_types_t t, unsigned int index, double time)
    {
      if (sensor_times.count(t) == 0)
      {
        observations.push_back(std::make_pair(t, index));
        sensor_times.insert(std::make_pair(t, std::vector<double>()));
        sensor_times[t].push_back(time);
      }
      else
      {
        bool is_duplicate = false;
        for (std::vector<double>::const_iterator it = sensor_times[t].begin();
             it != sensor_times[t].end(); ++it)
        {
          if (fabs(*it - time) < 1e-9)
          {
            is_duplicate = true;
            break;
          }
        }

        if (!is_duplicate)
        {
          observations.push_back(std::make_pair(t, index));
          sensor_times[t].push_back(time);
        }
      }
    }

    typedef boost::shared_ptr<TimedObservations> Ptr;
    typedef boost::shared_ptr<const TimedObservations> ConstPtr;
  };

  void printContents();

  void sortPendingMessagesByTime(double time_epsilon);
  bool getNextPendingTimedObservations(unsigned int& index);
  const TimedObservations::ConstPtr& getTimedObservations(unsigned int i);
  bool nextPendingTimedObservationsExists();
  unsigned int getNumberOfTimeKeyGroups();

private:
  struct StampedSensorType
  {
    double time;
    sensor_types_t type;
    unsigned int index;
    StampedSensorType(double t, sensor_types_t s, unsigned int i) :
      time(t), type(s), index(i) {}
    typedef boost::shared_ptr<StampedSensorType> Ptr;
    typedef boost::shared_ptr<const StampedSensorType> ConstPtr;
  };

  static bool compareSensorTimestamps(const StampedSensorType::ConstPtr& lhs,
                                      const StampedSensorType::ConstPtr& rhs)
  {
    return ((lhs->time < rhs->time) ||
            ((lhs->time == rhs->time) && (lhs->type < rhs->type)));
  }

  unsigned int pending_index;
  std::vector< StampedSensorType::ConstPtr > sensor_ordering;
  scan_queue pending_scans;
  trimmed_scan_queue pending_trimmed_scans;
  imu_queue pending_imu;
  attitude_queue pending_attitude;
  odom_queue pending_odom;
  icp_queue pending_icp;
  vo_queue pending_vo;
  height_queue pending_height;
  dh_queue pending_dh;
  twist_queue pending_twist;
  tof_queue pending_tof;
  trimmed_tof_queue pending_trimmed_tof;
  image_queue pending_image;

  std::vector< TimedObservations::ConstPtr > time_ordering;
  unsigned int time_ordering_index;
};
#endif
