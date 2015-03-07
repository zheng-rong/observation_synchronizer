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

#include <observation_synchronizer/ObservationSynchronizer.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <ros/ros.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;

ObservationSynchronizer::ObservationSynchronizer() :
  pending_index(0), time_ordering_index(0) {}
ObservationSynchronizer::~ObservationSynchronizer() {}

void ObservationSynchronizer::sortPendingMessages()
{
  sensor_ordering.clear();

  // Sort the messages based on timestamps, across all input
  unsigned int i = 0;
  for (imu_queue::const_iterator it = pending_imu.begin();
       it != pending_imu.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     IMU, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (attitude_queue::const_iterator it = pending_attitude.begin();
       it != pending_attitude.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     ATTITUDE, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (scan_queue::const_iterator it = pending_scans.begin();
       it != pending_scans.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     LASER, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (odom_queue::const_iterator it = pending_odom.begin();
       it != pending_odom.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     ODOM, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (icp_queue::const_iterator it = pending_icp.begin();
       it != pending_icp.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     ICP, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (vo_queue::const_iterator it = pending_vo.begin();
       it != pending_vo.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     VO, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (trimmed_scan_queue::const_iterator it = pending_trimmed_scans.begin();
       it != pending_trimmed_scans.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->scan.header.stamp.toSec(),
                                                     TRIMMEDLASER, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (height_queue::const_iterator it = pending_height.begin();
       it != pending_height.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     HEIGHT, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (dh_queue::const_iterator it = pending_dh.begin();
       it != pending_dh.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     DH, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (tof_queue::const_iterator it = pending_tof.begin();
       it != pending_tof.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     TOF, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (trimmed_tof_queue::const_iterator it = pending_trimmed_tof.begin();
       it != pending_trimmed_tof.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->tof.header.stamp.toSec(),
                                                     TRIMMEDTOF, i));
      sensor_ordering.push_back(p);
    }

  i = 0;
  for (image_queue::const_iterator it = pending_image.begin();
       it != pending_image.end(); ++it, ++i)
    {
      StampedSensorType::Ptr p =
        StampedSensorType::Ptr(new StampedSensorType((*it)->msg->header.stamp.toSec(),
                                                     IMAGE, i));
      sensor_ordering.push_back(p);
    }

  std::sort(sensor_ordering.begin(), sensor_ordering.end(),
            ObservationSynchronizer::compareSensorTimestamps);

  pending_index = 0;
}

bool ObservationSynchronizer::getNextPendingMessage(sensor_types_t& type,
                                                    unsigned int& index)
{
  if (pending_index >= sensor_ordering.size())
  {
    return false;
  }

  type = sensor_ordering[pending_index]->type;
  index = sensor_ordering[pending_index]->index;

  pending_index++;

  return true;
}

bool ObservationSynchronizer::nextPendingMessageExists()
{
  if(pending_index >= sensor_ordering.size())
    return false;
  else
    return true;
}

void ObservationSynchronizer::clearPendingMessages()
{
  pending_imu.clear();
  pending_attitude.clear();
  pending_scans.clear();
  pending_odom.clear();
  pending_icp.clear();
  pending_vo.clear();
  pending_trimmed_scans.clear();
  pending_height.clear();
  pending_dh.clear();
  pending_twist.clear();
  pending_tof.clear();
  pending_trimmed_tof.clear();
  pending_image.clear();
}

const ObservationSynchronizer::scan_queue&
ObservationSynchronizer::getPendingLaserScanMessages()
{
  return pending_scans;
}

const ObservationSynchronizer::trimmed_scan_queue&
ObservationSynchronizer::getPendingTrimmedLaserScanMessages()
{
  return pending_trimmed_scans;
}

const ObservationSynchronizer::imu_queue&
ObservationSynchronizer::getPendingImuMessages()
{
  return pending_imu;
}

const ObservationSynchronizer::attitude_queue&
ObservationSynchronizer::getPendingAttitudeMessages()
{
  return pending_attitude;
}

const ObservationSynchronizer::odom_queue&
ObservationSynchronizer::getPendingOdometryMessages()
{
  return pending_odom;
}

const ObservationSynchronizer::icp_queue&
ObservationSynchronizer::getPendingIcpMessages()
{
  return pending_icp;
}

const ObservationSynchronizer::vo_queue&
ObservationSynchronizer::getPendingVoMessages()
{
  return pending_vo;
}

const ObservationSynchronizer::height_queue&
ObservationSynchronizer::getPendingHeightMessages()
{
  return pending_height;
}

const ObservationSynchronizer::dh_queue&
ObservationSynchronizer::getPendingDhMessages()
{
  return pending_dh;
}

const ObservationSynchronizer::twist_queue&
ObservationSynchronizer::getPendingTwistMessages()
{
  return pending_twist;
}

const ObservationSynchronizer::tof_queue&
ObservationSynchronizer::getPendingTOFMessages()
{
  return pending_tof;
}

const ObservationSynchronizer::trimmed_tof_queue&
ObservationSynchronizer::getPendingTrimmedTOFMessages()
{
  return pending_trimmed_tof;
}

const ObservationSynchronizer::image_queue&
ObservationSynchronizer::getPendingImageMessages()
{
  return pending_image;
}

const ObservationSynchronizer::Tagged<nav_msgs::Odometry>::ConstPtr&
ObservationSynchronizer::getPendingOdometryMessage(unsigned int index)
{
  return pending_odom[index];
}

const ObservationSynchronizer::Tagged<geometry_msgs::PoseWithCovarianceStamped>::ConstPtr&
ObservationSynchronizer::getPendingIcpMessage(unsigned int index)
{
  return pending_icp[index];
}

const ObservationSynchronizer::Tagged<quadrotor_msgs::LocalOdometry>::ConstPtr&
ObservationSynchronizer::getPendingVoMessage(unsigned int index)
{
  return pending_vo[index];
}

const ObservationSynchronizer::Tagged<sensor_msgs::Imu>::ConstPtr&
ObservationSynchronizer::getPendingImuMessage(unsigned int index)
{
  return pending_imu[index];
}

const ObservationSynchronizer::Tagged<sensor_msgs::Imu>::ConstPtr&
ObservationSynchronizer::getPendingAttitudeMessage(unsigned int index)
{
  return pending_attitude[index];
}

const ObservationSynchronizer::Tagged<sensor_msgs::LaserScan>::ConstPtr&
ObservationSynchronizer::getPendingLaserScanMessage(unsigned int index)
{
  return pending_scans[index];
}

const ObservationSynchronizer::Tagged<scan_utils::TrimmedLaserScan>::ConstPtr&
ObservationSynchronizer::getPendingTrimmedLaserScanMessage(unsigned int index)
{
  return pending_trimmed_scans[index];
}

const ObservationSynchronizer::Tagged<quadrotor_msgs::AltitudeObservation>::ConstPtr&
ObservationSynchronizer::getPendingHeightMessage(unsigned int index)
{
  return pending_height[index];
}

const ObservationSynchronizer::Tagged<quadrotor_msgs::HeightDelta>::ConstPtr&
ObservationSynchronizer::getPendingDhMessage(unsigned int index)
{
  return pending_dh[index];
}

const ObservationSynchronizer::Tagged<geometry_msgs::TwistWithCovarianceStamped>::ConstPtr&
ObservationSynchronizer::getPendingTwistMessage(unsigned int index)
{
  return pending_twist[index];
}

const ObservationSynchronizer::Tagged<sensor_msgs::PointCloud2>::ConstPtr&
ObservationSynchronizer::getPendingTOFMessage(unsigned int index)
{
  return pending_tof[index];
}

const ObservationSynchronizer::Tagged<tof_utils::TrimmedTOF>::ConstPtr&
ObservationSynchronizer::getPendingTrimmedTOFMessage(unsigned int index)
{
  return pending_trimmed_tof[index];
}

const ObservationSynchronizer::Tagged<sensor_msgs::Image>::ConstPtr&
ObservationSynchronizer::getPendingImageMessage(unsigned int index)
{
  return pending_image[index];
}

void ObservationSynchronizer::addOdometryMessage(const nav_msgs::Odometry::ConstPtr& msg,
                                                 const std::string& tag)
{
  Tagged<nav_msgs::Odometry>::Ptr p =
    Tagged<nav_msgs::Odometry>::Ptr(new Tagged<nav_msgs::Odometry>(msg, tag));
  pending_odom.push_back(p);
}

void ObservationSynchronizer::addIcpMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                                            const std::string& tag)
{
  Tagged<geometry_msgs::PoseWithCovarianceStamped>::Ptr p =
    Tagged<geometry_msgs::PoseWithCovarianceStamped>::Ptr(new Tagged<geometry_msgs::PoseWithCovarianceStamped>(msg, tag));
  pending_icp.push_back(p);
}

void ObservationSynchronizer::addVoMessage(const quadrotor_msgs::LocalOdometry::ConstPtr& msg,
                                           const std::string& tag)
{
  Tagged<quadrotor_msgs::LocalOdometry>::Ptr p =
    Tagged<quadrotor_msgs::LocalOdometry>::Ptr(new Tagged<quadrotor_msgs::LocalOdometry>(msg, tag));
  pending_vo.push_back(p);
}

void ObservationSynchronizer::addLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& msg,
                                                  const std::string& tag)
{
  Tagged<sensor_msgs::LaserScan>::Ptr p =
    Tagged<sensor_msgs::LaserScan>::Ptr(new Tagged<sensor_msgs::LaserScan>(msg, tag));
  pending_scans.push_back(p);
}

void ObservationSynchronizer::addImuMessage(const sensor_msgs::Imu::ConstPtr& msg,
                                            const std::string& tag)
{
  Tagged<sensor_msgs::Imu>::Ptr p =
    Tagged<sensor_msgs::Imu>::Ptr(new Tagged<sensor_msgs::Imu>(msg, tag));
  pending_imu.push_back(p);
}

void ObservationSynchronizer::addAttitudeMessage(const sensor_msgs::Imu::ConstPtr& msg,
                                                 const std::string& tag)
{
  Tagged<sensor_msgs::Imu>::Ptr p =
    Tagged<sensor_msgs::Imu>::Ptr(new Tagged<sensor_msgs::Imu>(msg, tag));
  pending_attitude.push_back(p);
}

void
ObservationSynchronizer::addTrimmedLaserScanMessage(const scan_utils::TrimmedLaserScan::ConstPtr& msg,
                                                    const std::string& tag)
{
  Tagged<scan_utils::TrimmedLaserScan>::Ptr p =
    Tagged<scan_utils::TrimmedLaserScan>::Ptr(new Tagged<scan_utils::TrimmedLaserScan>(msg, tag));
  pending_trimmed_scans.push_back(p);
}

void ObservationSynchronizer::addHeightMessage(const quadrotor_msgs::AltitudeObservation::ConstPtr& msg,
                                                 const std::string& tag)
{
  Tagged<quadrotor_msgs::AltitudeObservation>::Ptr p =
    Tagged<quadrotor_msgs::AltitudeObservation>::Ptr(new Tagged<quadrotor_msgs::AltitudeObservation>(msg, tag));
  pending_height.push_back(p);
}

void ObservationSynchronizer::addDhMessage(const quadrotor_msgs::HeightDelta::ConstPtr& msg,
                                           const std::string& tag)
{
  Tagged<quadrotor_msgs::HeightDelta>::Ptr p =
    Tagged<quadrotor_msgs::HeightDelta>::Ptr(new Tagged<quadrotor_msgs::HeightDelta>(msg, tag));
  pending_dh.push_back(p);
}

void ObservationSynchronizer::addTwistMessage(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg,
                                              const std::string& tag)
{
  Tagged<geometry_msgs::TwistWithCovarianceStamped>::Ptr p =
    Tagged<geometry_msgs::TwistWithCovarianceStamped>::Ptr(new Tagged<geometry_msgs::TwistWithCovarianceStamped>(msg, tag));
  pending_twist.push_back(p);
}

void ObservationSynchronizer::addTOFMessage(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                            const std::string& tag)
{
  Tagged<sensor_msgs::PointCloud2>::Ptr p =
    Tagged<sensor_msgs::PointCloud2>::Ptr(new Tagged<sensor_msgs::PointCloud2>(msg, tag));
  pending_tof.push_back(p);
}

void ObservationSynchronizer::addTrimmedTOFMessage(const tof_utils::TrimmedTOF::ConstPtr& msg,
                                                   const std::string& tag)
{
  Tagged<tof_utils::TrimmedTOF>::Ptr p =
    Tagged<tof_utils::TrimmedTOF>::Ptr(new Tagged<tof_utils::TrimmedTOF>(msg, tag));
  pending_trimmed_tof.push_back(p);
}

void ObservationSynchronizer::addImageMessage(const sensor_msgs::Image::ConstPtr& msg,
                                              const std::string& tag)
{
  Tagged<sensor_msgs::Image>::Ptr p =
    Tagged<sensor_msgs::Image>::Ptr(new Tagged<sensor_msgs::Image>(msg, tag));
  pending_image.push_back(p);
}

void ObservationSynchronizer::sortPendingMessagesByTime(double time_epsilon)
{
  // First sort pending messages
  sortPendingMessages();

  time_ordering.clear();
  time_ordering_index = 0;

  unsigned total_msg_num = 0;
  unsigned obervation_num = 0;

  if (sensor_ordering.empty())
    return;

  // Iterate through pending messages checking for similar time stamps
  TimedObservations::Ptr t =
    TimedObservations::Ptr(new TimedObservations(sensor_ordering[0]->time, time_epsilon));

    // ================================================
    // Modified by Zheng Rong / Feb 20, 2015
    // every observation has only one message 
    // no need to sync messages here
    // avoid the problems of message lost
    // ================================================

    for (unsigned int i=0; i<sensor_ordering.size(); i++)
    {
        t = TimedObservations::Ptr(new TimedObservations(sensor_ordering[i]->time,
                                                       time_epsilon));
        t->addObservation(  sensor_ordering[i]->type, 
                            sensor_ordering[i]->index,
                            sensor_ordering[i]->time);

        time_ordering.push_back(t);
    }

/*
  t->addObservation(sensor_ordering[0]->type, 
                    sensor_ordering[0]->index,
                    sensor_ordering[0]->time);
  total_msg_num++;

  for (unsigned int i = 1; i < sensor_ordering.size(); i++)
  {
    if (t->inSet(sensor_ordering[i]->time))
    {
      t->addObservation(sensor_ordering[i]->type, 
                        sensor_ordering[i]->index,
                        sensor_ordering[i]->time);
      total_msg_num++;
    }
    else
    {
      time_ordering.push_back(t);
      t = TimedObservations::Ptr(new TimedObservations(sensor_ordering[i]->time,
                                                       time_epsilon));
      t->addObservation(sensor_ordering[i]->type, 
                        sensor_ordering[i]->index,
                        sensor_ordering[i]->time);
      total_msg_num++;
      obervation_num++;
    }
  }
  time_ordering.push_back(t);
  obervation_num++;
*/
  // ROS_INFO("time_ordering size = %d", time_ordering.size()); 
  // ROS_INFO("sensor_ordering = %d", sensor_ordering.size());
  // ROS_INFO("obervation_num = %d", obervation_num);
  // ROS_INFO("total_msg_num = %d", total_msg_num);
  
  
}

const ObservationSynchronizer::TimedObservations::ConstPtr&
ObservationSynchronizer::getTimedObservations(unsigned int i)
{
  return time_ordering[i];
  //return sensor_ordering[i];
}

bool ObservationSynchronizer::getNextPendingTimedObservations(unsigned int& index)
{
  if (time_ordering_index >= time_ordering.size())
  {
    //ROS_INFO("time_ordering size = %d", time_ordering.size());
    //ROS_INFO("sensor_ordering = %d", sensor_ordering.size());
    //ROS_INFO("pending_index = %d", pending_index);
    return false;
  }

  //ROS_INFO("time_ordering_index = %d", time_ordering_index);
  index = time_ordering_index;
  time_ordering_index++;

  return true;
}

bool ObservationSynchronizer::nextPendingTimedObservationsExists()
{
  if((time_ordering_index >= time_ordering.size()) && (time_ordering.size() > 1))
    return false;
  else
    return true;
}

unsigned int ObservationSynchronizer::getNumberOfTimeKeyGroups()
{
   return time_ordering.size();
}

void ObservationSynchronizer::printContents()
{
  unsigned int fake_pending_index = 0;
  for(unsigned int packet_ctr = 0;
                   packet_ctr < time_ordering.size();
                   packet_ctr++)
  {
     double packet_time = time_ordering[packet_ctr]->time;

     for(unsigned int obs_ctr = 0; obs_ctr < time_ordering[packet_ctr]->observations.size();
         obs_ctr++)
     {
        sensor_types_t type = sensor_ordering[fake_pending_index + obs_ctr]->type;
        std::string ps = getTypeString(type);
        ps.insert(ps.size(), 6 - ps.size(), ' ');
        unsigned int index = sensor_ordering[fake_pending_index + obs_ctr]->index;
        double obs_timestamp = sensor_ordering[fake_pending_index + obs_ctr]->time;
        printf("synchronizer(%u) = %s, \t timestamp = %f, \t packet # = %u \t packet timestamp = %f \n",
            fake_pending_index + obs_ctr,
            ps.c_str(), obs_timestamp, packet_ctr, packet_time);
     }
     fake_pending_index += time_ordering[packet_ctr]->observations.size();
  }
}
