^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package observation_synchronizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.8 (2014-10-24)
------------------
* Merge branch 'feature/local_odometry' into develop
* added support for quadrotor_msgs::LocalOdometry
* Contributors: John Yao

0.0.7 (2014-10-16)
------------------
* Merge branch 'feature/do_not_add_duplicates' into 'develop'
  Feature/do not add duplicates
  Prior version ensured monotonic time changes while we wanted unique times.
* Use std::map for key-based timed observation duplicate check
* removed checks from functions that add messages to observation synchronizer, added a check for duplicates when forming timed observation packets
* Adding validation check for monotonic time and rejection if not held
* merged in changes to support visual odometry observations
* added vo message type
* Contributors: John Yao, Nathan Michael

0.0.6 (2014-09-20)
------------------
* removed function clearAllPendingMessagesExceptForThoseInPacket, updated printContents() to include packet index
* Contributors: John Yao

0.0.5 (2014-09-16)
------------------
* Adding sensor_msgs/Image support
* Removing file left over from merge. Updating gitignore to prevent future inclusion
* Fixing merge issues and handling twist during inpacket clear
* Merge branch 'develop' into feature/tof_message
  Conflicts:
  include/observation_synchronizer/ObservationSynchronizer.h
  src/ObservationSynchronizer.cc
* Adding TOF and TrimmedTOF messages
* adding relative height measurement type
* added timestamp to printContents() printout
* Format printf output as expected and silence warning
* add support for PoseWithCovarianceStamped messages
* Merge branch 'feature/alt_msg_change' into feature/time_ordering
* added ICP message type
* Changing altitude observation msg to pull from quadrotor_msgs pkg
* added convenience functions to work with time-keyed packets of messages
* removed cumulative_pending_index
* local changes
* Support for timed observation sets vs sequential indexing
* Contributors: Erik Nelson, John Yao, Nathan Michael

0.0.4 (2014-05-05)
------------------
* Manual merge from twistwithcovariance and altitude_pf_height_msg
* Adding support for APF height messages rather than geometry_utils::Vector3Stamped
* Contributors: Erik

0.0.3 (2014-03-26)
------------------
* Merge branch 'feature-twistwithcovariance_observation' into 'develop'
  Feature Twistwithcovariance Observation
* added twist with covariance message type
* Contributors: John Yao, Nathan Michael

0.0.2 (2014-02-25)
------------------
* Merge branch 'feature-peek_at_next_pending_message' into develop
* added function to get type and time of next pending message
* Fixing incorrect call to find_package without specifying components
* removed isEmpty() function that was added in previous commit
* added height observation type
* Merge branch 'feature/attitude_message_handling' into 'develop'
  Feature/Attitude Message Handling
  Request for a merge.
  Functionality removed: None
  Functionality added: Attitude message handling, where an attitude message is a sensor_msgs/Imu type message containing 3DOF quaternion orientation as well as 3D angular rates.
* Added handling for attitude messages: sensor_msgs::Imu type messages which contain 3DOF orientation as well as angular rates taken from the onboard complimentary filter.
* Contributors: Erik, John Yao, Nathan Michael

0.0.1 (2014-01-20)
------------------
* Adding GPLv2 license and documentation base
* Moving to templatized object. Fixes bug that arose when sorting with c++11.
* Moving to hydro and catkin
* Initial commit
* Contributors: Nathan Michael
