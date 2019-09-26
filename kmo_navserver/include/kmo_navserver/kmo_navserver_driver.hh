#ifndef VMCNAVSERVERDRIVER_HH
#define VMCNAVSERVERDRIVER_HH

#include "kmo_navserver.hh"

#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <string.h>

#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <kmo_navserver/kmo_navserver_parser.h>


class VMCNavserverDriver
{
public:
     enum RobotType { SNOWWHITE = 0, MALTATRUCK };
     class Params
     {
     public:
	  Params()
	  {
	       navParams = VMCNavserver::Params();
	       setStateAsInitOdom = true;
	       debug = 0;
	       combinedSteerAndDrive = true;
	       useVmcTimestamp = true;
	       steeringFixWheelDistance = 1.380; // Arla 0.68; // Snowwhite...
	       steeringFixWheelDistanceY = -0.233; // Arla 0.0 // Snowwhite...
	       robotType = SNOWWHITE;
	       useEPUtimeOffset = true;
	       additionalEPUoffset = 0.;
	       ignoreState2 = true;
         minTimeLocDelta = 0.06;
	  }

	  VMCNavserver::Params navParams;
          // Parameters...
	  bool setStateAsInitOdom;
	  int debug;
	  bool combinedSteerAndDrive;
	  bool useVmcTimestamp;
	  double steeringFixWheelDistance; //X
	  double steeringFixWheelDistanceY;
	  int robotType;
    bool useEPUtimeOffset;
    double additionalEPUoffset;
    bool ignoreState2;
    double minTimeLocDelta;

	  friend std::ostream& operator<<(std::ostream &os, const VMCNavserverDriver::Params &obj)
	  {
	       os << obj.navParams;
	       os << "\nsetStateAsInitOdom       : " << obj.setStateAsInitOdom;
	       os << "\ndebug                    : " << obj.debug;
	       os << "\ncombinedSteerAndDrive    : " << obj.combinedSteerAndDrive;
	       os << "\nuseVmcTimestamp          : " << obj.useVmcTimestamp;
	       os << "\nsteeringFixWheelDistance : " << obj.steeringFixWheelDistance;
	       os << "\nsteeringFixWheelDistanceY: " << obj.steeringFixWheelDistanceY;
	       os << "\nuseEPUtimeOffset         : " << obj.useEPUtimeOffset;
	       os << "\nadditionalEPUoffset      : " << obj.additionalEPUoffset;
	       os << "\nignoreState2             : " << obj.ignoreState2;
         os << "\nminTimeLocDelta          : " << obj.minTimeLocDelta;
         return os;
	  }

	  
     };

     VMCNavserverDriver(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, const VMCNavserverDriver::Params& params);
     ~VMCNavserverDriver();
     bool init();
     bool spin(ros::NodeHandle &node);
private:
     bool protocol2SyncCheck(const double &timestamp) const;
     bool processDataSocket();
     bool processData(const std::string &mesg);
     void updateTimeOffsets(double offset);
     void process_loc(const nav_msgs::OdometryConstPtr &msg);

     ros::NodeHandle n;
     tf::TransformBroadcaster odom_broadcaster, state_broadcaster;
     ros::Publisher pub_odom_, pub_state_, pub_state2_, pub_enc_, pub_laserway_, pub_sick_id0_, pub_sick_id1_;
     ros::Subscriber sub_loc_;

     std::string global_frame_id_;
     std::string odom_frame_id_;
     std::string state_frame_id_;
     std::string base_footprint_frame_id_;
     std::string tf_frame_state2_id_;
     std::string laserscan0_frame_id_;
     std::string laserscan1_frame_id_;

     std::string laserscan0_topic_name_;
     std::string laserscan1_topic_name_;
     std::string mcl_pose_topic_;

     VMCNavserver navserver_;
     VMCNavserverDriver::Params params_;

     double baseTime_;
     double lastTimeStampEnc_;
     double lastTimeStampState_;
     double lastTimeStampState2_;
     double lastTimeStampSick_;
     double lastTimeStampLoc_;

     size_t timeOffsetSize_;
     std::vector<double> timeOffsets_;
     size_t timeOffsetsCounter_;

     VMCStateMessage odomState_;
     VMCStateMessage state_;
     VMCStateMessage oldState_;
     bool initOdom_;

     std::string socket_file_;
     std::string tf_prefix_;
     
public:
     geometry_msgs::Twist cmdvel_;


};




#endif
