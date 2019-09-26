/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <kmo_navserver/kmo_navserver_driver.hh>
#include <kmo_navserver/VMCEncodersStamped.h>
#include <kmo_navserver/VMCLaserWayStamped.h>

template<class T> std::string _toString_ (const T& x)
{
     std::ostringstream o;

     if (!(o << x))
	  throw std::runtime_error ("::toString()");

     return o.str ();
}

inline double median_sample(std::vector<double>& samples)
{
     std::vector<double>::iterator  i = samples.begin();
     std::vector<double>::size_type m = samples.size() / 2;

     std::nth_element(i, i + m, samples.end());

     return samples.at(m);
}

/** \brief Constructor */
VMCNavserverDriver::VMCNavserverDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh, const VMCNavserverDriver::Params& params)
  : params_(params), baseTime_(-1.), lastTimeStampEnc_(-1.), lastTimeStampState_(-1.), lastTimeStampState2_(-1.), lastTimeStampSick_(-1.), timeOffsetSize_(100), timeOffsetsCounter_(0), initOdom_(true), lastTimeStampLoc_(-1.)
       //reconfigure_server_(param_nh), 
    
{
     // Set up reconfigure server
//  ReconfigureServer::CallbackType f = boost::bind(&uEyeDriver::configCb, this, _1, _2);
//  reconfigure_server_.setCallback(f);

  param_nh.param<std::string>("odom_frame_id", odom_frame_id_, std::string ("odom"));
  param_nh.param<std::string>("global_frame_id", global_frame_id_, std::string ("map"));
  param_nh.param<std::string>("base_footprint_frame_id", base_footprint_frame_id_, std::string("base_footprint"));
  param_nh.param<std::string>("state_frame_id", state_frame_id_, std::string("base_link_ground_truth"));
  param_nh.param<std::string>("laserscan0_frame_id", laserscan0_frame_id_, "laserscan0_frame");
  param_nh.param<std::string>("laserscan1_frame_id", laserscan1_frame_id_, "laserscan1_frame");
  
  param_nh.param<std::string>("host", params_.navParams.host, std::string("192.168.0.100"));
  param_nh.param<int>("port", params_.navParams.port, 5432);
  param_nh.param<bool>("enc", params_.navParams.enc, false);
  param_nh.param<bool>("laser", params_.navParams.laser, false);
  param_nh.param<bool>("sick", params_.navParams.sick, false);
  param_nh.param<bool>("state", params_.navParams.state, false);
  param_nh.param<std::string>("received_logname", params_.navParams.received_logname, std::string(""));
  param_nh.param<std::string>("sent_logname", params_.navParams.sent_logname, std::string(""));
  param_nh.param<bool>("set_state_as_init_odom", params_.setStateAsInitOdom, true);
  param_nh.param<int>("debug", params_.debug, 0);
  param_nh.param<int>("protocol_ver", params_.navParams.protocol_ver, 8);
  param_nh.param<bool>("combined_steer_and_drive", params_.combinedSteerAndDrive, true);
  param_nh.param<bool>("use_vmc_timestamp", params_.useVmcTimestamp, true);
  param_nh.param<int>("robot_type", params_.robotType, static_cast<int>(SNOWWHITE));
  param_nh.param<double>("steering_fix_wheel_distance", params_.steeringFixWheelDistance, 0.68); // Snowwhite...
  param_nh.param<double>("steering_fix_wheel_distance_y", params_.steeringFixWheelDistanceY, 0.); // Snowwhite...
  param_nh.param<std::string>("tf_prefix", tf_prefix_, std::string(""));
  std::string tf_prefix_own;
  param_nh.param<std::string>("tf_prefix_own", tf_prefix_own, std::string(""));
  tf_prefix_ += tf_prefix_own;
  param_nh.param<std::string>("tf_frame_state_id", tf_frame_state2_id_, std::string("state2_base_link"));
  param_nh.param<bool>("epu", params_.navParams.epu, false);
  param_nh.param<bool>("use_epu_time_offset", params_.useEPUtimeOffset, true);
  param_nh.param<double>("additional_epu_offset", params_.additionalEPUoffset, 0.);
  param_nh.param<std::string>("socket_file", socket_file_, std::string(""));
  param_nh.param<double>("min_time_loc_delta", params_.minTimeLocDelta, 0.06);

  // Get the parameters for the topic names.
  param_nh.param<std::string>("laserscan0_topic_name", laserscan0_topic_name_, "laserscan0");
  param_nh.param<std::string>("laserscan1_topic_name", laserscan1_topic_name_, "laserscan1");

  param_nh.param<std::string>("mcl_pose_topic", mcl_pose_topic_, "mcl_pose_estimate");

  if (params_.navParams.epu)
       params_.navParams.protocol2Hack = true;

     std::ostringstream os;
     os << params_;
     ROS_INFO("[VMCNavserverDriver]: params%s", os.str().c_str());
     navserver_.setParams(params_.navParams);

     // Publishers and subscribers
     pub_odom_ = comm_nh.advertise<nav_msgs::Odometry>("odom", 50);
     pub_state_ = comm_nh.advertise<nav_msgs::Odometry>("state", 50);
     if (!params_.ignoreState2) {
       pub_state2_ = comm_nh.advertise<nav_msgs::Odometry>("state2", 50);
     }
     pub_enc_ = comm_nh.advertise<kmo_navserver::VMCEncodersStamped>("encoders", 50);
     pub_laserway_ = comm_nh.advertise<kmo_navserver::VMCLaserWayStamped>("laserway", 50);
     pub_sick_id0_ = comm_nh.advertise<sensor_msgs::LaserScan>(laserscan0_topic_name_,10);
     pub_sick_id1_ = comm_nh.advertise<sensor_msgs::LaserScan>(laserscan1_topic_name_,10);

     //sub_loc_ = param_nh.subscribe<nav_msgs::Odometry>("input_loc",10,&VMCNavserverDriver::process_loc,this);
     sub_loc_ = param_nh.subscribe<nav_msgs::Odometry>(mcl_pose_topic_, 10, &VMCNavserverDriver::process_loc, this);

     odomState_.setZero();
     oldState_.setZero();

     if (params_.setStateAsInitOdom)
     {
	  initOdom_ = false;
     }

}

bool
VMCNavserverDriver::init ()
{
     ROS_DEBUG ("[VMCNavserver::init]");
     if (!socket_file_.empty())
         return true;

     if (!navserver_.connect())
     	  return false;
     return true;
}

/** \brief Destructor */
VMCNavserverDriver::~VMCNavserverDriver ()
{
     if (!socket_file_.empty())
         return;

     //do cleanup
     navserver_.disconnect();
}

bool
VMCNavserverDriver::protocol2SyncCheck(const double &timestamp) const
{
    if (!socket_file_.empty())
        return true;

     assert(baseTime_ > 0);
     
     if (timestamp < baseTime_)
     {
	  return false;
     }
     return true;
}

void
VMCNavserverDriver::updateTimeOffsets(double offset)
{
     if (timeOffsets_.size() < timeOffsetSize_)
     {
	  timeOffsets_.push_back(offset);
     }
     else
     {
	  timeOffsets_[timeOffsetsCounter_] = offset;
	  timeOffsetsCounter_++;
	  timeOffsetsCounter_ = timeOffsetsCounter_ % timeOffsets_.size();
     }
}

bool
VMCNavserverDriver::processDataSocket() 
{
    std::string mesg;
    int byte = navserver_.receive(mesg);
    
    if(byte>0) {
        return processData(mesg);
    }
    return false;
}


bool
VMCNavserverDriver::processData(const std::string &mesg)
{
//     std::string mesg;
//     int byte = navserver_.receive(mesg);

       if(!mesg.empty()) {
	  const char* delim = " ";
	  char* msg = (char*)mesg.c_str();
	  char *tok = strtok(msg,delim);
	  double current_time = ros::Time::now().toSec();
  	  double timestamp;
	  
	  if(tok!=NULL) {
	       // -------------------------------------------
	       // ----------------- restart -----------------
	       // -------------------------------------------
	       if(strncmp("navserver",tok,9) == 0) 
	       {
		    std::cout << "navserver (re)started" << std::endl;
		    //std::cout << " enc timestamp : " << _lastTimeStampEnc << std::endl;
		    //std::cout << " laser timestamp : " << _lastTimeStampLaser << std::endl;
		    //std::cout << " state timestamp : " << _lastTimeStampState << std::endl;
	       }
	       // -------------------------------------------
	       // ---------------- basetime -----------------
	       // -------------------------------------------
	       else if(strncmp("basetime",tok,8) == 0) {
		    //receiving state msg
		    //timestamp
		    tok = strtok(NULL," ");
		    double basetime_sec;
		    basetime_sec = atof(tok);
		    
		    tok = strtok(NULL," ");
		    double basetime_usec;
		    basetime_usec = atof(tok);
		    
		    baseTime_ = basetime_sec + basetime_usec * 0.000001;
        std::cout << "got basetime : " << baseTime_ << std::endl;

         }
	       // -------------------------------------------
	       // ------------------ sick -------------------
	       // -------------------------------------------
	       else if(strncmp("sick",tok,4) == 0) {
                 
		 // Note this currently assumes that we use the latest protocol.
		 VMCSickMessage sm;
		 sm = parseSickMessage(tok);
		 timestamp = sm.getTs();
                
		 updateTimeOffsets(current_time - timestamp);

		 if (this->lastTimeStampSick_ < 0.) // _lastTimeStamp always -1 the first time.
		   {
		     this->lastTimeStampSick_ = timestamp;
		     return true;
		   }
		 this->lastTimeStampSick_ = timestamp;
                
		 ros::Time stamp_time = ros::Time(current_time);
		 if (params_.useVmcTimestamp)
		   {
		     stamp_time = ros::Time(timestamp + median_sample(timeOffsets_));
		   }
            
		 // Don't send the transform here, add a static transform publisher to the launch files...
		 sensor_msgs::LaserScan scan;
		 sm.getRosScan(scan);
		 scan.header.stamp = stamp_time;
		 if (sm.sensor_id == 0) {
		   scan.header.frame_id = laserscan0_frame_id_; 
		   pub_sick_id0_.publish(scan);
		 }
		 else if (sm.sensor_id == 1) {
		   scan.header.frame_id = laserscan1_frame_id_; 
		   pub_sick_id1_.publish(scan);
		 }
		 else {
		   ROS_WARN_STREAM_THROTTLE(30, "[kmo_navserver]: sensor id is > 1 (" << sm.sensor_id << ")");
		 }
	       }
               
	       // -------------------------------------------
	       // ------------------ state2 ------------------
	       // -------------------------------------------
	       else if(strncmp("state2",tok,6) == 0) {

		 if (!params_.ignoreState2) {
		   VMCStateMessage state;
		   state.parseStateMessage(tok);
		   timestamp = state.getTs();
		   
		   if (!protocol2SyncCheck(timestamp))
		     return true; // discard this message
		   
		   updateTimeOffsets(current_time - timestamp);
		   
		   if (this->lastTimeStampState2_ < 0.) // _lastTimeStamp always -1 the first time.
		     {
		       this->lastTimeStampState2_ = timestamp;
		       return true;
		     }
		   
		   double time_diff = (timestamp - this->lastTimeStampState2_);
		   this->lastTimeStampState2_ = timestamp;
		   
		   ros::Time stamp_time = ros::Time(current_time);
		   if (params_.useVmcTimestamp)
		     {
		       stamp_time = ros::Time(timestamp + median_sample(timeOffsets_));
		     }
		   nav_msgs::Odometry ros_state = state.getRosOdometry();
		   ros_state.header.frame_id = global_frame_id_;
		   ros_state.header.stamp = stamp_time;
		   
		   geometry_msgs::TransformStamped stf = state.getTF();
		   stf.header.frame_id = global_frame_id_;
		   stf.header.stamp = stamp_time;
		   stf.child_frame_id = tf_frame_state2_id_;
		   
		   //send the transform
		   state_broadcaster.sendTransform(stf);
		   pub_state2_.publish(ros_state);
		 }
	       }
	       // -------------------------------------------
	       // ------------------ state ------------------
	       // -------------------------------------------
	       else if(strncmp("state",tok,5) == 0) {
		    //receiving state msg
		    //timestamp
		    VMCStateMessage state;
		    state.parseStateMessage(tok);
		    timestamp = state.getTs();
		    
		    if (!protocol2SyncCheck(timestamp))
			 return true; // discard this message
	 
		    updateTimeOffsets(current_time - timestamp);

		    if (this->lastTimeStampState_ < 0.) // _lastTimeStamp always -1 the first time.
		    {
			 this->lastTimeStampState_ = timestamp;
			 oldState_ = state;
			 return true;
		    }

		    double time_diff = (timestamp - this->lastTimeStampState_);
		    this->lastTimeStampState_ = timestamp;

		    ros::Time stamp_time = ros::Time(current_time);
		    if (params_.useVmcTimestamp)
		    {
			 stamp_time = ros::Time(timestamp + median_sample(timeOffsets_));
		    }

		    geometry_msgs::TransformStamped stf = state.getTF();
		    stf.header.frame_id = global_frame_id_;
		    stf.header.stamp = stamp_time;
		    stf.child_frame_id = state_frame_id_;

		    //send the transform
		    state_broadcaster.sendTransform(stf);
		    

		    if (!initOdom_)
		    {
			 odomState_ = state;
			 initOdom_ = true;
		    }
		    
		    nav_msgs::Odometry ros_state = state.getRosOdometry();
		    ros_state.header.frame_id = global_frame_id_;
		    ros_state.child_frame_id = state_frame_id_;
		    ros_state.header.stamp = stamp_time;
		    //set the velocity
		    ros_state.twist.twist.linear.x = (state.x - oldState_.x)/time_diff;
		    ros_state.twist.twist.linear.y = (state.y - oldState_.y)/time_diff;
		    ros_state.twist.twist.angular.z = (state_.a - oldState_.a)/time_diff;
		    
		    oldState_ = state;

		    //publish the message
		    pub_state_.publish(ros_state);
	       }
	       // -------------------------------------------
	       // ------------------ laser ------------------
	       // -------------------------------------------
	       // Note there is no specific paser type for this one...
	       else if(strncmp("laser",tok,5) == 0) 
	       {
		    double timestamp = 0.;
		    tok = strtok(NULL," ");
		    if (tok!= NULL)
			 timestamp = atof(tok);
		    
		    if (!protocol2SyncCheck(timestamp))
			 return true; // discard this message
		    
		    ros::Time stamp_time = ros::Time(current_time);
		    if (params_.useVmcTimestamp)
		    {
			 stamp_time = ros::Time(timestamp + median_sample(timeOffsets_));
		    }
		    
		    tok = strtok(NULL," ");
		    // Bearing to the reflector
		    double bearing = 0.;
		    if(tok!=NULL) {
			 bearing = atof(tok);
		    }
		    // Distance to the reflector
		    double distance = 0.;
		    tok = strtok(NULL," ");
		    if(tok!=NULL) {
		     distance = atof(tok);
		    }
		    int status = -1;
		    tok = strtok(NULL," ");
		    if (tok!=NULL)
			 status = atoi(tok);
		    
		    // status = quality of the reading
		    // 0- very good
		    // 1- so so good
		    // 2- bad
		    
		    // This is an estimated value of the rotation of the reflector relative to the incoming laser beam. Seems not to be very accurate but it is used in Danaher's 'SLAM' tool for estimating the reflex positions.
		    double reflector_ang=0.;
		    tok = strtok(NULL," ");
		    if (tok!=NULL)
			 reflector_ang = atof(tok);
		    
		    // Fill in the data.
		    kmo_navserver::VMCLaserWayStamped vmc_laserway;
		    //vmc_laserway.header.frame_id = frame_id_;
		    vmc_laserway.header.stamp = stamp_time;
		    vmc_laserway.data.bearing = bearing;
		    vmc_laserway.data.distance = distance;
		    vmc_laserway.data.status = status;
		    vmc_laserway.data.reflectorAngle = reflector_ang;

		    pub_laserway_.publish(vmc_laserway);
	       } 
	       // -------------------------------------------
	       // ------------------  enc  ------------------
	       // -------------------------------------------
	       // encoder readings from navserver look like: enc timestamp speed1 angle1 speed2 angle2 ... speedx anglex, currenty not using the parser.
	       else if(strncmp("enc",tok,3) == 0) 
	       {
		    double timestamp = 0.;
		    tok = strtok(NULL," ");
		    if (tok!= NULL)
			 timestamp = atof(tok);
		    
		    if (!protocol2SyncCheck(timestamp))
			 return true; // discard this message
		    
		    updateTimeOffsets(current_time - timestamp);
		    
		    if (this->lastTimeStampEnc_ < 0.) // _lastTimeStamp always -1 the first time.
		    {
			 this->lastTimeStampEnc_ = timestamp;
			 return true;
		    }
		    double time_diff = (timestamp - this->lastTimeStampEnc_);
		    this->lastTimeStampEnc_ = timestamp;
		    
		    ros::Time stamp_time = ros::Time(current_time);
		    if (params_.useVmcTimestamp)
		    {
			 stamp_time = ros::Time(timestamp + median_sample(timeOffsets_));
		    }

		    double odom_alpha = 0.;
		    double odom_drive_vel = 0.;
		    
		    // Use a different struct here compared to protocol 0.
		    // Get steering angle + velocities for left / right wheel.
		    std::vector<std::pair<double, double> > encoders;

		    while (true)
		    {
			 //enc steering angle (radians)
			 tok = strtok(NULL," ");
			 double drive_vel = 0.;
			 if(tok!=NULL) {
			      drive_vel = atof(tok);
			 }
			 else
			      break;
			 //drive wheel vel
			 double alpha = 0.;
			 tok = strtok(NULL," ");
			 if(tok!=NULL) {
			      alpha = atof(tok);
			 }
			 else
			      break;
			 encoders.push_back(std::pair<double, double>(drive_vel, alpha));
		    }
                    // For the MALTA truck - we have:
		    // [0] - steering wheel
		    // [1] - left front wheel
		    // [2] - right front wheel
		    switch (params_.robotType)
		    {
		    case SNOWWHITE:
		    {
			 assert(encoders.size() == 1);
			 odom_drive_vel = encoders[0].first;
			 odom_alpha = encoders[0].second;
			 break;
		    }
		    case MALTATRUCK:
		    {
			 assert(encoders.size() == 3);
			 odom_drive_vel = 0.5*(encoders[1].first + encoders[2].first);
			 odom_alpha = encoders[0].second;
			 break;
		    }
		    default:
			 assert(false);
			 break;
		    };
		    
		    kmo_navserver::VMCEncodersStamped vmc_encoders;
		    //vmc_encoders.header.frame_id = frame_id_;
		    vmc_encoders.header.stamp = stamp_time;
		    vmc_encoders.encoders.resize(encoders.size());
		    for(unsigned int i=0;i<encoders.size();i++){
			 vmc_encoders.encoders[i].fwd_vel = encoders[i].first;
			 vmc_encoders.encoders[i].steering_ang = encoders[i].second;
		    }
		    pub_enc_.publish(vmc_encoders);

		    if (initOdom_) // This only wait until we got a state message (otherwise we don't have any state to set as the inital odometry).
		    {
			 double drive_dist = (odom_drive_vel)*time_diff;
		     
			 // Changes in distance and rotation
			 double dd = 0.;
			 double da = 0.;
		     
			 if (fabs(odom_alpha) < 0.000001) // Avoid dividing with a very small number...
			 {
			      dd = drive_dist;
			      da = 0.;
			 }
			 else
			 {
			      if (params_.combinedSteerAndDrive) // Snowwhite
			      {
				   double r_stear = params_.steeringFixWheelDistance / sin(odom_alpha);
           double r_fix = r_stear * cos(odom_alpha) + params_.steeringFixWheelDistanceY;
			       
				   dd = r_fix / r_stear * drive_dist; // Adjust the actual forward movement (located between the fixed front wheels) based on the radius of the drive wheel).
				   da = drive_dist / r_stear;
			      }
			      else // the MALTA truck
			      {
				   double r_stear = params_.steeringFixWheelDistance / sin(odom_alpha);
				   double r_fix = r_stear * cos(odom_alpha);

				   dd = drive_dist;
				   da = drive_dist / r_fix;
			      }
			 }
			 if (params_.debug == 1)
			      std::cout << "odom_alpha : " << odom_alpha << " odom_drive_vel : " << odom_drive_vel << " drive_dist : " << drive_dist << " dd : " << dd << " da : " << da << " time_diff : " << time_diff << std::endl;
			 
			 // Update the current estimate
			 double dx = dd * cos( odomState_.a + da / 2. );
			 double dy = dd * sin( odomState_.a + da / 2. );
			 odomState_.x += dx;
			 odomState_.y += dy;
			 odomState_.a += da;
			 
			 geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odomState_.a);
			 geometry_msgs::TransformStamped odom_trans;
			 odom_trans.header.stamp = stamp_time;
			 odom_trans.header.frame_id = odom_frame_id_;
			 odom_trans.child_frame_id = base_footprint_frame_id_;
		    
			 odom_trans.transform.translation.x = odomState_.x;
			 odom_trans.transform.translation.y = odomState_.y;
			 odom_trans.transform.translation.z = 0.0;
			 odom_trans.transform.rotation = odom_quat;
		    
			 //send the transform
			 odom_broadcaster.sendTransform(odom_trans);

			 //next, we'll publish the odometry message over ROS
			 nav_msgs::Odometry odom;
			 odom.header.stamp = stamp_time;
			 odom.header.frame_id = odom_frame_id_;
		    
			 //set the position
			 odom.pose.pose.position.x = odomState_.x;
			 odom.pose.pose.position.y = odomState_.y;
			 odom.pose.pose.position.z = 0.0;
			 odom.pose.pose.orientation = odom_quat;
		    
			 
			 //set the velocity
			 odom.child_frame_id = base_footprint_frame_id_;
			 odom.twist.twist.linear.x = dx / time_diff;
			 odom.twist.twist.linear.y = dy / time_diff;
			 odom.twist.twist.angular.z = da / time_diff;

			 //publish the message
			 pub_odom_.publish(odom);
		    }
	       }
               else 
               {
                   // Unknown mesage
                   return false;
               }
	  }
     }
     else
     {
	  return false;
     }
     return true;
}


bool 
VMCNavserverDriver::spin(ros::NodeHandle &node)
{
    if (!socket_file_.empty()) {
        // Try to open and read it...
        std::ifstream ifs;
        ifs.open(socket_file_.c_str());
        if (!ifs.is_open()) {
            std::cerr << __FILE__ << ":" << __LINE__ << " cannot open file : " << socket_file_ << std::endl;
        }
        std::cout << "reading socket file : " << socket_file_ << std::endl;
        while (!ifs.eof() && node.ok())
        {
            std::string line;
            getline(ifs, line);
            
            if (!processData(line)) {
                std::cout << "." << std::flush;
            }
            else {
                ros::spinOnce();
                usleep(10000); // TODO, put the sleep in the parsing to get a proper replay of the data...
            }
	}
    }
    else { // Using the socket
        while (node.ok())
        {
            ros::spinOnce();
            if (processDataSocket())
            {
                // OK.
            }
            else
            {
//                ROS_ERROR("nothing to process...");
                usleep(100);
            }
        }
        // Shutdown...
    }
    return true;
}
 
void
VMCNavserverDriver::process_loc(const nav_msgs::OdometryConstPtr &msg)
{
  if (!params_.navParams.epu)
    return;

  double current_stamp = msg->header.stamp.toSec();
  if (this->lastTimeStampLoc_ < 0) {
    this->lastTimeStampLoc_ = current_stamp;
    return;
  }
		
  if (current_stamp - this->lastTimeStampLoc_ < this->params_.minTimeLocDelta) {
    return;
  }

  this->lastTimeStampLoc_ = current_stamp;

	double off=median_sample(timeOffsets_);
  	ros::Time stamp=msg->header.stamp;

	double timestamp = stamp.toSec() - off- params_.additionalEPUoffset;
	if (!params_.useEPUtimeOffset) {
	  timestamp = lastTimeStampEnc_;// VERY HACKY :-).
	}
	//	fprintf(stderr,"VMC: Offset=%lf, localization=%lf, lastTimeStampEnc_=%lf\n", off, timestamp, lastTimeStampEnc_);
  float pos_x = msg->pose.pose.position.x;
  float pos_y = msg->pose.pose.position.y;
  float pos_th = tf::getYaw(msg->pose.pose.orientation);
	 
  // Confidence value if 0 -> no confidence, 100 -> max confidence.
  // Will decrease by 1 every 60 ms when no data has arrived.
  int conf = 100;
  
  // NOTE: These values are not used by the EPU inteface.
  
  // float cxx = msg->pose.covariance[0]; //0.f;
  // float cyy = msg->pose.covariance[7]; //0.f;
  // float cpp = msg->pose.covariance[35]; //0.f;
  
  // float rxy = msg->pose.covariance[1]; // 0.f;
  // float rxp = msg->pose.covariance[5]; // 0.f;
  // float ryp = msg->pose.covariance[11]; //0.f;

  float cxx = 0.f;
  float cyy = 0.f;
  float cpp = 0.f;
  
  float rxy = 0.f;
  float rxp = 0.f;
  float ryp = 0.f;

  
  if (params_.navParams.protocol2Hack)
    timestamp = timestamp - baseTime_;
	 
  if (params_.debug == 2) {
    std::cout << "timestamp : " << timestamp << std::endl;
    std::cout << "baseTime  : " << baseTime_ << std::endl;
  }

  std::string cmd = "epu " + _toString_(timestamp) + " " + _toString_(conf) + " " + _toString_(pos_x) + " " + _toString_(pos_y) + " " + _toString_(pos_th) + " " + _toString_(cxx) + " " + _toString_(cyy) + " " + _toString_(cpp) + " " + _toString_(rxy) + " " + _toString_(rxp) + " " + _toString_(ryp);
	 
  if (params_.debug == 2)
    std::cout << "cmd : " << cmd << std::endl;

  // It seems that the epu interface is only available in protocol 0.
  if (params_.navParams.protocol2Hack)
    navserver_.send("protocol 0");
  
  navserver_.send(cmd);
  
  if (params_.navParams.protocol2Hack) {
    std::string protocol_cmd = "protocol " + _toString_(params_.navParams.protocol_ver);
    navserver_.send(protocol_cmd);
  }
}
