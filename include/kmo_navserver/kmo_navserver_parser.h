#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

template<class T> std::string toString (const T& x)
{
    std::ostringstream o;

    if (!(o << x))
        throw std::runtime_error ("::toString()");

    return o.str ();
}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/// SICK 
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
struct VMCSickMessage{
	double ts;
	int scan_num;
	int sensor_id;
        double range_max;
	double start_ang;
	double ang_inc;
	int num_dist;
        std::vector<double> r;

   
/*struct NDTMessage{
  	
  	double resolution;

};*/
    //std::vector<Matrix2d> cov_array;
    //std::Vector<Vector3d> mean_array;
	void printHeader(){
            //             <ts>    <sn> <id><rm><nd> <sa> <ai>  <.......>
            //sick 1450875080.6804 48510 0 29.96 541 -135 0.500 1.19 1.17 1.17 1.15 1.16 1.14 1.13 1.13 1.11 1.11 1.09 1.09 -> new log file version
            //sick 1369157522.6773 147 0 751 -135 0.360 0.885 0.898 0.915 0.908 0.915 0.918 0.928 -> old log file version
            fprintf(stderr,"Sick: %lf %d %d %lf %lf %d r:[%d -- last = %lf]\n", ts, scan_num, sensor_id, start_ang,ang_inc,num_dist,(int)r.size(), r[r.size()-1]);
	}
	
	void getRosScan(sensor_msgs::LaserScan &scan){
		ros::Time rt(ts);
		scan.header.stamp = rt;
		scan.header.frame_id = "laserscan" + toString(sensor_id);
		
		scan.angle_min = start_ang*M_PI/180.0;
		scan.angle_max = start_ang*M_PI/180.0 + num_dist*ang_inc*M_PI/180.0;
		scan.angle_increment = ang_inc*M_PI/180.0;
		
		scan.range_min = 0.1;
		scan.range_max = range_max;
		
		for(int i=0;i<num_dist;i++) scan.ranges.push_back(r[i]);		 
		
	}

  double getTs() const {
    return ts;
  }

};

geometry_msgs::TransformStamped getStaticTF(double lx, double ly, double la, ros::Time ts, std::string frame, std::string child_frame){
		geometry_msgs::Quaternion state_quat = tf::createQuaternionMsgFromYaw(la);
		geometry_msgs::TransformStamped state_trans;
		state_trans.header.stamp = ts;
		state_trans.header.frame_id = frame;
		state_trans.child_frame_id = child_frame;
		
		state_trans.transform.translation.x = lx;
		state_trans.transform.translation.y = ly;
		state_trans.transform.translation.z = 0.0;
		state_trans.transform.rotation = state_quat;
		
		return state_trans;
	}


/**
 * Parse SICK message -- 'new' Protocol 4
 */
VMCSickMessage parseSickMessage(char *tok){
	VMCSickMessage sm;
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.ts = atof(tok);  ///TS
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.scan_num = atoi(tok); ///Scan number (rotating 0...255)
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.sensor_id = atoi(tok); ///Sensor ID
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.range_max = atof(tok); ///Range max value

        if (sm.range_max > 200.)
          std::cerr << "range_max > 200. - is the correct protocol used?" << std::endl;

	tok = strtok(NULL," ");
	if(tok!=NULL) sm.num_dist = atoi(tok); ///Number of distances
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.start_ang = atof(tok); ///Start Ang (deg)
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.ang_inc = atof(tok); ///Angle incr
	
	tok = strtok(NULL," ");
	while(tok!=NULL){
		double range = atof(tok);
		if(tok != NULL) sm.r.push_back(range);
		tok = strtok(NULL," ");
	}
	
	//if(sm.r.size() != sm.num_dist) fprintf(stderr,"(%d != %d)INVALID NUMBER OF ELEMENTS IN SCAN!!!\n",sm.r.size(),sm.num_dist);
	
//	sm.printHeader();
	return sm;
}
/**
 * Parse SICK message -- The Protocol 3
 */
VMCSickMessage parseSickMessageProtocol3(char *tok){
	VMCSickMessage sm;
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.ts = atof(tok);  ///TS
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.scan_num = atoi(tok); ///Scan number (rotating 0...255)
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.sensor_id = atoi(tok); ///Sensor ID
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.num_dist = atoi(tok); ///Number of distances
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.start_ang = atof(tok); ///Start Ang (deg)
	
	tok = strtok(NULL," ");
	if(tok!=NULL) sm.ang_inc = atof(tok); ///Angle incr
	
	tok = strtok(NULL," ");
	while(tok!=NULL){
		double range = atof(tok);
		if(tok != NULL) sm.r.push_back(range);
		tok = strtok(NULL," ");
	}
	
	//if(sm.r.size() != sm.num_dist) fprintf(stderr,"(%d != %d)INVALID NUMBER OF ELEMENTS IN SCAN!!!\n",sm.r.size(),sm.num_dist);
	
//	sm.printHeader();
	return sm;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///STATE
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
struct VMCStateMessage{
	double ts;
	double x;
	double y;
	double a;
	int pos_known;
	int certainty;

        void setZero() {
	  ts = 0.;
	  x = 0.;
	  y = 0.;
	  a = 0.;
	  pos_known = 0;
	  certainty = 0;
	}
	
	void parseStateMessage(char *tok){
		tok = strtok(NULL," ");
		if(tok!=NULL) ts = atof(tok);  ///TS
		tok = strtok(NULL," ");
		if(tok!=NULL) x = atof(tok);  ///TS
		tok = strtok(NULL," ");
		if(tok!=NULL) y = atof(tok);  ///TS
		tok = strtok(NULL," ");
		if(tok!=NULL) a = atof(tok);  ///TS
		tok = strtok(NULL," ");
		if(tok!=NULL) pos_known = atoi(tok);  ///TS
		tok = strtok(NULL," ");
		if(tok!=NULL) certainty = atoi(tok);  ///TS
	}
	
	geometry_msgs::TransformStamped getTF(){
		geometry_msgs::Quaternion state_quat = tf::createQuaternionMsgFromYaw(a);
		geometry_msgs::TransformStamped state_trans;
		state_trans.header.stamp = ros::Time(ts);
		state_trans.header.frame_id = "/world";
		state_trans.child_frame_id = "/state_base_link";
		
		state_trans.transform.translation.x = x;
		state_trans.transform.translation.y = y;
		state_trans.transform.translation.z = 0.0;
		state_trans.transform.rotation = state_quat;
		
		return state_trans;
	}
	
	nav_msgs::Odometry getRosOdometry(){
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry state;
		state.header.stamp = ros::Time(ts);
    		state.header.frame_id = "/world";
		
		//set the position
		state.pose.pose.position.x = x;
		state.pose.pose.position.y = y;
		state.pose.pose.position.z = 0.0;
		geometry_msgs::Quaternion state_quat = tf::createQuaternionMsgFromYaw(a);
		state.pose.pose.orientation = state_quat;
		
		
		//set the velocity
		state.child_frame_id = "state_base_link";
		return state;
	}

  double getTs() const {
    return ts;
  }
  
};
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/// Encoders
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
struct VMCEncoderMessage{
	///These are from the message
	double ts;
	double driveVel;
	double alpha;
	
	///Derived odometry
	double prev_ts;
	double x;
	double y;
	double a;
	
	VMCEncoderMessage(){
		ts = -1;
		prev_ts = -1.;
		x = 0;
		y = 0;
		a = 0;
	}
	
        void parseMessage(char *tok, double steering_wheel_distance, double steering_wheel_distance_y){
		tok = strtok(NULL," ");
		if(tok!=NULL) ts = atof(tok);  ///TS
		bool vel_ok=false;
		bool a_ok = false;
		tok = strtok(NULL," ");
		
		if(tok!=NULL){
			driveVel = atof(tok);  
			vel_ok = true;	
		}
		
		tok = strtok(NULL," ");
		if(tok!=NULL){
			alpha = atof(tok);  
			a_ok = true;
		}

		if(a_ok && vel_ok){
                        if(prev_ts < 0.) {
                            prev_ts = ts;
                        }
			
			double dt = ts-prev_ts;
	
			double odom_drive_vel = driveVel;
			double drive_dist = (odom_drive_vel)*dt;
			double odom_alpha = alpha;
			double dd = 0.;
			double da = 0.;
		     
			if (fabs(odom_alpha) < 0.001) // Avoid dividing with a very small number...
			{
					dd = drive_dist;
					da = 0.;
			}
			else{
				double r_stear = steering_wheel_distance / sin(odom_alpha);
				double r_fix = r_stear * cos(odom_alpha) + steering_wheel_distance_y;
					
				dd = r_fix / r_stear * drive_dist; // Adjust the actual forward movement (located between the fixed front wheels) based on the radius of the drive wheel).
				da = drive_dist / r_stear;
			}
	
			// Update the current estimate
			double dx = dd * cos( a + da/2.0 );
			double dy = dd * sin( a + da/2.0 );
			
			x+=dx;
			y+=dy;
			a+=da;
			prev_ts = ts;
		}	
	}
	
	///////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////
	geometry_msgs::TransformStamped getTF(){
		geometry_msgs::Quaternion state_quat = tf::createQuaternionMsgFromYaw(a);
		geometry_msgs::TransformStamped state_trans;
		state_trans.header.stamp = ros::Time(ts);
		state_trans.header.frame_id = "/world";
		state_trans.child_frame_id = "/odom_base_link";
		
		state_trans.transform.translation.x = x;
		state_trans.transform.translation.y = y;
		state_trans.transform.translation.z = 0.0;
		state_trans.transform.rotation = state_quat;
		
		return state_trans;
	}
		
	nav_msgs::Odometry getRosOdometry(){
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry state;
		state.header.stamp = ros::Time(ts);
		state.header.frame_id = "/world";
		
		//set the position
		state.pose.pose.position.x = x;
		state.pose.pose.position.y = y;
		state.pose.pose.position.z = 0.0;
		geometry_msgs::Quaternion state_quat = tf::createQuaternionMsgFromYaw(a);
		state.pose.pose.orientation = state_quat;
		
		
		//set the velocity
		state.child_frame_id = "odom_base_link";
		return state;
	}	
	
  double getTs() const {
    return ts;
  }
	
};

    
