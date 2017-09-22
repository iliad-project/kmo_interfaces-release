/*
 *
 * sicks300.cpp
 *
 *
 * Copyright (C) 2010
 * Autonomous Intelligent Systems Group
 * University of Bonn, Germany
 *
 *
 * Authors: Andreas Hochrath, Torsten Fiolka
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * Origin:
 *  Player - One Hell of a Robot Server
 *  serialstream.cc & sicks3000.cc
 *  Copyright (C) 2003
 *     Brian Gerkey
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 */

#include "sick_s3x/sicks300.h"

#include "termios.h"

SickS300::SickS300()
{

  ros::NodeHandle param_node("~");
  ros::NodeHandle nodeHandle("/");

  int param;
  double x, y, z, roll, pitch, yaw;

  // reading transformation parameters from parameter server
  param_node.param(std::string("frame"), scan_data_.header.frame_id, std::string("base_laser_link"));
  param_node.param(std::string("send_transform"), param, 1);
  if (param)
  {
    send_transform_ = true;
  }
  else
  {
    send_transform_ = false;
  }
  // TODO, need also the rotation of the laser...
  param_node.param(std::string("tf_x"), x, 0.115);
  param_node.param(std::string("tf_y"), y, 0.0);
  param_node.param(std::string("tf_z"), z, 0.21);

  param_node.param(std::string("tf_roll"), roll, 0.);
  param_node.param(std::string("tf_pitch"), pitch, 0.);
  param_node.param(std::string("tf_yaw"), yaw, 0.);

  param_node.param<std::string>(std::string("base_link"), base_link_, std::string("/state_base_link"));

  transform_vector_ = tf::Vector3(x, y, z);
  transform_quat_.setRPY(roll,pitch,yaw);
  param_node.param(std::string("model"), model_, std::string("S300"));

  // Setting full field of view (270 degree) or reduced (180 degree)
  param_node.param(std::string("reduced_fov"), param, 0);
	
	
	
  if (param != 0){
    reduced_FOV_ = true;
    ROS_INFO("INFO: Starting Sick3x-Laser with reduced field ov view of 180 degree");
  }
  else{
    reduced_FOV_ = false;
  }
  
  param_node.param(std::string("laser_flipped"), param, 0);
	if(param!=0){
		laser_flipped = true;
		ROS_INFO("INFO: Laser Flipped (upside-down) enabled!!! -- THIS WORKS ONLY WITHOUT REDUCED FOV!");
	}else{
		laser_flipped = false;
	}
	
  
  

  if (model_ == std::string("S3000"))
  {
		ROS_INFO("INFO: Using S3000 scanner model");
		if (!reduced_FOV_){
			scan_data_.angle_min = -95.f / 180.f * M_PI;
	    scan_data_.angle_max = 95.f / 180.f * M_PI;
	    scan_data_.angle_increment = 0.5f / 180.f * M_PI;
	    scan_data_.time_increment = 0;
	    scan_data_.scan_time = 0.08; // same?
	    scan_data_.range_min = 0.1;
	    scan_data_.range_max = 49.0;
	    scan_data_.ranges.resize(381);
	    scan_data_.intensities.resize(381);
		}else{
	    scan_data_.angle_min = -90.f / 180.f * M_PI;
	    scan_data_.angle_max = 90.f / 180.f * M_PI;
	    scan_data_.angle_increment = 0.5f / 180.f * M_PI;
	    scan_data_.time_increment = 0;
	    scan_data_.scan_time = 0.08;
	    scan_data_.range_min = 0.1;
	    scan_data_.range_max = 49.0;
	    scan_data_.ranges.resize(361);
	    scan_data_.intensities.resize(361);
		}
  }
  else
  {
       ROS_INFO("INFO: Using S300 scanner model");
       if (!reduced_FOV_)
       {
	    scan_data_.angle_min = -135.f / 180.f * M_PI;
	    scan_data_.angle_max = 135.f / 180.f * M_PI;
	    scan_data_.angle_increment = 0.5f / 180.f * M_PI;
	    scan_data_.time_increment = 0;
	    scan_data_.scan_time = 0.08;
	    scan_data_.range_min = 0.1;
	    scan_data_.range_max = 29.0;
	    scan_data_.ranges.resize(541);
	    scan_data_.intensities.resize(541);
       }
       else
       {
	    scan_data_.angle_min = -90.f / 180.f * M_PI;
	    scan_data_.angle_max = 90.f / 180.f * M_PI;
	    scan_data_.angle_increment = 0.5f / 180.f * M_PI;
	    scan_data_.time_increment = 0;
	    scan_data_.scan_time = 0.08;
	    scan_data_.range_min = 0.1;
	    scan_data_.range_max = 29.0;
	    scan_data_.ranges.resize(361);
	    scan_data_.intensities.resize(361);
       }
       
  }
  
  // Reading device parameters
  param_node.param(std::string("devicename"), device_name_, std::string("/dev/ttyUSB0"));
  param_node.param(std::string("baudrate"), baud_rate_, 500000);

  connected_ = serial_comm_.connect(device_name_, baud_rate_);

  scan_data_publisher_ = nodeHandle.advertise<sensor_msgs::LaserScan> ("laserscan", 10);

}

SickS300::~SickS300()
{
}

bool SickS300::update()
{

  if (connected_ != 0){
    connected_ = serial_comm_.connect(device_name_, baud_rate_);
  }

  if (connected_ == 0 && serial_comm_.readData(scan_data_.header.stamp) == 0){
    //scan_data_.header.stamp = ros::Time::now();
    float* ranges = serial_comm_.getRanges();
    unsigned int numRanges = serial_comm_.getNumRanges();
    if (model_ == std::string("S3000")){
			if (!reduced_FOV_){
				scan_data_.ranges.resize(numRanges);
				if(!laser_flipped){ 
					for (unsigned int i = 0; i < numRanges; i++) scan_data_.ranges[i] = ranges[i];
				}else{
					for (unsigned int i = 0; i < numRanges; i++) scan_data_.ranges[i] = ranges[numRanges-1-i];
				}
			}
			else{
				 for (unsigned int i = 0; i < 361; i++) scan_data_.ranges[i] = ranges[i + 9];
			}
    }///S3000
    else{
			if (!reduced_FOV_)
			{
						scan_data_.ranges.resize(numRanges);
						if(!laser_flipped){ 
							for (unsigned int i = 0; i < numRanges; i++) scan_data_.ranges[i] = ranges[i];
						}else{
							for (unsigned int i = 0; i < numRanges; i++) scan_data_.ranges[i] = ranges[numRanges-1-i];
						}
			}
			else
			{
						for (unsigned int i = 0; i < 361; i++) scan_data_.ranges[i] = ranges[i + 89];
			}
    }
    this->broadcast_transform();
    ros::Time tmp = ros::Time::now();
    scan_data_.header.stamp = tmp; 
    scan_data_publisher_.publish(scan_data_);
    //ROS_INFO("System time %i, NTime : %llu", tmp.useSystemTime(), tmp.toNSec());
    return true;
  }
  return false;
}

void SickS300::broadcast_transform()
{
  if (send_transform_)
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(transform_quat_, transform_vector_),
                                                      ros::Time::now(), base_link_, "base_laser_link"));
  }

}


std::string
SickS300::get_model() const
{
     return this->model_;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "sicks300");
  ros::Time::init();
  ros::Rate loop_rate(150); // was before 20, this needed to be increased to not loose packages for me - Henrik 

  ROS_INFO("Opening connection to Sick300-Laser...");

  SickS300 sickS300;

  ROS_INFO("%s connected.", sickS300.get_model().c_str());

  while (ros::ok())
  {

    if( sickS300.update())
      {
	//sickS300.broadcast_transform();
      }
    else {
      //      usleep(1000*1000);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Laser shut down.");

  return 0;
}
