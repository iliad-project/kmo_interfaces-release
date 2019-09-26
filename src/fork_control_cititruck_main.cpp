#include <ros/ros.h>
#include <orunav_msgs/ForkCommand.h>
#include <orunav_msgs/ForkReport.h>
#include <visualization_msgs/Marker.h>
#include <kmo_cpi_interface/CPISocket.h>
#include <boost/thread/mutex.hpp>


class ForkControlCitiTruckNode {

    private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher forkreport_pub_;
    ros::Subscriber forkcommand_sub_;
    ros::Timer heartbeat_reports_;
    bool visualize;
  double lift_threshold_;
  double support_leg_threshold_;
    orunav_msgs::ForkReport current_report_;
    double fork_max_moving_time_;
  double fork_moving_time_support_to_low_;
  double delay_support_to_low_;  
  std::string cpi_host_;
    boost::mutex reports_mutex_;
    bool fake_movement_;
public: 
	ForkControlCitiTruckNode (ros::NodeHandle &paramHandle) {
	  

	    forkcommand_sub_ = nh_.subscribe<orunav_msgs::ForkCommand>("control/fork_command",10,&ForkControlCitiTruckNode::process_forkcommand,this);
            double hb_report;
            paramHandle.param("heartbeat",hb_report,0.1);
	    paramHandle.param<bool>("visualize",visualize,true);
            paramHandle.param<double>("lift_threshold", lift_threshold_, 0.05);
            paramHandle.param<double>("support_leg_threshold", support_leg_threshold_, -0.01);
            paramHandle.param<std::string>("cpi_host", cpi_host_, std::string("192.168.100.100"));
            paramHandle.param<double>("fork_max_moving_time", fork_max_moving_time_, 2.5);
            paramHandle.param<bool>("fake_movement", fake_movement_, false);
            paramHandle.param<double>("fork_moving_time_support_to_low", fork_moving_time_support_to_low_, 0.2);
            paramHandle.param<double>("delay_support_to_low", delay_support_to_low_, 0.2);
            
            forkreport_pub_ = nh_.advertise<orunav_msgs::ForkReport>("control/fork_report", 1000);            
  	    if (visualize)
	    {
                std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
                marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	    }
            
           
	    heartbeat_reports_   = nh_.createTimer(ros::Duration(hb_report),&ForkControlCitiTruckNode::publish_reports,this);

            current_report_.status = current_report_.FORK_POSITION_UNKNOWN;
            current_report_.state.position_z = 0.;
        }

	~ForkControlCitiTruckNode() {

        }
  
	void publish_reports(const ros::TimerEvent& event) {
	  reports_mutex_.lock();
	  forkreport_pub_.publish(current_report_);
	  reports_mutex_.unlock();
	}

    


    void process_forkcommand(const orunav_msgs::ForkCommand::ConstPtr &msg) {

        ROS_INFO("Got fork command");
        if (msg->state.position_z > lift_threshold_) {
            // Lift the forks (unless they are already up).
            if (current_report_.status != current_report_.FORK_POSITION_HIGH)
            {
                ROS_INFO("Lifting forks");
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_MOVING_UP;
                reports_mutex_.unlock();
                
                if (fake_movement_) {
                    usleep(fork_max_moving_time_*1000*1000);
                }
                else {
                    CPISocket cpi(cpi_host_, 5677);
                    cpi.liftForksBlocking(fork_max_moving_time_);
                }
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_POSITION_HIGH;
                reports_mutex_.unlock();
            }
        }
        else if (msg->state.position_z < support_leg_threshold_) {
          if (current_report_.status != current_report_.FORK_POSITION_SUPPORT_LEGS)
          {
                ROS_INFO("Lowering forks - enable support legs");
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_MOVING_DOWN;
                reports_mutex_.unlock();
                
                if (fake_movement_) {
                    usleep(fork_max_moving_time_*1000*1000);
                }
                else {
                    CPISocket cpi(cpi_host_, 5677);
                    cpi.lowerForksBlocking(fork_max_moving_time_);
                }
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_POSITION_SUPPORT_LEGS;
                reports_mutex_.unlock();
            }
        }
        else
        {
            // Lower the forks (unless they are already low)
            if (current_report_.status != current_report_.FORK_POSITION_LOW)
            {
                ROS_INFO("Lowering forks (1: lower full, 2: lift slightly)");
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_MOVING_DOWN;
                reports_mutex_.unlock();

                if (fake_movement_) {
                    usleep(fork_max_moving_time_*1000*1000);
                    usleep((fork_moving_time_support_to_low_+delay_support_to_low_)*1000*1000);
                }
                else {
                    CPISocket cpi(cpi_host_, 5677);
                    cpi.lowerForksBlocking(fork_max_moving_time_);
                    usleep(delay_support_to_low_*1000*1000);
                    cpi.liftForksBlocking(fork_moving_time_support_to_low_);
                }
                // This means that we have lowered the forks all the way down...
                // we now need to lift them up in order to be in the LOW position state.
                // Now we're done.
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_POSITION_LOW;
                reports_mutex_.unlock();
                

            }
        }
    }
};

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc,argv,"fork_control_cititruck");
    ros::NodeHandle params ("~");

    ForkControlCitiTruckNode node(params);

    ros::spin();

}
