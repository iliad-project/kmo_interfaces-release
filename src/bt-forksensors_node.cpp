#include <ros/ros.h>
#include <orunav_msgs/ForkReport.h>
#include <visualization_msgs/Marker.h>
#include <kmo_cpi_interface/CPISocket.h>
#include <boost/thread/mutex.hpp>


class BTForkSensorsNode {

    private:
    ros::NodeHandle nh_;
    ros::Publisher forkreport_pub_;
    ros::Timer heartbeat_reports_;

    CPISocket cpi;

public:
  BTForkSensorsNode(ros::NodeHandle &paramHandle) {

            double hb_report;
            paramHandle.param("heartbeat",hb_report,0.1);
            std::string cpi_host;
            paramHandle.param<std::string>("cpi_host", cpi_host, std::string("192.168.100.100"));
            cpi.setParams(cpi_host, 5677);
            cpi.reconnect();

            forkreport_pub_ = nh_.advertise<orunav_msgs::ForkReport>("control/fork_report", 1000);

            heartbeat_reports_   = nh_.createTimer(ros::Duration(hb_report),&BTForkSensorsNode::publish_reports,this);
        }

  ~BTForkSensorsNode() {
  }

  void publish_reports(const ros::TimerEvent& event) {

    if (!cpi.isConnected()) {
        ROS_ERROR_STREAM("Socket to CPI is not connected(!) - host : " << cpi.getHost() << " port : " << cpi.getPort());
        return;
    }
    orunav_msgs::ForkReport report;

    report.status = report.FORK_POSITION_KNOWN;
    report.stamp = ros::Time::now();
    report.state.center = cpi.getBTForkCenter();
    report.state.right = cpi.getBTForkRight();
    report.state.left = cpi.getBTForkLeft();
    report.state.position_z = cpi.getBTForkHeight();

    forkreport_pub_.publish(report);
  }


};

using namespace std;

int main(int argc, char** argv) {

    ros::init(argc,argv,"bt-forksensors");
    ros::NodeHandle params ("~");

    BTForkSensorsNode node(params);

    ros::spin();

}
