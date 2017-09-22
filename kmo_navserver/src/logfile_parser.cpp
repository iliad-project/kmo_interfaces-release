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
#include <rosbag/bag.h>
#include <boost/program_options.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <kmo_navserver/kmo_navserver_parser.h>

namespace po = boost::program_options;

using Eigen::Matrix2d;
using Eigen::Vector2d;
using namespace Eigen;
using namespace std;

double steering_wheel_distance;
double steering_wheel_distance_y;
bool LogInitialTimeSet=false;

ros::Time LogInitialTime;
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/// KolmorgenMapParser - A class for parsing the NDT maps from kollmorgen contained in their log files
///////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////// 
 class KolmorgenMapParser{
 private: //A map contain the following variables: Mean covariance for a set of NDT uncertainties, resolution of map, position of center (x,y)
   std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > mean_array_;
   std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > cov_array_;

   double resolution_;
   double center_x_;
   double center_y_;
   double scaleFactor=1/1000;

   void RemoveQuotation(char ** tok);
 public:
   visualization_msgs::MarkerArray marker_array_msg_;
 	 bool ContainNDTMessage(char * tok); // check if tok contain an NDT MAP Rule: "/><ns2:NdtCells"
 	 void ReadNDTMessage(char * tok);   // parse all data into the class instance.
   void CovarToScaleRotation(Matrix3d &cov,Quaterniond &q, Vector3d &eigenValues);
   void CreateMarkerArray();
   void Write();//Test class
   bool mapIsEmpty_=true;

};

void KolmorgenMapParser::Write(){
center_x_=0;
center_y_=0;
resolution_=300.0;
Vector3d mean;
Matrix3d cov;
/*mean<<1,2,0;
cov<<1,0,0,
     0,2,0,
     0,0,3;
cov_array_.push_back(cov);
mean_array_.push_back(mean); */
/*mean<<1,2,0;
cov.block<1,1>(2,2)<<3, 1,
                     1,3;

cov_array_.push_back(cov);
mean_array_.push_back(mean);

*/

}


bool KolmorgenMapParser::ContainNDTMessage(char * tok){
    	tok=strtok(NULL," ");

    	while(tok!=NULL){
          if(strncmp("/><ns2:NdtCells",tok,15)==0 ||strncmp("/><ns0:NdtCells",tok,15) ==0){ //tok before start of data
              cout<<"contain NDT map"<<endl;
              return true;
            }
            else
            	tok=strtok(NULL," ");
    	}
    	return false;
    	
}
void KolmorgenMapParser::ReadNDTMessage(char *tok)
{
  int i=1;
  double parsedVal;
  Vector3d tmp_vek;
  tmp_vek<<0, 0, 0.001;
  Matrix3d tmp_mat;
  tmp_mat<<0.001, 0, 0,
           0, 0.001, 0,
           0, 0, 0.001;
  //cout<<"is a NDT message"<<endl;
 //First sequence should contain the variables CenterX="0.000000" CenterY="0.000000" Resolution="300.0000" ..
  tok = strtok(NULL," ");
  RemoveQuotation(&tok); // read center coordinate
  if(tok!=NULL) center_x_ = atof(tok);

	    tok = strtok(NULL," ");
    RemoveQuotation(&tok);// read center coordinate
  if(tok!=NULL) center_y_ = atof(tok);

	    tok = strtok(NULL," ");
    RemoveQuotation(&tok);		// read resulotion
  if(tok!=NULL) resolution_ = atof(tok);


    tok = strtok(NULL," ");
    if(strncmp("CoordX",tok,6)!=0 ){ // if not coordX then the NDT map is empty
      cout<<"NDT Field of file empty"<<endl;
      return;
    }
    while(true){
      tok = strtok(NULL," ");
      if(strncmp("/></ns2:NdtCells>",tok,17)==0 || strncmp("/></ns0:NdtCells>",tok,17)==0){ // if last cell
       break;
      }
      else if(tok!=NULL && (strncmp("/><ns2:NdtCell",tok,14)==0 || strncmp("/><ns0:NdtCell",tok,14)==0)){ //NDT XML separator do nothing

      }
      else{
        mapIsEmpty_=false;
        RemoveQuotation(&tok);
        parsedVal=atof(tok);
        //cout<<"parsade"<<parsedVal<<endl;
        switch(i%5){
          case 0: tmp_vek(0)=parsedVal;
          break;
        case 1: tmp_vek(1)=parsedVal;
          break;
          case 2: tmp_mat(0,0)=parsedVal;
				break;
          case 3: tmp_mat(1,0)=parsedVal; tmp_mat(0,1)=parsedVal; //diagonal elements
          break;
				case 4: tmp_mat(1,1)=parsedVal;
          mean_array_.push_back(tmp_vek);
          cov_array_.push_back(tmp_mat);
          //cout<<"Vector =\n"<<tmp_vek<<endl;
          //cout<<"Matrix =\n"<<tmp_mat<<endl;
        break;
			  }
        i++;
      }
    }
    cout<<"A total of "<<mean_array_.size()<<" NDT cells were found in the log" <<endl;
}
void KolmorgenMapParser::RemoveQuotation(char ** tok)//return true if succesfull
{
	char* str=(*tok);
	int index=0, lastindex=0, i=0;
  while(i<30){//Maximum of 20 signs
    	if(str[i]=='"' && index==0){
      		index=i+1;
      	}
		else if(str[i]=='"' && index!=0){
			lastindex=i;
	    	str[i]='\0';
	    	break;
	    }
	    i++;
	}
    (*tok)=(*tok+index);
}
void KolmorgenMapParser::CovarToScaleRotation(Matrix3d &cov,Quaterniond &q,Vector3d &eigenValues){
//Matrix3d eigenVectors;

    Matrix2d mat= cov.block(0,0,2,2);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(mat);
    //cout<<"solving :\n"<<mat<<endl;
    Eigen::Vector2d eigValues = eig.eigenvalues();
    Eigen::Matrix2d eigVectors= eig.eigenvectors();
    //cout<<"solution =\n "<<eigVectors<<endl;
    double angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

    double lengthMajor = eigValues(0);
    double lengthMinor = eigValues(1);
    eigenValues<<sqrt(lengthMajor),sqrt(lengthMinor),0;
    q.x() = 0;
    q.y() = 0;
    q.z() = sin(angle*0.5);
    q.w() = cos(angle*0.5);
}

void KolmorgenMapParser::CreateMarkerArray(){
  Quaterniond q;
  sleep(1);
  Vector3d eigenValues;

  if(!LogInitialTimeSet)
    abort();
 visualization_msgs::Marker marker;
  //cout<<"there exists a total of "<<mean_array_.size()<<" of markers"<<endl;
 for(int i=0;i<mean_array_.size();i++){


  marker.header.frame_id = "world";
  marker.header.stamp =LogInitialTime+ ros::Duration(2.0);
  marker.ns = "NDT";
  marker.id = i;
  marker.type=visualization_msgs::Marker::SPHERE;
  marker.pose.position.x=(mean_array_[i](0)+center_x_)/1000.0;
  marker.pose.position.y=(mean_array_[i](1)+center_y_)/1000.0;
  marker.pose.position.z=0.01;
  CovarToScaleRotation(cov_array_[i],q, eigenValues);
  //cout<<"eig =\n"<<eigenValues<<endl<<"______________________________"<<endl;
  marker.pose.orientation.x=q.x();
  marker.pose.orientation.y=q.y();
  marker.pose.orientation.z=q.z();
  marker.pose.orientation.w=q.w();
  //cout<<"Got following eigen values\n "<<eigenValues<<endl;
  marker.scale.x=eigenValues(0)*3.0/1000.0;
  marker.scale.y=eigenValues(1)*3.0/1000.0;
  marker.scale.z=0.01;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.9f;
  marker.lifetime=ros::Duration(50000);
  marker.frame_locked=false;
  //marker.text="test";
  marker.action= visualization_msgs::Marker::ADD;
  marker_array_msg_.markers.push_back(marker);
 }

}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/// End of KolmorgenMapParser - A class for parsing the NDT maps from kollmorgein contained in their log files
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
/// MAIN
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
int main (int ac, char* av[])
{

    std::string file_name;
    int protocol_ver;
    double range_max;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("file_name", po::value<string>(&file_name)->required(), "input file name (Kollmorgen format)")
        ("steering_wheel_distance", po::value<double>(&steering_wheel_distance)->default_value(1.380), "steering wheel offset (fwd direction) relative to the fixed wheels / the base origin")
        ("steering_wheel_distance_y", po::value<double>(&steering_wheel_distance_y)->default_value(-0.233), "steering wheel offset (side direction) relative to the fixed wheels / the base origin")
      ("protocol_ver", po::value<int>(&protocol_ver)->default_value(4), "protocol version (3: this has no max range data in the sick messages and was used before 2016)")
      ("range_max", po::value<double>(&range_max)->default_value(29.), "maximum range parameter used for the sick data struct, only available if protocol < 4")
      ;

    // Arla parameters
    // steering_wheel_distance 1380 mm
    // steering_wheel_distance_y -233 mm

    // Survey parameters
    // steering_wheel_distance 1357 mm
    // steering_wheel_distance_y -233 mm


    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);    
   // string kollmorgenMap_file_name="KollmorgenMap";
    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    string str;
    ifstream infile;
    std::cout << "Loading : " << file_name << std::endl;
    infile.open (file_name.c_str()); ///<Input 
    std::cout << "... done." << std::endl;
    rosbag::Bag obag; //,KollmorgenMapBag;
    std::cout << "Writing to : " << file_name+".bag" << std::endl;
    //std::cout << "Writing to : " << kollmorgenMap_file_name+".bag" << std::endl;
    obag.open(file_name+".bag", rosbag::bagmode::Write); ///output
   // KollmorgenMapBag.open(kollmorgenMap_file_name+".bag", rosbag::bagmode::Write); ///output
    KolmorgenMapParser mapParser;
    VMCEncoderMessage Enc; // This keeps track of the odometry internally. Need to have it outside the loop.
    tf::tfMessage tfmsg; // Seems that we need to push more before publishing? Having this inside each line will only allow for 2 tf to be written(!?!).
    while(!infile.eof()) // To get you all the lines.
    {
       
       getline(infile, str);
			 //cout << str << endl;
			 
			 const char* delim = " ";
			 char* msg = (char*)str.c_str();
			 char *tok = strtok(msg,delim);
                         std_msgs::Header header; // Simply to store the time.
			 
			 if(tok!=NULL) {
                             if(strncmp("sick",tok,4) == 0){
                                         VMCSickMessage sm;
                                         if (protocol_ver == 3) {
                                           sm = parseSickMessageProtocol3(tok);
                                           sm.range_max = range_max;
                                         }
                                         else {
                                           sm = parseSickMessage(tok);
                                         }
					 sensor_msgs::LaserScan scan;
					 sm.getRosScan(scan);

					 if(!LogInitialTimeSet){
					   LogInitialTime=ros::Time(sm.getTs());
					   LogInitialTimeSet=true;
					 }

					 obag.write("/laserscan"+toString(sm.sensor_id),scan.header.stamp,scan);
                                         // It is better to add a static broadcaster here instead (if you want to change the sensor pose due to update calibration params)
                                         // Also for visualization etc. it is easy to change to which coordsystem you fit the laser scan data to.
                                         geometry_msgs::TransformStamped ltf = getStaticTF(0., 0., 0.,scan.header.stamp,std::string("/base_laser_link"),std::string("/laserscan")); // 1.415,0.485,45*M_PI/180.0,
					 tfmsg.transforms.push_back(ltf);
                                         header = ltf.header;
                                         					 
                             }else if(strncmp("state2",tok,6) == 0){ // Probably this entries could be ignored?
					VMCStateMessage state;
					state.parseStateMessage(tok);
                                        nav_msgs::Odometry ros_state = state.getRosOdometry();
					if(!LogInitialTimeSet){
					  LogInitialTime=ros::Time(state.getTs());
					  LogInitialTimeSet=true;
					 }
                                        obag.write("/state2",ros_state.header.stamp,ros_state);

                                        geometry_msgs::TransformStamped stf = state.getTF();
                                        stf.child_frame_id = "/state2_base_link"; // Important to not mix the tf's up.
					tfmsg.transforms.push_back(stf);
                                        header = stf.header;
					
                             }else if(strncmp("state",tok,5) == 0){
					VMCStateMessage state;
					state.parseStateMessage(tok);
                                        if(!LogInitialTimeSet){
					  LogInitialTime=ros::Time(state.getTs());
					  LogInitialTimeSet=true;
					 }
                                        nav_msgs::Odometry ros_state = state.getRosOdometry();
                                        obag.write("/state",ros_state.header.stamp,ros_state);

					geometry_msgs::TransformStamped stf = state.getTF();
					tfmsg.transforms.push_back(stf);
                                        header = stf.header;
					
                             }else if(strncmp("enc",tok,3) == 0){
                                         Enc.parseMessage(tok, steering_wheel_distance, steering_wheel_distance_y);
					 if(!LogInitialTimeSet){
					   LogInitialTime=ros::Time(Enc.getTs());
					   LogInitialTimeSet=true;
					 }
                                         nav_msgs::Odometry ros_odom = Enc.getRosOdometry();
                                         obag.write("/odom",ros_odom.header.stamp,ros_odom);

					 geometry_msgs::TransformStamped odotf = Enc.getTF();
					 tfmsg.transforms.push_back(odotf);
                                         header = odotf.header;
                             }else if(strncmp("</SdoServer>",tok,12)==0 || strncmp("<NavLab",tok,7)==0){

                                if(mapParser.ContainNDTMessage(tok)){
                                  mapParser.ReadNDTMessage(tok);
                                }

                             }
         
			 }
                         if (tfmsg.transforms.size() > 3) {
                           obag.write("/tf",header.stamp,tfmsg);
                           tfmsg.transforms.clear();
                         }

			 
			 
			 
    }
    if(!mapParser.mapIsEmpty_){
      mapParser.CreateMarkerArray();
      obag.write("/KollmorgenMap",mapParser.marker_array_msg_.markers[0].header.stamp,mapParser.marker_array_msg_);
    }

    infile.close();
    obag.close();
    //KollmorgenMapBag.close();
    std::cout << "... done." << std::endl;

}


