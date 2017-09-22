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

#include <boost/program_options.hpp>
#include <kmo_navserver/kmo_navserver_driver.hh>

namespace po = boost::program_options;

#include <iostream>
#include <iterator>
using namespace std;


/* ---[ */
int 
main (int argc, char **argv)
{
  // Init ROS
  ros::init (argc, argv, "kmo_navserver_node"); 

  ros::NodeHandle comm_nh ("kmo_navserver"); // for topics, services
  ros::NodeHandle param_nh ("~");     // for parameters

  po::options_description desc("Allowed options");
  desc.add_options()
       ("help", "produce help message")
       ("host", po::value<string>(), "IP to host")
       ("port", po::value<string>(), "port value (default 5432)")
       ("enc", "collect encoder values")
       ("laser", "collect laserway laser values")
       ("sick", "collect sick data")
       ("state", "collect state (pose) data")
       ("received_logname", "received log filename")
       ("sent_logname", "sent log filename");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    
  
  if (vm.count("help")) {
       cout << desc << "\n";
       return 1;
  }
  string host = "192.168.0.100";
  if (vm.count("host"))
       host = vm["host"].as<string>();
  int port = 5432;
  if (vm.count("port"))
       port = vm["port"].as<int>();
  
  string received_logname = "nav_received.log";
  if (vm.count("received_logname"))
       received_logname = vm["received_logname"].as<string>();
  string sent_logname = "nav_sent.log";
  if (vm.count("sent_logname"))
       sent_logname = vm["sent_logname"].as<string>();
  
  VMCNavserverDriver::Params params;
  params.navParams.host = host;
  params.navParams.port = port;
  params.navParams.enc = vm.count("enc");
  params.navParams.laser = vm.count("laser");
  params.navParams.sick = vm.count("sick");
  params.navParams.state = vm.count("state");
  params.navParams.received_logname = received_logname;
  params.navParams.sent_logname = sent_logname;
  params.setStateAsInitOdom = true;
  params.debug = 0;
  params.combinedSteerAndDrive = true;
  params.useVmcTimestamp = true;
  params.steeringFixWheelDistance = 0.68;

  // Create the Driver object
  VMCNavserverDriver d (comm_nh, param_nh, params);
  if (!d.init ())
    return (-1);

  d.spin(comm_nh);
  return (0);
}
/* ]--- */
