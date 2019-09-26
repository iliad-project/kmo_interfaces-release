#include <boost/program_options.hpp>
#include <iostream>
#include <iterator>
#include <kmo_cpi_interface/CPISocket.h>

namespace po = boost::program_options;
using namespace std;


int main(int ac, char* av[])
{
     try
     {
// Declare the supported options.
	  po::options_description desc("Allowed options");
	  desc.add_options()
	       ("help", "produce help message")
         ("debug", "print additional CPI debug communication info")
         ("host", po::value<string>(), "IP to host (default 192.168.100.100)")
               ("port", po::value<string>(), "port value (default 5677)")
              ;
	  po::variables_map vm;
	  po::store(po::parse_command_line(ac, av, desc), vm);
	  po::notify(vm);    
	  
	  if (vm.count("help")) {
	       cout << desc << "\n";
	       return 1;
	  }
	  
    bool debug = vm.count("debug");

    string host = "192.168.100.100";
	  if (vm.count("host"))
	       host = vm["host"].as<string>();
	  int port = 5677;
	  if (vm.count("port"))
	       port = vm["port"].as<int>();

          CPISocket cpi(host, port);
          if (debug) {
            cpi.setDebugFlag();
          }
          // These variables are of interest
          // Right / Left is refering to the forward direction of the vehicle -> when the steering wheel goes first.
          //SDIO_2­_1.RightForkSensor_i     - object in front of right fork at close proximity of 0 <-> ~350 mm
          //SDIO_2­_1.LeftForkSensor_i      - object in front of left fork at close proximity of 0 <-> ~350 mm
          //SDIO_2­_1.CentreForkSensor_i    - if there is an object at the center (between the fork), if both left and right are 0 then we're safe to drive.
          //SDIO_2_2.AuxEnc1Position       - the encoder value -> the height of the forks

          for (int i = 0; i < 100; i++) {
          std::cout << "cpi.getBTForkLeft()   : " << cpi.getBTForkLeft() << std::endl;
          std::cout << "cpi.getBTForkCenter() : " << cpi.getBTForkCenter() << std::endl;
          std::cout << "cpi.getBTForkRight()  : " << cpi.getBTForkRight() << std::endl;
          std::cout << "cpi.getBTForkHeight() : " << cpi.getBTForkHeight() << std::endl;
          usleep(100*1000);
          }
          // TODO - the CPI needs to be fixed to handle multiple CPI calls at once. Currently there is only
          // one 'get' call and one response.

     }
     catch (exception& e)
     {
	  cerr << "error: " << e.what() << "\n";
	  return 1;
     }
     catch(...) 
     {
	  cerr << "Exception of unknown type!\n";
     }
}

