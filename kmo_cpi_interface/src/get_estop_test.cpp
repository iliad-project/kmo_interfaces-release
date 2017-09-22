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
	       ("host", po::value<string>(), "IP to host")
               ("port", po::value<string>(), "port value (default 5677)")
               ("stop", "listen to the 'Stop' message and not the 'EStop'")
              ;
	  po::variables_map vm;
	  po::store(po::parse_command_line(ac, av, desc), vm);
	  po::notify(vm);    
	  
	  if (vm.count("help")) {
	       cout << desc << "\n";
	       return 1;
	  }
	  
          bool estop = !vm.count("stop");
          string host = "192.168.100.100";
	  if (vm.count("host"))
	       host = vm["host"].as<string>();
	  int port = 5677;
	  if (vm.count("port"))
	       port = vm["port"].as<int>();

          CPISocket cpi(host, port);
          cpi.setDebugFlag();

          for (int i = 0; i < 10; i++) {
            if (estop) {
              std::cout << "Is E-Stop set? : " << cpi.getEStopFlag() << std::endl;
            }
            else {
              std::cout << "Is Stop set? : " << cpi.getStopFlag() << std::endl;
            }
            usleep(1000*1000);
          }
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

