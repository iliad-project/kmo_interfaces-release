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
         double forks_delay;
	  po::options_description desc("Allowed options");
	  desc.add_options()
	       ("help", "produce help message")
	       ("host", po::value<string>(), "IP to host")
               ("port", po::value<string>(), "port value (default 5677)")
              ("forks_delay", po::value<double>(&forks_delay)->default_value(2.5), "max time used to move the forks");
	       
	  po::variables_map vm;
	  po::store(po::parse_command_line(ac, av, desc), vm);
	  po::notify(vm);    
	  
	  if (vm.count("help")) {
	       cout << desc << "\n";
	       return 1;
	  }
	  string host = "192.168.100.100";
	  if (vm.count("host"))
	       host = vm["host"].as<string>();
	  int port = 5677;
	  if (vm.count("port"))
	       port = vm["port"].as<int>();

          CPISocket cpi(host, port);
          cpi.setDebugFlag();

          std::cout << "--> Lifting forks. " << std::endl;
          // This call will block until the forks are lifted.
          cpi.liftForksBlocking(forks_delay);

          std::cout << "--> Pausing (5 secs.)" << std::endl;
          usleep(5000*1000);
          
          std::cout << "--> Lowering the forks." << std::endl;
          // This call will block until the forks are lowered.
          cpi.lowerForksBlocking(forks_delay);
          
          std::cout << "--> Pausing (2 secs.)" << std::endl;
          usleep(2000*1000);
          std::cout << "Done. - quitting" << std::endl;

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

