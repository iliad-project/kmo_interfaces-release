//! Small tool to set the state using the EPU. Quite useful if you running the vehicles 'in the air' and want to reset the pose estimates.
#include <boost/program_options.hpp>
#include <kmo_navserver/kmo_navserver.hh>

namespace po = boost::program_options;

#include <iostream>
#include <iterator>
using namespace std;


template<class T> std::string _toString_ (const T& x)
{
     std::ostringstream o;

     if (!(o << x))
	  throw std::runtime_error ("::toString()");

     return o.str ();
}

int main(int ac, char* av[])
{
     try
     {
// Declare the supported options.
	  po::options_description desc("Allowed options");
          double pos_x,pos_y,pos_th, timestamp;
	  desc.add_options()
	      ("help", "produce help message")
	      ("host", po::value<string>(), "IP to host")
	      ("port", po::value<string>(), "port value (default 5432)")
              ("pos_x", po::value<double>(&pos_x)->default_value(0.), "x pos (meters)")
              ("pos_y", po::value<double>(&pos_y)->default_value(0.), "y pos (meters)")
              ("pos_th", po::value<double>(&pos_th)->default_value(0.), "th pos (rads)")
              ("timestamp", po::value<double>(&timestamp)->default_value(0.), "use time stamp")
              ;
	  
	  po::variables_map vm;
	  po::store(po::parse_command_line(ac, av, desc), vm);
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

	  VMCNavserver::Params nav_params;
	  nav_params.host = host;
	  nav_params.port = port;
	  nav_params.state = true;
	  cout << "Params used : " << nav_params << endl;

	  VMCNavserver nav(nav_params);

	  if (!nav.connect())
	  {
	       cerr << "Failed to connect to host : " << nav_params.host << " (port: " << nav_params.port << "].\n";
	       return -1;
	  }
	  else
	  {
	       cout << "Connected to : " << nav_params.host << " (port: " << nav_params.port << "].\n";
	  }
	  bool stop = false;

          // Set the state (here just use the epu...)
          nav.send("nav epu");
          // Confidence value if 0 -> no confidence, 100 -> max confidence.
          // Will decrease by 1 every 60 ms when no data has arrived.
          int conf = 100;
          
          float cxx = 0.f;
          float cyy = 0.f;
          float cpp = 0.f;
          
          float rxy = 0.f;
          float rxp = 0.f;
          float ryp = 0.f;
          
          std::string cmd = "epu " + _toString_(timestamp) + " " + _toString_(conf) + " " + _toString_(pos_x) + " " + _toString_(pos_y) + " " + _toString_(pos_th) + " " + _toString_(cxx) + " " + _toString_(cyy) + " " + _toString_(cpp) + " " + _toString_(rxy) + " " + _toString_(rxp) + " " + _toString_(ryp);
          nav.send(cmd);
         
          
	  while (!stop)
	  {
	       std::string data;
	       nav.receive(data);
	       usleep(1000);
	  }
	  cout << "Done." << endl;
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
