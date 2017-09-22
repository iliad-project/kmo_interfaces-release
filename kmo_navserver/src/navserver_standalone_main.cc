// This logs data directly from the navserver by simply listening to the socket.
#include <boost/program_options.hpp>
#include <kmo_navserver/kmo_navserver.hh>

namespace po = boost::program_options;

#include <iostream>
#include <iterator>
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
	       ("port", po::value<string>(), "port value (default 5432)")
	       ("enc", "collect encoder values")
	       ("laser", "collect laserway laser values")
	       ("sick", "collect sick data")
	       ("state", "collect state (pose) data")
	       ("received_logname", "received log filename")
	       ("sent_logname", "sent log filename");
	  
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

	  string received_logname = "nav_received.log";
	  if (vm.count("received_logname"))
	       received_logname = vm["received_logname"].as<string>();
	  string sent_logname = "nav_sent.log";
	  if (vm.count("sent_logname"))
	       sent_logname = vm["sent_logname"].as<string>();

	  VMCNavserver::Params nav_params;
	  nav_params.host = host;
	  nav_params.port = port;
	  nav_params.enc = vm.count("enc");
	  nav_params.laser = vm.count("laser");
	  nav_params.sick = vm.count("sick");
	  nav_params.state = vm.count("state");
	  nav_params.received_logname = received_logname;
	  nav_params.sent_logname = sent_logname;
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
