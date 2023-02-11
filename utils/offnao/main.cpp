#include <QtCore/qglobal.h>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <QtGui/QApplication>
#include <iostream>
#include <fstream>
#include "visualiser.hpp"
#include "utils/options.hpp"
#include "utils/version.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace boost;

po::variables_map config;

int main(int argc, char *argv[]) {
   cerr << __FILE__ << ":" << __LINE__ << endl;
   /**
    * Read command line options. QT removes its recognized options
    * through the QApplication call. Currently only used for preloading
    * Vision files. If this gets too large, we may want to move this to
    * an Options.cpp helper file
    */
   po::options_description cmdline_options;
   po::options_description generic_options("Generic options");
   generic_options.add_options()
      ("help,h", "produce help message")
      ("version,v", "print version string")
      ("dump,d", po::value<string>(), "open dump.[ofn|yuv|bbd|ofn2|bbd2]")
      ("host,H", po::value<string>(), "connect to runswift hostname")
      ("port,p", po::value<uint16_t>()->default_value(10125), "connect to runswift port");


   po::options_description vision_options("Vision options");

   cmdline_options.add(generic_options);
   cmdline_options.add(vision_options);

   po::variables_map vm;
   po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
   po::notify(vm);

   if (vm.count("help")) {
      std::cout << cmdline_options << endl;
      return 0;
   }

   if (vm.count("version")) {
      std::cout << getVersion();
      return 0;
   }

   std::cout << getVersion() << endl;

   /** Load options from config file into global 'config' variable */
   po::options_description config_file_options;
   populate_options(config_file_options);
   ifstream ifs;
   string runswift_cfg_filename = getenv("RUNSWIFT_CHECKOUT_DIR");
   runswift_cfg_filename += "/utils/offnao/runswift.cfg";
   ifs.open(runswift_cfg_filename.c_str());
   store(parse_config_file(ifs, config_file_options), config);
   ifs.close();
   ifs.open("/etc/runswift/runswift.cfg");
   store(parse_config_file(ifs, config_file_options), config);
   ifs.close();
   po::notify(config);

   /** Start the QT application */
   QApplication a(argc, argv);
   Visualiser w;

   if(vm.count("host")) {
      w.connectToNao(vm["host"].as<string>().c_str(), vm["port"].as<uint16_t>());
   } else if (vm.count("dump")) {
      w.openFile(vm["dump"].as<string>().c_str());
   }

   w.show();

   cerr << "logpath is: " << config["debug.logpath"].as<string>() << endl;

   return a.exec();
}
