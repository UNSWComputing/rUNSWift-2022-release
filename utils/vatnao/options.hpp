#include <string>
#ifndef Q_MOC_RUN
#include <boost/program_options.hpp>
#endif


namespace po = boost::program_options;

po::variables_map parseOptions(int argc, char *argv[]);
std::string getHelp(po::options_description desc);
