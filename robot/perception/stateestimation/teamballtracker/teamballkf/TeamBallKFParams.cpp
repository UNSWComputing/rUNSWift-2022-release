#include "TeamBallKFParams.hpp"

#include <fstream>
#include <boost/program_options.hpp>
#include <iostream>

#include "utils/home_nao.hpp"

namespace po = boost::program_options;


TeamBallKFParams::TeamBallKFParams()
{
    readParams();
}

TeamBallKFParams::~TeamBallKFParams()
{}

void TeamBallKFParams::readParams()
{
    po::options_description parameters("Parameters");
        parameters.add_options()
            ("BallStdAccelerationX", po::value<float>(&ballStdAccelerationX)->default_value(70000))
            ("BallStdAccelerationY", po::value<float>(&ballStdAccelerationY)->default_value(70000))
            ;

    po::options_description config_file_options;
    config_file_options.add(parameters);

    po::variables_map vm;

    std::ifstream ifs(getHomeNao("data/TeamBallKFParams.cfg").c_str());
    if (!ifs.is_open())
    {
        std::cerr << __FILE__ <<  ": could not open param file" << std::endl;
    }
    
    store(parse_config_file(ifs, config_file_options), vm);
    notify(vm);
    ifs.close();
}
