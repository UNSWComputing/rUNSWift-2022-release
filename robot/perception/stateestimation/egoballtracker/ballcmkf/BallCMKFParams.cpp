#include "BallCMKFParams.hpp"

#include <fstream>
#include <boost/program_options.hpp>
#include <iostream>

#include "utils/home_nao.hpp"

namespace po = boost::program_options;


BallCMKFParams::BallCMKFParams()
{
    readParams();
}

BallCMKFParams::~BallCMKFParams()
{}

void BallCMKFParams::readParams()
{
    po::options_description parameters("Parameters");
        parameters.add_options()
            ("BallStdAccelerationX", po::value<float>(&ballStdAccelerationX)->default_value(70000))
            ("BallStdAccelerationY", po::value<float>(&ballStdAccelerationY)->default_value(70000))
            ("BallStdObservationDistBaseWalking", po::value<float>(&BallStdObservationDistBaseWalking)->default_value(20))
            ("BallStdObservationDistIncreaseRateWalking", po::value<float>(&BallStdObservationDistIncreaseRateWalking)->default_value(0.5))
            ("BallStdObservationHeadWalking", po::value<float>(&BallStdObservationHeadWalking)->default_value(20))
            ("BallStdObservationDistBaseStanding", po::value<float>(&BallStdObservationDistBaseStanding)->default_value(20))
            ("BallStdObservationDistIncreaseRateStanding", po::value<float>(&BallStdObservationDistIncreaseRateStanding)->default_value(0.3))
            ("BallStdObservationHeadStanding", po::value<float>(&BallStdObservationHeadStanding)->default_value(20))
            ;

    po::options_description config_file_options;
    config_file_options.add(parameters);

    po::variables_map vm;

    std::ifstream ifs(getHomeNao("data/BallCMKFParams.cfg").c_str());
    if (!ifs.is_open())
    {
        std::cerr << __FILE__ <<  ": could not open param file" << std::endl;
    }

    store(parse_config_file(ifs, config_file_options), vm);
    notify(vm);
    ifs.close();
}
