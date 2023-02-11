#include "MultiModalCMKFParams.hpp"

#include <fstream>
#include <boost/program_options.hpp>
#include <iostream>

#include "utils/home_nao.hpp"

namespace po = boost::program_options;


MultiModalCMKFParams::MultiModalCMKFParams()
{

    readParams();
}

MultiModalCMKFParams::~MultiModalCMKFParams()
{}

void MultiModalCMKFParams::readParams()
{

    po::options_description parameters("Parameters");
        parameters.add_options()
            ("ModeSplitWeightMultiplyFactor", po::value<float>(&modeSplitWeightMultiplyFactor)->default_value(0.5))
            ("LineModeSplitWeightMultiplyFactor", po::value<float>(&lineModeSplitWeightMultiplyFactor)->default_value(0.2))            
            ("OdometryForwardMultiplyFactor", po::value<float>(&odometryForwardMultiplyFactor)->default_value(20))
            ("OdometryLeftMultiplyFactor", po::value<float>(&odometryLeftMultiplyFactor)->default_value(20))
            ("OdometryHeadingMultiplyFactor", po::value<float>(&odometryHeadingMultiplyFactor)->default_value(0.5))
            ("AngleUncertainty", po::value<float>(&angleUncertainty)->default_value(20))
            ("UpdateHeadingUncertainty", po::value<float>(&updateHeadingUncertainty)->default_value(20))
            ("SimilarHeadingThresh", po::value<float>(&similarHeadingThresh)->default_value(30))
            ("SimilarXThresh", po::value<float>(&similarXThresh)->default_value(500))
            ("SimilarYThresh", po::value<float>(&similarYThresh)->default_value(500))
            ("MinCMKFWeight", po::value<float>(&minCMKFWeight)->default_value(0.03))
            ;
    po::options_description config_file_options;
    config_file_options.add(parameters);

    po::variables_map vm;

    std::ifstream ifs(getHomeNao("data/MultiModalCMKFParams.cfg").c_str());
    if (!ifs.is_open())
    {
        std::cerr << __FILE__ <<  ": could not open param file" << std::endl;
    }
    
    store(parse_config_file(ifs, config_file_options), vm);
    notify(vm);
    ifs.close();
}
