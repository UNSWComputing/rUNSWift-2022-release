#include "perception/stateestimation/StateEstimationAdapter.hpp"
#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoMiddle.hpp"
#include "types/EstimatorInfoOut.hpp"
#include "types/AbsCoord.hpp"

#include <fstream>
#include <boost/archive/text_iarchive.hpp>

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Not the correct number of arguments, please provide in and out file as arguments\n");
        printf("%i arguments provided\n", argc-1);
        printf("(eg. state-estimation-simulator in.txt out.txt)\n");
        exit(1);
    }

    std::ifstream ifs(argv[1]);
    if (!ifs.is_open())
    {
        printf("Couldn't find input file provided in argument\n");
    }
    std::ofstream ofs(argv[2]);

    EstimatorInfoInit *estimatorInfoInit = new EstimatorInfoInit();

    try {
        boost::archive::text_iarchive ia(ifs);
        ia >> estimatorInfoInit;
    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }

    StateEstimationAdapter stateEstimationAdapter(estimatorInfoInit);

    EstimatorInfoIn *estimatorInfoIn = NULL;
    EstimatorInfoMiddle *estimatorInfoMiddle = NULL;
    EstimatorInfoOut *estimatorInfoOut = NULL;

    while (true)
    {
        int64_t timestamp;

        estimatorInfoMiddle = new EstimatorInfoMiddle();
        estimatorInfoOut = new EstimatorInfoOut();

        try {
            boost::archive::text_iarchive ia(ifs);
            ia >> estimatorInfoIn;
            ia >> timestamp;
        } catch (const std::exception &e) {
            std::cout << e.what() << std::endl;
            break;
        }

        stateEstimationAdapter.tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);

        AbsCoord robotPos = estimatorInfoOut->robotPos;
        std::cout << "(x,y,theta,timestamp): " << robotPos.x() << ", " << robotPos.y() << ", "  << robotPos.theta()<< ", " << timestamp << "\n";
        ofs << robotPos.x() << " " << robotPos.y() << " "  << robotPos.theta() << " " << timestamp << "\n";

        // Delete objects
        if (estimatorInfoIn)
            delete estimatorInfoIn;
        if (estimatorInfoMiddle)
            delete estimatorInfoMiddle;
        if (estimatorInfoOut)
            delete estimatorInfoOut;
    }

    // estimatorInfoInit, estimatorInfoIn, estimatorInfoMiddle and estimatorInfoOut
    // get deleted in StateEstimationAdapter, so no need to delete here. This is very
    // implicit and it's bad.

    ifs.close();
    ofs.close();
}
