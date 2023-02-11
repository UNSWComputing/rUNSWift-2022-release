#ifndef BALL_CMKF_HPP
#define BALL_CMKF_HPP

#include <vector>
#include "BallCMKFConstants.hpp"
#include "types/AbsCoord.hpp"
#include "types/ActionCommand.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
struct Odometry;
class BallInfo;
class BallCMKFParams;
class SensorValues;


class BallCMKF
{
  public:
    explicit BallCMKF(const EstimatorInfoInit &estimatorInfoInit);
    ~BallCMKF();

    /* Function that gets called for every frame */
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

  private:

    void predictCollision(); // Predict any possible ball collisions with myself
    void predict(float dtInSeconds);
    void predictWithOdometry(const Odometry &odometry);
    void update(const BallInfo &ballInfo, const ActionCommand::Body &actionCommandBody, const SensorValues &sensorValues);

    AbsCoord getBallCoordRRC();
    AbsCoord getBallVelCoordRRC();

    BallStateVector state;
    BallCovarianceMatrix covariance;

    /* Reference to initialisation information */
    const EstimatorInfoInit &estimatorInfoInit;

    BallCMKFParams *params;

    /* Whether we have some ball updates we can send out, since last time we sent data out */
    bool haveOutgoingBallUpdate;
};

#endif // BALL_CMKF_HPP
