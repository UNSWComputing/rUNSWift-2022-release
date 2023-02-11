#include "MultiModalCMKF.hpp"
#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoMiddle.hpp"
#include "types/EstimatorInfoOut.hpp"
#include "perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFTransitioner.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "perception/stateestimation/localiser/FieldFeatureLocations.hpp"
#include "perception/stateestimation/localiser/FieldFeature.hpp"
#include "MultiModalCMKFParams.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/eigen_helpers.hpp"

// #define DEBUG

MultiModalCMKF::MultiModalCMKF(const EstimatorInfoInit &estimatorInfoInit)
    : bestCMKF(NULL)
    , estimatorInfoInit(estimatorInfoInit)
{
    transitioner = new MultiModalCMKFTransitioner(estimatorInfoInit, this);
    fieldFeatureLocations = new FieldFeatureLocations();
    params = new MultiModalCMKFParams();
}

MultiModalCMKF::~MultiModalCMKF()
{
    if (fieldFeatureLocations) delete fieldFeatureLocations;
    if (params) delete params;
}

void MultiModalCMKF::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    // Handle Transition
    transitioner->handleTransition(estimatorInfoIn);

    if (estimatorInfoMiddle.canLocaliseInState)
    {
        // Create reference to fieldfeatures
        const std::vector<FieldFeatureInfo> &fieldFeatures = estimatorInfoIn.fieldFeatures;

        std::vector<CMKF> newCMKFs;
        for (unsigned i = 0; i < kfs.size(); ++i)
        {
            CMKF &kf = kfs[i];
            predict(kf, estimatorInfoIn.odometryDiff);
            if (estimatorInfoMiddle.canDoObservations)
            {
                for (unsigned j = 0; j < fieldFeatures.size(); ++j)
                {

                    const FieldFeatureInfo &observation = fieldFeatures[j];

                    // Sometimes we get nan in the observations, ignore these
                    if (!check_finite(observation.rr.vec, __FILE__ "\tobservation.rr.vec") ||
                        !check_finite(observation.rr.var, __FILE__ "\tobservation.rr.var"))
                    {
                        std::cerr << "observation:\t" << observation << "\n";
                        continue;
                    }

                    if (observation.type == FieldFeatureInfo::fCorner)
                    {
                        for (unsigned k = 0; k < fieldFeatureLocations->corners.size(); ++k)
                        {
                            CMKF newCMKF(kf.state, kf.covariance, kf.weight * params->modeSplitWeightMultiplyFactor);
                            update(newCMKF, observation, fieldFeatureLocations->corners[k]);
                            newCMKFs.push_back(newCMKF);
                        }
                    }
                    if (observation.type == FieldFeatureInfo::fTJunction)
                    {
                        for (unsigned k = 0; k < fieldFeatureLocations->t_junctions.size(); ++k)
                        {
                            CMKF newCMKF(kf.state, kf.covariance, kf.weight * params->modeSplitWeightMultiplyFactor);
                            update(newCMKF, observation, fieldFeatureLocations->t_junctions[k]);
                            newCMKFs.push_back(newCMKF);
                        }
                    }
                    if (observation.type == FieldFeatureInfo::fCentreCircle)
                    {
                        for (unsigned k = 0; k < fieldFeatureLocations->centre_circles.size(); ++k)
                        {
                            CMKF newCMKF(kf.state, kf.covariance, kf.weight * params->modeSplitWeightMultiplyFactor);
                            update(newCMKF, observation, fieldFeatureLocations->centre_circles[k]);
                            newCMKFs.push_back(newCMKF);
                        }
                    }
                    if (observation.type == FieldFeatureInfo::fLine)
                    {
                        // Case where the line is a constant x line
                        for (unsigned k = 0; k < fieldFeatureLocations->constantXLines.size(); ++k)
                        {
                            // Case where we're on positive side of line
                            CMKF newCMKF1(kf.state, kf.covariance, kf.weight * params->lineModeSplitWeightMultiplyFactor);
                            updateUsingConstantXLine(newCMKF1, observation, fieldFeatureLocations->constantXLines[k], true);
                            newCMKFs.push_back(newCMKF1);

                            // Case where we're on the negative side of line
                            CMKF newCMKF2(kf.state, kf.covariance, kf.weight * params->lineModeSplitWeightMultiplyFactor);
                            updateUsingConstantXLine(newCMKF2, observation, fieldFeatureLocations->constantXLines[k], false);
                            newCMKFs.push_back(newCMKF2);
                        }

                        // Case where the line is a constant y line
                        for (unsigned k = 0; k < fieldFeatureLocations->constantYLines.size(); ++k)
                        {
                            // Case where we're on positive side of line
                            CMKF newCMKF1(kf.state, kf.covariance, kf.weight * params->lineModeSplitWeightMultiplyFactor);
                            updateUsingConstantYLine(newCMKF1, observation, fieldFeatureLocations->constantYLines[k], true);
                            newCMKFs.push_back(newCMKF1);

                            // Case where we're on the negative side of line
                            CMKF newCMKF2(kf.state, kf.covariance, kf.weight * params->lineModeSplitWeightMultiplyFactor);
                            updateUsingConstantYLine(newCMKF2, observation, fieldFeatureLocations->constantYLines[k], false);
                            newCMKFs.push_back(newCMKF2);
                        }
                    }
                }
            }
        }
        kfs.insert(kfs.end(), newCMKFs.begin(), newCMKFs.end());

        if (fieldFeatures.size() > 0 && estimatorInfoMiddle.canDoObservations)
        {
            deleteOffFieldCMKFs();
            mergeCMKFs();
            normaliseCMKFWeights();
            deleteLowWeightCMKFs();
        }

    }

    // Determine best kalman filter
    determineBestCMKF();


    // Fill out estimatorInfoOut
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        estimatorInfoOut.allRobotPos.push_back(kfs[i].getRobotAbsCoord());
    }

    AbsCoord robotPos = bestCMKF->getRobotAbsCoord();
    estimatorInfoOut.robotPos = robotPos;
    estimatorInfoOut.robotPosUncertainty = bestCMKF->getRobotPosUncertainty();
    estimatorInfoOut.robotHeadingUncertainty = bestCMKF->getRobotHeadingUncertainty();

    estimatorInfoOut.sharedStateEstimationBundle.robotPos = robotPos;
}

void MultiModalCMKF::predict(CMKF &kf, const Odometry &odometry)
{
    float delta_x = odometry.forward * cosf(kf.state(ME_H_DIM, 0)) - odometry.left * sinf(kf.state(ME_H_DIM, 0));
    float delta_y = odometry.forward * sinf(kf.state(ME_H_DIM, 0)) + odometry.left * cosf(kf.state(ME_H_DIM, 0));

    // Update the robot pose estimate.
    kf.state(ME_H_DIM, 0) = normaliseTheta(kf.state(ME_H_DIM, 0) + odometry.turn);
    kf.state(ME_X_DIM, 0) += delta_x;
    kf.state(ME_Y_DIM, 0) += delta_y;
    check_finite(kf.state, "MMCMKF predict state");

    // Update the robot pose covariance.
    Eigen::Matrix<float, 2, 2> relativeCovariance = Eigen::Matrix<float, 2, 2>::Zero();
    relativeCovariance(0, 0) = odometry.forward * odometry.forward * params->odometryForwardMultiplyFactor;
    relativeCovariance(1, 1) = odometry.left * odometry.left * params->odometryLeftMultiplyFactor;

    Eigen::Matrix2f rotation;
    float heading = kf.state(ME_H_DIM, 0);
    rotation << cosf(heading), -sinf(heading), sinf(heading), cosf(heading);
    Eigen::Matrix2f rotationInv = rotation.transpose(); // Rotation matrix is orthogonal, and hence transpose=inverse
    Eigen::Matrix2f odometryCovariance = rotation * relativeCovariance * rotationInv;
    check_finite(odometryCovariance, "MMCMKF predict odom covariance");

    // Calculate uncertainty propagation thing (http://www.cs.cmu.edu/~rasc/Download/AMRobots5.pdf)
    Eigen::Matrix<float, 3, 3> propagationJacobian = Eigen::Matrix<float, 3, 3>::Identity();
    float distanceTravelled = sqrt(pow(odometry.forward, 2) + pow(odometry.left, 2));
    propagationJacobian(0, 2) = -distanceTravelled * sinf(kf.state(ME_H_DIM, 0));
    propagationJacobian(1, 2) = distanceTravelled * cosf(kf.state(ME_H_DIM, 0));
    kf.covariance.block<3, 3>(ME_X_DIM, ME_X_DIM)  = propagationJacobian * kf.covariance.block<3, 3>(ME_X_DIM, ME_X_DIM) * propagationJacobian.transpose();

    kf.covariance.block<2, 2>(ME_X_DIM, ME_X_DIM) += odometryCovariance;
    kf.covariance(ME_H_DIM, ME_H_DIM) += params->odometryHeadingMultiplyFactor * (odometry.turn * odometry.turn);
    check_finite(kf.covariance, "MMCMKF covariance");

    #ifdef DEBUG
        std::cout << "Relative Covariance\n" << relativeCovariance << std::endl;
    #endif
}


void MultiModalCMKF::updateUsingConstantXLine(CMKF &kf, const FieldFeatureInfo &observation, float x, bool onPositiveSideOfLine)
{
    // Take part of the kalman filter
    Eigen::Block<StateVector, NUM_DIM_ROBOT, 1> blockedState = kf.state.segment<NUM_DIM_ROBOT>(ME_X_DIM);
    Eigen::Block<CovarianceMatrix, NUM_DIM_ROBOT, NUM_DIM_ROBOT> blockedCovariance = kf.covariance.block<NUM_DIM_ROBOT, NUM_DIM_ROBOT>(ME_X_DIM, ME_X_DIM);

    #ifdef DEBUG
        std::cout << "State initial: \n" << blockedState << std::endl;                
    #endif

    // Calculate innovation vector
    Eigen::Matrix<float, NUM_DIM_ROBOT, 1> innovationVector = Eigen::Matrix<float, NUM_DIM_ROBOT, 1>::Zero();

    // Calculate innovation covariance
    float obs_dist = observation.rr.distance();
    float myXFromObservation = x + (onPositiveSideOfLine ? obs_dist : -obs_dist);
    innovationVector(ME_X_DIM, 0) = myXFromObservation - blockedState(ME_X_DIM, 0);

    float obs_heading = observation.rr.heading();
    float myHFromObservation;
    if (onPositiveSideOfLine)
        myHFromObservation = normaliseTheta(DEG2RAD(180) - obs_heading);
    else
        myHFromObservation = -obs_heading;

    innovationVector(ME_H_DIM, 0) = normaliseTheta(myHFromObservation - blockedState(ME_H_DIM, 0));

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> observationCovariance = Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT>::Zero();

    double bigNumber = 1000000;
    observationCovariance(ME_X_DIM, ME_X_DIM) = observation.rr.var(0,0);
    observationCovariance(ME_Y_DIM, ME_Y_DIM) = bigNumber * bigNumber;
    observationCovariance(ME_H_DIM, ME_H_DIM) = observation.rr.var(1,1);

    check_finite(observationCovariance, "MMCMKF observation covariance");

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> innovationCovariance =
        observationCovariance + blockedCovariance;
    check_finite(innovationCovariance, "MMCMKF update innovation covariance");

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> innovationCovarianceInv =
        innovationCovariance.inverse();
    bool is_finite = check_finite(innovationCovarianceInv, "MMCMKF update innovation covariance inverse");

    // For now, we just ignore the update if this has NaN or +/- inf
    if (is_finite) {
        // Calculate optimal Kalman gain
        Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> kalmanGain =
            blockedCovariance * innovationCovarianceInv;
        check_finite(kalmanGain, "MMCMKF update kalman gain");

        // Update state vector
        blockedState += kalmanGain * innovationVector;

        // Update covariance matrix
        Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> identity = Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT>::Identity();
        blockedCovariance = (identity - kalmanGain) * blockedCovariance * (identity - kalmanGain).transpose() + kalmanGain * observationCovariance * kalmanGain.transpose();
        check_finite(blockedCovariance, "MMCMKF update blocked covariance");

        // Adjust weight
        float weightAdjustment = 1.0;
        float weightUpdate = innovationVector.transpose() * innovationCovarianceInv * innovationVector;
        weightAdjustment = exp(-0.5 * weightUpdate);
        weightAdjustment = std::max(std::min(weightAdjustment, 1.0f), 0.01f);
        kf.weight *= weightAdjustment;

        #ifdef DEBUG
            std::cout << "Innovation Vector\n" << innovationVector << std::endl;
            std::cout << "Innovation Covariance\n" << innovationCovariance << std::endl;
            std::cout << "Innovation Covariance Inverse\n" << innovationCovarianceInv << std::endl;
            std::cout << "Observation covariance x: \n" << observationCovariance << std::endl;
            std::cout << "Kalman gain\n" << kalmanGain << std::endl;
            std::cout << "myheadingfromobservationY: " << myHFromObservation<<std::endl;
            std::cout << "Observation distX: " << obs_dist << std::endl;
            std::cout << "Observation headingX: " << obs_heading << std::endl;
            std::cout << "Covariance\n" << blockedCovariance << std::endl;
            std::cout << "State End: \n" << blockedState << std::endl;        
        #endif
    }
}

void MultiModalCMKF::updateUsingConstantYLine(CMKF &kf, const FieldFeatureInfo &observation, float y, bool onPositiveSideOfLine)
{   
   // Take part of the kalman filter
    Eigen::Block<StateVector, NUM_DIM_ROBOT, 1> blockedState = kf.state.segment<NUM_DIM_ROBOT>(ME_X_DIM);
    Eigen::Block<CovarianceMatrix, NUM_DIM_ROBOT, NUM_DIM_ROBOT> blockedCovariance = kf.covariance.block<NUM_DIM_ROBOT, NUM_DIM_ROBOT>(ME_X_DIM, ME_X_DIM);
    #ifdef DEBUG
        std::cout << "state \n" << blockedState << std::endl;
    #endif 
    // Calculate innovation vector
    Eigen::Matrix<float, NUM_DIM_ROBOT, 1> innovationVector = Eigen::Matrix<float, NUM_DIM_ROBOT, 1>::Zero();

    // Calculate innovation covariance
    float obs_dist = observation.rr.distance();
    float myYFromObservation = y + (onPositiveSideOfLine ? obs_dist : -obs_dist);
    innovationVector(ME_Y_DIM, 0) = myYFromObservation - blockedState(ME_Y_DIM, 0);

    float obs_heading = observation.rr.heading();
    float myHFromObservation;
    if (onPositiveSideOfLine)
        myHFromObservation = normaliseTheta(-DEG2RAD(90) - obs_heading);
    else
        myHFromObservation = normaliseTheta(DEG2RAD(90) - obs_heading);

    innovationVector(ME_H_DIM, 0) = normaliseTheta(myHFromObservation - blockedState(ME_H_DIM, 0));

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> observationCovariance = Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT>::Zero();

    double bigNumber = 1000000;
    observationCovariance(ME_X_DIM, ME_X_DIM) = bigNumber * bigNumber;
    observationCovariance(ME_Y_DIM, ME_Y_DIM) = observation.rr.var(0,0);
    observationCovariance(ME_H_DIM, ME_H_DIM) =observation.rr.var(1,1);

    check_finite(observationCovariance, "MMCMKF observation covariance");

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> innovationCovariance =
        observationCovariance + blockedCovariance;
    check_finite(innovationCovariance, "MMCMKF update innovation covariance");

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> innovationCovarianceInv =
        innovationCovariance.inverse();
    bool is_finite = check_finite(innovationCovarianceInv, "MMCMKF update innovation covariance inverse");

    // For now, we just ignore the update if this has NaN or +/- inf
    if (is_finite) {
        // Calculate optimal Kalman gain
        Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> kalmanGain =
            blockedCovariance * innovationCovarianceInv;
        check_finite(kalmanGain, "MMCMKF update kalman gain");

        // Update state vector
        blockedState += kalmanGain * innovationVector;

        // Update covariance matrix
        Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> identity = Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT>::Identity();
        blockedCovariance = (identity - kalmanGain) * blockedCovariance * (identity - kalmanGain).transpose() + kalmanGain * observationCovariance * kalmanGain.transpose();
        check_finite(blockedCovariance, "MMCMKF update blocked covariance");

        // Adjust weight
        float weightAdjustment = 1.0;
        float weightUpdate = innovationVector.transpose() * innovationCovarianceInv * innovationVector;
        weightAdjustment = exp(-0.5 * weightUpdate);
        weightAdjustment = std::max(std::min(weightAdjustment, 1.0f), 0.01f);
        kf.weight *= weightAdjustment;

        #ifdef DEBUG
            std::cout << "Observation covariance: \n" << observationCovariance << std::endl;
            std::cout << "myheadingfromobservationY: " << myHFromObservation;
            std::cout << "Innovation Vector\n" << innovationVector << std::endl;
            std::cout << "Innovation Covariance\n" << innovationCovariance << std::endl;
            std::cout << "Innovation Covariance Inverse\n" << innovationCovarianceInv << std::endl;
            std::cout << "Observation distY: " << obs_dist << std::endl;
            std::cout << "Observation headingY: " << obs_heading << std::endl;
            std::cout << "Kalman gain\n" << kalmanGain << std::endl;
            std::cout << "Covariance\n" << blockedCovariance << std::endl;
            std::cout << "State End: \n" << blockedState << std::endl;
        #endif
    }
}


void MultiModalCMKF::update(
    CMKF &kf,
    const FieldFeatureInfo &observation,
    FieldFeature &ff)
{
    // Take part of the kalman filter
    Eigen::Block<StateVector, NUM_DIM_ROBOT, 1> blockedState = kf.state.segment<NUM_DIM_ROBOT>(ME_X_DIM);
    Eigen::Block<CovarianceMatrix, NUM_DIM_ROBOT, NUM_DIM_ROBOT> blockedCovariance = kf.covariance.block<NUM_DIM_ROBOT, NUM_DIM_ROBOT>(ME_X_DIM, ME_X_DIM);

    // Calculate innovation vector
    Eigen::Matrix<float, NUM_DIM_ROBOT, 1> innovationVector = Eigen::Matrix<float, NUM_DIM_ROBOT, 1>::Zero();

    float x = ff.x + observation.rr.distance() * cosf(ff.orientation + observation.rr.orientation());
    float y = ff.y + observation.rr.distance() * sinf(ff.orientation + observation.rr.orientation());
    float theta = normaliseTheta(atan2f(ff.y - y, ff.x - x) - observation.rr.heading());

    innovationVector(ME_X_DIM, 0) = x - kf.state(ME_X_DIM, 0);
    innovationVector(ME_Y_DIM, 0) = y - kf.state(ME_Y_DIM, 0);
    innovationVector(ME_H_DIM, 0) = theta - kf.state(ME_H_DIM, 0);

    // Calculate innovation covariance
    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> observationCovariance = Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT>::Zero();

    // varience is distance is somewhat dependant on distance from object
    // TODO do this better/ with actual maths not just testing
    double distUncertainty = observation.rr.distance()/4.0;  // mm

    double angleUncertainty = DEG2RAD(params->angleUncertainty); // rad
    // This is the heading uncertainy AND the observation angle uncertainty
    double headingUncertainty = DEG2RAD(params->updateHeadingUncertainty);

    // Convert from polar to cartesian covariance matrix as per
    // http://resource.npl.co.uk/docs/networks/anamet/members_only/meetings/30/hall.pdf
    // Heading is treated as independant from observation distance and angle

    double Ut = observation.rr.distance() * tanf(angleUncertainty);
    double Ur = distUncertainty;

    double a = Ur * Ur;
    double b = Ut * Ut;
    double c = a - b;

    observationCovariance(0, 0) = pow(cos(theta), 2) * a + pow(sinf(theta), 2) * b;
    observationCovariance(0, 1) = 0.5 * sinf(2 * theta) * c;
    observationCovariance(1, 0) = 0.5 * sinf(2 * theta) * c;
    observationCovariance(1, 1) = pow(cos(theta), 2) * b + pow(sinf(theta), 2) * a;
    observationCovariance(2, 2) = headingUncertainty * headingUncertainty; //heading uncertainty hard coded
    check_finite(observationCovariance, "MMCMKF observation covariance");

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> innovationCovariance =
        observationCovariance + blockedCovariance;
    check_finite(innovationCovariance, "MMCMKF update innovation covariance");

    Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> innovationCovarianceInv =
        innovationCovariance.inverse();
    bool is_finite = check_finite(innovationCovarianceInv, "MMCMKF update innovation covariance inverse");

    // For now, we just ignore the update if this has NaN or +/- inf
    if (is_finite) {
        // Calculate optimal Kalman gain
        Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> kalmanGain =
            blockedCovariance * innovationCovarianceInv;
        check_finite(kalmanGain, "MMCMKF update kalman gain");

        // Update state vector
        blockedState += kalmanGain * innovationVector;

        // Update covariance matrix
        Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT> identity = Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT>::Identity();
        blockedCovariance = (identity - kalmanGain) * blockedCovariance * (identity - kalmanGain).transpose() + kalmanGain * observationCovariance * kalmanGain.transpose();
        check_finite(blockedCovariance, "MMCMKF update blocked covariance");

        // Adjust weight
        float weightAdjustment = 1.0;
        float weightUpdate = innovationVector.transpose() * innovationCovarianceInv * innovationVector;
        weightAdjustment = exp(-0.5 * weightUpdate);
        weightAdjustment = std::max(std::min(weightAdjustment, 1.0f), 0.01f);
        kf.weight *= weightAdjustment;

        #ifdef DEBUG
            std::cout << "Observation covariance: \n" << observationCovariance << std::endl;
            std::cout << "Innovation Vector\n" << innovationVector << std::endl;
            std::cout << "Innovation Covariance\n" << innovationCovariance << std::endl;
            std::cout << "Innovation Covariance Inverse\n" << innovationCovarianceInv << std::endl;
            std::cout << "Kalman gain\n" << kalmanGain << std::endl;
            std::cout << "Covariance\n" << blockedCovariance << std::endl;
            std::cout << "State End: \n" << blockedState << std::endl;
        #endif
    }
}

bool noWeight(CMKF &kf)
{
    return kf.weight < 0;
}

void MultiModalCMKF::mergeCMKFs()
{
    for (std::vector<CMKF>::iterator belief = kfs.begin(); belief != kfs.end(); ++belief)
    {
        if (belief->weight > 0)
        {
            for (std::vector<CMKF>::iterator belief2 = belief + 1; belief2 != kfs.end(); ++belief2)
            {
                if (belief2->weight > 0)
                {
                    if (similar((*belief), (*belief2)))
                    {
                        merge((*belief), (*belief2));
                    }
                }
            }
        }
    }

    kfs.erase(std::remove_if(kfs.begin(), kfs.end(), noWeight), kfs.end());
}

bool MultiModalCMKF::similar(CMKF &kf1, CMKF &kf2)
{
    // For now, just compare states.
    // TODO: This should probably be extended to check covariances too
    float xDiff = kf1.state(ME_X_DIM, 0) - kf2.state(ME_X_DIM, 0);
    float yDiff = kf1.state(ME_Y_DIM, 0) - kf2.state(ME_Y_DIM, 0);
    float hDiff = MIN_THETA_DIFF(kf1.state(ME_H_DIM, 0), kf2.state(ME_H_DIM, 0));

    if (fabs(hDiff) < DEG2RAD(params->similarHeadingThresh) &&
        fabs(xDiff) < params->similarXThresh && fabs(yDiff) < params->similarYThresh)
    {
        return true;
    }

    return false;
}

void MultiModalCMKF::merge(CMKF &kf1, CMKF &kf2)
{
    /*
     * We take a weight average if the weights are relatively similar to each other.
     * We don't take the weighted average if one weight is significanlty greater than
     * the other, since it causes drift (the drift is explained in the following paper)
     * https://www.cs.utexas.edu/~pstone/Courses/393Rfall11/resources/RC09-Quinlan.pdf
     */
    float sumWeights = kf1.weight + kf2.weight;

    if (kf1.weight > 10.0 * kf2.weight)
    {
        kf1.weight = sumWeights;
        kf2.weight = -1;
    }
    else if (kf2.weight > 10.0 * kf1.weight)
    {
        kf1.state = kf2.state;
        kf1.covariance = kf2.covariance;
        kf1.weight = sumWeights;
        kf2.weight = -1;
    }
    else
    {
        float kf1Ratio = kf1.weight / sumWeights;
        float kf2Ratio = kf2.weight / sumWeights;

        // Weighted sum of angles is a bit special (https://stackoverflow.com/questions/1686994/weighted-average-of-angles)
        float alphaSinSum = kf1Ratio * sinf(kf1.state(ME_H_DIM, 0)) + kf2Ratio * sinf(kf2.state(ME_H_DIM, 0));
        float alphaCosSum = kf1Ratio * cosf(kf1.state(ME_H_DIM, 0)) + kf2Ratio * cosf(kf2.state(ME_H_DIM, 0));
        float weightedAvgH = atan2f(alphaSinSum, alphaCosSum);

        kf1.state = kf1Ratio * kf1.state + kf2Ratio * kf2.state;
        kf1.state(ME_H_DIM, 0) = weightedAvgH;

        kf1.covariance = kf1Ratio * kf1.covariance + kf2Ratio * kf2.covariance;

        kf1.weight = sumWeights;
        kf2.weight = -1;
    }
}

void MultiModalCMKF::normaliseCMKFWeights()
{
    float weightSum = 0.0;
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        weightSum += kfs[i].weight;
    }

    float scale = 1.0 / weightSum;
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        kfs[i].weight *= scale;
    }
}

void MultiModalCMKF::deleteLowWeightCMKFs()
{
    std::vector<CMKF> newCMKFs;
    for (unsigned i = 0; i < kfs.size(); ++i)
    {
        if (kfs[i].weight > params->minCMKFWeight)
            newCMKFs.push_back(kfs[i]);
    }

    kfs = newCMKFs;
}

void MultiModalCMKF::deleteOffFieldCMKFs()
{
    float fieldXClip = FIELD_LENGTH/2 + 700;
    float fieldYClip = FIELD_WIDTH/2 + 700;

    std::vector<CMKF> newCMKFs;
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        CMKF &kf = kfs[i];
        if (fabs(kf.state(ME_X_DIM, 0)) > fieldXClip)
        {
            continue;
        }
        if (fabs(kf.state(ME_Y_DIM, 0)) > fieldYClip)
        {
            continue;
        }
        newCMKFs.push_back(kfs[i]);
    }
    kfs = newCMKFs;
}

void MultiModalCMKF::determineBestCMKF()
{
    float maxWeight = 0.0;
    for (unsigned i = 0; i < kfs.size(); ++i)
    {
        if (kfs[i].weight > maxWeight)
        {
            bestCMKF = &kfs[i];
            maxWeight = kfs[i].weight;
        }
    }
}
