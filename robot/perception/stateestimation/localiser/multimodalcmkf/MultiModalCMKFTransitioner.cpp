#include "MultiModalCMKFTransitioner.hpp"

#include "types/EstimatorInfoInit.hpp"
#include "MultiModalCMKF.hpp"
// #include "../LocalisationDefs.hpp"
// #include "../LocalisationUtils.hpp"
#include <Eigen/Eigen>
#include "utils/TransitionPoses.hpp"

MultiModalCMKFTransitioner::MultiModalCMKFTransitioner(
    const EstimatorInfoInit &estimatorInfoInit,
    MultiModalCMKF *mmcmkf)
    : LocaliserTransitioner(estimatorInfoInit)
    , mmcmkf(mmcmkf)
{
    LocaliserTransitioner::resetToInitialPose();
}

void MultiModalCMKFTransitioner::resetToGameInitialPose()
{
    StateVector state = StateVector::Zero();

    switch(estimatorInfoInit.playerNumber)
    {
    case 1:
        state(ME_X_DIM, 0) = INITIAL_POSE_P1_X;
        state(ME_Y_DIM, 0) = INITIAL_POSE_P1_Y;
        state(ME_H_DIM, 0) = INITIAL_POSE_P1_THETA;
        break;

    case 2:
        state(ME_X_DIM, 0) = INITIAL_POSE_P2_X;
        state(ME_Y_DIM, 0) = INITIAL_POSE_P2_Y;
        state(ME_H_DIM, 0) = INITIAL_POSE_P2_THETA;
        break;

    case 3:
        state(ME_X_DIM, 0) = INITIAL_POSE_P3_X;
        state(ME_Y_DIM, 0) = INITIAL_POSE_P3_Y;
        state(ME_H_DIM, 0) = INITIAL_POSE_P3_THETA;
        break;

    case 4:
        state(ME_X_DIM, 0) = INITIAL_POSE_P4_X;
        state(ME_Y_DIM, 0) = INITIAL_POSE_P4_Y;
        state(ME_H_DIM, 0) = INITIAL_POSE_P4_THETA;
        break;

    case 5:
        state(ME_X_DIM, 0) = INITIAL_POSE_P5_X;
        state(ME_Y_DIM, 0) = INITIAL_POSE_P5_Y;
        state(ME_H_DIM, 0) = INITIAL_POSE_P5_THETA;
        break;

    case 6:
        state(ME_X_DIM, 0) = INITIAL_POSE_P6_X;
        state(ME_Y_DIM, 0) = INITIAL_POSE_P6_Y;
        state(ME_H_DIM, 0) = INITIAL_POSE_P6_THETA;
        break;

    default:
        state(ME_X_DIM, 0) = INITIAL_POSE_DEFAULT_X;
        state(ME_Y_DIM, 0) = INITIAL_POSE_DEFAULT_Y;
        state(ME_H_DIM, 0) = INITIAL_POSE_DEFAULT_THETA;
        break;
    }

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    mmcmkf->kfs.clear();
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
}

void MultiModalCMKFTransitioner::resetToOneVsOneInitialPose()
{
    mmcmkf->kfs.clear();
    StateVector state = StateVector::Zero();

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    state(ME_X_DIM, 0) = -3500;
    state(ME_Y_DIM, 0) = 3000;
    state(ME_H_DIM, 0) = -M_PI / 2;
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

    state(ME_X_DIM, 0) = -3500;
    state(ME_Y_DIM, 0) = -3000;
    state(ME_H_DIM, 0) = M_PI / 2;
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
}

void MultiModalCMKFTransitioner::resetToSpecifiedInitialPose()
{
    StateVector state = StateVector::Zero();

    state(ME_X_DIM, 0) = estimatorInfoInit.specifiedInitialPose.x();
    state(ME_Y_DIM, 0) = estimatorInfoInit.specifiedInitialPose.y();
    state(ME_H_DIM, 0) = estimatorInfoInit.specifiedInitialPose.theta();

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    mmcmkf->kfs.clear();
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
}

void MultiModalCMKFTransitioner::resetToUnpenalisedPose()
{
    mmcmkf->kfs.clear();
    StateVector state = StateVector::Zero();

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    state(ME_X_DIM, 0) = UNPENALISED_H1_X;
    state(ME_Y_DIM, 0) = UNPENALISED_H1_Y;
    state(ME_H_DIM, 0) = UNPENALISED_H1_THETA;
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

    state(ME_X_DIM, 0) = UNPENALISED_H2_X;
    state(ME_Y_DIM, 0) = UNPENALISED_H2_Y;
    state(ME_H_DIM, 0) = UNPENALISED_H2_THETA;
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

    state(ME_X_DIM, 0) = UNPENALISED_H3_X;
    state(ME_Y_DIM, 0) = UNPENALISED_H3_Y;
    state(ME_H_DIM, 0) = UNPENALISED_H3_THETA;
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

    state(ME_X_DIM, 0) = UNPENALISED_H4_X;
    state(ME_Y_DIM, 0) = UNPENALISED_H4_Y;
    state(ME_H_DIM, 0) = UNPENALISED_H4_THETA;
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
}

void MultiModalCMKFTransitioner::resetToManualPlacementPoseOffense()
{
    StateVector state = StateVector::Zero();

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    mmcmkf->kfs.clear();

    if (estimatorInfoInit.playerNumber == 1)
    {
        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_GOALIE_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_GOALIE_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_GOALIE_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
    }
    else
    {
        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H1_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H1_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H1_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H2_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H2_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H2_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H3_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H3_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H3_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H4_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H4_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H4_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_KICKOFF_PLAYER_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_KICKOFF_PLAYER_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_KICKOFF_PLAYER_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
    }
}

void MultiModalCMKFTransitioner::resetToManualPlacementPoseDefense()
{
    StateVector state = StateVector::Zero();

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    mmcmkf->kfs.clear();


    if (estimatorInfoInit.playerNumber == 1)
    {
        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_GOALIE_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_GOALIE_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_GOALIE_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
    }
    else
    {
        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H1_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H1_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H1_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H2_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H2_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H2_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H3_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H3_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H3_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));

        state(ME_X_DIM, 0) = MANUAL_PLACEMENT_H4_X;
        state(ME_Y_DIM, 0) = MANUAL_PLACEMENT_H4_Y;
        state(ME_H_DIM, 0) = MANUAL_PLACEMENT_H4_THETA;
        mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
    }
}

void MultiModalCMKFTransitioner::resetToPenaltyshootPhasePoseOffense()
{
    StateVector state = StateVector::Zero();
    state(ME_X_DIM, 0) = PENALTY_SHOOT_OFFENSE_POSE_X;
    state(ME_Y_DIM, 0) = PENALTY_SHOOT_OFFENSE_POSE_Y;
    state(ME_H_DIM, 0) = PENALTY_SHOOT_OFFENSE_POSE_THETA;

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    mmcmkf->kfs.clear();
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
}

void MultiModalCMKFTransitioner::resetToPenaltyshootPhasePoseDefense()
{
    StateVector state = StateVector::Zero();
    state(ME_X_DIM, 0) = PENALTY_SHOOT_DEFENSE_POSE_X;
    state(ME_Y_DIM, 0) = PENALTY_SHOOT_DEFENSE_POSE_Y;
    state(ME_H_DIM, 0) = PENALTY_SHOOT_DEFENSE_POSE_THETA;

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    mmcmkf->kfs.clear();
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
}

void MultiModalCMKFTransitioner::resetToPenaltyshootPhasePlayerSelectedPose()
{
    StateVector state = StateVector::Zero();
    state(ME_X_DIM, 0) = PENALTY_SHOOT_SELECTED_POSE_X;
    state(ME_Y_DIM, 0) = PENALTY_SHOOT_SELECTED_POSE_Y;
    state(ME_H_DIM, 0) = PENALTY_SHOOT_SELECTED_POSE_THETA;

    CovarianceMatrix covariance = CovarianceMatrix::Zero();
    covariance(ME_X_DIM, ME_X_DIM) = 100000;
    covariance(ME_Y_DIM, ME_Y_DIM) = 100000;
    covariance(ME_H_DIM, ME_H_DIM) = 0.5;

    mmcmkf->kfs.clear();
    mmcmkf->kfs.push_back(CMKF(state, covariance, 1.0));
}

