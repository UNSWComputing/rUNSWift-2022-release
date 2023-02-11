#ifndef TEAM_BALL_KF_PARAMS_HPP
#define TEAM_BALL_KF_PARAMS_HPP

class TeamBallKFParams {

    public:
        TeamBallKFParams();
        ~TeamBallKFParams();
        void readParams();

        float ballStdAccelerationY;
        float ballStdAccelerationX;
};

#endif // TEAM_BALL_KF_PARAMS_HPP
