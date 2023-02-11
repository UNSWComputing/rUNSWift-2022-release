#include <algorithm>
#include <fstream>
#include <iostream>
#include <jpeglib.h>

#include "blackboard/Blackboard.hpp"
#include "Blackboard.pb.h"

using namespace std;

extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP;
extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT;

// not sure why i need to forward declare these, since they exist when the templated callers are instantiated
static void serialise(const RobotObstacle &, offnao::StateEstimation_RobotObstacle &);

static void serialise(const AbsCoord &, offnao::AbsCoord &);

static void serialise(const BehaviourRequest &, offnao::Behaviour_BehaviourRequest &);

static void serialise(const BallInfo &, offnao::Vision_BallInfo &);

static void serialise(const RobotVisionInfo &, offnao::Vision_RobotVisionInfo &);

static void serialise(const FieldBoundaryInfo &, offnao::Vision_FieldBoundaryInfo &);

static void serialise(const FieldFeatureInfo &, offnao::Vision_FieldFeatureInfo &);

static void serialise(const SPLStandardMessage &, offnao::Receiver_SPLStandardMessage &);

static void serialise(const BroadcastData &, offnao::Receiver_BroadcastData &);

static void serialise(const BehaviourDebugInfo &, offnao::BehaviourDebugInfo &);

static void serialise(const BehaviourSharedData &, offnao::BehaviourSharedData &);

//
static void serialise(const string &cpp, string &pb) {
   pb = cpp;
}

template<unsigned int size>
static void serialise(const char(&array)[size], string &pb) {
   serialise(string(array, size), pb);
}

template<typename T, typename Container>
static void serialise(const Container &container, ::google::protobuf::RepeatedField<T> &repeatedField, int size) {
   repeatedField.Reserve(size);
   for (int i = 0; i < size; ++i) {
      repeatedField.Add(container[i]);
   }
}

template<typename T, int size>
static void serialise(const T(&array)[size], ::google::protobuf::RepeatedField<T> &repeatedField) {
   serialise(array, repeatedField, size);
}

// different name so it's not used accidentally
template<typename T, typename U, int size>
static void serialiseWithImplicitCast(const T(&array)[size], ::google::protobuf::RepeatedField<U> &repeatedField) {
   serialise(array, repeatedField, size);
}

template<typename T>
static void serialise(const vector<T> &vector, ::google::protobuf::RepeatedField<T> &repeatedField) {
   serialise(vector, repeatedField, vector.size());
}

template<typename T, int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<T, size1_Rows, size2_Cols> &cpp, google::protobuf::RepeatedField<T> &pb) {
   serialise(cpp.data(), pb, size1_Rows * size2_Cols);
}

template<typename T, typename U>
static void serialise(const T *array, ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField, int size) {
   repeatedPtrField.Reserve(size);
   for (int i = 0; i < size; ++i) {
      U *u = repeatedPtrField.Add();
      serialise(array[i], *u);
   }
}

template<typename T, typename U, int size>
static void serialise(const T(&array)[size], ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   serialise(array, repeatedPtrField, size);
}

template<typename T, typename U>
static void serialise(const vector <T> &vector, ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   serialise(vector.data(), repeatedPtrField, vector.size());
}

static void serialise(const pair<int, int> &cpp, offnao::PairIntInt &pb) {
   pb.set_first(cpp.first);
   pb.set_second(cpp.second);
}

static void
serialise(const boost::numeric::ublas::matrix<float> &cpp, offnao::FloatMatrix &pb) {
   unsigned int size1 = cpp.size1();
   unsigned int size2 = cpp.size2();
   pb.set_size1(size1);
   pb.set_size2(size2);
   serialise(cpp.data(), *pb.mutable_data(), size1 * size2);
}

template<int size1_Rows, int size2_Cols>
static void serialise(const Eigen::Matrix<float, size1_Rows, size2_Cols> &cpp, offnao::FloatMatrix &pb) {
   pb.set_size1(size1_Rows);
   pb.set_size2(size2_Cols);
   serialise(cpp.data(), *pb.mutable_data(), size1_Rows * size2_Cols);
}

// // I couldn't decide if I wanted these in their own .cpp files, but I'm pretty sure I don't want these in their
// // respective classes, because then they have to include the protobuf which changes often, and I don't want to split
// // the protobuf into multiple files because that's annoying.

static void serialise(const CameraSettings &cpp, offnao::Vision_CameraSettings &pb) {
   pb.set_hflip(cpp.hflip);
   pb.set_vflip(cpp.vflip);
   pb.set_brightness(cpp.brightness);
   pb.set_contrast(cpp.contrast);
   pb.set_saturation(cpp.saturation);
   pb.set_hue(cpp.hue);
   pb.set_sharpness(cpp.sharpness);
   pb.set_backlightcompensation(cpp.backlightCompensation);
   pb.set_exposure(cpp.exposure);
   pb.set_gain(cpp.gain);
   pb.set_whitebalance(cpp.whiteBalance);
   pb.set_exposureauto(cpp.exposureAuto);
   pb.set_autowhitebalance(cpp.autoWhiteBalance);
   pb.set_exposurealgorithm(cpp.exposureAlgorithm);
   pb.set_aetargetavgluma(cpp.aeTargetAvgLuma);
   pb.set_aetargetavglumadark(cpp.aeTargetAvgLumaDark);
   pb.set_aetargetgain(cpp.aeTargetGain);
   pb.set_aeminvirtgain(cpp.aeMinVirtGain);
   pb.set_aemaxvirtgain(cpp.aeMaxVirtGain);
   pb.set_aeminvirtagain(cpp.aeMinVirtAGain);
   pb.set_aemaxvirtagain(cpp.aeMaxVirtAGain);
   pb.set_autofocus(cpp.autoFocus);
   pb.set_aetargetexposure(cpp.aeTargetExposure);
   pb.set_aeuseweighttable(cpp.aeUseWeightTable);
}

#ifndef CTC_2_1

// offnao is compiled for v5 and so it expects the v5 order
static vector<float> v6JointsToV5Joints(const float v6JointValues[Joints::NUMBER_OF_JOINTS]) {
   vector<float> v5JointValues(V5Joints::NUMBER_OF_JOINTS);
   v5JointValues[V5Joints::HeadYaw]        = v6JointValues[Joints::HeadYaw];
   v5JointValues[V5Joints::HeadPitch]      = v6JointValues[Joints::HeadPitch];
   v5JointValues[V5Joints::LShoulderPitch] = v6JointValues[Joints::LShoulderPitch];
   v5JointValues[V5Joints::LShoulderRoll]  = v6JointValues[Joints::LShoulderRoll];
   v5JointValues[V5Joints::LElbowYaw]      = v6JointValues[Joints::LElbowYaw];
   v5JointValues[V5Joints::LElbowRoll]     = v6JointValues[Joints::LElbowRoll];
   v5JointValues[V5Joints::LWristYaw]      = v6JointValues[Joints::LWristYaw];
   v5JointValues[V5Joints::LHipYawPitch]   = v6JointValues[Joints::LHipYawPitch];
   v5JointValues[V5Joints::LHipRoll]       = v6JointValues[Joints::LHipRoll];
   v5JointValues[V5Joints::LHipPitch]      = v6JointValues[Joints::LHipPitch];
   v5JointValues[V5Joints::LKneePitch]     = v6JointValues[Joints::LKneePitch];
   v5JointValues[V5Joints::LAnklePitch]    = v6JointValues[Joints::LAnklePitch];
   v5JointValues[V5Joints::LAnkleRoll]     = v6JointValues[Joints::LAnkleRoll];
   v5JointValues[V5Joints::RHipRoll]       = v6JointValues[Joints::RHipRoll];
   v5JointValues[V5Joints::RHipPitch]      = v6JointValues[Joints::RHipPitch];
   v5JointValues[V5Joints::RKneePitch]     = v6JointValues[Joints::RKneePitch];
   v5JointValues[V5Joints::RAnklePitch]    = v6JointValues[Joints::RAnklePitch];
   v5JointValues[V5Joints::RAnkleRoll]     = v6JointValues[Joints::RAnkleRoll];
   v5JointValues[V5Joints::RShoulderPitch] = v6JointValues[Joints::RShoulderPitch];
   v5JointValues[V5Joints::RShoulderRoll]  = v6JointValues[Joints::RShoulderRoll];
   v5JointValues[V5Joints::RElbowYaw]      = v6JointValues[Joints::RElbowYaw];
   v5JointValues[V5Joints::RElbowRoll]     = v6JointValues[Joints::RElbowRoll];
   v5JointValues[V5Joints::RWristYaw]      = v6JointValues[Joints::RWristYaw];
   v5JointValues[V5Joints::LHand]          = v6JointValues[Joints::LHand];
   v5JointValues[V5Joints::RHand]          = v6JointValues[Joints::RHand];
   return v5JointValues;
}

// offnao is compiled for v5 and so it expects the v5 order
static vector<float> v6SensorsToV5Sensors(const float v6SensorValues[Sensors::NUMBER_OF_SENSORS]) {
   vector<float> v5SensorValues(V5Sensors::NUMBER_OF_SENSORS);
   v5SensorValues[V5Sensors::InertialSensor_AngleX]         = v6SensorValues[Sensors::InertialSensor_AngleX];
   v5SensorValues[V5Sensors::InertialSensor_AngleY]         = v6SensorValues[Sensors::InertialSensor_AngleY];
   v5SensorValues[V5Sensors::InertialSensor_GyroscopeX]     = v6SensorValues[Sensors::InertialSensor_GyroscopeX];
   v5SensorValues[V5Sensors::InertialSensor_GyroscopeY]     = v6SensorValues[Sensors::InertialSensor_GyroscopeY];
   v5SensorValues[V5Sensors::InertialSensor_GyroscopeZ]     = v6SensorValues[Sensors::InertialSensor_GyroscopeZ];
   v5SensorValues[V5Sensors::InertialSensor_AccelerometerX] = v6SensorValues[Sensors::InertialSensor_AccelerometerX];
   v5SensorValues[V5Sensors::InertialSensor_AccelerometerY] = v6SensorValues[Sensors::InertialSensor_AccelerometerY];
   v5SensorValues[V5Sensors::InertialSensor_AccelerometerZ] = v6SensorValues[Sensors::InertialSensor_AccelerometerZ];
   v5SensorValues[V5Sensors::LFoot_FSR_FrontLeft]           = v6SensorValues[Sensors::LFoot_FSR_FrontLeft];
   v5SensorValues[V5Sensors::LFoot_FSR_FrontRight]          = v6SensorValues[Sensors::LFoot_FSR_FrontRight];
   v5SensorValues[V5Sensors::LFoot_FSR_RearLeft]            = v6SensorValues[Sensors::LFoot_FSR_RearLeft];
   v5SensorValues[V5Sensors::LFoot_FSR_RearRight]           = v6SensorValues[Sensors::LFoot_FSR_RearRight];
//   v5SensorValues[V5Sensors::LFoot_FSR_CenterOfPressure_X] = v6SensorValues[Sensors::LFoot_FSR_CenterOfPressure_X];
//   v5SensorValues[V5Sensors::LFoot_FSR_CenterOfPressure_Y] = v6SensorValues[Sensors::LFoot_FSR_CenterOfPressure_Y];
   v5SensorValues[V5Sensors::RFoot_FSR_FrontLeft]           = v6SensorValues[Sensors::RFoot_FSR_FrontLeft];
   v5SensorValues[V5Sensors::RFoot_FSR_FrontRight]          = v6SensorValues[Sensors::RFoot_FSR_FrontRight];
   v5SensorValues[V5Sensors::RFoot_FSR_RearLeft]            = v6SensorValues[Sensors::RFoot_FSR_RearLeft];
   v5SensorValues[V5Sensors::RFoot_FSR_RearRight]           = v6SensorValues[Sensors::RFoot_FSR_RearRight];
//   v5SensorValues[V5Sensors::RFoot_FSR_CenterOfPressure_X] = v6SensorValues[Sensors::RFoot_FSR_CenterOfPressure_X];
//   v5SensorValues[V5Sensors::RFoot_FSR_CenterOfPressure_Y] = v6SensorValues[Sensors::RFoot_FSR_CenterOfPressure_Y];
   v5SensorValues[V5Sensors::LFoot_Bumper_Left]             = v6SensorValues[Sensors::LFoot_Bumper_Left];
   v5SensorValues[V5Sensors::LFoot_Bumper_Right]            = v6SensorValues[Sensors::LFoot_Bumper_Right];
   v5SensorValues[V5Sensors::RFoot_Bumper_Left]             = v6SensorValues[Sensors::RFoot_Bumper_Left];
   v5SensorValues[V5Sensors::RFoot_Bumper_Right]            = v6SensorValues[Sensors::RFoot_Bumper_Right];
   v5SensorValues[V5Sensors::ChestBoard_Button]             = v6SensorValues[Sensors::ChestBoard_Button];
   v5SensorValues[V5Sensors::Head_Touch_Front]              = v6SensorValues[Sensors::Head_Touch_Front];
   v5SensorValues[V5Sensors::Head_Touch_Rear]               = v6SensorValues[Sensors::Head_Touch_Rear];
   v5SensorValues[V5Sensors::Head_Touch_Middle]             = v6SensorValues[Sensors::Head_Touch_Middle];
   v5SensorValues[V5Sensors::Battery_Charge]                = v6SensorValues[Sensors::Battery_Charge];
   v5SensorValues[V5Sensors::Battery_Current]               = v6SensorValues[Sensors::Battery_Current];
   v5SensorValues[V5Sensors::SonarLeft]                     = v6SensorValues[Sensors::SonarLeft];
   v5SensorValues[V5Sensors::SonarRight]                    = v6SensorValues[Sensors::SonarRight];
   return v5SensorValues;
}

#endif

static void serialise(const JointValues &cpp, offnao::JointValues &pb) {
#ifdef CTC_2_1
   serialise(cpp.angles, *pb.mutable_angles());
   serialise(cpp.stiffnesses, *pb.mutable_stiffnesses());
   serialise(cpp.temperatures, *pb.mutable_temperatures());
   serialise(cpp.currents, *pb.mutable_currents());
#else
   // offnao is compiled for v5 and so it expects the v5 order
   serialise(v6JointsToV5Joints(cpp.angles), *pb.mutable_angles());
   serialise(v6JointsToV5Joints(cpp.stiffnesses), *pb.mutable_stiffnesses());
   serialise(v6JointsToV5Joints(cpp.temperatures), *pb.mutable_temperatures());
   serialise(v6JointsToV5Joints(cpp.currents), *pb.mutable_currents());
#endif
}

static void serialise(const SensorValues &cpp, offnao::SensorValues &pb) {
   serialise(cpp.joints, *pb.mutable_joints());
#ifdef CTC_2_1
   serialise(cpp.sensors, *pb.mutable_sensors());
#else
   serialise(v6SensorsToV5Sensors(cpp.sensors), *pb.mutable_sensors());
#endif
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void serialise(const Pose &cpp, offnao::Motion_Pose &pb) {
   serialise(cpp.topCameraToWorldTransform, *pb.mutable_topcameratoworldtransform());
   serialise(cpp.botCameraToWorldTransform, *pb.mutable_botcameratoworldtransform());
   serialise(cpp.neckToWorldTransform, *pb.mutable_necktoworldtransform());
   serialise(cpp.origin, *pb.mutable_origin());
   serialise(cpp.zunit, *pb.mutable_zunit());
   serialise(cpp.topCOrigin, *pb.mutable_topcorigin());
   serialise(cpp.botCOrigin, *pb.mutable_botcorigin());
   serialise(cpp.horizon, *pb.mutable_horizon());
   serialiseWithImplicitCast(cpp.topExclusionArray, *pb.mutable_topexclusionarray());
   serialiseWithImplicitCast(cpp.botExclusionArray, *pb.mutable_botexclusionarray());
}

void serialise(const Odometry &cpp, offnao::Motion_Odometry &pb) {
   pb.set_forward(cpp.forward);
   pb.set_left(cpp.left);
   pb.set_turn(cpp.turn);
}

static void serialise(const XYZ_Coord &cpp, offnao::XYZ_Coord &pb) {
   pb.set_x(cpp.x);
   pb.set_y(cpp.y);
   pb.set_z(cpp.z);
}

static void serialise(const ActionCommand::Head &cpp, offnao::ActionCommandAll_Head &pb) {
   pb.set_yaw(cpp.yaw);
   pb.set_pitch(cpp.pitch);
   pb.set_isrelative(cpp.isRelative);
   pb.set_yawspeed(cpp.yawSpeed);
   pb.set_pitchspeed(cpp.pitchSpeed);
}

static void serialise(const ActionCommand::Body &cpp, offnao::ActionCommandAll_Body &pb) {
   pb.set_actiontype(static_cast<offnao::ActionType>(cpp.actionType));
   pb.set_forward(cpp.forward);
   pb.set_left(cpp.left);
   pb.set_turn(cpp.turn);
   pb.set_power(cpp.power);
}

static void serialise(const ActionCommand::rgb &cpp, offnao::ActionCommandAll_LED_rgb &pb) {
   pb.set_red(cpp.red);
   pb.set_green(cpp.green);
   pb.set_blue(cpp.blue);
}

static void serialise(const ActionCommand::LED &cpp, offnao::ActionCommandAll_LED &pb) {
   pb.set_leftear(cpp.leftEar);
   pb.set_rightear(cpp.rightEar);
   serialise(cpp.leftEye, *pb.mutable_lefteye());
   serialise(cpp.rightEye, *pb.mutable_righteye());
   serialise(cpp.chestButton, *pb.mutable_chestbutton());
   serialise(cpp.leftFoot, *pb.mutable_leftfoot());
   serialise(cpp.rightFoot, *pb.mutable_rightfoot());
}

static void serialise(const ActionCommand::All &cpp, offnao::ActionCommandAll &pb) {
   serialise(cpp.head, *pb.mutable_head());
   serialise(cpp.body, *pb.mutable_body());
   serialise(cpp.leds, *pb.mutable_leds());
}

static void serialise(const FootPosition &cpp, offnao::Motion_FootPosition &pb)
{
   pb.set_x(cpp.x);
   pb.set_y(cpp.y);
   pb.set_theta(cpp.theta);
}

static void serialise(const FeetPosition &cpp, offnao::Motion_FeetPosition &pb)
{
   serialise(cpp.left, *pb.mutable_left());
   serialise(cpp.right, *pb.mutable_right());
}

static void serialise(const MotionDebugInfo &cpp, offnao::Motion_MotionDebugInfo &pb) {
   serialise(cpp.feetPosition, *pb.mutable_feetposition());
}

static void serialise(const Parameters &cpp, offnao::Kinematics_Parameters &pb) {
   pb.set_camerapitchtop(cpp.cameraPitchTop);
   pb.set_camerayawtop(cpp.cameraYawTop);
   pb.set_camerarolltop(cpp.cameraRollTop);
   pb.set_camerayawbottom(cpp.cameraYawBottom);
   pb.set_camerapitchbottom(cpp.cameraPitchBottom);
   pb.set_camerarollbottom(cpp.cameraRollBottom);
   pb.set_bodypitch(cpp.bodyPitch);
}

static void serialise(const AbsCoord &cpp, offnao::AbsCoord &pb) {
   serialise(cpp.vec, *pb.mutable_vec());
   serialise(cpp.var, *pb.mutable_var());
   pb.set_weight(cpp.weight);
}

static void serialise(const RRCoord &cpp, offnao::RRCoord &pb) {
   serialise(cpp.vec, *pb.mutable_vec());
   serialise(cpp.var, *pb.mutable_var());
}

static void serialise(const SharedStateEstimationBundle &cpp, offnao::SharedStateEstimationBundle &pb) {
   serialise(cpp.robotPos, *pb.mutable_robotpos());
   serialise(cpp.ballPosRRC, *pb.mutable_ballposrrc());
   serialise(cpp.ballVelRRC, *pb.mutable_ballvelrrc());
   pb.set_haveballupdate(cpp.haveBallUpdate);
}

static void serialise(const RobotObstacle &cpp, offnao::StateEstimation_RobotObstacle &pb) {
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_type(static_cast<offnao::RobotVisionInfoType>(cpp.type));
   serialise(cpp.rrc, *pb.mutable_rrc());
   serialise(cpp.pos, *pb.mutable_pos());
   pb.set_tangentheadingleft(cpp.tangentHeadingLeft);
   pb.set_tangentheadingright(cpp.tangentHeadingRight);
   serialise(cpp.evadeVectorLeft, *pb.mutable_evadevectorleft());
   serialise(cpp.evadeVectorRight, *pb.mutable_evadevectorright());
}

static void serialise(const BehaviourRequest &cpp, offnao::Behaviour_BehaviourRequest &pb) {
   serialise(cpp.actions, *pb.mutable_actions());
   serialise(cpp.behaviourSharedData, *pb.mutable_behaviourshareddata());
   serialise(cpp.behaviourDebugInfo, *pb.mutable_behaviourdebuginfo());
}

static void serialise(const BehaviourDebugInfo &cpp, offnao::BehaviourDebugInfo &pb) {
   pb.set_bodybehaviourhierarchy(cpp.bodyBehaviourHierarchy);
   pb.set_headbehaviourhierarchy(cpp.headBehaviourHierarchy);
   pb.set_haveballmanoeuvretarget(cpp.haveBallManoeuvreTarget);
   pb.set_ballmanoeuvretargetx(cpp.ballManoeuvreTargetX);
   pb.set_ballmanoeuvretargety(cpp.ballManoeuvreTargetY);
   pb.set_ballmanoeuvreheadingerror(cpp.ballManoeuvreHeadingError);
   pb.set_ballmanoeuvretype(cpp.ballManoeuvreType);
   pb.set_ballmanoeuvrehard(cpp.ballManoeuvreHard);
   pb.set_anticipating(cpp.anticipating);
   pb.set_anticipatex(cpp.anticipateX);
   pb.set_anticipatey(cpp.anticipateY);
   pb.set_anticipateh(cpp.anticipateH);
}

static void serialise(const Eigen::Matrix<int, 2, 1> &cpp, offnao::Point &pb) {
   pb.set_x(cpp[0]);
   pb.set_y(cpp[1]);
}

static void serialise(const BBox &cpp, offnao::BBox &pb) {
   serialise(cpp.a, *pb.mutable_a());
   serialise(cpp.b, *pb.mutable_b());
}

static void serialise(const BallInfo &cpp, offnao::Vision_BallInfo &pb) {
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_radius(cpp.radius);
   serialise(cpp.imageCoords, *pb.mutable_imagecoords());
   pb.set_topcamera(cpp.topCamera);
}

static void serialise(const RobotVisionInfo &cpp, offnao::Vision_RobotVisionInfo &pb) {
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_type(static_cast<offnao::RobotVisionInfoType>(cpp.type));
   pb.set_cameras(static_cast<offnao::Vision_Cameras>(cpp.cameras));
   serialise(cpp.imageCoords, *pb.mutable_imagecoords());
   serialise(cpp.topImageCoords, *pb.mutable_topimagecoords());
   serialise(cpp.botImageCoords, *pb.mutable_botimagecoords());
}

static void serialise(const RANSACLine &cpp, offnao::Vision_RANSACLine &pb) {
   pb.set_t1(cpp.t1);
   pb.set_t2(cpp.t2);
   pb.set_t3(cpp.t3);
   pb.set_var(cpp.var);
   serialise(cpp.p1, *pb.mutable_p1());
   serialise(cpp.p2, *pb.mutable_p2());
}

static void serialise(const FieldBoundaryInfo &cpp, offnao::Vision_FieldBoundaryInfo &pb) {
   serialise(cpp.rrBoundary, *pb.mutable_rrboundary());
   serialise(cpp.imageBoundary, *pb.mutable_imageboundary());
}

static void serialise(const FieldFeatureInfo &cpp, offnao::Vision_FieldFeatureInfo &pb) {
   serialise(cpp.rr, *pb.mutable_rr());
   pb.set_type(static_cast<offnao::Vision_FieldFeatureInfoType>(cpp.type));
   serialise(cpp.p1, *pb.mutable_p1());
   serialise(cpp.p2, *pb.mutable_p2());
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void serialise(const RegionI &cpp, offnao::Vision_RegionI &pb) {
   pb.set_is_top_camera_(cpp.is_top_camera_);
   serialise(cpp.bounding_box_rel_, *pb.mutable_bounding_box_rel_());
   serialise(cpp.bounding_box_fovea_, *pb.mutable_bounding_box_fovea_());
   serialise(cpp.bounding_box_raw_, *pb.mutable_bounding_box_raw_());
   pb.set_n_raw_cols_in_region_(cpp.n_raw_cols_in_region_);
   pb.set_n_raw_rows_in_region_(cpp.n_raw_rows_in_region_);
   pb.set_density_to_raw_(cpp.density_to_raw_);
   pb.set_y_offset_raw_(cpp.y_offset_raw_);
   pb.set_x_offset_raw_(cpp.x_offset_raw_);
   pb.set_raw_total_width_(cpp.raw_total_width_);
   pb.set_raw_to_fovea_density_(cpp.raw_to_fovea_density_);
   pb.set_fovea_width_(cpp.fovea_width_);
}

static void serialise(const SPLStandardMessage &cpp, offnao::Receiver_SPLStandardMessage &pb) {
   serialise(cpp.header, *pb.mutable_header());
   pb.set_version(cpp.version);
   pb.set_playernum(cpp.playerNum);
   pb.set_teamnum(cpp.teamNum);
   pb.set_fallen(cpp.fallen);
   serialise(cpp.pose, *pb.mutable_pose());
   pb.set_ballage(cpp.ballAge);
   serialise(cpp.ball, *pb.mutable_ball());
   pb.set_numofdatabytes(cpp.numOfDataBytes);
   pb.set_data(cpp.data, cpp.numOfDataBytes);
}

static void serialise(const BehaviourSharedData &cpp, offnao::BehaviourSharedData &pb) {
   pb.set_role(cpp.role);
   pb.set_playingball(cpp.playingBall);
   pb.set_needassistance(cpp.needAssistance);
   pb.set_isassisting(cpp.isAssisting);
   pb.set_secondssincelastkick(cpp.secondsSinceLastKick);
   pb.set_iskickedoff(cpp.isKickedOff);
   pb.set_walkingtox(cpp.walkingToX);
   pb.set_walkingtoy(cpp.walkingToY);
   pb.set_walkingtoh(cpp.walkingToH);
}

static void serialise(const BroadcastData &cpp, offnao::Receiver_BroadcastData &pb) {
   pb.set_playernum(cpp.playerNum);
   serialise(cpp.robotPos, *pb.mutable_robotpos());
   serialise(cpp.ballPosAbs, *pb.mutable_ballposabs());
   serialise(cpp.ballPosRR, *pb.mutable_ballposrr());
   serialise(cpp.sharedStateEstimationBundle, *pb.mutable_sharedstateestimationbundle());
   serialise(cpp.behaviourSharedData, *pb.mutable_behaviourshareddata());
   pb.set_acb(static_cast<offnao::ActionType>(cpp.acB));
   pb.set_uptime(cpp.uptime);
   pb.set_gamestate(cpp.gameState);
}

static void serialise(offnao::Blackboard &pb,
                      int bytesPerPixel,
                      const uint8_t *frame,
                      int8_t quality,
                      int height,
                      int width,
                      void (::offnao::Vision::*setframejpeg)(const void *, size_t),
                      void (::offnao::Vision::*setframe)(const void *, size_t)) {
   if (frame) {
      if (quality >= 0) {
         // only works with yuvyuv not yuyv
         if (bytesPerPixel == 2) {
            const int      newBytesPerPixel = 3;
            const int      yuvFrameSize     = height * width * newBytesPerPixel;
            static uint8_t *yuvFrame        = new uint8_t[yuvFrameSize];

            const uint8_t *yuyvFrameRead = frame;
            uint8_t       *yuvFrameWrite = yuvFrame;
            const uint8_t *const yuvFrameWriteEnd = yuvFrame + yuvFrameSize;
            do {
               // convert two pixels at a time
               yuvFrameWrite[0] = yuyvFrameRead[0];
               yuvFrameWrite[1] = yuyvFrameRead[1];
               yuvFrameWrite[2] = yuyvFrameRead[3];
               yuvFrameWrite[3] = yuyvFrameRead[2];
               yuvFrameWrite[4] = yuyvFrameRead[1];
               yuvFrameWrite[5] = yuyvFrameRead[3];
               yuyvFrameRead += 4;
               yuvFrameWrite += 6;
            } while (yuvFrameWrite < yuvFrameWriteEnd);

            frame         = yuvFrame;
            bytesPerPixel = newBytesPerPixel;
         }

         // os 2.1 has turbo jpeg 1.1.1
         // os 2.8 has turbo jpeg 1.4.2, which has some more convenient functions, supposedly
         // https://github.com/libjpeg-turbo/libjpeg-turbo/blob/1.1.x/example.c
         struct jpeg_compress_struct cinfo;
         struct jpeg_error_mgr       jerr; // TODO: not exit on error
         unsigned long               outsize    = height * width * bytesPerPixel;
         static unsigned char        *outbuffer = new unsigned char[outsize];
         JSAMPROW                    row_pointer[1]; /* pointer to JSAMPLE row[s] */
         int                         row_stride;     /* physical row width in image buffer */
         cinfo.err = jpeg_std_error(&jerr);
         jpeg_create_compress(&cinfo);
         jpeg_mem_dest(&cinfo, &outbuffer, &outsize);
         cinfo.image_width      = width;             /* image width and height, in pixels */
         cinfo.image_height     = height;
         cinfo.input_components = bytesPerPixel;     /* # of color components per pixel */
         cinfo.in_color_space   = JCS_YCbCr;         /* colorspace of input image */
         jpeg_set_defaults(&cinfo);
         jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);
         jpeg_start_compress(&cinfo, TRUE);
         row_stride = width * bytesPerPixel;         /* JSAMPLEs per row in image_buffer */
         while (cinfo.next_scanline < cinfo.image_height) {
            row_pointer[0] = const_cast<JSAMPROW>(&frame[cinfo.next_scanline * row_stride]);
            (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
         }
         jpeg_finish_compress(&cinfo);
         (pb.mutable_vision()->*setframejpeg)(outbuffer, outsize);
         jpeg_destroy_compress(&cinfo);
      } else {
         (pb.mutable_vision()->*setframe)(frame, sizeof(uint8_t[height * width * bytesPerPixel]));
      }
   }
}

// the if/mask statements belong in serialise not serialise, but we keep them here anyway so we don't need `if (!pb.vision().landmarks().empty())` or `if(pb.vision().has_timestamp())`
void Blackboard::serialise(const Blackboard &cpp, offnao::Blackboard &pb) {
   pb.set_mask(cpp.mask);
   if (cpp.mask & BLACKBOARD_MASK) {
      pb.mutable_gamecontroller()->set_player_number(cpp.gameController.player_number);
      pb.mutable_gamecontroller()->mutable_our_team()->set_teamnumber(cpp.gameController.our_team.teamNumber);

      ::serialise(cpp.motion.sensors, *pb.mutable_motion()->mutable_sensors());
      ::serialise(cpp.motion.pose, *pb.mutable_motion()->mutable_pose());
      ::serialise(cpp.motion.com, *pb.mutable_motion()->mutable_com());
      ::serialise(cpp.motion.odometry, *pb.mutable_motion()->mutable_odometry());
      ::serialise(cpp.motion.active, *pb.mutable_motion()->mutable_active());
      ::serialise(cpp.motion.jointRequest, *pb.mutable_motion()->mutable_jointrequest());
      ::serialise(cpp.motion.motionDebugInfo, *pb.mutable_motion()->mutable_motiondebuginfo());

      pb.mutable_perception()->set_behaviour(cpp.perception.behaviour);
      pb.mutable_perception()->set_kinematics(cpp.perception.kinematics);
      pb.mutable_perception()->set_stateestimation(cpp.perception.stateEstimation);
      pb.mutable_perception()->set_total(cpp.perception.total);
      pb.mutable_perception()->set_vision(cpp.perception.vision);

      // This request was updated for version 19 but not sure if more is required
      ::serialise(cpp.behaviour.request, *pb.mutable_behaviour()->mutable_request());

      ::serialise(cpp.kinematics.parameters, *pb.mutable_kinematics()->mutable_parameters());

      if (cpp.mask & ROBOT_FILTER_MASK) {
         ::serialise(cpp.stateEstimation.robotObstacles, *pb.mutable_stateestimation()->mutable_robotobstacles());
      }

      /* Only serialise the things below if WHITEBOARD_MASK is not set.
       * We also ONLY want this to happen when we are serialising in Offnao,
       * which occurs when we save the dump. WHITEBOARD_MASK can only be set
       * in the save function in offnao.
       */
      if (!(cpp.mask & WHITEBOARD_MASK)) {
         pb.mutable_vision()->set_timestamp(cpp.vision.timestamp);
         ::serialise(cpp.vision.balls, *pb.mutable_vision()->mutable_balls());
         ::serialise(cpp.vision.robots, *pb.mutable_vision()->mutable_robots());
         ::serialise(cpp.vision.fieldBoundaries, *pb.mutable_vision()->mutable_fieldboundaries());
         ::serialise(cpp.vision.fieldFeatures, *pb.mutable_vision()->mutable_fieldfeatures());

         ::serialise(cpp.vision.regions, *pb.mutable_vision()->mutable_regions());
      }

      ::serialise(cpp.vision.topCameraSettings, *pb.mutable_vision()->mutable_topcamerasettings());
      ::serialise(cpp.vision.botCameraSettings, *pb.mutable_vision()->mutable_botcamerasettings());

      pb.mutable_vision()->set_horizontalfieldofview(cpp.vision.horizontalFieldOfView);
      pb.mutable_vision()->set_verticalfieldofview(cpp.vision.verticalFieldOfView);

      pb.mutable_vision()->set_atwindowsizetop(ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP);
      pb.mutable_vision()->set_atwindowsizebot(ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT);
      pb.mutable_vision()->set_atpercentagetop(ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP);
      pb.mutable_vision()->set_atpercentagebot(ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT);

      ::serialise(cpp.receiver.message, *pb.mutable_receiver()->mutable_message());
      ::serialise(cpp.receiver.data, *pb.mutable_receiver()->mutable_data());
      serialiseWithImplicitCast(cpp.receiver.lastReceived, *pb.mutable_receiver()->mutable_lastreceived());
      ::serialise(cpp.receiver.incapacitated, *pb.mutable_receiver()->mutable_incapacitated());


      // seems to happen irregardless of everything else, so why do it twice?
//     ::serialise(cpp.stateEstimation.robotPos, *pb.mutable_stateestimation()->mutable_robotpos());
      ::serialise(cpp.stateEstimation.allRobotPos, *pb.mutable_stateestimation()->mutable_allrobotpos());

      ::serialise(cpp.stateEstimation.ballPosRR, *pb.mutable_stateestimation()->mutable_ballposrr());
      ::serialise(cpp.stateEstimation.ballPosRRC, *pb.mutable_stateestimation()->mutable_ballposrrc());
      ::serialise(cpp.stateEstimation.ballVelRRC, *pb.mutable_stateestimation()->mutable_ballvelrrc());
      ::serialise(cpp.stateEstimation.ballVel, *pb.mutable_stateestimation()->mutable_ballvel());
      ::serialise(cpp.stateEstimation.ballPos, *pb.mutable_stateestimation()->mutable_ballpos());
      ::serialise(cpp.stateEstimation.teamBallPos, *pb.mutable_stateestimation()->mutable_teamballpos());
      ::serialise(cpp.stateEstimation.teamBallVel, *pb.mutable_stateestimation()->mutable_teamballvel());
      ::serialise(cpp.stateEstimation.sharedStateEstimationBundle,
                  *pb.mutable_stateestimation()->mutable_sharedstateestimationbundle());
      pb.mutable_stateestimation()
            ->set_havependingoutgoingsharedbundle(cpp.stateEstimation.havePendingOutgoingSharedBundle);
      ::serialise(cpp.stateEstimation.havePendingIncomingSharedBundle,
                  *pb.mutable_stateestimation()->mutable_havependingincomingsharedbundle());
   }

   if (cpp.mask & SALIENCY_MASK) {
      // TODO(jayen): RLE
      if (cpp.vision.topSaliency) {
         pb.mutable_vision()->set_topsaliency(cpp.vision.topSaliency, sizeof(TopSaliency));
      }
      if (cpp.vision.botSaliency) {
         pb.mutable_vision()->set_botsaliency(cpp.vision.botSaliency, sizeof(BotSaliency));
      }
   }
   if (cpp.mask & RAW_IMAGE_MASK) {
      // NULL in simulation
      ::serialise(pb,
                  2,
                  cpp.vision.topFrame,
                  cpp.vision.topFrameJPEGQuality,
                  TOP_IMAGE_ROWS,
                  TOP_IMAGE_COLS,
                  &::offnao::Vision::set_topframejpeg,
                  &::offnao::Vision::set_topframe);
      ::serialise(pb,
                  2,
                  cpp.vision.botFrame,
                  cpp.vision.botFrameJPEGQuality,
                  BOT_IMAGE_ROWS,
                  BOT_IMAGE_COLS,
                  &::offnao::Vision::set_botframejpeg,
                  &::offnao::Vision::set_botframe);
   }

   ::serialise(cpp.stateEstimation.robotPos, *pb.mutable_stateestimation()->mutable_robotpos());
}

void Blackboard::serialise(ostream &os) const {
   offnao::Blackboard bb;
   serialise(*this, bb);

   // https://developers.google.com/protocol-buffers/docs/techniques#streaming
   unsigned int size = bb.ByteSize();
   os.write(reinterpret_cast<char *>(&size), sizeof(size));
   bb.SerializeToOstream(&os);
}

// not sure why i need to forward declare these, since they exist when the templated callers are instantiated
static void deserialise(RobotObstacle &, const offnao::StateEstimation_RobotObstacle &);

static void deserialise(AbsCoord &, const offnao::AbsCoord &);

static void deserialise(BehaviourRequest &, const offnao::Behaviour_BehaviourRequest &);

static void deserialise(BallInfo &, const offnao::Vision_BallInfo &);

static void deserialise(RobotVisionInfo &, const offnao::Vision_RobotVisionInfo &);

static void deserialise(FieldBoundaryInfo &, const offnao::Vision_FieldBoundaryInfo &);

static void deserialise(FieldFeatureInfo &, const offnao::Vision_FieldFeatureInfo &);

static void deserialise(SPLStandardMessage &, const offnao::Receiver_SPLStandardMessage &);

static void deserialise(BroadcastData &, const offnao::Receiver_BroadcastData &);

static void deserialise(BehaviourDebugInfo &, const offnao::BehaviourDebugInfo &);

static void deserialise(BehaviourSharedData &, const offnao::BehaviourSharedData &);

//
template<typename T>
static void deserialise(T &cpp, const T &pb) {
   cpp = pb;
}

static void deserialise(uint8_t &cpp, const uint32_t &pb) {
   cpp = (uint8_t) pb;
}

static void deserialise(int16_t &cpp, const int32_t &pb) {
   cpp = (int16_t) pb;
}

// only when compiling on 64-bit
static void deserialise(long long int &cpp, const google::protobuf::int64 &pb) {
   cpp = (long long int) pb;
}

static void deserialise(time_t &cpp, const int32_t &pb) {
   cpp = (time_t) pb;
}

static void deserialise(uint16_t &cpp, const google::protobuf::uint32 &pb) {
   cpp = (uint16_t) pb;
}

// Generic ones for enums
#ifdef CTC_2_1

template<typename T>
static void deserialise(T &cpp,
                        const int &pb,
                        typename boost::enable_if<boost::is_enum<T> >::type *dummyT = 0) {
#else

template<typename T,
      typename std::enable_if<std::is_enum<T>::value>::type * = nullptr>
static void deserialise(T &cpp, const int &pb) {
#endif
   cpp = static_cast<T>(pb);
}

#ifdef CTC_2_1

template<typename T, typename U>
static void deserialise(T &cpp,
                        const U &pb,
                        typename boost::enable_if<boost::is_enum<T> >::type *dummyT = 0,
                        typename boost::enable_if<boost::is_enum<U> >::type *dummyU = 0) {
#else

template<typename T,
         typename U,
      typename std::enable_if<std::is_enum<T>::value>::type * = nullptr,
      typename std::enable_if<std::is_enum<U>::value>::type * = nullptr>
static void deserialise(T &cpp, const U &pb) {
#endif
   cpp = static_cast<T>(pb);
}

template<size_t fixedSize>
static void deserialise(char (&cpp)[fixedSize], const string &pb) {
   size_t size = fixedSize;
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << endl;
      size = pb.size();
   }
   pb.copy(cpp, size);
}

template<typename T, typename Container>
static void deserialise(Container &cpp, const ::google::protobuf::RepeatedField<T> &pb, int size) {
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << endl;
      size = pb.size();
   }
   for (int i = 0; i < size; ++i) {
      deserialise(cpp[i], pb.Get(i));
   }
}

template<typename T, size_t size>
static void deserialise(T (&cpp)[size], const ::google::protobuf::RepeatedField<T> &pb) {
   deserialise(cpp, pb, size);
}

// different name so it's not used accidentally
template<typename T, typename U, int size>
static void deserialiseWithImplicitCast(T(&cpp)[size], const ::google::protobuf::RepeatedField<U> &pb) {
   deserialise(cpp, pb, size);
}

// vector<bool>::data() returns void because it's a bitset
static void deserialise(vector<bool> &cpp, const ::google::protobuf::RepeatedField<bool> &pb, int size) {
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << endl;
      size = pb.size();
   }
   cpp.resize(static_cast<unsigned int>(size));
   for (int i = 0; i < size; ++i) {
      cpp[i] = pb.Get(i);
   }
}

template<typename T>
static void deserialise(vector<T> &vector, const ::google::protobuf::RepeatedField<T> &repeatedField, int size) {
   vector.resize(static_cast<unsigned int>(size));
   T *cpp = vector.data();
   deserialise(cpp, repeatedField, size);
}

template<typename T, int size1_Rows, int size2_Cols>
static void deserialise(Eigen::Matrix<T, size1_Rows, size2_Cols> &cpp, const google::protobuf::RepeatedField<T> &pb) {
   float tmp[size1_Rows * size2_Cols];
   deserialise(tmp, pb);
   cpp = Eigen::Map<Eigen::Matrix<float, size1_Rows, size2_Cols> >(tmp);
}

template<typename T, typename U>
static void deserialise(T *cpp, const ::google::protobuf::RepeatedPtrField<U> &pb, int size) {
   if (pb.size() < size) {
      llog(ERROR) << "trying to deserialise " << pb.size() << " elements to a field of " << size
                  << " elements" << endl;
      size = pb.size();
   }
   for (int i = 0; i < size; ++i) {
      deserialise(cpp[i], pb.Get(i));
   }
}

template<typename T, typename U>
static void deserialise(T *array, const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   deserialise(array, repeatedPtrField, repeatedPtrField.size());
}

// I'm not sure how to overload the one above such that this one takes precendence.  Would be nice if we used it more often, but I'm too lazy to find all the fixed-size arrays.
template<typename T, typename U, int size>
static void deserialiseArray(T(&cpp)[size], const ::google::protobuf::RepeatedPtrField<U> &pb) {
   deserialise(cpp, pb, size);
}

template<typename T, typename U>
static void deserialise(vector<T> &vector, const ::google::protobuf::RepeatedPtrField<U> &repeatedPtrField) {
   vector.resize(static_cast<unsigned int>(repeatedPtrField.size()));
   deserialise(vector.data(), repeatedPtrField);
}

static void deserialise(pair<int, int> &cpp, const offnao::PairIntInt &pb) {
   cpp.first  = pb.first();
   cpp.second = pb.second();
}

static void
deserialise(boost::numeric::ublas::matrix<float> &cpp, const offnao::FloatMatrix &pb) {
   cpp.resize(pb.size1(), pb.size2());
   deserialise(cpp.data(), pb.data(), pb.size1() * pb.size2());
}

template<int size1_Rows, int size2_Cols>
static void deserialise(Eigen::Matrix<float, size1_Rows, size2_Cols> &cpp, const offnao::FloatMatrix &pb) {
   if (size1_Rows != pb.size1() || size2_Cols != pb.size2())
      llog(ERROR) << "trying to deserialise " << pb.size1() << "x" << pb.size2() << " elements to a field of "
                  << size1_Rows << "x" << size2_Cols << " elements" << endl;
   float tmp[size1_Rows * size2_Cols];
   deserialise(tmp, pb.data());
   cpp = Eigen::Map<Eigen::Matrix<float, size1_Rows, size2_Cols> >(tmp);
}

// // I couldn't decide if I wanted these in their own .cpp files, but I'm pretty sure I don't want these in their
// // respective classes, because then they have to include the protobuf which changes often, and I don't want to split
// // the protobuf into multiple files because that's annoying.

static void deserialise(CameraSettings &cpp, const offnao::Vision_CameraSettings &pb) {
   cpp.hflip                 = pb.hflip();
   cpp.vflip                 = pb.vflip();
   cpp.brightness            = pb.brightness();
   cpp.contrast              = pb.contrast();
   cpp.saturation            = pb.saturation();
   cpp.hue                   = pb.hue();
   cpp.sharpness             = pb.sharpness();
   cpp.backlightCompensation = pb.backlightcompensation();
   cpp.exposure              = pb.exposure();
   cpp.gain                  = pb.gain();
   cpp.whiteBalance          = pb.whitebalance();
   cpp.exposureAuto          = pb.exposureauto();
   cpp.autoWhiteBalance      = pb.autowhitebalance();
   cpp.exposureAlgorithm     = pb.exposurealgorithm();
   cpp.aeTargetAvgLuma       = pb.aetargetavgluma();
   cpp.aeTargetAvgLumaDark   = pb.aetargetavglumadark();
   cpp.aeTargetGain          = pb.aetargetgain();
   cpp.aeMinVirtGain         = pb.aeminvirtgain();
   cpp.aeMaxVirtGain         = pb.aemaxvirtgain();
   cpp.aeMinVirtAGain        = pb.aeminvirtagain();
   cpp.aeMaxVirtAGain        = pb.aemaxvirtagain();
   cpp.autoFocus             = pb.autofocus();
   cpp.aeTargetExposure      = pb.aetargetexposure();
   cpp.aeUseWeightTable      = pb.aeuseweighttable();
}

static void deserialise(JointValues &cpp, const offnao::JointValues &pb) {
   deserialise(cpp.angles, pb.angles());
   deserialise(cpp.stiffnesses, pb.stiffnesses());
   deserialise(cpp.temperatures, pb.temperatures());
   deserialise(cpp.currents, pb.currents());
}

static void deserialise(SensorValues &cpp, const offnao::SensorValues &pb) {
   deserialise(cpp.joints, pb.joints());
   deserialise(cpp.sensors, pb.sensors());
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void deserialise(Pose &cpp, const offnao::Motion_Pose &pb) {
   deserialise(cpp.topCameraToWorldTransform, pb.topcameratoworldtransform());
   deserialise(cpp.botCameraToWorldTransform, pb.botcameratoworldtransform());
   deserialise(cpp.neckToWorldTransform, pb.necktoworldtransform());
   deserialise(cpp.origin, pb.origin());
   deserialise(cpp.zunit, pb.zunit());
   deserialise(cpp.topCOrigin, pb.topcorigin());
   deserialise(cpp.botCOrigin, pb.botcorigin());
   deserialise(cpp.horizon, pb.horizon());
   deserialiseWithImplicitCast(cpp.topExclusionArray, pb.topexclusionarray());
   deserialiseWithImplicitCast(cpp.botExclusionArray, pb.botexclusionarray());
   if (pb.has_necktoworldtransform())
      cpp.makeConstants();
}

void deserialise(Odometry &cpp, const offnao::Motion_Odometry &pb) {
   cpp.forward = pb.forward();
   cpp.left    = pb.left();
   cpp.turn    = pb.turn();
}

static void deserialise(XYZ_Coord &cpp, const offnao::XYZ_Coord &pb) {
   cpp.x = pb.x();
   cpp.y = pb.y();
   cpp.z = pb.z();
}

static void deserialise(ActionCommand::Head &cpp, const offnao::ActionCommandAll_Head &pb) {
   deserialise(cpp.yaw, pb.yaw());
   deserialise(cpp.pitch, pb.pitch());
   deserialise(cpp.isRelative, pb.isrelative());
   deserialise(cpp.yawSpeed, pb.yawspeed());
   deserialise(cpp.pitchSpeed, pb.pitchspeed());
}

static void deserialise(ActionCommand::Body &cpp, const offnao::ActionCommandAll_Body &pb) {
   deserialise(cpp.actionType, pb.actiontype());
   deserialise(cpp.forward, pb.forward());
   deserialise(cpp.left, pb.left());
   deserialise(cpp.turn, pb.turn());
   deserialise(cpp.power, pb.power());
}

static void deserialise(ActionCommand::rgb &cpp, const offnao::ActionCommandAll_LED_rgb &pb) {
   cpp.red   = pb.red();
   cpp.green = pb.green();
   cpp.blue  = pb.blue();
}

static void deserialise(ActionCommand::LED &cpp, const offnao::ActionCommandAll_LED &pb) {
   deserialise(cpp.leftEar, pb.leftear());
   deserialise(cpp.rightEar, pb.rightear());
   deserialise(cpp.leftEye, pb.lefteye());
   deserialise(cpp.rightEye, pb.righteye());
   deserialise(cpp.chestButton, pb.chestbutton());
   deserialise(cpp.leftFoot, pb.leftfoot());
   deserialise(cpp.rightFoot, pb.rightfoot());
}

static void deserialise(ActionCommand::All &cpp, const offnao::ActionCommandAll &pb) {
   deserialise(cpp.head, pb.head());
   deserialise(cpp.body, pb.body());
   deserialise(cpp.leds, pb.leds());
}

static void deserialise(FootPosition &cpp, const offnao::Motion_FootPosition &pb) {
   cpp.x = pb.x();
   cpp.y = pb.y();
   cpp.theta = pb.theta();
}

static void deserialise(FeetPosition &cpp, const offnao::Motion_FeetPosition &pb) {
   deserialise(cpp.left, pb.left());
   deserialise(cpp.right, pb.right());
}

static void deserialise(MotionDebugInfo &cpp, const offnao::Motion_MotionDebugInfo &pb) {
   deserialise(cpp.feetPosition, pb.feetposition());
}

static void deserialise(Parameters &cpp, const offnao::Kinematics_Parameters &pb) {
   cpp.cameraPitchTop    = pb.camerapitchtop();
   cpp.cameraYawTop      = pb.camerayawtop();
   cpp.cameraRollTop     = pb.camerarolltop();
   cpp.cameraYawBottom   = pb.camerayawbottom();
   cpp.cameraPitchBottom = pb.camerapitchbottom();
   cpp.cameraRollBottom  = pb.camerarollbottom();
   cpp.bodyPitch         = pb.bodypitch();
}

static void deserialise(AbsCoord &cpp, const offnao::AbsCoord &pb) {
   deserialise(cpp.vec, pb.vec());
   deserialise(cpp.var, pb.var());
   cpp.weight = pb.weight();
}

static void deserialise(RRCoord &cpp, const offnao::RRCoord &pb) {
   deserialise(cpp.vec, pb.vec());
   deserialise(cpp.var, pb.var());
}

static void deserialise(SharedStateEstimationBundle &cpp, const offnao::SharedStateEstimationBundle &pb) {
   deserialise(cpp.robotPos, pb.robotpos());
   deserialise(cpp.ballPosRRC, pb.ballposrrc());
   deserialise(cpp.ballVelRRC, pb.ballvelrrc());
   cpp.haveBallUpdate = pb.haveballupdate();
}

static void deserialise(RobotObstacle &cpp, const offnao::StateEstimation_RobotObstacle &pb) {
   deserialise(cpp.rr, pb.rr());
   deserialise(cpp.type, pb.type());
   deserialise(cpp.rrc, pb.rrc());
   deserialise(cpp.pos, pb.pos());
   deserialise(cpp.tangentHeadingLeft, pb.tangentheadingleft());
   deserialise(cpp.tangentHeadingRight, pb.tangentheadingright());
   deserialise(cpp.evadeVectorLeft, pb.evadevectorleft());
   deserialise(cpp.evadeVectorRight, pb.evadevectorright());
}

static void deserialise(BehaviourRequest &cpp, const offnao::Behaviour_BehaviourRequest &pb) {
   deserialise(cpp.actions, pb.actions());
   deserialise(cpp.behaviourSharedData, pb.behaviourshareddata());
   deserialise(cpp.behaviourDebugInfo, pb.behaviourdebuginfo());
}

static void deserialise(BehaviourDebugInfo &cpp, const offnao::BehaviourDebugInfo &pb) {
   deserialise(cpp.bodyBehaviourHierarchy, pb.bodybehaviourhierarchy());
   deserialise(cpp.headBehaviourHierarchy, pb.headbehaviourhierarchy());
   deserialise(cpp.haveBallManoeuvreTarget, pb.haveballmanoeuvretarget());
   deserialise(cpp.ballManoeuvreTargetX, pb.ballmanoeuvretargetx());
   deserialise(cpp.ballManoeuvreTargetY, pb.ballmanoeuvretargety());
   deserialise(cpp.ballManoeuvreHeadingError, pb.ballmanoeuvreheadingerror());
   deserialise(cpp.ballManoeuvreType, pb.ballmanoeuvretype());
   deserialise(cpp.ballManoeuvreHard, pb.ballmanoeuvrehard());
   deserialise(cpp.anticipating, pb.anticipating());
   deserialise(cpp.anticipateX, pb.anticipatex());
   deserialise(cpp.anticipateY, pb.anticipatey());
   deserialise(cpp.anticipateH, pb.anticipateh());
}

static void deserialise(Eigen::Matrix<int, 2, 1> &cpp, const offnao::Point &pb) {
   cpp[0] = pb.x();
   cpp[1] = pb.y();
}

static void deserialise(BBox &cpp, const offnao::BBox &pb) {
   deserialise(cpp.a, pb.a());
   deserialise(cpp.b, pb.b());
}

static void deserialise(BallInfo &cpp, const offnao::Vision_BallInfo &pb) {
   deserialise(cpp.rr, pb.rr());
   deserialise(cpp.radius, pb.radius());
   deserialise(cpp.imageCoords, pb.imagecoords());
   deserialise(cpp.topCamera, pb.topcamera());
}

static void deserialise(RobotVisionInfo &cpp, const offnao::Vision_RobotVisionInfo &pb) {
   deserialise(cpp.rr, pb.rr());
   deserialise(cpp.type, pb.type());
   deserialise(cpp.cameras, pb.cameras());
   deserialise(cpp.imageCoords, pb.imagecoords());
   deserialise(cpp.topImageCoords, pb.topimagecoords());
   deserialise(cpp.botImageCoords, pb.botimagecoords());
}

static void deserialise(RANSACLine &cpp, const offnao::Vision_RANSACLine &pb) {
   deserialise(cpp.t1, pb.t1());
   deserialise(cpp.t2, pb.t2());
   deserialise(cpp.t3, pb.t3());
   deserialise(cpp.var, pb.var());
   deserialise(cpp.p1, pb.p1());
   deserialise(cpp.p2, pb.p2());
}

static void deserialise(FieldBoundaryInfo &cpp, const offnao::Vision_FieldBoundaryInfo &pb) {
   deserialise(cpp.rrBoundary, pb.rrboundary());
   deserialise(cpp.imageBoundary, pb.imageboundary());
}

static void deserialise(FieldFeatureInfo &cpp, const offnao::Vision_FieldFeatureInfo &pb) {
   deserialise(cpp.rr, pb.rr());
   deserialise(cpp.type, pb.type());
   deserialise(cpp.p1, pb.p1());
   deserialise(cpp.p2, pb.p2());
}

// making this static creates a warning in the friend declaration.  there are some GCC pragmas to squelch the warning, but they don't work in g++...
void deserialise(RegionI &cpp, const offnao::Vision_RegionI &pb) {
   deserialise(cpp.is_top_camera_, pb.is_top_camera_());
   deserialise(cpp.bounding_box_rel_, pb.bounding_box_rel_());
   deserialise(cpp.bounding_box_fovea_, pb.bounding_box_fovea_());
   deserialise(cpp.bounding_box_raw_, pb.bounding_box_raw_());
   deserialise(cpp.n_raw_cols_in_region_, pb.n_raw_cols_in_region_());
   deserialise(cpp.n_raw_rows_in_region_, pb.n_raw_rows_in_region_());
   deserialise(cpp.density_to_raw_, pb.density_to_raw_());
   deserialise(cpp.y_offset_raw_, pb.y_offset_raw_());
   deserialise(cpp.x_offset_raw_, pb.x_offset_raw_());
   deserialise(cpp.raw_total_width_, pb.raw_total_width_());
   deserialise(cpp.raw_to_fovea_density_, pb.raw_to_fovea_density_());
   deserialise(cpp.fovea_width_, pb.fovea_width_());
}

static void deserialise(SPLStandardMessage &cpp, const offnao::Receiver_SPLStandardMessage &pb) {
   deserialise(cpp.header, pb.header());
   deserialise(cpp.version, pb.version());
   deserialise(cpp.playerNum, pb.playernum());
   deserialise(cpp.teamNum, pb.teamnum());
   deserialise(cpp.fallen, pb.fallen());
   deserialise(cpp.pose, pb.pose());
   deserialise(cpp.ballAge, pb.ballage());
   deserialise(cpp.ball, pb.ball());
   deserialise(cpp.numOfDataBytes, pb.numofdatabytes());

   const string &pbField = pb.data();
   size_t       pbSize   = pbField.size();
   size_t       cppSize  = sizeof(uint8_t[SPL_STANDARD_MESSAGE_DATA_SIZE]);
   if (cppSize < pbSize)
      llog(ERROR) << "trying to deserialise " << pbSize << " bytes to a field of " << cppSize << " bytes" << endl;
   if (cpp.numOfDataBytes != pbSize)
      llog(ERROR) << "trying to deserialise " << pbSize << " bytes but numOfDataBytes is " << cpp.numOfDataBytes
                  << endl;
   memcpy(cpp.data, pbField.data(), min(cppSize, pbSize));
}

static void deserialise(BehaviourSharedData &cpp, const offnao::BehaviourSharedData &pb) {
   deserialise(cpp.role, pb.role());
   deserialise(cpp.playingBall, pb.playingball());
   deserialise(cpp.needAssistance, pb.needassistance());
   deserialise(cpp.isAssisting, pb.isassisting());
   deserialise(cpp.secondsSinceLastKick, pb.secondssincelastkick());
   deserialise(cpp.isKickedOff, pb.iskickedoff());
   deserialise(cpp.walkingToX, pb.walkingtox());
   deserialise(cpp.walkingToY, pb.walkingtoy());
   deserialise(cpp.walkingToH, pb.walkingtoh());
}

static void deserialise(BroadcastData &cpp, const offnao::Receiver_BroadcastData &pb) {
   deserialise(cpp.playerNum, pb.playernum());
   deserialise(cpp.robotPos, pb.robotpos());
   deserialise(cpp.ballPosAbs, pb.ballposabs());
   deserialise(cpp.ballPosRR, pb.ballposrr());
   deserialise(cpp.sharedStateEstimationBundle, pb.sharedstateestimationbundle());
   deserialise(cpp.behaviourSharedData, pb.behaviourshareddata());
   deserialise(cpp.acB, pb.acb());
   deserialise(cpp.uptime, pb.uptime());
   deserialise(cpp.gameState, pb.gamestate());
}

static const uint8_t *deserialise(const offnao::Blackboard &pb,
                                  int height,
                                  int width,
                                  bool (::offnao::Vision::*hasframe)() const,
                                  const string &(::offnao::Vision::*getframe)() const,
                                  bool (::offnao::Vision::*hasframejpeg)() const,
                                  const string &(::offnao::Vision::*getframejpeg)() const) {
   if ((pb.vision().*hasframe)()) {
      const string &pbField = (pb.vision().*getframe)();
      uint8_t      *frame   = new uint8_t[height * width * 2];

      size_t pbSize  = pbField.size();
      size_t cppSize = sizeof(uint8_t[height * width * 2]);

      if (cppSize < pbSize)
         llog(ERROR) << "trying to deserialise " << pbSize << " bytes to a field of " << cppSize << " bytes" << endl;
      memcpy(frame, pbField.data(), min(cppSize, pbSize));
      return frame;
   } else if ((pb.vision().*hasframejpeg)()) {
      static uint8_t                *yuvFrame = new uint8_t[height * width * 3];
      // os 2.1 has turbo jpeg 1.1.1
      // os 2.8 has turbo jpeg 1.4.2, which has some more convenient functions, supposedly
      // https://github.com/libjpeg-turbo/libjpeg-turbo/blob/1.1.x/example.c
      struct jpeg_decompress_struct cinfo;
      struct jpeg_error_mgr         jerr; // TODO: not exit on error
      JSAMPARRAY                    buffer; /* Output row buffer */
      int                           row_stride; /* physical row width in output buffer */
      cinfo.err = jpeg_std_error(&jerr);
      jpeg_create_decompress(&cinfo);
      jpeg_mem_src(&cinfo,
                   (unsigned char *) (pb.vision().*getframejpeg)().data(),
                   (pb.vision().*getframejpeg)().size());
      (void) jpeg_read_header(&cinfo, TRUE);
      cinfo.output_components = 3;
      cinfo.out_color_space   = JCS_YCbCr;
      (void) jpeg_start_decompress(&cinfo);
      row_stride = cinfo.output_width * cinfo.output_components;
      buffer     = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
      while (cinfo.output_scanline < cinfo.output_height) {
         (void) jpeg_read_scanlines(&cinfo, buffer, 1);
         memcpy(&yuvFrame[(cinfo.output_scanline - 1) * row_stride], buffer[0], row_stride);
      }
      (void) jpeg_finish_decompress(&cinfo);

      // only works with yuvyuv not yuyv
      const int     yuyvFrameSize   = height * width * 2;
      uint8_t       *yuyvFrame      = new uint8_t[yuyvFrameSize];
      const uint8_t *yuvFrameRead   = yuvFrame;
      uint8_t       *yuyvFrameWrite = yuyvFrame;
      const uint8_t *const yuyvFrameWriteEnd = yuyvFrame + yuyvFrameSize;
      do {
         // convert two pixels at a time
         yuyvFrameWrite[0] = yuvFrameRead[0];
         yuyvFrameWrite[1] = yuvFrameRead[1]; // ignore u of the second pixel
         yuyvFrameWrite[2] = yuvFrameRead[3];
         yuyvFrameWrite[3] = yuvFrameRead[2]; // ignore v of the second pixel
         yuvFrameRead += 6;
         yuyvFrameWrite += 4;
      } while (yuyvFrameWrite < yuyvFrameWriteEnd);
      jpeg_destroy_decompress(&cinfo);

      return yuyvFrame;
   }
   return NULL;
}

// the if/mask statements belong in serialise not deserialise, but we keep them here anyway so we don't need `if (!pb.vision().landmarks().empty())` or `if(pb.vision().has_timestamp())`
void Blackboard::deserialise(Blackboard &cpp, const offnao::Blackboard &pb) {
   cpp.mask = pb.mask();
   if (cpp.mask & BLACKBOARD_MASK) {
      cpp.gameController.player_number = pb.gamecontroller().player_number();
      if (pb.gamecontroller().has_our_team())
         cpp.gameController.our_team.teamNumber = pb.gamecontroller().our_team().teamnumber();

      ::deserialise(cpp.motion.sensors, pb.motion().sensors());
      ::deserialise(cpp.motion.pose, pb.motion().pose());
      ::deserialise(cpp.motion.com, pb.motion().com());
      ::deserialise(cpp.motion.odometry, pb.motion().odometry());
      ::deserialise(cpp.motion.active, pb.motion().active());
      ::deserialise(cpp.motion.jointRequest, pb.motion().jointrequest());
      ::deserialise(cpp.motion.motionDebugInfo, pb.motion().motiondebuginfo());

      cpp.perception.behaviour       = pb.perception().behaviour();
      cpp.perception.kinematics      = pb.perception().kinematics();
      cpp.perception.stateEstimation = pb.perception().stateestimation();
      cpp.perception.total           = pb.perception().total();
      cpp.perception.vision          = pb.perception().vision();

      // This request was updated for version 19 but not sure if more is required
      deserialiseArray(cpp.behaviour.request, pb.behaviour().request());

      ::deserialise(cpp.kinematics.parameters, pb.kinematics().parameters());

      if (cpp.mask & ROBOT_FILTER_MASK) {
         ::deserialise(cpp.stateEstimation.robotObstacles, pb.stateestimation().robotobstacles());
      }

      /* Only serialise the things below if WHITEBOARD_MASK is not set.
       * We also ONLY want this to happen when we are serialising in Offnao,
       * which occurs when we save the dump. WHITEBOARD_MASK can only be set
       * in the save function in offnao.
       */
      if (!(cpp.mask & WHITEBOARD_MASK)) {
         cpp.vision.timestamp = pb.vision().timestamp();
         ::deserialise(cpp.vision.balls, pb.vision().balls());
         ::deserialise(cpp.vision.robots, pb.vision().robots());
         ::deserialise(cpp.vision.fieldBoundaries, pb.vision().fieldboundaries());
         ::deserialise(cpp.vision.fieldFeatures, pb.vision().fieldfeatures());

         ::deserialise(cpp.vision.regions, pb.vision().regions());
      }

      ::deserialise(cpp.vision.topCameraSettings, pb.vision().topcamerasettings());
      ::deserialise(cpp.vision.botCameraSettings, pb.vision().botcamerasettings());

      cpp.vision.horizontalFieldOfView = pb.vision().horizontalfieldofview();
      cpp.vision.verticalFieldOfView   = pb.vision().verticalfieldofview();

      ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP = pb.vision().atwindowsizetop();
      ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT = pb.vision().atwindowsizebot();
      ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP = pb.vision().atpercentagetop();
      ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT = pb.vision().atpercentagebot();

      deserialiseArray(cpp.receiver.message, pb.receiver().message());
      deserialiseArray(cpp.receiver.data, pb.receiver().data());
      ::deserialiseWithImplicitCast(cpp.receiver.lastReceived, pb.receiver().lastreceived());
      // TODO: add the sizes to the message
      ::deserialise(cpp.receiver.incapacitated, pb.receiver().incapacitated(), pb.receiver().incapacitated_size());


      // seems to happen irregardless of everything else, so why do it twice?
//      ::deserialise(cpp.stateEstimation.robotPos, pb.stateestimation().robotpos());
      ::deserialise(cpp.stateEstimation.allRobotPos, pb.stateestimation().allrobotpos());

      ::deserialise(cpp.stateEstimation.ballPosRR, pb.stateestimation().ballposrr());
      ::deserialise(cpp.stateEstimation.ballPosRRC, pb.stateestimation().ballposrrc());
      ::deserialise(cpp.stateEstimation.ballVelRRC, pb.stateestimation().ballvelrrc());
      ::deserialise(cpp.stateEstimation.ballVel, pb.stateestimation().ballvel());
      ::deserialise(cpp.stateEstimation.ballPos, pb.stateestimation().ballpos());
      ::deserialise(cpp.stateEstimation.teamBallPos, pb.stateestimation().teamballpos());
      ::deserialise(cpp.stateEstimation.teamBallVel, pb.stateestimation().teamballvel());
      ::deserialise(cpp.stateEstimation.sharedStateEstimationBundle,
                    pb.stateestimation().sharedstateestimationbundle());
      cpp.stateEstimation.havePendingOutgoingSharedBundle = pb.stateestimation().havependingoutgoingsharedbundle();
      ::deserialise(cpp.stateEstimation.havePendingIncomingSharedBundle,
                    pb.stateestimation().havependingincomingsharedbundle(),
            // TODO: add the sizes to the message
                    pb.stateestimation().havependingincomingsharedbundle_size());
   }

   if (cpp.mask & SALIENCY_MASK) {
      // TODO(jayen): RLE
      if (pb.vision().has_topsaliency()) {
         const string &pbField = pb.vision().topsaliency();
         cpp.vision.topSaliency = (Colour *) new TopSaliency;

         size_t pbSize  = pbField.size();
         size_t cppSize = sizeof(TopSaliency);

         if (cppSize < pbSize)
            llog(ERROR) << "trying to deserialise " << pbSize << " bytes to a field of " << cppSize << " bytes" << endl;
         memcpy(cpp.vision.topSaliency, pbField.data(), min(cppSize, pbSize));
      }
      if (pb.vision().has_botsaliency()) {
         const string &pbField = pb.vision().botsaliency();
         cpp.vision.botSaliency = (Colour *) new BotSaliency;

         size_t pbSize  = pbField.size();
         size_t cppSize = sizeof(BotSaliency);

         if (cppSize < pbSize)
            llog(ERROR) << "trying to deserialise " << pbSize << " bytes to a field of " << cppSize << " bytes" << endl;
         memcpy(cpp.vision.botSaliency, pbField.data(), min(cppSize, pbSize));
      }
   }
   if (cpp.mask & RAW_IMAGE_MASK) {
      cpp.vision.topFrame = ::deserialise(pb,
                                          TOP_IMAGE_ROWS,
                                          TOP_IMAGE_COLS,
                                          &::offnao::Vision::has_topframe,
                                          &::offnao::Vision::topframe,
                                          &::offnao::Vision::has_topframejpeg,
                                          &::offnao::Vision::topframejpeg);
      cpp.vision.botFrame = ::deserialise(pb,
                                          BOT_IMAGE_ROWS,
                                          BOT_IMAGE_COLS,
                                          &::offnao::Vision::has_botframe,
                                          &::offnao::Vision::botframe,
                                          &::offnao::Vision::has_botframejpeg,
                                          &::offnao::Vision::botframejpeg);
   }

   ::deserialise(cpp.stateEstimation.robotPos, pb.stateestimation().robotpos());
}

void Blackboard::deserialise(istream &is) {
   offnao::Blackboard bb;

   // https://developers.google.com/protocol-buffers/docs/techniques#streaming
   unsigned int size;
   is.read(reinterpret_cast<char *>(&size), sizeof(size));
   char *array = new char[size];
   is.read(array, size);
   bb.ParseFromArray(array, size);

   deserialise(*this, bb);
}
