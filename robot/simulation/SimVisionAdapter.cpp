#include "SimVisionAdapter.hpp"
#include "Observation.hpp"
#include "FieldFeature.hpp"

#include "blackboard/Blackboard.hpp"

#include "perception/vision/camera/CombinedCamera.hpp"
#include "types/CombinedCameraSettings.hpp"
#include "perception/vision/VisionDefinitions.hpp"

#include <utility>

namespace Simulation {

    SimVisionAdapter::SimVisionAdapter(Blackboard* bb)
        : Adapter(bb)
    {
        // TODO @jez find a more elegant solution than this workaround
        //
        // Here we write an initial vision timestamp on construction.
        //
        // The problem is that we don't start writing to the blackboard
        // until the simulation connection is established. While we are
        // connecting we wait 2 seconds for the robot to settle (it falls
        // from the sky).
        //
        // During this 2 seconds, localisation is reading from the blackboard
        // and its reading the default vision timestamp of 0. As such, when we
        // write the first actual vision timestamp it's huge increase from 0 and
        // localisation thinks we've passed hours (?) of time and all of our
        // estimations become very uncertain due to decay (stdvar is MASSIVE).
        //
        // Workaround: write intial vision timestamp before connection takes
        // place.
        acquireLock(serialization);
        struct timeval tv;
        gettimeofday(&tv, 0);
        int64_t vision_timestamp = tv.tv_sec * 1000000ull + tv.tv_usec;
        writeTo(vision, timestamp, vision_timestamp);
        releaseLock(serialization);

        limitVisionRange = bb->config["simulation.limit_vision_range"].as<bool>();
        if (limitVisionRange)
        {
            visionRange = bb->config["simulation.vision_range"].as<float>();
        } else
        {
            visionRange = std::numeric_limits<float>::infinity();
        }
    }

    RRCoord SimVisionAdapter::getRR(Observation& o, double head_pitch, double head_yaw)
    {
        // We calculate ball distance from FEET in mm
        //   - o.pol[0] is distance from the HEAD.
        //   - o.pol[2] is vertical angle from HEAD to ball
        double dist = o.pol[0] * cos(DEG2RAD(o.pol[2] + head_pitch)) * 1000; // This assumes nao is standing upright.

        // Find heading using heading of ball from camera plus head yaw
        double heading = DEG2RAD(o.pol[1] + head_yaw);       // Looks like sim is in degrees, RR uses radians

        return RRCoord(dist, heading);
    }

    RRCoord SimVisionAdapter::getRR(FieldFeature& ff, double head_pitch, double head_yaw)
    {
        // We calculate field feature distance from FEET in mm
        //   - ff.ffs[0] is distance from the HEAD.
        //   - ff.ffs[2] is vertical angle from HEAD to field feature
        double dist = ff.ffs[0] * cos(DEG2RAD(ff.ffs[2] + head_pitch)) * 1000; // This assumes nao is standing upright.

        // Find heading using heading of ball from camera plus head yaw
        double heading = DEG2RAD(ff.ffs[1] + head_yaw);       // Looks like sim is in degrees, RR uses radians

        return RRCoord(dist, heading);
    }


    BallInfo SimVisionAdapter::handleBall(Observation& o, RRCoord rr)
    {
        // Behaviours moves the head depending on the position of the ball
        // in the image. So when we see the ball we have to simulate its
        // spot in the camera frame.
        //
        // Move to right spot in image (at least X coord) (640 is width)
        // We want this to be around 320
        // NOTE image coords depend on camera selection (top/bottom)
        Point image_coords((640/2.0) - o.pol[1]*10, 1356.0);

        BallInfo ball(rr, 5, image_coords);
        ball.topCamera = 0;
        return ball;
    }

    RobotVisionInfo SimVisionAdapter::handleRobot(Observation& o, RRCoord rr)
    {
        RobotVisionInfo robot(rr, RobotVisionInfo::rUnknown, BBox(Point(0,0), Point(0,0)), RobotVisionInfo::TOP_CAMERA); // The bounding box, we don't care
        return robot;
    }

    FieldFeatureInfo SimVisionAdapter::handleFieldFeature(FieldFeature& ff, RRCoord rr)
    {
        rr.setOrientation(DEG2RAD(ff.ffs[3]));

        if (ff.type==FieldFeature::CORNER) return FieldFeatureInfo(rr, FieldFeatureInfo::fCorner);
        else if (ff.type==FieldFeature::T_JUNCTION) return FieldFeatureInfo(rr, FieldFeatureInfo::fTJunction);
        else if (ff.type==FieldFeature::CENTRE_CIRCLE) return FieldFeatureInfo(rr, FieldFeatureInfo::fCentreCircle);
        else return FieldFeatureInfo();
    }


    void SimVisionAdapter::tick(const PerceptorInfo& perceived)
    {
        acquireLock(serialization);
        struct timeval tv;
        gettimeofday(&tv, 0);
        int64_t vision_timestamp = tv.tv_sec * 1e6 + tv.tv_usec;

        double head_yaw = perceived.joints.hj1;
        double head_pitch = perceived.joints.hj2;

        // Do camera frame
        for (unsigned int i = 0; i < s_num_frame_bytes_top; ++i)
        {
            frameTop[i] = (uint8_t) i;
        }
        for (unsigned int i = 0; i < s_num_frame_bytes_bot; ++i)
        {
            frameBot[i] = (uint8_t) i;
        }
        writeTo(vision, topFrame, (const uint8_t*)frameTop);
        writeTo(vision, botFrame, (const uint8_t*)frameBot);

        // Do saliency image (just make a noise image)
        for (unsigned int i=0U; i < s_num_saliency_pixels_top; ++i)
            saliencyTop[i] = (Colour)(i % cNUM_COLOURS);
        for (unsigned int i=0U; i < s_num_saliency_pixels_bot; ++i)
            saliencyBot[i] = (Colour)(i % cNUM_COLOURS);
        writeTo(vision, topSaliency, (Colour*)&saliencyTop);
        writeTo(vision, botSaliency, (Colour*)&saliencyBot);

        CombinedCameraSettings settings;
        writeTo(vision, topCameraSettings, settings.top_camera_settings);
        writeTo(vision, botCameraSettings, settings.bot_camera_settings);

        // Do vision info
        std::vector<BallInfo> sim_balls;
        std::vector<FieldFeatureInfo> sim_field_features;
        std::vector<RobotVisionInfo> sim_robots;

        // Handle observations
        for (unsigned int i=0; i < perceived.observations.size(); ++i)
        {
            Observation o = perceived.observations[i];
            RRCoord rr = getRR(o, head_pitch, head_yaw);

            if (o.type == Observation::BALL)
            {
                if (rr.distance() <= visionRange)
                {
                    sim_balls.push_back(handleBall(o, rr));
                }
            }
            else if (o.type == Observation::GOAL)
            {
                // Uncomment this if we have goal post detection (commented out since we don't)
                // sim_posts.push_back(handlePost(o, rr));
            }
            else if (o.type == Observation::PLAYER)
            {
                if (rr.distance() <= visionRange)
                {
                    sim_robots.push_back(handleRobot(o, rr));
                }
            }
        }

        // Handle fieldfeatures
        for (unsigned int i=0; i < perceived.fieldFeatures.size(); ++i)
        {
            FieldFeature ff = perceived.fieldFeatures[i];
            RRCoord rr = getRR(ff, head_pitch, head_yaw);

            if (rr.distance() <= visionRange)
            {
                sim_field_features.push_back(handleFieldFeature(ff, rr));
            }
        }

        writeTo(vision, timestamp, vision_timestamp);

        // TODO debug
        writeTo(vision, balls,          sim_balls);
        writeTo(vision, fieldFeatures,  sim_field_features);
        //writeTo(vision, fieldBoundaries,std::vector<FieldBoundaryInfo>());
        writeTo(vision, robots, sim_robots);
        releaseLock(serialization);

    }

};
