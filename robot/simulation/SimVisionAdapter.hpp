#ifndef SIMULATION_SIMVISIONADAPTER_H_
#define SIMULATION_SIMVISIONADAPTER_H_

#include "blackboard/Adapter.hpp"
#include "PerceptorInfo.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "types/BallInfo.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/RobotVisionInfo.hpp"

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#endif

#include <vector>

class Blackboard;

namespace Simulation
{
    class SimVisionAdapter : Adapter
    {
    public:

        /**
         * Constructor
         *
         * @param bb The shared Blackboard instance
         */
        SimVisionAdapter(Blackboard* bb);

        /**
         * Ticks the simulated vision adapter with the vision information
         * received from the simulator.
         *
         * @param perceived Information perceived by the simulated robot.
         */
        void tick(const PerceptorInfo& perceived);

        /**
         * Converts an observation position from the robot camera to robot
         * relative coordinates from the feet.
         *
         * @param o The Observation to find the RRCoord for.
         * @param head_yaw The head yaw joint value at the time of the observation
         * @return RRCoord Robot relative coordinates of the observation.
         */
        static RRCoord getRR(Observation& o, double head_pitch, double head_yaw);
        static RRCoord getRR(FieldFeature& o, double head_pitch, double head_yaw);

    private:
        BallInfo handleBall(Observation& o, RRCoord rr); /**< Convert form observation to BallInfo */
        RobotVisionInfo handleRobot(Observation& o, RRCoord rr); /**< Convert form observation to RobotVisionInfo */
        FieldFeatureInfo handleFieldFeature(FieldFeature& ff, RRCoord rr); /**< Convert from FieldFeature to FieldFeatureInfo */
        RobotVisionInfo handleRobotVisionInfo(Observation& o, RRCoord rr); /**< Convert from observation to RobotVisionInfo */

        // static buffers so we don't end up:
        // a) using a local (function scope) buffer that is on the stack
        // b) call new on every tick() without a delete
        static const size_t s_num_frame_bytes_top = TOP_IMAGE_ROWS * TOP_IMAGE_COLS * 2;
        static const size_t s_num_frame_bytes_bot = BOT_IMAGE_ROWS * BOT_IMAGE_COLS * 2;
        uint8_t frameTop[s_num_frame_bytes_top];
        uint8_t frameBot[s_num_frame_bytes_bot];
        static const size_t s_num_saliency_pixels_top =sizeof(TopSaliency) / sizeof(Colour);
        static const size_t s_num_saliency_pixels_bot =sizeof(BotSaliency) / sizeof(Colour);
        Colour saliencyTop[s_num_saliency_pixels_top];
        Colour saliencyBot[s_num_saliency_pixels_bot];

        bool limitVisionRange;
        float visionRange;
    };
}

#endif // SIMULATION_SIMVISIONADAPTER_H_


