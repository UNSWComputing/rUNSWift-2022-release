#include "jointsTab.hpp"
#include <QGridLayout>
#include "tabs/plots.hpp"
#include "types/SensorValues.hpp"
#include "naoData.hpp"

#define COLS 5

using namespace std;

JointsTab::JointsTab(QTabWidget *parent, QMenuBar *menuBar,
                     Vision *vision)
{
    initMenu(menuBar);
    init();
    this->vision = vision;
    this->parent = parent;
}

void JointsTab::initMenu(QMenuBar *)
{
}

void JointsTab::init()
{

    layout = new QGridLayout(this);
    setLayout(layout);
    layout->setAlignment(layout, Qt::AlignTop);
    layout->setHorizontalSpacing(5);
    layout->setVerticalSpacing(5);

    last_frame = -1;

    for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
    {
        jointPlots[i] = new MultiPlot(this, Joints::jointNames[i], 2, RAD2DEG(Joints::Radians::MinAngle[i]), RAD2DEG(Joints::Radians::MaxAngle[i]));
        layout->addWidget(jointPlots[i], i / COLS, i % COLS);
    }
}

void JointsTab::updatePlots(SensorValues &s, JointValues &jr, bool left)
{
    for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
    {
        std::vector<float> vals;
        vals.push_back(RAD2DEG(s.joints.angles[i]));
        vals.push_back(RAD2DEG(jr.angles[i]));
        jointPlots[i]->push(vals, left);
    }
}

void JointsTab::updatePlots(std::vector<SensorValues> &s, std::vector<JointValues> &jr, bool left)
{
    if (left)
        for (int i = s.size() - 1; i > 0; i--)
        {
            updatePlots(s[i], jr[i], left);
        }
    else
        for (u_int i = 0; i < s.size(); i++)
        {
            updatePlots(s[i], jr[i], left);
        }
}

void JointsTab::newNaoData(NaoData *naoData)
{

    if (!naoData || !naoData->getCurrentFrame().blackboard)
    {
        // clean up display, as read is finished
    }
    else if (naoData->getFramesTotal() != 0)
    {
        int new_frame = naoData->getCurrentFrameIndex();
        if (new_frame == last_frame + 1)
        {
            // special case for one frame at a time
            Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
            SensorValues s = readFrom(motion, sensors);
            JointValues jr = readFrom(motion, jointRequest);
            updatePlots(s, jr);
        }
        else if (ABS(new_frame - last_frame) > PLOT_SIZE)
        {
            // scrap all data and pass in new array
            std::vector<SensorValues> s;
            std::vector<JointValues> jr;
            SensorValues emptyS;
            JointValues emptyJR;
            if (new_frame < PLOT_SIZE - 1)
                for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
                    emptyS.sensors[i] = 0.0f;
            for (int i = new_frame - PLOT_SIZE + 1; i <= new_frame; ++i)
            {
                if (i >= 0)
                {
                    s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
                    s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
                    jr.push_back(naoData->getFrame(i).blackboard->motion.jointRequest);
                    jr.push_back(naoData->getFrame(i).blackboard->motion.jointRequest);
                }
                else
                {
                    s.push_back(emptyS);
                    jr.push_back(emptyJR);
                }
            }
            updatePlots(s, jr);
        }
        else if (new_frame < last_frame)
        {
            // push some new data to the front of graph
            std::vector<SensorValues> s;
            std::vector<JointValues> jr;
            SensorValues emptyS;
            JointValues emptyJR;
            if (new_frame < PLOT_SIZE - 1)
                for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
                    emptyS.sensors[i] = 0.0f;
            for (int i = new_frame - PLOT_SIZE + 1; i <= new_frame; ++i)
            {
                if (i >= 0)
                {
                    s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
                    jr.push_back(naoData->getFrame(i).blackboard->motion.jointRequest);
                }
                else
                {
                    s.push_back(emptyS);
                    jr.push_back(emptyJR);
                }
            }

            updatePlots(s, jr, true);
        }
        else if (new_frame > last_frame)
        {
            // push some new data to the end of graph
            std::vector<SensorValues> s;
            for (int i = last_frame + 1; i <= new_frame; ++i)
            {
                s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
            }

            std::vector<JointValues> jr;
            for (int i = last_frame + 1; i <= new_frame; ++i)
            {
                jr.push_back(naoData->getFrame(i).blackboard->motion.jointRequest);
            }
            updatePlots(s, jr);
        }
        last_frame = new_frame;
    }
}

void JointsTab::readerClosed()
{
}
