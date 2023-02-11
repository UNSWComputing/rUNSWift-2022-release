#include "temperatureTab.hpp"
#include "tabs/plots.hpp"
#include "naoData.hpp"

#define COLS 5
#define MIN_TEMPERATURE 20
#define MAX_TEMPERATURE 100

using namespace std;

TemperatureTab::TemperatureTab(QTabWidget *parent, QMenuBar *menuBar,
                     Vision *vision)
{
    initMenu(menuBar);
    init();
    this->vision = vision;
    this->parent = parent;
}

void TemperatureTab::initMenu(QMenuBar *)
{
}

void TemperatureTab::init()
{

    layout = new QGridLayout(this);
    setLayout(layout);
    layout->setAlignment(layout, Qt::AlignTop);
    layout->setHorizontalSpacing(5);
    layout->setVerticalSpacing(5);

    last_frame = -1;

    for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
    {
        temperaturePlots[i] = new MultiPlot(this, Joints::jointNames[i], 1, MIN_TEMPERATURE, MAX_TEMPERATURE);
        layout->addWidget(temperaturePlots[i], i / COLS, i % COLS);
    }
}

void TemperatureTab::updatePlots(SensorValues &s, bool left)
{
    for (unsigned i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
    {
        std::vector<float> vals;
        vals.push_back(s.joints.temperatures[i]);
        temperaturePlots[i]->push(vals, left);
    }
}

void TemperatureTab::updatePlots(std::vector<SensorValues> &s, bool left)
{
    if (left)
        for (int i = s.size() - 1; i > 0; i--)
        {
            updatePlots(s[i], left);
        }
    else
        for (u_int i = 0; i < s.size(); i++)
        {
            updatePlots(s[i], left);
        }
}

void TemperatureTab::newNaoData(NaoData *naoData)
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
            updatePlots(s);
        }
        else if (ABS(new_frame - last_frame) > PLOT_SIZE)
        {
            // scrap all data and pass in new array
            std::vector<SensorValues> s;
            SensorValues emptyS;
            if (new_frame < PLOT_SIZE - 1)
                for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
                    emptyS.sensors[i] = 0.0f;
            for (int i = new_frame - PLOT_SIZE + 1; i <= new_frame; ++i)
            {
                if (i >= 0)
                {
                    s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
                    s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
                }
                else
                {
                    s.push_back(emptyS);
                }
            }
            updatePlots(s);
        }
        else if (new_frame < last_frame)
        {
            // push some new data to the front of graph
            std::vector<SensorValues> s;
            SensorValues emptyS;
            if (new_frame < PLOT_SIZE - 1)
                for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
                    emptyS.sensors[i] = 0.0f;
            for (int i = new_frame - PLOT_SIZE + 1; i <= new_frame; ++i)
            {
                if (i >= 0)
                {
                    s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
                }
                else
                {
                    s.push_back(emptyS);
                }
            }

            updatePlots(s, true);
        }
        else if (new_frame > last_frame)
        {
            // push some new data to the end of graph
            std::vector<SensorValues> s;
            for (int i = last_frame + 1; i <= new_frame; ++i)
            {
                s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
            }
            updatePlots(s);
        }
        last_frame = new_frame;
    }
}

void TemperatureTab::readerClosed()
{
}
