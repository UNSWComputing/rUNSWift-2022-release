#include <boost/function.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>

#include "types/CleverNaoInfo.hpp"

#include "utils/ConfigUtils.hpp"

/*
Creates a directory to save the next set of data in and returns the path to that directory.
*/
std::string CleverNaoInfoBehaviour::getNextDataDirectory(const std::string& bodyName)
{
    // The directory in which data is to be saved.
    std::string saveDir;

    // The ID of this entry.
    int entryID = 0;
    std::string entryIDString;

    // A string stream.
    std::stringstream conversionStream;

    // Data types for iterating through directories.
    DIR *dir;
    struct dirent *entry;

    // Get the directory to save data in.
    saveDir = "/home/nao/data/recordings";
    mkdir(saveDir.c_str(), ACCESSPERMS);
    saveDir += "/" + getNaoHeadName() + "With" + bodyName;
    mkdir(saveDir.c_str(), ACCESSPERMS);

    // Determine the ID of this set of recordings.
    dir = opendir(saveDir.c_str());
    entry = readdir(dir);
    while(entry != NULL)
    {
        if(entry->d_type == DT_DIR && atoi(entry->d_name) >= entryID)
            entryID = atoi(entry->d_name)+1;
        entry = readdir(dir);
    }
    closedir(dir);

    // Add the entry ID to the path.
    conversionStream << entryID;
    conversionStream >> entryIDString;
    saveDir += "/" + entryIDString;
    mkdir(saveDir.c_str(), ACCESSPERMS);

    // Return the path to the data directory.
    return(saveDir);
}

/*
Save the data recorded in CleverNaoInfoBehaviour to the directory provided.
*/
void CleverNaoInfoBehaviour::saveDataToDirectory(const CleverNaoInfoMotion& cleverNaoInfoMotion,
                                                                                             const std::string& saveDir)
{
    // The file currently being output to.
    std::ofstream curFileOut;

    // Open a file to save overall data in.
    curFileOut.open((saveDir + "/globalData.txt").c_str(), std::ofstream::out);
    curFileOut << "dict" << std::endl;  // Dictionary of ": " separated name value pairs.

    // Save whether CleverNao was being used to set behaviours.
    curFileOut << "cleverNaoBehaviour: " << cleverNaoBehaviour << std::endl;

    // Close the overall data file.
    curFileOut.close();

    // Save a file for the gyro readings.
    curFileOut.open((saveDir + "/gyroReadingsX.txt").c_str(), std::ofstream::out);
    curFileOut << "1df" << std::endl;   // 1 dimension of floats.
    for(unsigned int reading=0; reading < cleverNaoInfoMotion.gyroReadingsX.size(); ++reading)
        curFileOut << cleverNaoInfoMotion.gyroReadingsX[reading] << std::endl;
    curFileOut.close();

    // The CoM calculations, recorded for Clever Nao.
    curFileOut.open((saveDir + "/CoM.txt").c_str(), std::ofstream::out);
    curFileOut << "2df" << std::endl;   // 2 dimensions of floats.
    for(unsigned int reading=0; reading < cleverNaoInfoMotion.CoM.size(); ++reading)
    {
        curFileOut << cleverNaoInfoMotion.CoM[reading].x << "\t" << cleverNaoInfoMotion.CoM[reading].y << "\t" <<
                                                                        cleverNaoInfoMotion.CoM[reading].z << std::endl;
    }
    curFileOut.close();

    // Save a file for the ball pos readings.
    curFileOut.open((saveDir + "/rrBallPos.txt").c_str(), std::ofstream::out);
    curFileOut << "2df" << std::endl;   // 2 dimensions of floats.
    for(unsigned int reading=0; reading < rrBallPos.size(); ++reading)
        curFileOut << rrBallPos[reading].x() << "\t" << rrBallPos[reading].y() << std::endl;
    curFileOut.close();
}

/*
Save the data recorded in CleverNaoInfoBehaviour.
*/
void CleverNaoInfoBehaviour::saveData(const std::string& bodyName, const CleverNaoInfoMotion& cleverNaoInfoMotion)
{
    // The directory in which data is to be saved.
    std::string saveDir;

    // Create and get the path to the dircectory in which data is to be saved.
    saveDir = getNextDataDirectory(bodyName);

    // Save the data to the created directory.
    saveDataToDirectory(cleverNaoInfoMotion, saveDir);
}
