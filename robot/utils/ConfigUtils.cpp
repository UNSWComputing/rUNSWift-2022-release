#include "utils/ConfigUtils.hpp"

#include <boost/function.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>

/*
Overwrites or adds a single entry in a config file.
*/
void overwriteEntry(std::string targetSection, std::string targetEntry, std::string newValue, bool bodyConfig,
                                                                                                   std::string bodyName)
{
    // Buffer.
    char buffer[HOST_NAME_MAX] = {0};

    // The path/name of the configuration file.
    std::string name;
    std::string namecfg;
    std::string nametemp;

    // The input and temporary output files.
    std::ifstream config;
    std::ofstream tempConfig;

    // The current line in the file.
    std::string line;

    // Whether we're in the configuration section that should be changed.
    bool inSection = false;

    // Whether the configuration section that should be changed was found.
    bool foundSection = false;

    // Whether the entry has been changed.
    bool entryChanged = false;

    // Add the "=" to the target entry.
    targetEntry += "=";

    // Add the "[" and "]" to the target section.
    targetSection = "[" + targetSection + "]";

    // Get the nao's name.
    if(bodyConfig)
        name = "/home/nao/data/configs/body/" + bodyName;
    else
        name = "/home/nao/data/configs/" + getNaoHeadName();

    // Open the config file.
    strcpy(buffer, name.c_str());
    strcat(buffer, ".cfg");
    namecfg = std::string(buffer);
    strcpy(buffer, name.c_str());
    strcat(buffer, ".temp");
    nametemp = std::string(buffer);
    config.open(namecfg.c_str());
    tempConfig.open(nametemp.c_str(), std::ofstream::out);

    // Check the files were opened successfully.
    if(!tempConfig.is_open())
        std::cout << "Could not save configuration: Error creating temp file." << std::endl;
    else if(!config.is_open())
        std::cout << "No config found. Creating a new one." << std::endl;

    // Copy the file, editing the relevant lines.
    while(std::getline(config, line))
    {
        // If we're in the correct section change all the options.
        if(inSection)
        {
            // Look for characters indicating a possible end to the section.
            if(line[0] == '[' || line[0] == '\n')
            {
                // If another flag is found we're out of the section.
                if(line[0] == '[')
                    inSection = false;

                // If the entry hasn't been changed already add it here.
                if(!entryChanged)
                {
                    tempConfig << targetEntry << newValue << std::endl;
                    entryChanged = true;
                }
            }

            // If this is the target entry, overwrite it.
            if(strncmp(line.c_str(), targetEntry.c_str(), targetEntry.length()) == 0)
            {
                tempConfig << targetEntry << newValue << std::endl;
                entryChanged = true;
            }

            // This is not the target entry, so just copy the line across.
            else
                tempConfig << line << std::endl;
        }

        // Otherwise keep searching for the target section.
        else
        {
            // Check if this line is the header for the relevant section.
            if(line == targetSection)
            {
                inSection = true;
                foundSection = true;
            }

            // Copy the existing line across.
            tempConfig << line << std::endl;
        }
    }

    // If the relevant section hasn't been found, add it.
    if(!foundSection)
    {
        tempConfig << targetSection << std::endl;
        tempConfig << targetEntry << newValue << std::endl;
    }
    else if(!entryChanged)
        tempConfig << targetEntry << newValue << std::endl;

    // Close the files.
    config.close();
    tempConfig.close();

    // Rename the temporary file.
    remove(namecfg.c_str());
    rename(nametemp.c_str(), namecfg.c_str());

    // Output that the change was saved.
    std::cout << "Saved " << targetEntry << newValue << " to " << namecfg << std::endl;
}
