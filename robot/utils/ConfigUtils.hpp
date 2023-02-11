#ifndef CONFIG_UTILS_HPP
#define CONFIG_UTILS_HPP

#include <boost/function.hpp>
#include <string>
#include <iostream>

/*
Gets the hostname, which on a nao is also the name of the head.
*/
inline std::string getNaoHeadName()
{
    std::string headName = "";
    char buffer[HOST_NAME_MAX] = {0};
    gethostname(buffer, HOST_NAME_MAX);
    // Not sure why this is needed, but strange things happen if you use assign.
    for(int i=0; (int)buffer[i] != 0; ++i)
        headName += buffer[i];
    return(headName);
}

/*
Overwrites or adds a single entry in a config file.
*/
void overwriteEntry(std::string targetSection, std::string targetEntry, std::string newValue, bool bodyConfig=false,
                                                                                               std::string bodyName="");

#endif // CONFIG_UTILS_HPP
