#include "Logger.hpp"

#include <stdexcept>
#include <string>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <ostream>
#include <map>
#include "thread/Thread.hpp"
#include "utils/basic_onullstream.hpp"

Logger::Logger(const char *name) {
   if (!initialised) {
      throw std::runtime_error("Logging framework not initialized.");
   } else {
      // Replace motion logstreams with a nullstream, so motion won't make
      // syscalls. We do this after making an ofstream so the log files still
      // get wiped, so noone gets confused.
      if (name == NULL) {
         logStream = &std::cerr;
      } else if (!motion && (strcmp(name, "Motion") == 0)) {
         logStream = new onullstream;
      } else if (logOutput == "CERR") {
         logStream = &std::cerr;
      } else {
         logStream = new std::ofstream((logPath + std::string("/") + std::string(name)).c_str(), std::ios_base::out);
      }
   }
}

Logger::~Logger() {
   if (logStream != &std::cerr) {
      delete logStream;
   }
   logStream = NULL;
}

void Logger::init(std::string logLevel_, bool motion_) {
   std::map<std::string, LogLevel> logLevels;
   logLevels["NONE"] = NONE;
   logLevels["SILENT"] = SILENT;
   logLevels["QUIET"] = QUIET;
   logLevels["FATAL"] = FATAL;
   logLevels["ERROR"] = ERROR;
   logLevels["WARNING"] = WARNING;
   logLevels["INFO"] = INFO;
   logLevels["VERBOSE"] = VERBOSE;
   // logLevels["DEBUG"] = DEBUG;
   logLevels["DEBUG1"] = DEBUG1;
   logLevels["DEBUG2"] = DEBUG2;
   logLevels["DEBUG3"] = DEBUG3;
   logLevel = logLevels[logLevel_];
   motion = motion_;
}

void Logger::init(std::string logPath_, std::string logLevel_, bool motion_, std::string logOutput_) {
   init(logLevel_, motion_);
   logPath = logPath_;
   logOutput = logOutput_;
   system((std::string("/bin/mkdir -p ") + logPath).c_str());
   system((std::string("/bin/ln -sfT ") + logPath + " " + logPath + "/../latest").c_str());
   initialised = true;
}

void Logger::readOptions(const boost::program_options::variables_map &config) {
   Logger::init(config["debug.log"].as<std::string>(),
                config["debug.log.motion"].as<bool>());
}

std::string Logger::getLogDir() {
   return logPath;
}

Logger *Logger::instance() {
   if (logger == NULL) {
      logger = new Logger(Thread::name);
   }
   return logger;
}

std::ostream &Logger::realLlog(int logLevel_) {
   if (logLevel >= logLevel_) {
      return *logStream;
   } else {
      static onullstream nullStream;
      return nullStream;
   }
}

std::ostream &Logger::realLlog(int logLevel_, int indentInc_) {
   indentLevel = std::min(0, indentLevel + indentInc_);
   for (int i = 0; i < indentLevel; ++i) {
      *logStream << " ";
   }
   //if (indentInc_ > 0) *logStream << "BEGIN: ";
   //if (indentInc_ < 0) *logStream << "END: ";
   return Logger::realLlog(logLevel_);
}

__thread Logger *Logger::logger = NULL;
bool Logger::initialised = false;
bool Logger::motion;
std::string Logger::logOutput;
int Logger::indentLevel = 0;
std::string Logger::logPath;
enum LogLevel Logger::logLevel;
