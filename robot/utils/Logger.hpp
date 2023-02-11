#pragma once

#include <string>
#include <ostream>
#ifndef Q_MOC_RUN
#include <boost/program_options/variables_map.hpp>
#endif

#define llog(X) ((Logger::instance())->realLlog(X))
#define llog_open(X) ((Logger::instance())->realLlog(X, 1))
#define llog_middle(X) ((Logger::instance())->realLlog(X, 0))
#define llog_close(X) ((Logger::instance())->realLlog(X, -1))

/**
 * Possible log levels
 * When something is to be logged an associated level is passed with it. Only
 * messages with levels greater than logLevel are actually logged at runtime.
 * Levels should be set to an appropriate point, so that selecting a logLevel
 * such as INFO gives a corresponding level of logging.
 * A message is FATAL if the robot cannot function in its current state.
 * An ERROR, while still allowing a robot to run, greatly impairs its ability
 * to function accurately or as intended.
 * INFO gives useful, concise summaries of the robot's functioning.
 * VERBOSE details at length every step that INFO summarises.
 * DEBUGx gives multiple levels of debugging, for the coder's pleasure.
 * @see logLevel
 */
enum LogLevel {
   NONE  = -200,
   SILENT  = -100,
   QUIET   = -67,
   FATAL   = -33,
   ERROR   = 0,
   WARNING = 10,
   INFO    = 20,
   VERBOSE = 40,
   DEBUG   = 60,
   DEBUG1  = 60,
   DEBUG2  = 80,
   DEBUG3  = 100
};

class Logger {
   public:
      Logger(const char *name);
      virtual ~Logger();
      static void init(std::string logPath, std::string logLevel, bool motion, std::string logOutput);
      static Logger *instance();
      std::ostream &realLlog(int logLevel);
      std::ostream &realLlog(int logLevel, int indentInc);
      static void readOptions(const boost::program_options::variables_map &config);
      // so we can add more files to it
      static std::string getLogDir();

   private:
      static int indentLevel;
      static void init(std::string logLevel, bool motion);
      static __thread Logger *logger;
      static enum LogLevel logLevel;
      static bool motion;
      static std::string logOutput;
      static std::string logPath;
      static bool initialised;
      std::ostream *logStream;
};
