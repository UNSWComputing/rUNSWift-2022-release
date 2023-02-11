#include "PerceptionDumper.hpp"

#include "blackboard/Blackboard.hpp"

PerceptionDumper::PerceptionDumper(const char *path)
   : path(path), of(path)
{
}

PerceptionDumper::~PerceptionDumper()
{
}

const std::string &PerceptionDumper::getPath() const
{
   return path;
}

void PerceptionDumper::dump(Blackboard *blackboard)
{
   blackboard->serialise(of);
}
