SET(AGENT_SRCS
        libagent/libagent.cpp
        libagent/libagent.hpp
        libagent/AgentData.hpp
        thread/Thread.cpp
        utils/options.cpp
        utils/Logger.cpp
        utils/Timer.cpp
        types/ButtonPresses.cpp
        )
add_library(agent MODULE ${AGENT_SRCS})
