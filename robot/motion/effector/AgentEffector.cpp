#include "motion/effector/AgentEffector.hpp"
#include <sys/mman.h>        /* For shared memory */
#include <fcntl.h>           /* For O_* constants */
#include <stdexcept>
#include "utils/Logger.hpp"
#include "utils/speech.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

/*-----------------------------------------------------------------------------
 * Agent effector constructor
 *---------------------------------------------------------------------------*/
AgentEffector::AgentEffector(int team, int player_number, bool simulation) {

   std::string mem_path(AGENT_MEMORY);
   if (simulation) {
      // If we're running a simulator build, modify the memory path so we don't
      // use the same memory as another instance of the sim build running at
      // the same time
      int mod = (team * MAX_NUM_PLAYERS) + player_number;
      std::stringstream ss;
      ss << mod;
      mem_path += ss.str();
   }

   // open shared memory as RW
   shared_fd = shm_open(mem_path.c_str(), O_CREAT | O_RDWR, 0600);
   if (shared_fd < 0) {
      throw std::runtime_error("AgentEffector: shm_open() failed");
   }
   // map shared memory to process memory
   shared_data = (AgentData*) mmap(NULL, sizeof(AgentData),
                                   PROT_READ | PROT_WRITE,
                                   MAP_SHARED, shared_fd, 0);
   if (shared_data == MAP_FAILED) {
      throw std::runtime_error("AgentEffector: mmap() failed");
   }

   llog(INFO) << "AgentEffector constructed" << std::endl;
}

/*-----------------------------------------------------------------------------
 * Agent effector destructor
 *---------------------------------------------------------------------------*/
AgentEffector::~AgentEffector() {
   if (shared_data != MAP_FAILED) {
      munmap(shared_data, sizeof(AgentData));
   }
   if (shared_fd >= 0) {
      close(shared_fd);
   }
   llog(INFO) << "AgentEffector destroyed" << std::endl;
}

/*-----------------------------------------------------------------------------
 * Agent effector - actuate the joints to the desired position
 *---------------------------------------------------------------------------*/
void AgentEffector::actuate(JointValues joints, ActionCommand::LED leds,
                            ActionCommand::Stiffen stiffen) {
   static bool kill_standing = false;
   uint8_t i;
   // Find the right index to write to
   // Roger: this should have been actually swapping between slots right?
   for (i = 0; i != shared_data->actuators_latest &&
        i != shared_data->actuators_read; ++i) ;
   shared_data->leds[i] = leds;
   shared_data->joints[i] = joints;
   shared_data->stiffen[i] = stiffen;
   shared_data->actuators_latest = i;

   // effector needs to set standing to false if we got standing
   // we need to wait one cycle in case standing was set after AgentTouch is run
   shared_data->standing = kill_standing;
   if (kill_standing) {
      kill_standing = false;
      shared_data->standing = false;
   } else {
      kill_standing = true;
   }
}
