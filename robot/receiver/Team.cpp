#include <ctime>
#include <iostream>
#include "Team.hpp"
#include "types/SPLStandardMessage.hpp"
#include "utils/incapacitated.hpp"
#include "thread/Thread.hpp"
#include "blackboard/Blackboard.hpp"

using namespace std;

TeamReceiver::TeamReceiver(Blackboard *bb, void(TeamReceiver::*handler)
                           (const boost::system::error_code & error, std::size_t))
   : Adapter(bb), NaoReceiver(this,
                              handler,
                              (bb->config)["network.transmitter_base_port"].as<int>()
                              + (bb->config)["player.team"].as<int>()) {}

void TeamReceiver::naoHandler(const boost::system::error_code &error,
                              std::size_t size) {
   if (Thread::name == NULL) {
      Thread::name = "TeamReceiverBoostThread";
   }

   SPLStandardMessage* m = reinterpret_cast<SPLStandardMessage*>(recvBuffer);
   BroadcastData* bd = (BroadcastData*)m->data;
   if (size == sizeof(SPLStandardMessage)) {
      if (m->playerNum >= 1 && m->playerNum <= ROBOTS_PER_TEAM &&
          m->teamNum == readFrom(gameController, our_team).teamNumber &&
          m->numOfDataBytes == sizeof(BroadcastData) &&
          bd->sanityCheck()) {

         // Not sure why Oleg thought it was ok to modify someone else's blackboard, especially in a different thread
         // This should be based on lastReceived as that is lastReceived's purpose
         std::vector<bool> pendingIncomingUpdates = readFrom(stateEstimation, havePendingIncomingSharedBundle);
         pendingIncomingUpdates[m->playerNum - 1] = true;
         writeTo(stateEstimation, havePendingIncomingSharedBundle, pendingIncomingUpdates);

         writeTo(receiver, message[m->playerNum - 1], *m);
         writeTo(receiver, data[m->playerNum - 1], *bd);
         // TODO (jayen): use ms as we could get two packets in the same second
         writeTo(receiver, lastReceived[m->playerNum - 1], time(NULL));

         // calculate incapacitated
         bool incapacitated = false;
         if (readFrom(gameController, our_team).players[m->playerNum - 1].penalty
             != PENALTY_NONE) {
            incapacitated = true;
         }

         const ActionCommand::Body::ActionType &acB =
            readFrom(receiver, data)[m->playerNum - 1].acB;
         incapacitated |= isIncapacitated(acB);

         std::vector<bool> incapacitatedVec = readFrom(receiver, incapacitated);
         incapacitatedVec[m->playerNum - 1] = incapacitated;
         writeTo(receiver, incapacitated, incapacitatedVec);
      }
   } else {
      llog(WARNING) << "Received packet of " << size << " bytes, but expected "
                                                        "packet of " << sizeof(SPLStandardMessage) << " bytes."  << endl;
   }
   startReceive(this, &TeamReceiver::naoHandler);
}

void TeamReceiver::stdoutHandler(const boost::system::error_code &error,
                                 std::size_t size) {
   SPLStandardMessage* m = (SPLStandardMessage*)recvBuffer;
   BroadcastData* bd = (BroadcastData*)m->data;
   cout << "Received data from player " << bd->playerNum << endl;
   startReceive(this, &TeamReceiver::stdoutHandler);
}

void TeamReceiver::tick() {
   std::vector<bool> incapacitatedVec = readFrom(receiver, incapacitated);
   for (int robot = 0; robot < ROBOTS_PER_TEAM; ++robot) {
      if (time(NULL) - readFrom(receiver, lastReceived)[robot] > SECS_TILL_INCAPACITATED) {
         incapacitatedVec[robot] = true;
      }
   }
   writeTo(receiver, incapacitated, incapacitatedVec);
}
