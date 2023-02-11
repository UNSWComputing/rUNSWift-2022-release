#ifndef BEHAVIOUR_REQUEST_HPP
#define BEHAVIOUR_REQUEST_HPP

#include "types/ActionCommand.hpp"
#include "types/BehaviourDebugInfo.hpp"
#include "types/BehaviourSharedData.hpp"

class BehaviourRequest {
   public:

      // Action to send to motion
      ActionCommand::All actions;

      // Information to share to teammates
      BehaviourSharedData behaviourSharedData;

      // Debug information to show in offnao
      BehaviourDebugInfo behaviourDebugInfo;

      BehaviourRequest() {
      };

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {

         ar & actions;
         ar & behaviourSharedData;
      }
};

BOOST_CLASS_VERSION(BehaviourRequest, 1);

#endif // BEHAVIOUR_REQUEST_HPP
