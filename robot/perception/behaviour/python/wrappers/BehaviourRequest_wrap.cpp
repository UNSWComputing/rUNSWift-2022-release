class_<BehaviourRequest>("BehaviourRequest")
   .def_readwrite("actions", &BehaviourRequest::actions)
   .def_readwrite("behaviourSharedData", &BehaviourRequest::behaviourSharedData)
   .def_readwrite("behaviourDebugInfo", &BehaviourRequest::behaviourDebugInfo);
