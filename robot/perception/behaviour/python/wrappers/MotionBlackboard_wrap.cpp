class_<MotionBlackboard>("MotionBlackboard")
   .add_property("sensors", &MotionBlackboard::sensors) 
   .add_property("active", &MotionBlackboard::active)
   .def_readonly("isStiff", &MotionBlackboard::isStiff);
