class_<BallInfo>("BallInfo")
   .def_readonly("rr", &BallInfo::rr)
   .def_readonly("radius", &BallInfo::radius)
   .add_property("imageCoords", make_getter(&BallInfo::imageCoords, return_value_policy<return_by_value>()))
   .def_readonly("topCamera", &BallInfo::topCamera);
