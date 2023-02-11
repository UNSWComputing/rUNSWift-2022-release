class_<BroadcastData>("BroadcastData")
    .def_readonly("playerNum", &BroadcastData::playerNum)
    .def_readonly("robotPos", &BroadcastData::robotPos)
    .def_readonly("ballPosAbs", &BroadcastData::ballPosAbs)
    .def_readonly("ballPosRR", &BroadcastData::ballPosRR)
    .def_readonly("behaviourSharedData", &BroadcastData::behaviourSharedData)
    .def_readonly("uptime", &BroadcastData::uptime)
    .def_readonly("gameState", &BroadcastData::gameState)
    .def_readonly("sharedStateEstimationBundle", &BroadcastData::sharedStateEstimationBundle);
