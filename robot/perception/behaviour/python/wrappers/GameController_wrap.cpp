class_<RoboCupGameControlData>("RoboCupGameControlData")
   .def_readonly("header"        , &RoboCupGameControlData::header        )
   .def_readonly("version"       , &RoboCupGameControlData::version       )
   .def_readonly("playersPerTeam", &RoboCupGameControlData::playersPerTeam)
   .def_readonly("state"         , &RoboCupGameControlData::state         )
   .def_readonly("setPlay"       , &RoboCupGameControlData::setPlay       )
   .def_readonly("firstHalf"     , &RoboCupGameControlData::firstHalf     )
   .def_readonly("kickingTeam"   , &RoboCupGameControlData::kickingTeam   )
   .def_readonly("gamePhase"     , &RoboCupGameControlData::gamePhase     )
   .def_readonly("secsRemaining" , &RoboCupGameControlData::secsRemaining )
   .add_property("teams"         , &RoboCupGameControlData::teams         )
   .def_readonly("secondaryTime" , &RoboCupGameControlData::secondaryTime );

class_<RobotInfo>("RobotInfo")
   .def_readonly("penalty"              , &RobotInfo::penalty            )
   .def_readonly("secsTillUnpenalised"  , &RobotInfo::secsTillUnpenalised);

class_<TeamInfo>("TeamInfo")
   .def_readonly("teamNumber"           , &TeamInfo::teamNumber           )
   .def_readonly("score"                , &TeamInfo::score                )
   .add_property("players"              , &TeamInfo::players              );

   scope().attr("MAX_NUM_PLAYERS"                     ) = MAX_NUM_PLAYERS;
   scope().attr("TEAM_BLUE"                           ) = TEAM_BLUE;
   scope().attr("TEAM_RED"                            ) = TEAM_RED;
   scope().attr("TEAM_YELLOW"                         ) = TEAM_YELLOW;
   scope().attr("TEAM_BLACK"                          ) = TEAM_BLACK;
   scope().attr("TEAM_WHITE"                          ) = TEAM_WHITE;
   scope().attr("TEAM_GREEN"                          ) = TEAM_GREEN;
   scope().attr("TEAM_ORANGE"                         ) = TEAM_ORANGE;
   scope().attr("TEAM_PURPLE"                         ) = TEAM_PURPLE;
   scope().attr("TEAM_BROWN"                          ) = TEAM_BROWN;
   scope().attr("TEAM_GRAY"                           ) = TEAM_GRAY;
   scope().attr("STATE_INITIAL"                       ) = STATE_INITIAL;
   scope().attr("STATE_READY"                         ) = STATE_READY;
   scope().attr("STATE_SET"                           ) = STATE_SET;
   scope().attr("STATE_PLAYING"                       ) = STATE_PLAYING;
   scope().attr("STATE_FINISHED"                      ) = STATE_FINISHED;
   scope().attr("GAME_PHASE_NORMAL"                   ) = GAME_PHASE_NORMAL;
   scope().attr("GAME_PHASE_PENALTYSHOOT"             ) = GAME_PHASE_PENALTYSHOOT;
   scope().attr("GAME_PHASE_OVERTIME"                 ) = GAME_PHASE_OVERTIME;
   scope().attr("GAME_PHASE_TIMEOUT"                  ) = GAME_PHASE_TIMEOUT;
   scope().attr("SET_PLAY_NONE"                       ) = SET_PLAY_NONE;
   scope().attr("SET_PLAY_GOAL_KICK"                  ) = SET_PLAY_GOAL_KICK;
   scope().attr("SET_PLAY_PUSHING_FREE_KICK"          ) = SET_PLAY_PUSHING_FREE_KICK;
   scope().attr("SET_PLAY_CORNER_KICK"                ) = SET_PLAY_CORNER_KICK;
   scope().attr("SET_PLAY_KICK_IN"                    ) = SET_PLAY_KICK_IN;
   scope().attr("SET_PLAY_PENALTY_KICK"               ) = SET_PLAY_PENALTY_KICK;
   scope().attr("PENALTY_NONE"                        ) = PENALTY_NONE;
   scope().attr("PENALTY_SPL_ILLEGAL_BALL_CONTACT"    ) = PENALTY_SPL_ILLEGAL_BALL_CONTACT;
   scope().attr("PENALTY_SPL_PLAYER_PUSHING"          ) = PENALTY_SPL_PLAYER_PUSHING;
   scope().attr("PENALTY_SPL_ILLEGAL_MOTION_IN_SET"   ) = PENALTY_SPL_ILLEGAL_MOTION_IN_SET;
   scope().attr("PENALTY_SPL_INACTIVE_PLAYER"         ) = PENALTY_SPL_INACTIVE_PLAYER;
   scope().attr("PENALTY_SPL_ILLEGAL_POSITION"        ) = PENALTY_SPL_ILLEGAL_POSITION;
   scope().attr("PENALTY_SPL_LEAVING_THE_FIELD"       ) = PENALTY_SPL_LEAVING_THE_FIELD;
   scope().attr("PENALTY_SPL_REQUEST_FOR_PICKUP"      ) = PENALTY_SPL_REQUEST_FOR_PICKUP;
   scope().attr("PENALTY_SPL_LOCAL_GAME_STUCK"        ) = PENALTY_SPL_LOCAL_GAME_STUCK;
   scope().attr("PENALTY_SPL_ILLEGAL_POSITION_IN_SET" ) = PENALTY_SPL_ILLEGAL_POSITION_IN_SET;
   scope().attr("PENALTY_SUBSTITUTE"                  ) = PENALTY_SUBSTITUTE;
   scope().attr("PENALTY_MANUAL"                      ) = PENALTY_MANUAL;
