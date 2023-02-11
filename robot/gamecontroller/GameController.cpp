/**
 * GameController.hpp
 * Description: A thread to recieve game state information from the Game
 * Controller sever and implements the Button Interface. Adapted from 2009 code.
 */

#include "gamecontroller/GameController.hpp"
#include <arpa/inet.h>
#include <ctime>
#include <dirent.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <time.h>
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"

#define POLL_TIMEOUT 200

using namespace std;

/**
 * Average number of broadcast GameController packets
 * expected to be received per second.
 */
const uint8_t PACKETS_PER_SECOND = 2;

/**
 * Minimum number of seconds to wait before leaving WiFi.
 */
const uint8_t LEAVE_WIFI_SECONDS = 20;

/**
 * To handle spotty WiFi scenarios, allow up to this much network jitter,
 * specifically allow this proportion of packets dropped before leaving the
 * WiFi, as the second required guard to leave the WiFi and kill `runswift`.
 *
 * If set too high, robots will wait longer before actually leaving the WiFi,
 * if set too low, robots will depend more only on the time that the
 * first and last packets were received.
 *
 * Should be set to a value between 0.0 and 1.0
 */
const double JITTER_BUFFER = 0.5;

const uint8_t min_packets =
    JITTER_BUFFER * LEAVE_WIFI_SECONDS * PACKETS_PER_SECOND;

GameController::GameController(Blackboard *bb)
    : Adapter(bb), our_team(NULL), connected(false) {
    lastState = STATE_INITIAL;
    myLastPenalty = PENALTY_NONE;
    if (readFrom(gameController, connect)) {
        initialiseConnection();
    }
    actOnWhistle = (bb->config)["debug.act_on_whistle"].as<bool>();
    useGetups = (bb->config)["behaviour.use_getups"].as<bool>();
}

GameController::~GameController() {
    close(sock);
}

void GameController::tick() {

    // Notes:
    // gameState represents what we think the game state should be
    // It is the same as data.state unless we hear a whistle
    // If we hear a whistle, set gameState to playing to tell our teammates
    // If the team agrees, then override the official data.state with playing

    // -- Standard Game Controller Packet Update --
    uint8_t previousGameState = readFrom(gameController, gameState);
    data = readFrom(gameController, data);
    teamNumber = readFrom(gameController, our_team).teamNumber;
    playerNumber = readFrom(gameController, player_number);
    setOurTeam();

   if (!connected && readFrom(gameController, connect)) {
        initialiseConnection();
    }
    if (connected) wirelessUpdate();
    buttons = readFrom(motion, buttons);
    
    buttonUpdate();
   // Not sure why Dave thought it was ok to write to someone else's blackboard
    writeTo(motion, buttons, buttons);

    // -- Extras for Whistle Detection --
    uint8_t gameState = data.state;
    bool whistleDetected = false;

    // If we previously heard a whistle and changed our gameState to playing
    // Then don't let it get overriden by the game controller
    if (gameState == STATE_SET && previousGameState == STATE_PLAYING) {
        gameState = previousGameState;
        whistleDetected = true;
    }

    // Heard whistles - if we are in SET and whistle heard in last 3 seconds
    if (gameState == STATE_SET && whistleHeard(3)) {
        if (actOnWhistle == true) {
            gameState = STATE_PLAYING;
            whistleDetected = true;
            SAY("Whistle heard");
        }
        else {
            SAY("Heard whistle not moving");
        }
    }

    // Check the team opinion on whether a whistle has been heard
    if (data.state == STATE_SET) {
        float numTeammatesPlaying = 0;
        float numActiveTeammates = 0;
        for (int i = 0; i < ROBOTS_PER_TEAM; ++i) {
            if (!readFrom(receiver, incapacitated)[i]) {
                ++numActiveTeammates;
                if (readFrom(receiver, data)[i].gameState == STATE_PLAYING) {
                    ++numTeammatesPlaying;
                }
            }
        }
        AbsCoord robot_pos = readFrom(stateEstimation, robotPos);
        bool nearCenterCircle = abs(robot_pos.x()) < 1500 &&
                                        abs(robot_pos.y()) < 3000;

        if (numActiveTeammates > 0) {
            // If enough of the team thinks we should play,
            // and we're localised and close to center circle, lets play
            float ratio = numTeammatesPlaying / numActiveTeammates;
            if (ratio >= 0.30 && nearCenterCircle && actOnWhistle) {
                data.state = STATE_PLAYING;
                whistleDetected = true;
               SAY("Whistle heard by teammates");
            }
            else if (ratio >= 0.49 && actOnWhistle) {
                data.state = STATE_PLAYING;
                whistleDetected = true;
               SAY("Whistle heard by most teammates");
            }
        }
    }

    writeTo(gameController, data, data);
    writeTo(gameController, our_team, *our_team);
    writeTo(gameController, gameState, data.state);
    writeTo(gameController, whistleDetected, whistleDetected);

    // In the case where we've heard a whistle, but haven't decided to play yet
    // We want to keep the official data as set,
    // ... but tell the team we think its play time
    // So override our gameState variable, thus making gameState != data.state
    if (gameState == STATE_PLAYING && data.state == STATE_SET) {
        writeTo(gameController, gameState, gameState);
    }
}

void GameController::initialiseConnection() {
    llog(INFO) << "GameController: Connecting on port "
               << GAMECONTROLLER_DATA_PORT << endl;
    stringstream s;
    s << GAMECONTROLLER_DATA_PORT;

    struct addrinfo myInfo, *results;
    memset(&myInfo, 0, sizeof myInfo);
    myInfo.ai_family = AF_UNSPEC;
    myInfo.ai_socktype = SOCK_DGRAM;
    myInfo.ai_flags = AI_PASSIVE;  // use my IP

    if (getaddrinfo(NULL, s.str().c_str(), &myInfo, &results) == -1) {
        llog(ERROR) << "GameController: Invalid Address Information" << endl;
        return;
    }

    // loop through all the results and bind to the first we can
    struct addrinfo *p;
    for (p = results; p != NULL; p = p->ai_next) {
        if ((sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            llog(INFO) << "GameController: Cannot use Socket, trying next"
                       << endl;
            continue;
        }

        // set the socket to reuse ports (so we can have multiple instances of
        // rUNSWift listening to the same UDP port ;-) )
        int enable = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) != 0)
        {
             llog(ERROR) << "Could not set socket options: " << errno << endl;
             continue;
        }

        if (bind(sock, p->ai_addr, p->ai_addrlen) == -1) {
            close(sock);
            llog(INFO) << "GameController: Cannot Bind, trying next" << endl;
            continue;
        }

        break;
    }
    if (p == NULL) {
        llog(ERROR) << "GameController: Failed to bind socket" << endl;
        return;
    }

    // We don't want memory leaks...
    freeaddrinfo(results);

    llog(INFO) << "GameController: Connected on port - " << s.str() << endl;
    connected = true;
    writeTo(gameController, connected, connected);
}

void GameController::buttonUpdate() {
    if (buttons.remove(ButtonPresses::SINGLE_CHEST_TAP)) {
        llog(INFO) << "button pushed once, switching state" << endl;
        switch (our_team->players[playerNumber - 1].penalty) {
        case PENALTY_NONE:
            our_team->players[playerNumber - 1].penalty =
                PENALTY_MANUAL;
              SAY("Penalised");
            break;
        default:
            data.state = STATE_PLAYING;
            our_team->players[playerNumber - 1].penalty =
                PENALTY_NONE;
              SAY("Playing");
        }
    }
}

void GameController::wirelessUpdate() {
    // Setup receiving client
    int bytesRecieved;
    struct sockaddr_storage clientAddress;
    socklen_t addr_len = sizeof(clientAddress);

    // Setup buffer to write to
    int dataSize = sizeof(RoboCupGameControlData);
    unsigned char buffer[dataSize + 1];

    // Setup for polling
    struct pollfd ufds[1];
    ufds[0].fd = sock;
    ufds[0].events = POLLIN;                // For incoming packets

    // Congested WiFi: Try to grab several packets in one tick
    // to clear buffers and lower effective latency
    for (int i = 0; i < 5; i++) {
        int rv = poll(ufds, 1, POLL_TIMEOUT);  // Wait up to POLL_TIMEOUT ms

        // Check to see if we've received a packet
        if (rv > 0) {
            bytesRecieved = recvfrom(sock, buffer, dataSize, 0,
                                     (struct sockaddr *)&clientAddress,
                                     &addr_len);
            // Should use inet_ntop, but we don't need IPv6 support so meh
            writeTo(gameController, lastGameControllerIPAddress, inet_ntoa(
                ((struct sockaddr_in *)&clientAddress)->sin_addr
            ));
            if (bytesRecieved > 0) {
                parseData((RoboCupGameControlData*)buffer);
                handlePenaltyShootoutPacket();
                handleFinishedPacket();
            }
        }
    }
}

void GameController::handlePenaltyShootoutPacket() {
    if (data.gamePhase != GAME_PHASE_PENALTYSHOOT) {
        writeTo(behaviour, useGetups, useGetups);
        return;
    }
    // Hard coded as we've seen goalie dive, save successfully, but
    // hands then fling ball into goal
    if (playerNumber == 1) {
        writeTo(behaviour, useGetups, false);
    }
}

void GameController::handleFinishedPacket() {
    if (data.state != STATE_FINISHED ||
            data.gamePhase != GAME_PHASE_NORMAL) {
        // If not FINISHED, or in a timeout or penalty shootout game state
        finished_times.clear();
        return;
    }
    time_t now = time(0);  // get time now
    finished_times.push_back(now);
    double diff = difftime(finished_times.front(), finished_times.back());
    if (abs(diff) > LEAVE_WIFI_SECONDS &&
        finished_times.size() > min_packets
    ) {
       SAY("THANKS AND SEE YOU LATER");
//       sleep(3);
//       exit(EXIT_SUCCESS);
    }
}

bool GameController::whistleHeard(int numSeconds) {
    const char *WHISTLE_FILE_FORMAT = "whistle_%Y_%m_%d_%H%M%S.wav";
    const char *NAO_WHISTLE_LOCATION = "/home/nao/whistle/heard_whistles";
    DIR *dir;
    struct dirent *ent;
    struct tm fileDateTime;
    time_t now = time(0);  // get time now
    bool found = false;

    if ((dir = opendir (NAO_WHISTLE_LOCATION)) != NULL) {
        /* go through all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            strptime(ent->d_name, WHISTLE_FILE_FORMAT, &fileDateTime);
            double seconds = difftime(now, mktime(&fileDateTime));

            // If file created in last numSeconds
            if (abs(seconds) < numSeconds && seconds < numSeconds) {
                found = true;
            }
        }
        closedir (dir);
    } else {
        /* could not open directory */
    }
    return found;
}

void GameController::parseData(RoboCupGameControlData *update) {
    if (isValidData(update)) {
        // Heard whistles - if GameController still saying we are in SET
        // but we are already PLAYING, keep PLAYING
        if (int(update->state) == STATE_SET && data.state == STATE_PLAYING) {
            update->state = STATE_PLAYING;
        }

        // Update the data
        if (!gameDataEqual(update, &data)) {
            memcpy(&data, update, sizeof(RoboCupGameControlData));
            setOurTeam();
        }

        llog(VERBOSE) << "GameController: Valid data" << endl;
        if (data.state != lastState) {
            // Shamelessly copied from: http://stackoverflow.com/a/1995057/1101109
            char comboState[100];
            strcpy(comboState, gameControllerGamePhaseNames[update->gamePhase]);
            strcat(comboState, gameControllerStateNames[data.state]);
           SAY(comboState);
            lastState = data.state;
        }

        unsigned char myPenalty = our_team->players[playerNumber - 1].penalty;

        if (myPenalty != myLastPenalty) {
            if (myPenalty == PENALTY_NONE) {
               SAY("Unpenalised");
            } else {
               SAY((string("Penalised for ") +
                    gameControllerPenaltyNames[myPenalty]).c_str());
            }
            myLastPenalty = myPenalty;
        }
    } else {
       // no need to do anything since we already log an error during validation about why it's invalid
       // and we ignore TCM data
//        llog(ERROR) << "GameController: Invalid data" << endl;
    }
}

void GameController::setOurTeam() {// make our_team point to the my actual team, based on teamNumber
   if (data.teams[0].teamNumber == teamNumber) {
      our_team = &(data.teams[0]);
   } else if (data.teams[1].teamNumber == teamNumber) {
      our_team = &(data.teams[1]);
   } else {
      llog(ERROR) << "We are team " << teamNumber << " but GC is sending " <<
                  data.teams[0].teamNumber << " and " << data.teams[1].teamNumber << "\n";
   }
}

bool GameController::isValidData(RoboCupGameControlData *gameData) {
    // check the right structure header has come in
    if (!(checkHeader(gameData->header))) {
        // only log an error if it's not for us or the tcm (as we may run on the same computer as the tcm in simulation)
        if (strncmp(GAMECONTROLLER_TCM_HEADER, gameData->header, strlen(GAMECONTROLLER_TCM_HEADER)) != 0)
            llog(ERROR) << "GameController: DATA HEADER MISMATCH! "
                        << "Expected: " << GAMECONTROLLER_STRUCT_HEADER
                        << " received: " << gameData->header << endl;
        return false;
    }

    // check for partial packets
    // TODO(stuartr): these will always be equal since sizeof(type) is compile-time
    //                based on the type.  use bytesRecieved from wirelessUpdate
    if (sizeof(*gameData) != sizeof(RoboCupGameControlData)) {
        llog(ERROR) << "GameController: RECEIVED PARTIAL PACKET! "
                    << "Expected: " << sizeof(RoboCupGameControlData)
                    << " received: " << sizeof(*gameData) << endl;
        return false;
    }

    // check the right version of the structure is being used
    if (gameData->version != GAMECONTROLLER_STRUCT_VERSION) {
        llog(ERROR) << "GameController: DATA VERSION MISMATCH! "
                    << "Expected: " << GAMECONTROLLER_STRUCT_VERSION
                    << " received: " << gameData->version << endl;
        return false;
    }

    // check whether this packet belongs to this game at all
    if (!isThisGame(gameData)) {
        llog(ERROR) << "GameController: DATA NOT FOR THIS GAME!" << endl;
        return false;
    }

    // Data is valid ^_^
    return true;
}

bool GameController::checkHeader(char* header) {
    for (int i = 0; i < 4; i++) {
        if (header[i] != GAMECONTROLLER_STRUCT_HEADER[i]) return false;
    }
    return true;
}

bool GameController::isThisGame(RoboCupGameControlData* gameData) {
   return gameData->teams[0].teamNumber == teamNumber ||
          gameData->teams[1].teamNumber == teamNumber;
}

bool GameController::gameDataEqual(void* gameData, void* previous) {
    if (!memcmp(previous, gameData, sizeof(RoboCupGameControlData))) {
        return true;
    }
    return false;
}
