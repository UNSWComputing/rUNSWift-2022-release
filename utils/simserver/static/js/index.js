// Constants
const DEFAULT_POSITIONS = [
    {
        'top': '2%',
        'left': '5%',
    },
    {
        'top': '2%',
        'left': '12%',
    },
    {
        'top': '2%',
        'left': '19%',
    },
    {
        'top': '88%',
        'left': '9%',
    },
    {
        'top': '88%',
        'left': '16%',
    }
];
const OPP_DEFAULT_POSITIONS = [
    {
        'top': '2%',
        'left': '42%',
    },
    {
        'top': '2%',
        'left': '35%',
    },
    {
        'top': '2%',
        'left': '28%',
    },
    {
        'top': '88%',
        'left': '38%',
    },
    {
        'top': '88%',
        'left': '31%',
    }
]
const RELATIVE_PITCH_WIDTH = 90;
const RELATIVE_PITCH_LENGTH = 60;
const RELATIVE_PITCH_BORDER = 7;
const SCALE = 100;
const MAX_NUM_ROBOTS = 5;

// Default values
let numRobots = 1;
let opponentNumRobots = 0;
let team = 18;
let opponentTeam = 0;
let robotPositions = [];
let oppRobotPositions = [];
let behaviour = "FieldPlayer";
let opponentBehaviour = "FieldPlayer";
let extraBalls = false;
let time = 0;
let logInterval = 0.5;

// Other globals
let interval;
let selectedRobot;
let latestLogDisplayed = 0;
let rules = {};
let useGameController = true;

/**
 * Initialises the state. Called on onload for the body.
 */
function init() {
    // Create pitch
    let pitch = document.getElementById("pitch");
    let pitch_img = document.createElement("img");
    pitch_img.id = "pitch_img";
    pitch_img.src = "./static/images/pitch.png";
    pitch.appendChild(pitch_img);

    // Wait for the pitch to load as otherwise we can't calculate the dimensions correctly
    pitch_img.onload = () => {
        updateNumRobots(1);
        setUpDraggable()
        setUpSelect()
        // setUpRotate()
    };
}

/**
 * Updates the number of robots for our team and also creates new sprites.
 * @param {number} val number of new robots 
 */
function updateNumRobots(val) {
    numRobots = val;
    let pitch = document.getElementById("pitch");
    
    for (let i = 0; i < MAX_NUM_ROBOTS; i++) {
        // If the number of robots was reduced, remove any extra robots
        if (i >= numRobots) {
            if (document.getElementsByName("robot_img_" + i.toString()).length != 0) {
                let robot = document.getElementsByName("robot_img_" + i.toString())[0];
                robot.remove();    
            }
            continue;
        }
        
        // If a robot has already been created for this player number, skip and don't create another one
        if (document.getElementsByName("robot_img_" + i.toString()).length != 0) {
            continue;
        }

        let node = document.createElement("img");
        node.id = "robot";
        node.name = "robot_img_" + i.toString();        
        node.className = "draggable";
        node.src = "./static/images/18_" + (i + 1).toString() + ".png";
        node.style.top = DEFAULT_POSITIONS[i].top;
        node.style.left = DEFAULT_POSITIONS[i].left;
        pitch.appendChild(node);
        let position = getPitchPositionFromPagePixelPosition(node.x, node.y);
        robotPositions[i] = {x: position.x.toString(), y: position.y.toString(), theta: '0'};
    }
    setUpSelect();
    // setUpRotate();
}

/**
 * Same as updateNumRobots but for the opposition.
 * @param {number} val number of opp robots
 */
function updateOpponentNumRobots(val) {
    opponentNumRobots = val;
    let pitch = document.getElementById("pitch");

    for (let i = 0; i < MAX_NUM_ROBOTS; i++) {   
        if (i >= opponentNumRobots) {
            if (document.getElementsByName("opp_robot_img_" + i.toString()).length != 0) {
                let robot = document.getElementsByName("opp_robot_img_" + i.toString())[0];
                robot.remove();    
            }
            continue;
        }
        
        if (document.getElementsByName("opp_robot_img_" + i.toString()).length != 0) {
            continue;
        }

        let node = document.createElement("img");
        node.id = "opp_robot";
        node.name = "opp_robot_img_" + i.toString();        
        node.className = "draggable";
        node.src = "./static/images/19_" + (i + 1).toString() +  ".png";
        node.style.top = OPP_DEFAULT_POSITIONS[i].top;
        node.style.left = OPP_DEFAULT_POSITIONS[i].left;
        pitch.appendChild(node);
        let position = getPitchPositionFromPagePixelPosition(node.x, node.y);
        // Opposition positions are reversed
        const x = position.x * -1;
        const y = position.y * -1;
        oppRobotPositions[i] = {x: x.toString(), y: y.toString(), theta: '0'};
    }
    setUpSelect();
    // setUpRotate();
}

function updateBehaviour(val) {
    behaviour = val;
}

function updateOpponentBehaviour(val) {
    opponentBehaviour = val;
}

function updateExtraBalls(val) {
    extraBalls = val;
}

/** Called when a single simulation is started */
function onStartClick() {
    time = 0;
    let data = {
        'team': team,
        'num_robots': numRobots,
        'robot_positions': robotPositions,
        'behaviour': behaviour,
        'extra_balls': extraBalls,
        'opp_team': opponentTeam,
        'opp_num_robots': opponentNumRobots,
        'opp_robot_positions': oppRobotPositions,
        'opp_behaviour': opponentBehaviour,
        'use_gamecontroller': useGameController
    }

    let statusNode = document.getElementById("start_status");
    statusNode.textContent = "Starting sim...";
    document.getElementById("log").hidden = false
    resetLog()
    
    // Get logs
    // We poll the logs API ever 3 seconds after the first fetch.
    setTimeout(() => {
                clearInterval(interval)
                interval = setInterval(() => {
                    fetch("/get_logs?log_time=" + time.toFixed(4) + "&num_players=" + numRobots.toString() + dictToArgs(rules))
                        .then(res => res.json())
                        .then(res => {
                            time = parseFloat(res['log_time'])
                            appendLog(res);
                            if (res['message'] != "") {
                                onStopClick();
                                statusNode.textContent = res['message']
                            }
                        }).catch(err => console.log(err));
                }, 3000);
            }, 5000 + numRobots * 2000);
    
    return fetch("/start_sim", {
                method: 'POST',
                body: JSON.stringify(data),
                headers: {
                    'Content-Type': 'application/json',
                },
            })
            .then((res) => {
                return res.text();
            })
            .then((res) => {
                statusNode.textContent = "Started sim successfully."
            })
            .catch((err) => {
                console.log(err);
            })
}

/**
 * Called when the "Stop simulation" button is clicked.
 */
function onStopClick() {
    let statusNode = document.getElementById("stop_status");
    statusNode.textContent = "Stopping sim..."
    clearInterval(interval);
    return fetch("/stop_sim")
                .then((res) => {
                    return res.text();
                })
                .then((res) => {
                    statusNode.textContent = "Stopped sim successfully."
                })
                .catch((err) => {
                    console.log(err);
                })
}

/**
 * Called when user clicks on the multiple simulation button.
 */
function onStartMulClick() {
    let statusNode = document.getElementById("start_mul_status");
    statusNode.textContent = "Running multiple sims....";
    let num_trials = document.getElementById("num_simulations").value;
    let sd = document.getElementById("sd").value;
    let data = {
        'team': team,
        'num_robots': numRobots,
        'robot_positions': robotPositions,
        'opp_team': opponentTeam,
        'opp_num_robots': opponentNumRobots,
        'opp_robot_positions': oppRobotPositions,
        'num_trials': num_trials,
        'sd': sd
    }

    return fetch("/mul_sim", {
        method: 'POST',
        body: JSON.stringify(data),
        headers: {
            'Content-Type': 'application/json',
        },
    })
    .then(res => res.json())
    .then(res => {
        let num_goals = 0;
        let total_time = 0;
        // Create result text
        for (let i = 0; i < res.length; i++) {
            if (res[i].includes("Goal")) {
                num_goals++;
                total_time += parseFloat(res[i].match(/\d+\.\d+/)[0])
            }
        }
        statusNode.textContent = "Finished multiple sims. Scored " + num_goals.toString() + " goals. Average time for goal: " + Math.round((total_time/num_goals) * 100) / 100 + " seconds."
    })
    .catch(err => console.log(err));
}

/**
 * Appends logs that are polled from the server to the log table.
 * @param {*} log new log data received from the server
 */
function appendLog(log) {
    var table = document.getElementById("log_table_body");
    for (let i = 0; i < log['time_pos'].length; i++) {
        if (parseFloat(log['time_pos'][i]) >= latestLogDisplayed + logInterval) {
            latestLogDisplayed = parseFloat(log['time_pos'][i]);
            var row = table.insertRow(-1);
            row.insertCell(-1).innerHTML = log['time_pos'][i];
            for (let j = 0; j < numRobots; j++) {
                row.insertCell(-1).innerHTML = log['true_player_pos'][j][i];
                row.insertCell(-1).innerHTML = log['sim_player_pos'][j][i];
            }
            row.insertCell(-1).innerHTML = log['true_ball_pos'][i];;
            row.insertCell(-1).innerHTML = log['sim_ball_pos'][i];
        }
    }
}

/**
 * Function that converts a robot's position on the page to runswift co-ords.
 * @param {number} pageX x position of robot wrt page in px 
 * @param {number} pageY y position of robot wrt page in px
 */
function getPitchPositionFromPagePixelPosition(pageX, pageY) {
    let pitchImg = document.getElementById('pitch_img');
    let offset = getOffset(pitchImg);
    
    // These are including the pitch "border" and are in pixels
    let pitchTop = offset.top;
    let pitchBottom = offset.top + pitchImg.clientHeight;
    let pitchLeft = offset.left;
    let pitchRight = offset.left + pitchImg.clientWidth;

    // These are just ratios
    let relativePitchWidthWithBorder = RELATIVE_PITCH_BORDER + RELATIVE_PITCH_WIDTH + RELATIVE_PITCH_BORDER;
    let relativePitchLengthWithBorder = RELATIVE_PITCH_BORDER + RELATIVE_PITCH_LENGTH + RELATIVE_PITCH_BORDER;
    
    // All "sizes" are in pixels
    // Pitch border size is essentially the same for both width and length
    let pitchBorderSize = ((pitchRight - pitchLeft)/relativePitchWidthWithBorder) * RELATIVE_PITCH_BORDER;
    pitchBorderSize = Math.round(pitchBorderSize);

    let pitchWidthSize = ((pitchRight - pitchLeft)/relativePitchWidthWithBorder) * RELATIVE_PITCH_WIDTH;
    pitchWidthSize = Math.round(pitchWidthSize);

    let pitchLengthSize = ((pitchBottom - pitchTop)/relativePitchLengthWithBorder) * RELATIVE_PITCH_LENGTH;
    pitchLengthSize = Math.round(pitchLengthSize);

    // "FromEdge" refers to the pixels from the edge of the playable area
    let xFromEdge;
    if (pageX < (pitchLeft + pitchBorderSize) || (pageX > (pitchRight - pitchBorderSize))) {
        if (pageX < (pitchLeft + pitchBorderSize)) {
            // Reset to left most part of playable pitch
            xFromEdge = 0;
        } else {
            // Reset to right most part of playable pitch
            xFromEdge = pitchRight - pitchBorderSize;
        }
    } else {
        xFromEdge = pageX - pitchLeft - pitchBorderSize;
    }

    let yFromEdge;
    if (pageY < (pitchTop + pitchBorderSize) || pageY > (pitchBottom - pitchBorderSize)) {
        if (pageY < (pitchTop + pitchBorderSize)) {
            // Reset to top most part of playable pitch
            yFromEdge = 0;
        } else {
            // Rest to bottom most part of playable pitch
            yFromEdge = pitchBottom - pitchBorderSize;
        }
    } else {
        yFromEdge = pageY - pitchTop - pitchBorderSize;
    }

    // Ratio is to convert pixel distance to runswift coordinates
    let xFromEdgeRatio = xFromEdge/pitchWidthSize;
    let xRobotPos = (xFromEdgeRatio * RELATIVE_PITCH_WIDTH) - (RELATIVE_PITCH_WIDTH/2);
    
    let yFromEdgeRatio = yFromEdge/pitchLengthSize;
    let yRobotPos = (yFromEdgeRatio * RELATIVE_PITCH_LENGTH) - (RELATIVE_PITCH_LENGTH/2);
    // Y cords are other way round
    yRobotPos *= -1;

    return {x: Math.round(xRobotPos * SCALE), y: Math.round(yRobotPos* SCALE)};
}

/**
 * Makes the robot sprites draggable.
 */
function setUpDraggable() {
    const positions = [
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        { x: 0, y: 0 },
    ]

    // Parameters required to visually limit the dragging to the playable area
    // We essentially just need to calculate the coordinates of the playable area
    let pitchImg = document.getElementById('pitch_img');
    let offset = getOffset(pitchImg);
    
    // These are including the pitch "border" and are in pixels
    let pitchTop = offset.top;
    let pitchBottom = offset.top + pitchImg.clientHeight;
    let pitchLeft = offset.left;
    let pitchRight = offset.left + pitchImg.clientWidth;

    // These are just ratios
    let relativePitchWidthWithBorder = RELATIVE_PITCH_BORDER + RELATIVE_PITCH_WIDTH + RELATIVE_PITCH_BORDER;
    let relativePitchLengthWithBorder = RELATIVE_PITCH_BORDER + RELATIVE_PITCH_LENGTH + RELATIVE_PITCH_BORDER;
    
    // All "sizes" are in pixels
    // Pitch border size is essentially the same for both width and length
    let pitchBorderSize = ((pitchRight - pitchLeft)/relativePitchWidthWithBorder) * RELATIVE_PITCH_BORDER;
    pitchBorderSize = Math.round(pitchBorderSize);

    let pitchWidthSize = ((pitchRight - pitchLeft)/relativePitchWidthWithBorder) * RELATIVE_PITCH_WIDTH;
    pitchWidthSize = Math.round(pitchWidthSize);

    let pitchLengthSize = ((pitchBottom - pitchTop)/relativePitchLengthWithBorder) * RELATIVE_PITCH_LENGTH;
    pitchLengthSize = Math.round(pitchLengthSize);

    let restrictRect = {
        left: pitchLeft + pitchBorderSize,
        top: pitchTop + pitchBorderSize,
        right: pitchRight - pitchBorderSize,
        bottom: pitchBottom - pitchBorderSize,    
    }

    interact('.draggable').draggable({
        listeners: {
            start (event) {
                // console.log(event.type, event.target)
            },
            move (event) {
                // Not nice?
                const isOpp = event.currentTarget.name.startsWith("opp"); 

                let robotNumber = isOpp ? 
                                    parseInt(event.currentTarget.name.split("_")[3]) + 5:
                                    parseInt(event.currentTarget.name.split("_")[2]);
                
                positions[robotNumber].x += event.dx
                positions[robotNumber].y += event.dy

                event.target.style.transform =
                `translate(${positions[robotNumber].x}px, ${positions[robotNumber].y}px)`
            },
            end (event) {
                let isOpp = event.currentTarget.name.startsWith("opp");

                let robotNumber = isOpp ? 
                                    parseInt(event.currentTarget.name.split("_")[3]):
                                    parseInt(event.currentTarget.name.split("_")[2]);

                let robotPitchPosition = getPitchPositionFromPagePixelPosition(event.pageX, event.pageY);
                
                if (isOpp) {
                    oppRobotPositions[robotNumber] = {x: (robotPitchPosition.x * -1).toString(), y: (robotPitchPosition.y * -1).toString(), theta: '0'};
                } else {
                    robotPositions[robotNumber] = {x: robotPitchPosition.x.toString(), y: robotPitchPosition.y.toString(), theta: '0'};
                }
            },
        },
        modifiers: [
            interact.modifiers.restrict({
                restriction: restrictRect,
                endOnly: true
            })
        ]
    })
}

/**
 * Sets up selection for the robot sprites.
 */
function setUpSelect() {
    let robots = document.getElementsByClassName('draggable');

    let onClick = (event) => {
        if (selectedRobot) {
            selectedRobot.style.filter = "";
            selectedRobot.style.webkitFilter = "";
        }

        event.target.style.filter = "drop-shadow(black 1px 1px 0px) drop-shadow(black -1px 1px 0px) drop-shadow(black 1px -1px 0px) drop-shadow(black -1px -1px 0px)";
        event.target.style.webkitFilter = "drop-shadow(black 1px 1px 0px) drop-shadow(black -1px 1px 0px) drop-shadow(black 1px -1px 0px) drop-shadow(black -1px -1px 0px)";
        selectedRobot = event.target;
    }

    for (let i = 0; i < robots.length; i++) {
        robots[i].addEventListener('click', onClick, false)
    }
}

// function setUpRotate() {
//     let onkeydownFunction = (event) => {
//         if (event.keyCode == '38') {
//             console.log('askldhklasjdlk');
//             if (selectedRobot) {
//                 selectedRobot.style.transform = "rotate(10deg)";
//             }
//         }
//     }

//     document.onkeydown = onkeydownFunction;
// }

// https://stackoverflow.com/questions/442404/retrieve-the-position-x-y-of-an-html-element
function getOffset( el ) {
    var _x = 0;
    var _y = 0;
    while( el && !isNaN( el.offsetLeft ) && !isNaN( el.offsetTop ) ) {
        _x += el.offsetLeft - el.scrollLeft;
        _y += el.offsetTop - el.scrollTop;
        el = el.offsetParent;
    }
    return { top: _y, left: _x };
}

function updateTeam(val) {
    team = val;
}

function updateOpponentTeam(val) {
    opponentTeam = val;
}

function updateLogInterval(val) {
    logInterval = parseFloat(val);
}

function updateRule(key, val) {
    rules[key] = val;
}

function updateUseGameController(val) {
    useGameController = val;
}

function dictToArgs(dict) {
    var ans = "";
    for (var key in dict) {
        ans = `${ans}&${key}=${dict[key]}`;
    }
    return ans;
}

function resetLog() {

    document.getElementById("log_table_players_title").colSpan = numRobots*2

    const player_num_row = document.getElementById("log_table_players_row")
    while (player_num_row.firstChild) {
        player_num_row.removeChild(player_num_row.firstChild)
    }
    player_num_row.insertCell(-1).style.visibility='hidden' 
    for (let i = 1 ; i <= numRobots ; i++) {
        const node = player_num_row.insertCell(-1);
        node.innerHTML = i;
        node.colSpan = 2;
    }

    const true_percieved_row = document.getElementById("log_table_true_perceived")
    while (true_percieved_row.firstChild) {
        true_percieved_row.removeChild(true_percieved_row.firstChild)
    }
    true_percieved_row.insertCell(-1).style.visibility='hidden' 
    for (let i = 0 ; i <= numRobots ; i++) {
        true_percieved_row.insertCell(-1).innerHTML = "True";
        true_percieved_row.insertCell(-1).innerHTML = "Percieved";
    }
   
    document.getElementById('log_table_body').innerHTML = '';

    latestLogDisplayed = 0;
}   


function saveLog() {
    document.getElementById('save_log').textContent = 'Save log to file: WIP Unimplemented';
}
