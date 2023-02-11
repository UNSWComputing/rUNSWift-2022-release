import os, time, datetime
from flask import Flask, render_template, request, Response, jsonify
import multiple_sim
import glob
app = Flask(__name__)

log_filename = None


# When updating these constants make sure you update the constants in robot/utils/SPLDefs.hpp as well
# https://github.com/UNSWComputing/rUNSWift/blob/master/robot/utils/SPLDefs.hpp
FIELD_LENGTH = 9000 + 50
FIELD_WIDTH = 6000 + 50
PENALTY_AREA_LENGTH = 1650 + 50*2
PENALTY_AREA_WIDTH = 4000 + 50

PORT_NUMBER=5000

FALLBACK_TEAMS = [["0", "Invisibles"], ["18", "rUNSWift"]]

@app.route("/")
def index():
    try:
        cfgs = sorted(glob.glob("../../softwares/GameController2*/config/spl/teams.cfg"))
        if (len(cfgs) == 0):
            print("Could not find a teams.cfg")
        cfg = cfgs[-1]
        print(f"Generating teams from {cfg}")
        teams = []
        with open(cfg) as f:
            lines = f.readlines()
            teams = [x.strip().split('=') for x in lines]
    
        if (len(teams) == 0):
            print("Could not parse teams.cfg")

        return render_template("index.html", teams=teams)
    except Exception as e: 
        print(f"Caught {e}. Using fallback teams list")
        return render_template("index.html", teams=FALLBACK_TEAMS)


@app.route("/start_sim", methods=['POST'])
def start_sim():
    team = request.json['team']
    num_robots = request.json['num_robots']
    robot_positions = request.json['robot_positions']
    behaviour = request.json['behaviour']
    extra_balls = request.json['extra_balls']

    opp_team = request.json['opp_team']
    opp_num_robots = request.json['opp_num_robots']
    opp_robot_positions = request.json['opp_robot_positions']
    opp_behaviour = request.json['opp_behaviour']
    use_gamecontroller = request.json['use_gamecontroller']

    # TODO input validation?
    if (behaviour.replace(' ','') == ''):
        behaviour = 'FieldPlayer'
    if (opp_behaviour.replace(' ','') == ''):
        opp_behaviour = 'FieldPlayer'

    # remove the old log files
    #for f in glob.glob("*.tsv"):
    #    os.remove(f)

    runswift_args = ''
    if not use_gamecontroller:
        runswift_args = runswift_args + '--gamecontroller.state PLAYING'

    # remove file if it exists somehow
    if os.path.exists("pose.txt"):
        os.remove("pose.txt")

    # write positions to temp file
    with open("pose.txt", "w+") as pose_file:
        for pose in robot_positions:
            pose_file.write(pose['x'] + ' ' + pose['y'] + ' ' + pose['theta'])
            pose_file.write('\n')
    
    if extra_balls:
        os.system("sim_spawn_balls 3.9,1.1 3.9,-1.1 -3.9,1.1 -3.9,-1.1")

    command_with_args = f"sim_start -t {team} -n {num_robots} -s {behaviour} -ra '{runswift_args + ' --simulation.log_sensed_and_true_positions TRUE'}' -f pose.txt"
    print(command_with_args)
    os.system(command_with_args)
    
    if extra_balls:
        os.system("sim_spawn_balls 0,0")

    # remove file if it exists somehow
    if os.path.exists("opp_pose.txt"):
        os.remove("opp_pose.txt")

    # write positions to temp file
    with open("opp_pose.txt", "w+") as pose_file:
        for pose in opp_robot_positions:
            pose_file.write(pose['x'] + ' ' + pose['y'] + ' ' + pose['theta'])
            pose_file.write('\n')

    command_with_args = f"sim_run -t {opp_team} -n {opp_num_robots} -s {opp_behaviour} -ra '{runswift_args}' -f opp_pose.txt"
    print(command_with_args)
    os.system(command_with_args)


    # remove the temp files
    #os.remove("pose.txt")
    #os.remove("opp_pose.txt")

    return "Started sim"

@app.route("/stop_sim")
def stop_sim():
    os.system("sim_stop")
    print("stopping sim")
    return "stopping sim"

@app.route("/get_logs")
def get_logs():
    global log_filename

    log_time = float(request.args['log_time'])
    num_players = int(request.args['num_players'])

    field_bounds = request.args.get('field_bounds', False) == "true"
    illegal_positioning = request.args.get('illegal_positioning', False) == "true"
    goal_score = request.args.get('goal_score', False) == "true"
    time_limit = float(request.args.get('time_limit', -1))


    # Log files include the:
    #   time
    #   true position of the robot (x,y,theta)
    #   sensed position of the robot (x,y,theta)
    #   true position of the ball (x,y)
    #   sensed position of the ball (x,y)

    # Example log file line:
    # 44.57   -4280   220     -0.14399        -4287   26      -0.160194       2520    -820    2583    -776

    time_pos = []
    true_player_pos = []
    sim_player_pos = []
    true_ball_pos = []
    sim_ball_pos = []

    for p in range(num_players):
        with open(f"sim-position-log-18-{p + 1}.tsv") as f:
            t = 0
            while t < log_time:
                line = f.readline()
                if line == "":
                    break
                t = float(line.split("\t")[0])

            true_player = []
            sim_player = []

            for line in f.read().split("\n"):
                parts = line.split("\t")

                true_player.append(" ".join(parts[1:4]))
                sim_player.append(" ".join(parts[4:7]))

                # simswift instance agree somewhat on where the ball is
                # keep the numbers from the last robot (with the shortest log)
                if p == num_players - 1:
                    time_pos.append(parts[0])
                    true_ball_pos.append(" ".join(parts[7:9]))
                    sim_ball_pos.append(" ".join(parts[9:11]))

            true_player_pos.append(true_player)
            sim_player_pos.append(sim_player)

    log_time = log_time + (min([len(x) for x in true_player_pos])) * 0.03 # 0.03 is the period of the logging

    ret = {
        'message': "",
        'time_pos': time_pos,
        'true_player_pos': true_player_pos,
        'sim_player_pos': sim_player_pos,
        'true_ball_pos': true_ball_pos,
        'sim_ball_pos': sim_ball_pos,
        'log_time': log_time
    }
    for pos in true_ball_pos:
        parts = pos.split(" ")
        if len(parts) != 2:
            break
        x = int(parts[0])
        y = int(parts[1])
        if goal_score and x > FIELD_LENGTH/2 and -750 < y < 750:
            ret['message'] = "Left team scored"
        elif goal_score and x < -FIELD_LENGTH/2 and -750 < y < 750:
            ret['message'] = "Right team scored"
        elif field_bounds and (x > FIELD_LENGTH/2 or x < -FIELD_LENGTH/2 or y > FIELD_WIDTH/2 or y < -FIELD_WIDTH/2):
            ret['message'] = "The ball has left the field"


    transposed = map(lambda *a: list(a), *true_player_pos)
    for t in transposed:
        count_left_penalty_box = 0
        count_right_penalty_box = 0
        for player in t:
            parts = player.split(" ")
            if len(parts) != 3:
                continue
            x = int(parts[0])
            y = int(parts[1])
            if -4500 < x < -4500 + 1650 and -2000 < y < 2000:
                count_left_penalty_box = count_left_penalty_box + 1
            elif 4500 - 1650 < x < 4500 and -2000 < y < 2000:
                count_right_penalty_box = count_right_penalty_box + 1
            elif field_bounds and (x > FIELD_LENGTH/2 + 500 or x < -FIELD_LENGTH/2 - 500 or y > FIELD_WIDTH/2 + 500 or y < -FIELD_WIDTH/2 - 500):
                ret['message'] = "A robot has significantly left the field (by >500mm)"

        if illegal_positioning and count_left_penalty_box > 3:
            ret['message'] = "Illegal Positioning: More than 3 robots in the left penalty box"

        if illegal_positioning and count_right_penalty_box > 3:
            ret['message'] = "Illegal Positioning: More than 3 robots in the right penalty box"

    if time_limit != -1 and log_time > time_limit*60:
        ret['message'] = "Time limit reached"

    # write to a file
    # reset file if log_timeis 0
    # if (not log_filename or log_time== 0):
    #     log_filename = datetime.datetime.now().strftime"%Y%m%d%H%M%S") + ".log"
    #     with open(filename, "w+") as f:
    #         f.write("")

    # with open(filename, "w") as f:

    return jsonify(ret)

@app.route("/mul_sim", methods=['POST'])
def mul_sim():
    num_trials = int(request.json['num_trials'])
    num_robots = int(request.json['num_robots'])
    num_opp_robots = int(request.json['opp_num_robots'])
    sd = int(request.json['sd'])
    robot_positions = request.json['robot_positions']
    opp_robot_positions = request.json['opp_robot_positions']

    # remove file if it exists somehow
    if os.path.exists("pose.txt"):
        os.remove("pose.txt")

    # write positions to temp file
    with open("pose.txt", "w+") as pose_file:
        for pose in robot_positions:
            pose_file.write(pose['x'] + ' ' + pose['y'] + ' ' + pose['theta'])
            pose_file.write('\n')

    # remove file if it exists somehow
    if os.path.exists("opp_pose.txt"):
        os.remove("opp_pose.txt")

    # write positions to temp file
    with open("opp_pose.txt", "w+") as pose_file:
        for pose in opp_robot_positions:
            pose_file.write(pose['x'] + ' ' + pose['y'] + ' ' + pose['theta'])
            pose_file.write('\n')

    # TODO: Delete files after mul_sim uses them

    return jsonify(multiple_sim.main(num_trials, num_robots, num_opp_robots, sd))


if __name__ == "__main__":
    # https://stackoverflow.com/a/25504196/192798
    if 'WERKZEUG_RUN_MAIN' not in os.environ:
        os.system(f'fuser -k {PORT_NUMBER}/tcp')
    app.run(debug=True, load_dotenv=True, port=PORT_NUMBER)
