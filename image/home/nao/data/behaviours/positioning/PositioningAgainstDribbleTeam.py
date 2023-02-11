# Positioning against teams that kick a lot
#
# Teams include
# - HTWK
# - Any team that will not kick

import math
import itertools
from Positioning import Positioning, Role, OBJECT_INDEX
from util.TeamStatus import my_player_number
from util.Global import ballWorldPos, myPos, myHeading
from util.Vector2D import Vector2D, makeVector2DFromDistHeading
from util.FieldGeometry import OUR_GOAL_CENTRE, ENEMY_GOAL_CENTER, isInOurGoalBox, OUR_LEFT_POST, OUR_RIGHT_POST
from util.MathUtil import clamp, normalisedTheta
from util.Constants import FIELD_LENGTH, GOAL_BOX_LENGTH, GOAL_POST_ABS_Y
from math import radians
from util.TeamStatus import (
    teammate_ego_ball,
    player_role,
    player_number_is_incapacitated,
    get_teammate_pos,
    get_teammate_heading,
    get_teammate_seconds_since_last_ball_update,
    get_active_player_numbers,
    player_is_playing_ball,
)
from util.FieldGeometry import calculateTimeToReachPose
from util.GameStatus import in_goal_kick, in_pushing_free_kick, in_kick_in, we_are_kicking_team
from robot import (
    POSITIONING_AGAINST_DRIBBLE_TEAM_RIGHT_SUPPORTER,
    POSITIONING_AGAINST_DRIBBLE_TEAM_SHOOTER,
    POSITIONING_AGAINST_DRIBBLE_TEAM_LEFT_SUPPORTER,
    POSITIONING_AGAINST_DRIBBLE_TEAM_SWEEPER,
)


class Shooter(Role):
    """
    Defender:
    - Standing in the area where the ball would be most
    likely kicked to.
    - Heading will be calculated to balance the turning
    cost and vision coverage
    """

    # Default Position of robot when ball is not in a special position
    # Stand between ball and our goal, but offset to slightly to block
    # one side of the goal
    FORWARD_MAX_X = 3250
    OFFSET_MAX_Y = 2000
    OFFSET_VALUE = 1250

    def default_position(self, robot_pos, ball_pos):

        dist_to_ball_max = 4000.0
        dist_to_ball_min = 1200.0
        goal_to_ball_dist_to_use_max = 6000.0
        goal_to_ball_dist_to_use_min = 2000.0

        goal_to_ball = ball_pos.minus(ENEMY_GOAL_CENTER)
        goal_to_ball_dist = goal_to_ball.length()

        if goal_to_ball_dist < goal_to_ball_dist_to_use_min:
            dist_to_ball = dist_to_ball_min
        elif goal_to_ball_dist > goal_to_ball_dist_to_use_max:
            dist_to_ball = dist_to_ball_max
        else:
            # Use y = mx + b to have linear line
            m = (dist_to_ball_max - dist_to_ball_min) / (goal_to_ball_dist_to_use_max - goal_to_ball_dist_to_use_min)
            b = -m * goal_to_ball_dist_to_use_min + dist_to_ball_min
            dist_to_ball = m * goal_to_ball_dist + b

        ball_to_opponent_goal_heading = ENEMY_GOAL_CENTER.minus(ball_pos).heading()

        position = ball_pos.plus(makeVector2DFromDistHeading(dist_to_ball, ball_to_opponent_goal_heading))

        if ball_pos.y > 1.1:  # just dont be 0
            position.y -= self.OFFSET_VALUE
        else:
            position.y += self.OFFSET_VALUE

        if position.x > self.FORWARD_MAX_X:
            position.x = self.FORWARD_MAX_X
        if position.y > self.OFFSET_MAX_Y:
            position.y = self.OFFSET_MAX_Y
        if position.y < -self.OFFSET_MAX_Y:
            position.y = -self.OFFSET_MAX_Y

        return position

    # Evaluate positioin, heading, position_error and heading_error
    def evaluate(self, robot_pos=myPos(), ball_pos=ballWorldPos()):

        self.position = self.default_position(robot_pos, ball_pos)

        # Calculate heading to face ball, have the highest chance of
        # seeing the ball, and calculating the velocity of it.
        position_to_ball = ball_pos.minus(self.position)
        self.heading = position_to_ball.heading()
        if self.heading > 0:
            self.heading = self.heading - math.radians(30)
        else:
            self.heading = self.heading + math.radians(30)

        # Calculate size of error we're going to allow in the position
        # and heading of the robot
        self.position_error = 400
        self.heading_error = math.radians(20)


class Supporter(Role):
    """
    Two supporters form a triangle formation along with
    the ball player
    - Provide extract vision support
    - Take cover in case the ball player fails
    - Dense formation in ball playing area
    """

    # Some flags to prevent flickering between states
    in_left_side = False
    in_our_corner = False
    near_our_goal_box = False
    goalie_is_in_goal_box = False

    MAX_OFFSET_Y = 2700
    MAX_BACKWARD_X = -3500

    def __init__(self, position=None, seq=1):
        Role.__init__(self, position)
        if seq == 1:
            self.pos_offset = 750
        else:
            self.pos_offset = -750

    def evaluate(self, robot_pos=myPos(), ball_pos=ballWorldPos()):

        # Update some flags to decide our position
        self.update_in_left_side(ball_pos)
        self.update_in_our_corner(ball_pos)
        self.update_near_our_goal_box(ball_pos)
        self.update_goalie_is_in_goal_box()

        # Decide on where to position ourselves
        if in_goal_kick():
            if we_are_kicking_team():
                self.position = self.our_goal_kick_position(ball_pos)
            else:
                self.position = self.enemy_goal_kick_position(ball_pos)
        elif in_pushing_free_kick():
            if we_are_kicking_team():
                self.position = self.our_pushing_free_kick_position(ball_pos)
            else:
                self.position = self.enemy_pushing_free_kick_position(ball_pos)
        elif in_kick_in():
            if we_are_kicking_team():
                self.position = self.our_kick_in_position(ball_pos)
            else:
                self.position = self.enemy_kick_in_position(ball_pos)
        elif self.near_our_goal_box:
            self.position = self.near_our_goal_box_position(ball_pos)
        elif self.in_our_corner:
            self.position = self.our_corner_position()
        else:
            self.position = self.default_position(robot_pos, ball_pos)

        self.position.y += self.pos_offset
        if self.goalie_is_in_goal_box:
            if self.position.x < self.MAX_BACKWARD_X:
                self.position.x = self.MAX_BACKWARD_X
        if self.position.y > self.MAX_OFFSET_Y:
            self.position.y = self.MAX_OFFSET_Y
        if self.position.y < -self.MAX_OFFSET_Y:
            self.position.y = -self.MAX_OFFSET_Y

        # Calculate heading to face ball, have the highest chance of
        # seeing the ball, and calculating the velocity of it.
        position_to_ball = ball_pos.minus(self.position)
        self.heading = position_to_ball.heading()

        # Calculate size of error we're going to allow in the position
        # and heading of the robot
        self.position_error = 200
        self.heading_error = math.radians(15)

    # Default Position of robot when ball is not in a special position
    # Stand between ball and our goal, but offset to slightly to block
    # one side of the goal
    def default_position(self, robot_pos, ball_pos):

        dist_to_ball_max = 1500.0
        dist_to_ball_min = 800.0
        goal_to_ball_dist_to_use_max = 4000.0
        goal_to_ball_dist_to_use_min = 2000.0

        goal_to_ball = ball_pos.minus(OUR_GOAL_CENTRE)
        goal_to_ball_dist = goal_to_ball.length()

        if goal_to_ball_dist < goal_to_ball_dist_to_use_min:
            dist_to_ball = dist_to_ball_min
        elif goal_to_ball_dist > goal_to_ball_dist_to_use_max:
            dist_to_ball = dist_to_ball_max
        else:
            # Use y = mx + b to have linear line
            m = (dist_to_ball_max - dist_to_ball_min) / (goal_to_ball_dist_to_use_max - goal_to_ball_dist_to_use_min)
            b = -m * goal_to_ball_dist_to_use_min + dist_to_ball_min
            dist_to_ball = m * goal_to_ball_dist + b

        ball_to_our_goal_heading = OUR_GOAL_CENTRE.minus(ball_pos).heading()

        position = ball_pos.plus(makeVector2DFromDistHeading(dist_to_ball, ball_to_our_goal_heading))

        # Offset robot position slightly to the left or right from the line
        # between our goal and ball, to cover one side of our goal.

        # If we're missin a goalkeeper, then offset less so we block more
        # of the goal centre
        offset_distance = 300 if self.goalie_is_in_goal_box else 200

        # If ball is on:
        #   LHS of field - block right side of our goal
        #   RHS of field - block left side of our goal
        offset_heading = normalisedTheta(
            OUR_GOAL_CENTRE.headingTo(ball_pos) + radians(-90) if self.in_left_side else radians(90)
        )

        offset_vec = makeVector2DFromDistHeading(offset_distance, offset_heading)

        return position.plus(offset_vec)

    # Position of robot when ball is in the corners of the field
    # near our goal end
    def our_corner_position(self):
        if self.goalie_is_in_goal_box:
            # position outside the goal box, so we don't get called for illegal
            # defender if there are already two robots in the goal box, and
            # also to avoid standing in the way of the goalie
            return Vector2D(-FIELD_LENGTH / 2 + GOAL_BOX_LENGTH + 300, 400 if self.in_left_side else -400)
        else:
            # position near our right goal post and block the goal
            post = OUR_LEFT_POST if self.in_left_side else OUR_RIGHT_POST
            return post.plus(Vector2D(250, -200 if self.in_left_side else 200))

    # Position of robot when ball is near our goal box
    def near_our_goal_box_position(self, ball_pos):

        if self.goalie_is_in_goal_box:
            # stand to the side of the ball, that doesn't block
            # our teammates and goalie getting to the ball, but
            # somewhat blocks our own goal
            ball = ball_pos
            if self.in_left_side:
                y = clamp(ball.y - 700, -400, 100)
            else:
                y = clamp(ball.y + 700, -100, 400)
            return Vector2D(ball.x, y)
        else:
            # Stand just in front of our base line, or 200mm behind
            # ball, which ever is further back
            if abs(ball_pos.y) < GOAL_POST_ABS_Y - 200:
                max_x = -FIELD_LENGTH / 2 + 50
                min_x = -FIELD_LENGTH / 2 - 100
                x = clamp(ball_pos.x - 200, min_x, max_x)
            else:
                # If ball is far out in the y-direction, stand slighly in
                # front of base line to prevent hiding in the goals
                x = -FIELD_LENGTH / 2 + 50

            # Stand between left and right post, but with a margin of 250mm
            # to prevent getting caught on the post
            max_y_abs = GOAL_POST_ABS_Y - 250
            y = clamp(ball_pos.y, -max_y_abs, max_y_abs)

            return Vector2D(x, y)

    # Position when its our goal free kick
    def our_goal_kick_position(self, ball_pos):
        # Stand sort of near the ball
        ball_from_supporter_vector = Vector2D(500, 1000 if self.in_left_side else -1000)
        position = ball_pos.minus(ball_from_supporter_vector)
        X_MAX_ABS = FIELD_LENGTH / 2 - 400
        position.x = clamp(position.x, -X_MAX_ABS, X_MAX_ABS)
        return position

    def enemy_goal_kick_position(self, ball_pos):
        # Stand in a blocking position, slightly offset
        # to not overlap with ballplayer
        ball_from_supporter_vector = Vector2D(1600, 300 if self.in_left_side else -300)
        position = ball_pos.minus(ball_from_supporter_vector)
        X_MAX_ABS = FIELD_LENGTH / 2 - 400
        position.x = clamp(position.x, -X_MAX_ABS, X_MAX_ABS)
        return position

    def our_pushing_free_kick_position(self, ball_pos):
        # Stand sort of near the ball
        ball_from_supporter_vector = Vector2D(500, 1000 if self.in_left_side else -1000)
        position = ball_pos.minus(ball_from_supporter_vector)
        X_MAX_ABS = FIELD_LENGTH / 2 - 400
        position.x = clamp(position.x, -X_MAX_ABS, X_MAX_ABS)
        return position

    def enemy_pushing_free_kick_position(self, ball_pos):
        # Stand between ball and our goal, sligtly offset
        # to not overlap with ballplayer
        ball_from_supporter_vector = Vector2D(1600, 500 if self.in_left_side else -500)
        ball_from_supporter_vector.rotate(ball_pos.minus(OUR_GOAL_CENTRE).heading())
        position = ball_pos.minus(ball_from_supporter_vector)
        X_MAX_ABS = FIELD_LENGTH / 2 - 400
        position.x = clamp(position.x, -X_MAX_ABS, X_MAX_ABS)
        return position

    # Position when its opponents kick in
    def enemy_kick_in_position(self, ball_pos):
        if self.in_left_side:
            # Stand between ball and right goal post
            heading = ball_pos.minus(OUR_RIGHT_POST).heading()
            ball_from_supporter_vector = Vector2D(2000, 0).rotate(heading)
        else:
            # Stand between ball and left goal post
            heading = ball_pos.minus(OUR_LEFT_POST).heading()
            ball_from_supporter_vector = Vector2D(2000, 0).rotate(heading)
        position = ball_pos.minus(ball_from_supporter_vector)
        X_MAX_ABS = FIELD_LENGTH / 2 - 400
        position.x = clamp(position.x, -X_MAX_ABS, X_MAX_ABS)
        return position

    # Position when its our kick in
    def our_kick_in_position(self, ball_pos):
        # Stand sort of near the ball
        ball_from_supporter_vector = Vector2D(500, 1000 if self.in_left_side else -1000)
        ball_from_supporter_vector.rotate(ball_pos.minus(OUR_GOAL_CENTRE).heading())
        position = ball_pos.minus(ball_from_supporter_vector)
        X_MAX_ABS = FIELD_LENGTH / 2 - 400
        position.x = clamp(position.x, -X_MAX_ABS, X_MAX_ABS)
        return position

    # Update whether ball is in our goal box
    def update_near_our_goal_box(self, ball_pos):
        if self.near_our_goal_box:
            if not isInOurGoalBox(ball_pos, buffx=300, buffy=300):
                self.near_our_goal_box = False
        else:
            if isInOurGoalBox(ball_pos, buffx=100, buffy=100):
                self.near_our_goal_box = True

    # Update whether ball is in left side of field
    def update_in_left_side(self, ball_pos):
        if self.in_left_side:
            if ball_pos.y < -300:
                self.in_left_side = False
        else:
            if ball_pos.y > 300:
                self.in_left_side = True

    # Update whether ball is in our corner
    def update_in_our_corner(self, ball_pos):
        goal_to_ball_heading_abs = abs(OUR_GOAL_CENTRE.headingTo(ball_pos))
        if self.in_our_corner:
            if goal_to_ball_heading_abs < radians(60):
                self.in_our_corner = False
        else:
            if goal_to_ball_heading_abs > radians(70):
                self.in_our_corner = True

    # Update whether our goalie is in the goal box.
    # If goalie is in the goal box, we don't have to defend the center
    # of our goal. If goalie isn't, we have to defend our goal centre more.
    def update_goalie_is_in_goal_box(self):
        sweeper_player = -1
        for i in get_active_player_numbers():
            role_index = player_role(i)
            if role_index == POSITIONING_AGAINST_DRIBBLE_TEAM_SWEEPER:  # noqa
                sweeper_player = i
                break

        if sweeper_player == -1:
            return False

        if player_number_is_incapacitated(sweeper_player):
            # Goalie is incapacitated, so its not in the goalbox
            self.goalie_is_in_goal_box = False
        else:
            # Check if goalie is inside goal box, with a margin of 200mm
            goalie_pos = get_teammate_pos(sweeper_player)
            if self.goalie_is_in_goal_box:
                if not isInOurGoalBox(goalie_pos, buffx=200, buffy=200):
                    self.goalie_is_in_goal_box = False
            else:
                if isInOurGoalBox(goalie_pos):
                    self.goalie_is_in_goal_box = True


class Sweeper(Role):
    """
    Sweeper:
    - Standing behind the triangle formation
    - Sweep the ball if it gets passed our defense
    - Can assess goalbox but will not go beyond the midline
    """

    # Position of robot when ball is inside enemy's goal box
    def inside_enemy_goal_box_position(self, ball_pos):
        return ball_pos.minus(Vector2D(800, 0))

    # Position of robot when ball is in enemy corner
    def enemy_corner_position(self):
        return Vector2D(3500, 0)

    # Default Position of robot when ball isn't in a special position
    # Stand between ball and one of the opponent's goal posts, but
    # offset to ensure we're not blocking our teammate's shooting path
    def default_position(self, robot_pos, ball_pos):

        if ball_pos.x < -2000:
            position = OUR_GOAL_CENTRE.plus(Vector2D(300, 0))
            return position

        dist_to_ball = min(4000, OUR_GOAL_CENTRE.minus(ball_pos).length())

        ball_to_our_goal_heading = OUR_GOAL_CENTRE.minus(ball_pos).heading()

        position = ball_pos.plus(makeVector2DFromDistHeading(dist_to_ball, ball_to_our_goal_heading))

        if position.x > 0:
            position.x = -1000
        if position.x < -4500:
            position.x = -4300

        return position

    # Evaluate positioin, heading, position_error and heading_error
    def evaluate(self, robot_pos=myPos(), ball_pos=ballWorldPos()):

        self.position = self.default_position(robot_pos, ball_pos)

        # Calculate heading to face ball, have the highest chance of
        # seeing the ball, and calculating the velocity of it.
        position_to_ball = ball_pos.minus(self.position)
        self.heading = position_to_ball.heading()

        # Calculate size of error we're going to allow in the position
        # and heading of the robot
        self.position_error = 400
        self.heading_error = math.radians(15)


class PositioningAgainstDribbleTeam(Positioning):
    """
    Tactic follows following priority:
    1. A right supporter is the first priority whose duty is to provide
        extract vision and be prepare to be ball player
    2. A Shooter holds the position where the ball is most likely
        to be kicked to, facing towards a heading between ball and goal
    3. Left supporter is identical to right supporter except for the
        following position
    4. Sweeper is the additional FieldPlayer since we don't really need
        a goalkeeper in this case.

    Depending on number of robots available, we fill different roles,
    as following.

    | # of robots |           |          |           |
    --------------------------------------------------
    |      1      | right_supporter |          |           |
    |      2      | right_supporter | Shooter |           |
    |      3      | right_supporter | Shooter | left_supporter |
    |      4      | right_supporter | Shooter | left_supporter | sweeper |

    The role assignment is done by calculating the time of each
    teammate's current position to each role's desired position. It is
    important to note that the desired position of a role can depend on the
    robot's current position. One player's desired defender position
    will be different to another player's defender position. Therefore,
    we must compute the time of each teammate's current position
    to the teammate's desired position for all roles.
    We then enumerate through all possibilities of role assignment and find
    the role assignment that minimises the longest time a robot has to
    travel. If there are multiple role assignments that have the same longest
    time a robot has to travel, it then tries to minimise the second
    longest time a robot has to travel.
    """

    # Constructor
    def __init__(self):

        # Initialise role name, set to highest priority role
        self.my_role_name = "right_supporter"

        # Array containing all roles, in PRIORITY ORDER
        # we can only have up to three anticipating robots,
        # considering that one robot is a goalie and we
        # always have at least one playing the ball and not
        # anticipating
        self.roles = [
            ["right_supporter", POSITIONING_AGAINST_DRIBBLE_TEAM_RIGHT_SUPPORTER, Supporter(self, 2)],  # noqa
            ["shooter", POSITIONING_AGAINST_DRIBBLE_TEAM_SHOOTER, Shooter(self)],
            ["left_supporter", POSITIONING_AGAINST_DRIBBLE_TEAM_LEFT_SUPPORTER, Supporter(self, 1)],
            ["sweeper", POSITIONING_AGAINST_DRIBBLE_TEAM_SWEEPER, Sweeper(self)],
        ]

    # Evaluate position parameters
    def evaluate(self):

        anticipators_current_positions = []
        anticipators_current_headings = []
        anticipators_ball_positions = []
        player_numbers = []

        # always have our position at the first index
        # of the following arrays
        anticipators_current_positions.append(myPos())
        anticipators_current_headings.append(myHeading())
        anticipators_ball_positions.append(ballWorldPos())
        player_numbers.append(my_player_number())

        # num_ball_players
        num_ball_players = 0

        # check for team mates in a good position
        for player in get_active_player_numbers():
            if player is my_player_number():
                continue
            elif player_is_playing_ball(player):
                num_ball_players += 1
            else:
                anticipators_current_positions.append(get_teammate_pos(player))
                anticipators_current_headings.append(get_teammate_heading(player))
                if get_teammate_seconds_since_last_ball_update(player) > 5.0:
                    anticipators_ball_positions.append(ballWorldPos())
                else:
                    anticipators_ball_positions.append(teammate_ego_ball(player))  # noqa
                player_numbers.append(player)

        # Make decision, depending on how many floaters are
        # available
        num_anticipators = len(anticipators_current_positions)

        # In the case we have two ball players, we don't want role
        # switching to happen. To account for this, we ignore the number
        # of excessive ball players from the priority list
        num_excessive_ball_players = num_ball_players - 1

        # We've got same number of positions as number of players
        num_roles = num_anticipators

        # Cut list to number of floaters
        possible_roles = self.roles[num_excessive_ball_players : num_roles + num_excessive_ball_players]

        # Calculate distance of every anticipator's current position
        # to their role positions and store in array
        # x : floater index, y: position index
        time_to_reach_poses = [[10000000 for _ in range(num_anticipators)] for _ in range(num_roles)]
        for x, current_position in enumerate(anticipators_current_positions):
            ball_position = anticipators_ball_positions[x]
            for y, role in enumerate(possible_roles):
                role[OBJECT_INDEX].evaluate(current_position, ball_position)
                t = calculateTimeToReachPose(
                    current_position,
                    anticipators_current_headings[x],
                    role[OBJECT_INDEX].position,
                    role[OBJECT_INDEX].heading,
                )

                time_to_reach_poses[x][y] = t

        # Go through possible permutations, and find best arrangement of
        # floaters
        permutations = itertools.permutations(range(num_anticipators))
        best_role_index = None

        # Create array of times of robots to reach respective positioning,
        # with each times array being in descending order. Also store in the
        # array, what our role_enum should be
        all_times_and_my_role = []
        for permutation in permutations:
            times = []
            for anticipator_index, role_index in enumerate(permutation):
                times.append(time_to_reach_poses[anticipator_index][role_index])
            times.sort(reverse=True)  # sort list in descending order
            all_times_and_my_role.append([times, permutation[0]])

        # Sort in order of lowest times
        # NOTE: this will sort in order of first element of the times.
        # If there are multiple values that are equal, it will then
        # check the second element. That way, we can ensure that we're
        # not only looking at the "longest robot's time", but the "second
        # longest robot's time" in case there are multiple permutations that
        # have the same "longest robot's time"
        all_times_and_my_role.sort()
        # print(all_times_and_my_role[0], num_excessive_ball_players)
        best_role_index = all_times_and_my_role[0][1] + num_excessive_ball_players
        # print(best_role_index)

        # Evaluate my position again
        self.my_role_name = self.role_index_to_name(best_role_index)
        self.roles[self.get_my_role_index()][OBJECT_INDEX].evaluate(myPos(), ballWorldPos())  # noqa
