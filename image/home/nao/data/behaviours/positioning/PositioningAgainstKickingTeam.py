# Positioning against teams that kick a lot
#
# Teams include
# - rUNSWift
# - B-Human
# - HULKs
# - TJArk

import math
import itertools
from Positioning import Positioning, Role, OBJECT_INDEX
from util.TeamStatus import my_player_number
from util.Global import ballWorldPos, myPos, myHeading
from util.Vector2D import Vector2D, makeVector2DFromDistHeading
from util.FieldGeometry import (
    OUR_GOAL_CENTRE,
    ENEMY_LEFT_POST,
    ENEMY_RIGHT_POST,
    ENEMY_GOAL_CENTER,
    isInOpponentGoalBox,
    isInOurGoalBox,
    isInOurPenaltyBox,
    OUR_LEFT_POST,
    OUR_RIGHT_POST,
)
from util.MathUtil import clamp, normalisedTheta, closest_point_on_segment
from util.Constants import FIELD_LENGTH, GOAL_BOX_LENGTH, PENALTY_AREA_LENGTH, GOAL_POST_ABS_Y
from math import radians
from util.TeamStatus import (
    teammate_ego_ball,
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
    POSITIONING_AGAINST_KICKING_TEAM_SUPPORTER,
    POSITIONING_AGAINST_KICKING_TEAM_DEFENDER,
    POSITIONING_AGAINST_KICKING_TEAM_UPFIELDER,
)


class Defender(Role):
    """
    Defender:
    - Block one side of our goal from opponent shooting
    - Blocks centre of goal if ball is near our own goal, and goalie
      is incapacitated
    - Position in such a way to get out of the way of our teammates
      trying to clear the ball, while blocking the opponent's shooting
      path
    - Block "crossing" paths, where the opponent passes the ball
      in from the field corners at our end
    """

    # Some flags to prevent flickering between states
    in_left_side = False
    in_our_corner = False
    near_our_goal_box = False
    goalie_is_in_goal_box = False

    MIN_X_WHEN_GOALIE_IN_BOX = -FIELD_LENGTH / 2 + GOAL_BOX_LENGTH + 200

    # Update whether ball is in our goal box
    def update_near_our_goal_box(self, ball_pos):
        if self.near_our_goal_box:
            self.near_our_goal_box = isInOurGoalBox(ball_pos, buffx=700, buffy=700)
        else:
            self.near_our_goal_box = isInOurGoalBox(ball_pos, buffx=500, buffy=500)

    # Update whether ball is in left side of field
    def update_in_left_side(self, ball_pos):
        if self.in_left_side:
            self.in_left_side = ball_pos.y > -300
        else:
            self.in_left_side = ball_pos.y > 300

    # Update whether ball is in our corner
    def update_in_our_corner(self, ball_pos):
        goal_to_ball_heading_abs = abs(OUR_GOAL_CENTRE.headingTo(ball_pos))
        if self.in_our_corner:
            self.in_our_corner = goal_to_ball_heading_abs > radians(60)
        else:
            self.in_our_corner = goal_to_ball_heading_abs > radians(70)

    # Update whether our goalie is in the goal box.
    # If goalie is in the goal box, we don't have to defend the center
    # of our goal. If goalie isn't, we have to defend our goal centre more.
    def update_goalie_is_in_goal_box(self):
        if player_number_is_incapacitated(1):
            # Goalie is incapacitated, so its not in the goalbox
            self.goalie_is_in_goal_box = False
        else:
            # Check if goalie is inside goal box, with a margin of 200mm
            goalie_pos = get_teammate_pos(1)
            if self.goalie_is_in_goal_box:
                self.goalie_is_in_goal_box = isInOurGoalBox(goalie_pos, buffx=200, buffy=200)
            else:
                self.goalie_is_in_goal_box = isInOurGoalBox(goalie_pos)

    # Position of robot when ball is near our goal box
    def near_our_goal_box_position(self):

        if self.in_left_side:
            y = -400
        else:
            y = 400

        if self.goalie_is_in_goal_box:
            x = -FIELD_LENGTH / 2 + GOAL_BOX_LENGTH + 200
        else:
            x = -FIELD_LENGTH / 2

        return Vector2D(x, y)

    # Position of robot when ball is in the corners of the field
    # near our goal end
    def our_corner_position(self):
        if self.goalie_is_in_goal_box:
            # position outside the goal box, so we don't get called for illegal
            # defender if there are already two robots in the goal box, and
            # also to avoid standing in the way of the goalie
            return Vector2D(-FIELD_LENGTH / 2 + GOAL_BOX_LENGTH + 300, 0)
        else:
            return Vector2D(-FIELD_LENGTH / 2 + GOAL_BOX_LENGTH, 0)

    # Default Position of robot when ball is not in a special position
    # Stand between ball and our goal, but offset to slightly to block
    # one side of the goal
    def default_position(self, robot_pos, ball_pos):
        # Calculate closest point from me to segment robot has to be on
        P, Q = self.calculate_close_and_far_point_to_goal(ball_pos=ball_pos)
        point_on_segment = closest_point_on_segment(robot_pos, P, Q)

        # Offset robot position slightly to the left or right from the line
        # between our goal and ball, to cover one side of our goal.
        offset_distance = 300

        # If ball is on:
        #   LHS of field - block right side of our goal
        #   RHS of field - block left side of our goal
        offset_heading = normalisedTheta(
            OUR_GOAL_CENTRE.headingTo(ball_pos) + radians(-90) if self.in_left_side else radians(90)
        )

        offset_vec = makeVector2DFromDistHeading(offset_distance, offset_heading)

        return point_on_segment.plus(offset_vec)

    # Returns close and far point given to a post
    def calculate_close_and_far_point_to_goal(
        self,
        close_point_min_dist_to_goal=800,
        close_point_max_dist_to_goal=3000,
        far_point_min_dist_to_goal=900,
        far_point_max_dist_to_goal=3200,
        max_opponent_kick_distance=5000,
        ball_pos=ballWorldPos(),
    ):

        if self.goalie_is_in_goal_box:
            far_point_min_dist_to_goal += 300

        goal_to_ball = ball_pos.minus(OUR_GOAL_CENTRE)
        goal_to_ball_dist = goal_to_ball.length()
        goal_to_ball_heading = goal_to_ball.heading()

        # Position ourselves at a point where the opponent's strongest
        # possible kick would just reach, clamped by how far out we
        # allow the defender to come out and go in to the goals
        dist_close_point = clamp(
            goal_to_ball_dist - max_opponent_kick_distance, close_point_min_dist_to_goal, close_point_max_dist_to_goal
        )

        dist_far_point = clamp(
            goal_to_ball_dist - max_opponent_kick_distance, far_point_min_dist_to_goal, far_point_max_dist_to_goal
        )

        close_point = OUR_GOAL_CENTRE.plus(makeVector2DFromDistHeading(dist_close_point, goal_to_ball_heading))
        far_point = OUR_GOAL_CENTRE.plus(makeVector2DFromDistHeading(dist_far_point, goal_to_ball_heading))

        return close_point, far_point

    # Evaluate positioin, heading, position_error and heading_error
    def evaluate(self, robot_pos=myPos(), ball_pos=ballWorldPos()):

        # Update some flags to decide our position
        self.update_in_left_side(ball_pos)
        self.update_in_our_corner(ball_pos)
        self.update_near_our_goal_box(ball_pos)
        self.update_goalie_is_in_goal_box()

        # Decide on where to position ourselves
        if self.near_our_goal_box:
            self.position = self.near_our_goal_box_position()
        elif self.in_our_corner:
            self.position = self.our_corner_position()
        else:
            self.position = self.default_position(robot_pos, ball_pos)

        if self.goalie_is_in_goal_box:
            if self.position.x < self.MIN_X_WHEN_GOALIE_IN_BOX:
                self.position.x = self.MIN_X_WHEN_GOALIE_IN_BOX

        # Calculate heading to face ball, have the highest chance of
        # seeing the ball, and calculating the velocity of it.
        position_to_ball = ball_pos.minus(self.position)
        self.heading = position_to_ball.heading()

        # Calculate size of error we're going to allow in the position
        # and heading of the robot
        self.position_error = 300
        self.heading_error = math.radians(10)


class Supporter(Role):
    """
    Support the ball player near the ball, and also stand
    between our goal and the ball, on one side. This side
    is opposite from the defender, and depends on if the
    ball is on the LHS or RHS of the field.
    Supporter also goes into special positions in free kicks, since
    - we have to avoid clashing positions with ballplayer in enemy free kicks
    - we can be a bit more offensive on our free kicks
    """

    # Some flags to prevent flickering between states
    in_left_side = False
    in_our_corner = False
    near_our_penalty_box = False
    goalie_is_in_goal_box = False
    goalie_is_in_penalty_box = False

    MIN_X_WHEN_GOALIE_IN_BOX = -FIELD_LENGTH / 2 + PENALTY_AREA_LENGTH + 200

    # Update whether ball is in our goal box
    def update_near_our_penalty_box(self, ball_pos):
        if self.near_our_penalty_box:
            self.near_our_penalty_box = isInOurGoalBox(ball_pos, buffx=300, buffy=300)
        else:
            self.near_our_penalty_box = isInOurGoalBox(ball_pos, buffx=100, buffy=100)

    # Update whether ball is in left side of field
    def update_in_left_side(self, ball_pos):
        if self.in_left_side:
            self.in_left_side = ball_pos.y > -300
        else:
            self.in_left_side = ball_pos.y > 300

    # Update whether ball is in our corner
    def update_in_our_corner(self, ball_pos):
        goal_to_ball_heading_abs = abs(OUR_GOAL_CENTRE.headingTo(ball_pos))
        if self.in_our_corner:
            self.in_our_corner = goal_to_ball_heading_abs > radians(60)
        else:
            self.in_our_corner = goal_to_ball_heading_abs > radians(70)

    # Update whether our goalie is in the goal box.
    # If goalie is in the goal box, we don't have to defend the center
    # of our goal. If goalie isn't, we have to defend our goal centre more.
    def update_goalie_is_in_goal_box(self):
        if player_number_is_incapacitated(1):
            # Goalie is incapacitated, so its not in the goalbox
            self.goalie_is_in_goal_box = False
        else:
            # Check if goalie is inside goal box, with a margin of 200mm
            goalie_pos = get_teammate_pos(1)
            if self.goalie_is_in_goal_box:
                self.goalie_is_in_goal_box = isInOurGoalBox(goalie_pos, buffx=200, buffy=200)
            else:
                self.goalie_is_in_goal_box = isInOurGoalBox(goalie_pos)

    # Update whether our goalie is in the penalty box.
    # If goalie is in the penalty box, we shouldn't be otherwise we might get done
    # for illegal positioning. If goalie isn't, we have to defend our goal centre more.
    def update_goalie_is_in_penalty_box(self):
        if player_number_is_incapacitated(1):
            # Goalie is incapacitated, so its not in the goalbox
            self.goalie_is_in_penalty_box = False
        else:
            # Check if goalie is inside goal box, with a margin of 200mm
            goalie_pos = get_teammate_pos(1)
            if self.goalie_is_in_penalty_box:
                self.goalie_is_in_penalty_box = isInOurPenaltyBox(goalie_pos, buffx=200, buffy=200)
            else:
                self.goalie_is_in_penalty_box = isInOurPenaltyBox(goalie_pos)

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
        if self.goalie_is_in_penalty_box:
            # position outside the penalty box, so we don't get called for illegal
            # positioning if there are already three robots in the penalty box, and
            # also to avoid standing in the way of the goalie
            return Vector2D(-FIELD_LENGTH / 2 + PENALTY_AREA_LENGTH + 300, 400 if self.in_left_side else -400)
        else:
            # position near our right goal post and block the goal
            post = OUR_LEFT_POST if self.in_left_side else OUR_RIGHT_POST
            return post.plus(Vector2D(250, -200 if self.in_left_side else 200))

    # Position of robot when ball is near our goal box
    def near_our_penalty_box_position(self, ball_pos):

        if self.goalie_is_in_penalty_box:
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

    def evaluate(self, robot_pos=myPos(), ball_pos=ballWorldPos):
        # Update some flags to decide our position
        self.update_in_left_side(ball_pos)
        self.update_in_our_corner(ball_pos)
        self.update_near_our_penalty_box(ball_pos)
        self.update_goalie_is_in_goal_box()
        self.update_goalie_is_in_penalty_box()

        # Decide on where to position ourselves
        if in_goal_kick():
            if we_are_kicking_team():
                self.position = self.our_goal_free_kick_position(ball_pos)
            else:
                self.position = self.enemy_goal_free_kick_position(ball_pos)
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
        elif self.near_our_penalty_box:
            self.position = self.near_our_penalty_box_position(ball_pos)
        elif self.in_our_corner:
            self.position = self.our_corner_position()
        else:
            self.position = self.default_position(robot_pos, ball_pos)

        if self.goalie_is_in_penalty_box:
            if self.position.x < self.MIN_X_WHEN_GOALIE_IN_BOX:
                self.position.x = self.MIN_X_WHEN_GOALIE_IN_BOX

        # Calculate heading to face ball, have the highest chance of
        # seeing the ball, and calculating the velocity of it.
        position_to_ball = ball_pos.minus(self.position)
        self.heading = position_to_ball.heading()

        # Calculate size of error we're going to allow in the position
        # and heading of the robot
        self.position_error = 300
        self.heading_error = math.radians(15)


class Upfielder(Role):
    """
    Upfielder:
    - Prevent blocking teammate's shooting path
    - Position to follow up a teammate's kick that didn't score
    - Position to receive a cross from a teammate
    - Position to get out of the way of another robot scoring
    """

    # Some flags to prevent flickering between states
    in_left_side = False
    in_enemy_corner = False
    inside_enemy_goal_box = False

    # Update whether ball is in enemy goal box
    def update_inside_enemy_goal_box(self, ball_pos):
        if self.inside_enemy_goal_box:
            self.inside_enemy_goal_box = isInOpponentGoalBox(ball_pos, buffx=100, buffy=100)
        else:
            self.inside_enemy_goal_box = isInOpponentGoalBox(ball_pos, buffx=-100, buffy=-100)

    # Update whether ball is in left side of field
    def update_in_left_side(self, ball_pos):
        if self.in_left_side:
            self.in_left_side = ball_pos.y > -500
        else:
            self.in_left_side = ball_pos.y > 500

    # Update whether ball is in enemy corner
    def update_in_enemy_corner(self, ball_pos):
        ball_to_goal_heading_abs = abs(ENEMY_GOAL_CENTER.minus(ball_pos).heading())
        if self.in_enemy_corner:
            self.in_enemy_corner = ball_to_goal_heading_abs > radians(40)
        else:
            self.in_enemy_corner = ball_to_goal_heading_abs > radians(60)

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

        # If ball is on:
        #   LHS of field - stand on RHS of shooting path
        #   RHS of field - stand on LHS of shooting path
        post = ENEMY_RIGHT_POST if self.in_left_side else ENEMY_LEFT_POST

        # Calculate closest point from me to segment robot has to be on
        P, Q = self.calculate_close_and_far_point_to_post(post, ball_pos=ball_pos)
        point_on_segment = closest_point_on_segment(robot_pos, P, Q)

        # Offset robot position to the left/right from the line between ball
        # and enemy goal post to prevent accidentally blocking shots
        offset_distance = 400
        offset_heading = normalisedTheta(ball_pos.headingTo(post) + radians(-90) if self.in_left_side else radians(90))

        offset_vec = makeVector2DFromDistHeading(offset_distance, offset_heading)

        return point_on_segment.plus(offset_vec)

    # Returns close and far point given to a post
    def calculate_close_and_far_point_to_post(
        self,
        post_pos,
        close_point_min_distance_to_post=1200,
        far_point_min_distance_to_post=800,
        close_point_default_dist_to_ball=4000,
        far_point_default_dist_to_ball=6000,
        ball_pos=ballWorldPos(),
    ):

        ball_to_post = post_pos.minus(ball_pos)
        ball_to_post_dist = ball_to_post.length()
        ball_to_post_heading = ball_to_post.heading()

        # Calculate maximum distance of close and far point
        # to prevent being too close to right post
        max_close_point_dist = ball_to_post_dist - close_point_min_distance_to_post
        max_far_point_dist = ball_to_post_dist - far_point_min_distance_to_post

        dist_ball_to_close_point = min(max_close_point_dist, close_point_default_dist_to_ball)
        dist_ball_to_far_point = min(max_far_point_dist, far_point_default_dist_to_ball)

        close_point = ball_pos.plus(makeVector2DFromDistHeading(dist_ball_to_close_point, ball_to_post_heading))
        far_point = ball_pos.plus(makeVector2DFromDistHeading(dist_ball_to_far_point, ball_to_post_heading))

        return close_point, far_point

    # Evaluate positioin, heading, position_error and heading_error
    def evaluate(self, robot_pos=myPos(), ball_pos=ballWorldPos()):

        # Update some flags to decide our position
        self.update_in_left_side(ball_pos)
        self.update_in_enemy_corner(ball_pos)
        self.update_inside_enemy_goal_box(ball_pos)

        # Decide on where to position ourselves
        if self.in_enemy_corner:
            self.position = self.enemy_corner_position()
        elif self.inside_enemy_goal_box:
            self.position = self.inside_enemy_goal_box_position(ball_pos)
        else:
            self.position = self.default_position(robot_pos, ball_pos)

        # Calculate heading to face ball, have the highest chance of
        # seeing the ball, and calculating the velocity of it.
        position_to_ball = ball_pos.minus(self.position)
        self.heading = position_to_ball.heading()

        # Calculate size of error we're going to allow in the position
        # and heading of the robot
        self.position_error = 400
        self.heading_error = math.radians(15)


class PositioningAgainstKickingTeam(Positioning):
    """
    Tactic follows following priority:
    1. Have a supporter, that is near or slighly behind the ball.
       This robot should provide instant support when the opponent
       dribbles past our ballplayer.
    2. Have a defender, that is standing between the ball and our
       goal. This robot is positioned to be in the way of opponent's
       shooting paths, to prevent counter attacks, and also to
       help sweep the ball in case the goalie dived.
    3. An upfielder, stands between the ball and the opponent goal,
       but offset to one side to not stand in the way of our teammate's
       shooting path. Mainly in charge of following up shots blocked by
       the enemy's goalie or defender.

    Depending on number of robots available, we fill different roles,
    as following.

    | # of robots |           |          |           |
    --------------------------------------------------
    |      1      | supporter |          |           |
    |      2      | supporter | defender |           |
    |      3      | supporter | defender | upfielder |

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
        self.my_role_name = "supporter"

        # Array containing all roles, in PRIORITY ORDER
        # we can only have up to three anticipating robots,
        # considering that one robot is a goalie and we
        # always have at least one playing the ball and not
        # anticipating
        self.roles = [
            ["supporter", POSITIONING_AGAINST_KICKING_TEAM_SUPPORTER, Supporter(self)],
            ["defender", POSITIONING_AGAINST_KICKING_TEAM_DEFENDER, Defender(self)],
            ["upfielder", POSITIONING_AGAINST_KICKING_TEAM_UPFIELDER, Upfielder(self)],
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

        num_ball_players = 0

        # check for team mates in a good position
        for player in get_active_player_numbers():
            if player is my_player_number():
                continue
            elif player == 1:
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
        best_role_index = all_times_and_my_role[0][1] + num_excessive_ball_players

        # Evaluate my position again
        self.my_role_name = self.role_index_to_name(best_role_index)
        self.roles[self.get_my_role_index()][OBJECT_INDEX].evaluate(myPos(), ballWorldPos())  # noqa
