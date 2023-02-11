from Positioning import Positioning, Role
from util.Vector2D import Vector2D
from robot import POSITIONING_FIND_BALL_FINDER
from math import radians
from util.Global import myPos, myHeading
from util.FieldGeometry import calculateTimeToReachPose
from util.TeamStatus import (
    get_active_player_numbers,
    player_role,
    get_teammate_heading,
    get_teammate_pos,
    my_player_number,
)
import itertools


class Finder(Role):

    FINDER_POSITION_ONE_ROBOT = [Vector2D(-3000, 0)]
    FINDER_POSITION_TWO_ROBOTS = [Vector2D(-3500, 500), Vector2D(-1500, -500)]
    FINDER_POSITION_THREE_ROBOTS = [Vector2D(-3500, 500), Vector2D(-1500, -500), Vector2D(1000, 500)]
    FINDER_POSITION_FOUR_ROBOTS = [
        Vector2D(-3500, 500),
        Vector2D(-2000, -500),
        Vector2D(1000, 500),
        Vector2D(3000, -500),
    ]
    FINDER_POSITION_FIVE_ROBOTS = [
        Vector2D(-4000, 500),
        Vector2D(-2500, -500),
        Vector2D(500, 500),
        Vector2D(2000, -500),
        Vector2D(3000, 500),
    ]

    def evaluate(self):
        current_finder_positions = []
        current_finder_headings = []
        finder_player_numbers = []

        # always assume that you're in the calculation of
        # positioning, even if you're playing the ball
        current_finder_positions.append(myPos())
        current_finder_headings.append(myHeading())

        # check for team mates that are also finders
        for player in get_active_player_numbers():
            if self.positioning.role_enum_to_name(player_role(player)) == "finder" and player is not my_player_number():
                player_pos = get_teammate_pos(player)
                current_finder_positions.append(player_pos)
                player_heading = get_teammate_heading(player)
                current_finder_headings.append(player_heading)
                finder_player_numbers.append(player)

        # Make decision, depending on how many finders are
        # available
        num_finders = len(current_finder_positions)

        # Get array of possible positions
        possible_positions = self.FINDER_POSITION_ONE_ROBOT
        if num_finders == 2:
            possible_positions = self.FINDER_POSITION_TWO_ROBOTS
        elif num_finders == 3:
            possible_positions = self.FINDER_POSITION_THREE_ROBOTS
        elif num_finders == 4:
            possible_positions = self.FINDER_POSITION_FOUR_ROBOTS
        elif num_finders == 5:
            possible_positions = self.FINDER_POSITION_FIVE_ROBOTS

        # Calculate distance of every finder's current position
        # to possible positions and store in array
        # x : floater index, y: position index
        time_to_reach_poses = [[10000000 for _ in range(num_finders)] for _ in range(num_finders)]
        for x, current_position in enumerate(current_finder_positions):
            for y, possible_position in enumerate(possible_positions):
                t = calculateTimeToReachPose(current_position, current_finder_headings[y], possible_position)

                time_to_reach_poses[x][y] = t

        # Go through possible permutations, and find best arrangement of
        # floaters
        permutations = itertools.permutations(range(num_finders))
        best_position_index = None

        # Create array of times of robots to reach respective positioning,
        # with each times array being in descending order. Also store in the
        # array, what our position_enum should be
        all_times_and_my_position = []
        for permutation in permutations:
            times = []
            for finder_index, position_index in enumerate(permutation):
                times.append(time_to_reach_poses[finder_index][position_index])
            times.sort(reverse=True)  # sort list in descending order
            all_times_and_my_position.append([times, permutation[0]])

        # Sort in order of lowest times
        # NOTE: this will sort in order of first element of the times.
        # If there are multiple values that are equal, it will then
        # check the second element. That way, we can ensure that we're
        # not only looking at the "longest robot's time", but the "second
        # longest robot's time" in case there are multiple permutations that
        # have the same "longest robot's time"
        all_times_and_my_position.sort()
        best_position_index = all_times_and_my_position[0][1]

        # Evaluate my position again
        self.position = possible_positions[best_position_index]
        self.heading = 0
        self.position_error = 400
        self.heading_error = radians(30)


class PositioningFindBall(Positioning):
    def __init__(self):
        self.my_role_name = "finder"

        self.roles = [["finder", POSITIONING_FIND_BALL_FINDER, Finder(self)]]
