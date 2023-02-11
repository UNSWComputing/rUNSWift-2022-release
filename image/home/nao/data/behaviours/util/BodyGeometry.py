from util.Global import myPos, myHeading
from util.Vector2D import Vector2D
from util.Constants import TOE_CENTRE_X, HIP_OFFSET
from robot import Foot

left_toe_vector = Vector2D(TOE_CENTRE_X, HIP_OFFSET)
right_toe_vector = Vector2D(TOE_CENTRE_X, -HIP_OFFSET)


def toePos(foot=Foot.LEFT):
    toe_vec = left_toe_vector if foot is Foot.LEFT else right_toe_vector
    toe_vector_relative_to_me = toe_vec.rotated(myHeading())
    toe_pos = myPos().plus(toe_vector_relative_to_me)
    return toe_pos
