from util.Global import myPos

# Index of name, enum and object in roles array
NAME_INDEX = 0
ENUM_INDEX = 1
OBJECT_INDEX = 2


class Positioning:
    """
    Abstract class for positionings. Used by FieldPlayer to
    evaluate the optimal position, and heading of a FieldPlayer
    on the field.
    It decides what role the robot should be, and can provide to
    FieldPlayer, the optimal
    - position
    - heading
    - position error
    - heading error
    of a robot in that particular role
    """

    # Array containing all roles (name, enum, object).
    roles = []

    # __init__ function that gets called upon construction
    # You should initialise:
    # - self.my_role_name
    # - anything else you want to initialise
    def __init__(self):
        pass

    # Evaluate positioning
    # - position
    # - heading
    # - position_error
    # - heading_error
    #
    # This must be called by FieldPlayer.
    def evaluate(self, robot_pos=myPos()):
        self.evaluate_current_role()

    ##########################################
    # VARIABLES AND FUNCTIONS BELOW MUST NOT #
    #  BE OVERRIDDEN BY THE INHERITED CLASS  #
    ##########################################

    # What robot's current role is, this should be initalised in
    # init() and updated by `decide_my_role()` if there is dynamic
    # role assignment
    my_role_name = None

    # Function to convert role enum to role name
    def role_enum_to_name(self, role_enum):
        for name, enum, _ in self.roles:
            if role_enum == enum:
                return name
        return None

    # Function to convert role name to role enum
    def role_name_to_enum(self, role_name):
        for name, enum, _ in self.roles:
            if role_name == name:
                return enum
        return -1

    # Getter for my_role_name
    def get_my_role_name(self):
        return self.my_role_name

    # Getter for my_role's enum value
    def get_my_role_enum(self):
        return self.role_name_to_enum(self.my_role_name)

    def get_my_role_index(self):
        return self.role_name_to_index(self.my_role_name)

    # Get index of role name in roles array
    def role_name_to_index(self, name):
        for i in range(len(self.roles)):
            if self.roles[i][NAME_INDEX] == name:
                return i
        return -1

    # Get index of role name in roles array
    def role_enum_to_index(self, enum):
        for i in range(len(self.roles)):
            if self.roles[i][ENUM_INDEX] == enum:
                return i
        return -1

    def role_index_to_enum(self, index):
        if 0 <= index < len(self.roles):
            return self.roles[index][ENUM_INDEX]
        return -1

    def role_index_to_name(self, index):
        if 0 <= index < len(self.roles):
            return self.roles[index][NAME_INDEX]
        return None

    # Getter for the desired position (Vector2D(mm, mm))
    def get_position(self):
        return self.roles[self.get_my_role_index()][OBJECT_INDEX].position

    # Getter for the desired heading (rad)
    def get_heading(self):
        return self.roles[self.get_my_role_index()][OBJECT_INDEX].heading

    # Getter for the position error (mm)
    def get_position_error(self):
        return self.roles[self.get_my_role_index()][OBJECT_INDEX].position_error

    # Getter for the heading error (rad)
    def get_heading_error(self):
        return self.roles[self.get_my_role_index()][OBJECT_INDEX].heading_error

    def evaluate_current_role(self):
        self.roles[self.get_my_role_index()][OBJECT_INDEX].evaluate()


class Role:
    """
    Abstract class for a role. Used by Positioning and should
    be in charge of calculating the optimal position, and heading
    of the robot, assuming it is in this role. It should consider
    the current position of the robot, position of the ball,
    and what other robots are doing
    """

    # Desired position of role (Vector2D(mm, mm))
    position = None

    # Desired heading of role (rad)
    heading = None

    # Desired position_error of role (mm)
    position_error = None

    # Desired heading_error of role (rad)
    heading_error = None

    # A function called from the Positioning class, which should
    # update member variables:
    # - position
    # - heading
    # - position_error
    # - heading_error
    def evaluate(self):
        pass

    # Have reference to the positioning, so role can access
    # functions from the positioning class
    positioning = None

    def __init__(self, positioning):
        self.positioning = positioning
