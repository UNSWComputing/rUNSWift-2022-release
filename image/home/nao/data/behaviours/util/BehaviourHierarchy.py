import inspect


def get_behaviour_hierarchy():
    """Get behaviour hierarchy by listing all nested classes
    by inspecting the stack frames.
    """
    stack = inspect.stack()
    hierarchy = []
    prev_base_class_name = ""
    prev_class_name = ""

    i, parent_frame = 1, stack[1][0]
    while "self" in parent_frame.f_locals:
        class_name = parent_frame.f_locals["self"].__class__.__name__
        base_class_name = parent_frame.f_locals["self"].__class__.__base__.__name__
        # eg. class_name = "Striker" or "TeamSkill" or "WalkToPoint"
        # eg. base_class_name = "BehaviourTask" OR "TaskState"

        # make sure the same class isn't repeated. When adding a class,
        # compare it's class name and base class name with the previous class.
        # If either there name OR base class is different, add to hierarchy.
        # Otherwise, the same class is repeated. Ignore it.
        if prev_base_class_name != base_class_name or class_name != prev_class_name:
            hierarchy.append(parent_frame.f_locals["self"].__class__.__name__)
        i += 1
        parent_frame = stack[i][0]
        prev_base_class_name = base_class_name
        prev_class_name = class_name
    return ".".join(hierarchy[::-1])
