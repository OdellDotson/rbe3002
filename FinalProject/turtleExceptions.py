__author__ = 'Troy Hughes'


class FrontierException(Exception):
    """
        This class is used for creating and managing exceptions in the frontiers.
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class TurtlebotException(Exception):
    """
        This class is used for creating and managing exceptions in the turtlebot
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class PainterException(Exception):
    """
        This class is used for creating and managing exceptions in the rVizPainter
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class PathPlannerException(Exception):
    """
        This class is used for creating and managing exceptions in the rVizPainter
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)
