import math
import numpy as np
from shapely import geometry
from shapely.geometry import Point


def get_nearest_point_on_line(s1, s2, p, tol=1e-3):
    """Compute the nearest point on a line described by s1 and s2 to p

    Note that a point on the line can be parametrized by
        s(t) = s1 + t(s2 - s1).
    s(t) is on the line segment between s1 and s2 iff t \in [0, 1].

    The point on the line closest to p is s(t*) where
        t* = <p-s1, s2-s1> / ||s2 - s1||^2

    @return (s*, t*) where s* = s(t*)
    """
    ls = s2 - s1  # The line segment from s1 to s2
    len_ls2 = np.dot(ls, ls)  # the squared length of ls

    # If the line segment is too short, just return 0
    if len_ls2 < tol:
        return (s1, 0)

    tstar = np.dot(p - s1, ls) / len_ls2
    if tstar <= tol:
        return (s1, 0)
    if tstar >= 1 - tol:
        return (s2, 1)

    return (s1 + tstar * ls, tstar)


def get_euclidean_distance(s1, s2):
    """Compute the norm ||s2 - s1||"""
    ls = s2 - s1
    return math.sqrt(np.dot(ls, ls))


def is_inside_circle(c, r, p):
    """Return whether point p is inside a circle with radius r, centered at c"""
    # return (p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2 <= r ** 2

    # Editing for our turtlebot3
    # Width = 178, Length = 138
    # idea: We will draw a circle around our turtlebot and make sure this
    # circle doesn't intersect any of our obstacles
    # What if there is a big difference in size between our obstacles and our robot???
        # Do we need to make sure none of the obstacles contain our robot
        # And make sure our robot doesn't contain our obstacles as well?

    # print("This is c")
    # print(c)
    # print()
    # print("This is p")
    # print(p)
    # print()

    # we know our robot has Width = 178 and Length = 138.  We will just hard code these for now
    width = 178
    length = 138
    radius = findRadiusOfRobot(width, length)

    # Updating to avoid shapely
    return (p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2 <= (r + radius) ** 2

    # # p will be the center of our robot
    # robotPoint = (p[0], p[1])
    # # print("This is RobotPoint")
    # # print(robotPoint)
    # # print()
    #
    # # Make a circle surrounding the robot
    # robotCircle = Point(robotPoint).buffer(radius)
    # # print("This is the type of the robot circle")
    # # print(type (robotCircle))
    #
    # # Make a circle for the obstacle
    # obstacleCircle = Point(c).buffer(r)
    #
    # # Make sure the robot circle doesn't intersect with the obstacle
    # if robotCircle.intersects(obstacleCircle):
    #     return True
    #
    # # # Check to see if robot is inside obstacle
    # # if robotCircle.within(obstacleCircle):
    # #     print("robot circle is inside obstacle circle")
    # #     return True
    # #
    # # # Check to see if obstacle circle is inside robot circle
    # # if obstacleCircle.within(robotCircle):
    # #     print("Obstacle circle is inside robot circle")
    # #     print()
    # #     return True
    #
    # # print("Not In Collision")
    # return False


def findRadiusOfRobot(w, l):

    radius = math.sqrt(w**2 + l**2)
    # add one just to be on the safe side
    radius = radius + 1

    return radius

