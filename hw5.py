import sys, math, argparse
import numpy as np
import matplotlib.pyplot as plt
from planning import (
    rrt,
    prm,
    EdgeCreator,
    DistanceComputator,
    ObstacleCollisionChecker, StraightEdgeCreator,
)
from dubins import shortest_path
from edge import Edge
from obstacle import construct_circular_obstacles, WorldBoundary2D
from draw_cspace import draw
import time

ALG_RRT = "rrt"
ALG_PRM = "prm"


class DubinsEdgeCreator(EdgeCreator):
    def __init__(self, rho_min, step_size):
        self.rho_min = rho_min
        self.step_size = step_size

    def make_edge(self, s1, s2):
        return EdgeDubins(s1, s2, self.rho_min, self.step_size)


class DubinsDistanceComputator(DistanceComputator):
    def __init__(self, rho_min):
        self.rho_min = rho_min

    def get_distance(self, s1, s2):
        """Return the Euclidean distance between s1 and s2"""
        path = shortest_path(s1, s2, self.rho_min)
        return path.path_length()


class EdgeDubins(Edge):
    """Store the information about an edge representing a Dubins curve between 2 points"""

    def __init__(self, s1, s2, rho_min, step_size=0.5, length=None, states=None):
        super().__init__(s1, s2, step_size)

        # The shortest dubins curve and the discretized points along the path
        self.rho_min = rho_min
        if length is None or states is None:
            self._update_path()
        else:
            self.length = length
            self.states = states

    def _update_path(self):
        path = shortest_path(self.s1, self.s2, self.rho_min)
        self.length = path.path_length()

        # Change the step_size to make equal spacing between consecutive states
        # First, compute the number of states (excluding the first one) so that the new step_size
        # is not larger than teh given step_size
        num_states = math.ceil(self.length / self.step_size)
        self.step_size = self.length / num_states

        (self.states, _) = path.sample_many(self.step_size)
        self.states = [np.array(state) for state in self.states]

        # Make sure the last state s2 is also in self.states (possibly missing due to
        # numerical error
        if len(self.states) < num_states + 1:
            self.states.append(self.s2)
        assert len(self.states) == num_states + 1

    def get_path(self):
        """Return the path, representing the geometry of the edge"""
        return self.states

    def reverse(self):
        """Reverse the origin/destination of the edge"""
        super().reverse()
        self._update_path()

    def get_discretized_state(self, i):
        """Return the i^{th} discretized state"""
        if i >= len(self.states):
            return None
        return self.states[i]

    def get_nearest_point(self, state):
        """Compute the nearest point on this edge to the given state

        @return (s, t) where s is the point on the edge that is closest to state
        and it is at distance t*length from the beginning of the edge
        """
        nearest_dist = math.inf
        nearest_ind = None
        for ind, s in enumerate(self.states):
            path = shortest_path(s, state, self.rho_min)
            dist = path.path_length()
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_ind = ind

        if nearest_ind == 0:
            return (self.s1, 0)

        if nearest_ind == len(self.states) - 1:
            return (self.s2, 1)

        t = nearest_ind * self.step_size / self.length
        assert t > 0 and t < 1
        return (self.states[nearest_ind], t)

    def split(self, t):
        """Split the edge at distance t/length where length is the length of this edge

        @return (edge1, edge2) edge1 and edge2 are the result of splitting the original edge
        """
        split_length = t * self.length
        split_ind = round(split_length / self.step_size)
        assert split_ind > 0 and split_ind < len(self.states) - 1

        s = self.states[split_ind]
        edge1 = EdgeDubins(
            self.s1,
            s,
            self.rho_min,
            self.step_size,
            length=split_length,
            states=self.states[0 : split_ind + 1],
        )
        edge2 = EdgeDubins(
            s,
            self.s2,
            self.rho_min,
            self.step_size,
            length=self.length - split_length,
            states=self.states[split_ind:],
        )

        return (edge1, edge2)

    def get_length(self):
        """Return the length of the edge"""
        return self.length


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Run sampling-based motion planning algorithm"
    )
    parser.add_argument(
        "--alg",
        choices=[ALG_RRT, ALG_PRM],
        required=False,
        default=ALG_RRT,
        dest="alg",
        help="algorithm, default to rrt",
    )
    args = parser.parse_args(sys.argv[1:])
    return args


def findPathLength(G, path):
    # # a dictionary whose key = id of the vertex and value = state of the vertex
    # self.vertices = {}

    # a dictionary whose key = (v1, v2) and value = (cost, edge).
    # v1 is the id of the origin vertex and v2 is the id of the destination vertex.
    # cost is the cost of the edge.
    # edge is of type Edge and stores information about the edge, e.g.,
    # the origin and destination states and the discretized points along the edge
    # self.edges = {}

    totalLength = 0

    for i in range(len(path) - 1):

        # find the first vertex
        firstVertexId = path[i]
        firstVertex = G.vertices.get(firstVertexId)
        # print("This is the first vertex")
        # print(firstVertex)

        # find the second vertex
        secondVertexId = path[i + 1]
        secondVertex = G.vertices.get(secondVertexId)
        # print("This is the second vertex")
        # print(secondVertex)

        # Put the vertices in key format
        key = (firstVertexId, secondVertexId)

        # find the edge associated with those
        edge = G.edges.get(key)
        # print("This is the edge")
        # print(edge)
        # print()

        # find the length of that edge
        length = edge[0]

        # update our totalLength
        totalLength = totalLength + length


    return totalLength


if __name__ == "__main__":
    # TODO: Change min turning radius
    # rho_min = 0.5
    rho_min = 10
    # cspace = [(-3, 3), (-1, 1), (-math.pi / 2, math.pi / 2)]
    # qI = (-2, -0.5, 0)
    # qG = (2, -0.5, math.pi / 2)

    # cspace = [(-400, 400), (-300, 300), (-math.pi / 2, math.pi / 2)]
    # qI = (-300, -100, 0)
    # qG = (300, -100, math.pi / 2)

    cspace = [(-1000, 1000), (-1000, 1000), (-math.pi / 2, math.pi / 2)]
    qI = (-500, 0, 0)
    qG = (500, 0, math.pi / 2)

    # TODO: Edit how our obstacles are created.
    # Might have to pass them in as arguments
    # We will only have circular obstacles for now
    # obstacles = construct_circular_obstacles(0.2)
    obstacles = construct_circular_obstacles(95)

    obs_boundaries = [obstacle.get_boundaries() for obstacle in obstacles]
    world_boundary = WorldBoundary2D(cspace[0], cspace[1])
    obstacles.append(world_boundary)
    # edge_creator = DubinsEdgeCreator(rho_min, 0.1)
    edge_creator = StraightEdgeCreator(0.1)
    collision_checker = ObstacleCollisionChecker(obstacles)
    distance_computator = DubinsDistanceComputator(rho_min)

    # Our robot W=178 and L 138

    args = parse_args()
    numberOfIterations = 10
    sumOfVerticiesAlongAllPaths = 0
    numberOfFoundPaths = 0
    sumOfAllLengths = 0
    totalCount = 0
    startTime = time.time()
    endTime = 0
    totalTime = 0
    totalStopConfigTime = 0
    # PRM Timings
    # 25, 50, 100, 200, 500, 1000, 2000
    for i in range(numberOfIterations):
        startTime = time.time()
        if args.alg == ALG_RRT:
            title = "RRT planning"
            (G, root, goal, count, stoppingConfigTime) = rrt(
                cspace=cspace,
                qI=qI,
                qG=qG,
                edge_creator=edge_creator,
                distance_computator=distance_computator,
                collision_checker=collision_checker,
            )
        else:
            title = "PRM planning"
            (G, root, goal) = prm(
                cspace=cspace,
                qI=qI,
                qG=qG,
                edge_creator=edge_creator,
                distance_computator=distance_computator,
                collision_checker=collision_checker,
                k=15,
            )
            #TODO: If you edit this you need to edit PRM
            count = 25

        print("This is the number of iterations for the algorithm")
        print(count)
        totalCount = totalCount + count
        print("This is the sum of iterations so far for the algorithm")
        print(totalCount)

        totalStopConfigTime = totalStopConfigTime + stoppingConfigTime

        # End time if we have not found a path
        endTime = time.time()

        path = []
        if root is not None and goal is not None:

            numberOfFoundPaths = numberOfFoundPaths + 1
            path = G.getVertexIdsAlongPath(root, goal)

            # End time if we have found a path
            endTime = time.time()

            print("These are the path Ids")
            print(path)

            pathLength = findPathLength(G, path)
            print("This is the path length")
            print(pathLength)
            sumOfAllLengths = sumOfAllLengths + pathLength
            print("This is the sum of lengths so far")
            print(sumOfAllLengths)

            countOfVerticiesAlongPath = 0
            countOfVerticiesAlongPath = len(path)
            print("Number Of Vertices in Current Path")
            print(countOfVerticiesAlongPath)
            sumOfVerticiesAlongAllPaths = sumOfVerticiesAlongAllPaths + countOfVerticiesAlongPath
            print("Sum of Vertices in paths so far")
            print(sumOfVerticiesAlongAllPaths)
            print()

            # This gets a very discretized path
            # This is what I want to send to ROS
            # path = G.get_path(root, goal)

        # print("We are drawing the graph")
        # fig, ax = plt.subplots(1, 1)
        # draw(ax, cspace, obs_boundaries, qI, qG, G, path, title)
        # plt.show()

        totalTime = totalTime + (endTime - startTime)

    # Print Results for all iterations
    print("---------------------------------------------------------------------------------")
    print("You ran the algorithm this many times")
    print(numberOfIterations)
    print("This is the number of found paths")
    print(numberOfFoundPaths)
    print("This is the average number of vertices along the paths")
    print(sumOfVerticiesAlongAllPaths / numberOfIterations)
    print("This is the total number of iterations for the algorithm")
    print(totalCount)
    print("This is the average number of iterations for the algorithm")
    print(totalCount / numberOfIterations)
    print("This is the total lengths of all the paths")
    print(sumOfAllLengths)
    print("This is the average of all path lengths")
    print(sumOfAllLengths / numberOfIterations)
    print("This is the total time for all iterations of the algorithm")
    print(totalTime)
    print("This is the average time for each iteration of the algorithm")
    print(totalTime / numberOfIterations)
    print("This is the total stopping configurations time")
    print(totalStopConfigTime)
    print("This is the percentage of total time that was stopping configuration time")
    print(totalStopConfigTime / totalTime)



