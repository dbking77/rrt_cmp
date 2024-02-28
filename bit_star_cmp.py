#!/usr/bin/env python3

# MIT License

# Copyright (c) 2024 Derek King

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import matplotlib.pyplot as plt
from heapq import heappush, heappop
from math import sqrt, pi, sin, cos
import random
import time
from typing import Dict, List, Optional, Set, Tuple, Union
from environment import (Point, Line, Circle, point_dist,
                         is_point_in_collision, is_edge_in_collision,
                         generate_obstacles, find_free_point)

infinity = float("+inf")


class Vertex:
    def __init__(self, point: Point, parent: Optional['Vertex'] = None, dist=infinity):
        self.point = point
        self.children: List['Vertex'] = []
        self.dist = dist
        self.parent: Optional['Vertex'] = None
        if parent is not None:
            self.set_parent(parent)

    def set_parent(self, parent: 'Vertex'):
        assert self.parent is None
        self.dist = self.calc_dist(parent) + parent.dist
        parent.children.append(self)
        self.parent = parent

    def change_parent(self, new_parent: 'Vertex'):
        assert self.parent is not None
        self.parent.children.remove(self)
        prev_dist = self.dist
        new_dist = self.calc_dist(new_parent) + new_parent.dist
        self.reduce_dist(prev_dist - new_dist)
        new_parent.children.append(self)
        self.parent = new_parent

    def unconnect_from_tree(self):
        if self.parent is not None:
            self.parent.children.remove(self)
        self.unconnect_subtree()

    def unconnect_subtree(self):
        self.dist = infinity
        self.parent = None
        for child in self.children:
            child.unconnect_subtree()
        self.children = []

    def reduce_dist(self, d_reduction: float):
        self.dist -= d_reduction
        for child in self.children:
            child.reduce_dist(d_reduction)

    def get_all_descendants(self) -> List['Vertex']:
        descendants = self.children[:]
        for child in self.children:
            descendants += child.get_all_descendants()
        return descendants

    def validate(self, *, expect_unconnected: bool = False, recurse: bool = False):
        if self.parent is not None:
            assert self in self.parent.children
            expected_dist = self.parent.dist + self.calc_dist(self.parent)
            assert abs(self.dist - expected_dist) < 1e-3
        else:
            # unconnected vertexes are either the start, or just unconnected
            assert self.dist in (0.0, infinity)
        for child in self.children:
            assert child.parent is self
            expected_dist = self.dist + self.calc_dist(child)
            assert abs(child.dist - expected_dist) < 1e-3
        if expect_unconnected:
            assert self.dist == infinity
            assert self.parent is None
            assert len(self.children) == 0
        if recurse:
            for child in self.children:
                child.validate(recurse=True)

    @property
    def x(self):
        return self.point.x

    @property
    def y(self):
        return self.point.y

    def calc_dist(self, other: Union[Point, 'Vertex']) -> float:
        return point_dist(self.point, other)

    def __lt__(self, other: 'Vertex') -> bool:
        assert isinstance(other, Vertex)
        # the heapq sometimes falls back to comparing two vertexes,
        # so this overload is mostly needed to prevent a exception.
        return self.dist < other.dist


# Vertexes have properties that make them look like points
PointLike = Union[Point, Vertex]


class Edge:
    def __init__(self, src: Vertex, dst: Union[Vertex, Point]):
        self.src: Vertex = src
        self.dst: Union[Vertex, Point] = dst

    def to_line(self) -> Line:
        return Line(Point(self.src.x, self.src.y), Point(self.dst.x, self.dst.y))


class BitStarCompare():
    def __init__(self,
                 obstacles: List[Circle],
                 start: Point,
                 goal: Point,
                 *,
                 figsize: float = 3.0,
                 k_nearest: int = 4,
                 samples_per_batch: int = 100,
                 ):
        # environment
        self.obstacles = obstacles
        self.start = Vertex(start, dist=0.0)
        self.goal = Vertex(goal)

        # params
        self.samples_per_batch = samples_per_batch
        self.k_nearest = k_nearest
        self.figsize = figsize

        # stats
        self.iteration = 0
        self.batch_iteration = 0
        self.point_collision_checks = 0
        self.edge_collision_checks = 0

        # will be set once goal is found
        self.goal_iteration: Optional[int] = None

        # data
        self.vertexes: Dict[Point, Vertex] = {self.start.point: self.start}
        self.samples: Dict[Point, Vertex] = {self.goal.point: self.goal}
        self.old_vertexes: Set[Point] = set()

        # queues for ordered expansion of edges
        # queues are ordered based on first value
        self.edge_queue: List[Tuple[float, Edge]] = []
        self.vertex_queue: List[Tuple[float, Vertex]] = []
        self.max_edge_queue_value: float = 0.0

        # basically for debugging / visualization
        self.new_sample_collisions: List[Point] = []
        self.new_pruned_samples: List[Point] = []
        self.new_pruned_vertexes: List[Point] = []
        self.new_pruned_edges: List[Line] = []
        self.new_edge_collisions: List[Line] = []

        # Edges the were removed because their destination vertex was
        # connected with shorter path to src
        self.new_rewire_removed_edges: List[Line] = []

        # Edges that were taken of edge queue but were
        # not added to tree because there was already something
        # better connected
        self.new_failed_edges: List[Line] = []

        # Brand new edge (since last draw point)
        self.new_vertexes: Set[Point] = set()

        # Edges that had cost reduced because upstream node was re-wired
        self.new_reduced_edges: List[Line] = []

        # Edges that were removed from edge queue
        self.new_queue_removed_edges: List[Line] = []

        # Vertexes that have had their costs reduced
        self.new_reduced_vertexes: List[Vertex] = []

    @property
    def goal_dist(self) -> float:
        return self.goal.dist

    def is_point_in_collision(self, point: Point) -> bool:
        """Return true if point is in collision with any obstacle"""
        self.point_collision_checks += 1
        return is_point_in_collision(point, self.obstacles)

    def is_edge_in_collision(self, edge: Edge):
        self.edge_collision_checks += 1
        return is_edge_in_collision(edge.src, edge.dst, self.obstacles)

    def validate(self):
        # validate tree recursively
        self.start.validate(recurse=True)
        # validate every vertex (independently), just in case
        # some nodes are disconnected from the tree improperly
        for point, vertex in self.vertexes.items():
            assert point is vertex.point
            vertex.validate(recurse=False)
        # validate vertex in queue
        for _, vertex in self.vertex_queue:
            vertex.validate(recurse=False)
        # validate vertexes in edge queue
        for _, edge in self.edge_queue:
            edge.src.validate(recurse=False)
            edge.dst.validate(recurse=False)
        # validate samples (they are vertexes, but should be unconnected)
        for point, sample in self.samples.items():
            assert point is sample.point
            sample.validate(recurse=True, expect_unconnected=True)

    def admissible_point_path_dist(self, point: PointLike) -> float:
        """
        Returns the minimum posible distance from a path
        that goes from start to goal through this point.
        Assumes no obstacles
        """
        return point.calc_dist(self.start) + point.calc_dist(self.goal)

    def admissible_edge_path_dist(self, src: PointLike, dst: PointLike) -> float:
        """
        Returns the minimum posible distance from a path
        that goes from start to goal through this point.
        Assumes no obstacles
        """
        return src.calc_dist(self.start) + src.calc_dist(dst) + dst.calc_dist(self.goal)

    def sample_point(self):
        while True:
            new_point = Point(random.random(), random.random())
            if self.admissible_point_path_dist(new_point) <= self.goal_dist:
                # TODO is this checked now, or later?
                if self.is_point_in_collision(new_point):
                    self.new_sample_collisions.append(new_point)
                else:
                    break
        return new_point

    def prune(self):
        saved_samples = {}
        for sample in self.samples.values():
            if self.admissible_point_path_dist(sample) <= self.goal_dist:
                saved_samples[sample.point] = sample
            else:
                self.new_pruned_samples.append(sample.point)
        self.samples = saved_samples

        saved_vertexes = {}
        for vertex in self.vertexes.values():
            if self.admissible_point_path_dist(vertex) > self.goal_dist:
                self.new_pruned_vertexes.append(vertex.point)
                if vertex.parent is not None:
                    self.new_pruned_edges.append(Line(vertex.parent.point, vertex.point))
                vertex.unconnect_from_tree()
            elif vertex.dist >= infinity:
                # move vertex to samples, erase parent
                self.samples[vertex.point] = vertex
            else:
                saved_vertexes[vertex.point] = vertex
        self.vertexes = saved_vertexes
        # TODO Prune Edges

    def add_to_vertex_queue(self, vertex: Vertex):
        # value in queue is based on tree distance to vertex, plus admissable distance to goal
        value = vertex.dist + vertex.calc_dist(self.goal)
        heappush(self.vertex_queue, (value, vertex))

    def add_to_edge_queue(self, edge: Edge):
        assert isinstance(edge.src, Vertex)
        # value is actual distance to src vertex, admissiable edge length, + addmissable dist to goal
        value = edge.src.dist + edge.src.calc_dist(edge.dst) + edge.dst.calc_dist(self.goal)
        self.max_edge_queue_value = max(self.max_edge_queue_value, value)
        heappush(self.edge_queue, (value, edge))

    def expand_vertex(self, vertex: Vertex):
        # add k-nearest samples to edge queue (but only if they could reduce distance to goal)
        assert isinstance(vertex, Vertex)
        nearby_samples = []

        def add_nearby(edge_dist: float, sample: Vertex):
            nearby_samples.append((edge_dist, sample))
            nearby_samples.sort()
            while len(nearby_samples) > self.k_nearest:
                nearby_samples.pop()

        def is_nearby(edge_dist: float):
            return (len(nearby_samples) < self.k_nearest) or (edge_dist < nearby_samples[-1][0])

        for sample in self.samples.values():
            edge_dist = vertex.calc_dist(sample)
            if is_nearby(edge_dist):
                if self.admissible_edge_path_dist(vertex, sample) < self.goal_dist:
                    add_nearby(edge_dist, sample)

        if vertex.point in self.old_vertexes:
            print("  Not finding connections for vertex from older batch")
        else:
            # TODO slow use KD tree to find nearby vertexes
            for dst_vertex in self.vertexes.values():
                # don't try connecting a node to itself, or something it is already connected to
                if (dst_vertex is not vertex) and (dst_vertex.parent is not vertex):
                    edge_dist = vertex.calc_dist(dst_vertex)
                    if is_nearby(edge_dist):
                        # could edge do better than before
                        if ((self.admissible_edge_path_dist(vertex, dst_vertex) < self.goal_dist) and
                                (vertex.dist + edge_dist < dst_vertex.dist)):
                            # could path to existing connected vertex do better than before
                            add_nearby(edge_dist, dst_vertex)

        assert len(nearby_samples) <= self.k_nearest
        for _, sample in nearby_samples:
            self.add_to_edge_queue(Edge(vertex, sample))

    def prune_edge_queue(self, dst: Vertex):
        saved_edge_queue = []
        # remove anything from edge queue that couldn't reach vertex faster
        for cost, edge in self.edge_queue:
            if (edge.dst is dst) and (edge.src.dist + edge.src.calc_dist(dst) >= dst.dist):
                self.new_queue_removed_edges.append(edge.to_line())
            else:
                saved_edge_queue.append((cost, edge))
        print(f"  Pruned {len(self.edge_queue) - len(saved_edge_queue)} edges from queue")
        self.edge_queue = saved_edge_queue

    def plan_one(self):
        self.iteration += 1
        print("### Iteration", self.iteration, "###")
        if len(self.vertex_queue) == 0 and len(self.edge_queue) == 0:
            print("  Queues are empty : pruning and adding samples")
            self.prune()
            self.sample_batch()
            self.max_edge_queue_value = 0.0
            # TODO track old vertexes  : V_old <- V
            for vertex in self.vertexes.values():
                self.add_to_vertex_queue(vertex)
            self.old_vertexes = set(self.vertexes.keys())

        while len(self.vertex_queue):
            # move vertexes from queue until the best case distance of
            # vertex is worse that best case distance of edge queue
            if len(self.edge_queue) and (self.vertex_queue[0][0] > self.edge_queue[0][0]):
                break
            self.expand_vertex(heappop(self.vertex_queue)[1])

        if True:
            print("  After expanding vertexes to edges:")
            print(f"    Vertex queue size : {len(self.vertex_queue)}")
            print(f"    Edge queue size : {len(self.edge_queue)}")

        if len(self.edge_queue):
            edge = heappop(self.edge_queue)[1]
            # if tree dist to first vertex of edge + remaininging ideal path to goal is less than
            # current best distance to the goal, then ...
            dist1 = edge.src.dist + edge.src.calc_dist(edge.dst) + edge.dst.calc_dist(self.goal)
            if dist1 < self.goal_dist:
                # theoretically this would compute cost of the edge.. and check if it could
                # improve potetentially improve on goal distance...
                # however in this setup the edge is either in collision (cost == infinity) or
                # the cost is just the edge distance
                if self.is_edge_in_collision(edge):
                    # TODO add edge to list of colliding edges
                    self.new_edge_collisions.append(edge.to_line())
                    print("  Edge is in collision")
                elif edge.src.dist + edge.src.calc_dist(edge.dst) < edge.dst.dist:
                    assert isinstance(edge.dst, Vertex)
                    if edge.dst.point in self.samples:
                        print("    Expanding edge connects to new sample")
                        del self.samples[edge.dst.point]
                        # convert edge dst from a point to a vertex
                        edge.dst.set_parent(edge.src)
                        self.vertexes[edge.dst.point] = edge.dst
                        self.add_to_vertex_queue(edge.dst)
                    else:
                        d_prev = edge.dst.dist
                        # TODO remove edge from list of edges
                        # Figure out where vertex was previously connected to by looking at parent
                        self.new_rewire_removed_edges.append(Line(edge.dst.parent.point, edge.dst.point))
                        edge.dst.change_parent(edge.src)
                        print(f"    Rewiring to reduce dist from {d_prev} to {edge.dst.dist}")
                        self.new_reduced_vertexes += edge.dst.get_all_descendants()

                    self.prune_edge_queue(edge.dst)

                    # visualization
                    print(f"    Adding edge with distance {edge.src.calc_dist(edge.dst)} to tree")
                    self.new_vertexes.add(edge.dst.point)

                    if True:
                        print("  After evaluating edge:")
                        print(f"    Vertex queue size : {len(self.vertex_queue)}")
                        print(f"    Edge queue size : {len(self.edge_queue)}")
                else:
                    print("  New edge reaches vertex with larger path length")
                    self.new_failed_edges.append(edge.to_line())
            else:
                # best case distance of stuff on queue can't do better than goal distance -- reset
                print("Edge queue has nothing that improve on goal distance resetting queeus")
                self.vertex_queue = []
                self.edge_queue = []

        # TODO
        print(f"  Vertexes {len(self.vertexes)}")
        print(f"  Point Collision Checks : {self.point_collision_checks}")
        print(f"  Edge Collision Checks: {self.edge_collision_checks}")

        if self.goal_dist < infinity and self.goal_iteration is None:
            self.goal_iteration = self.iteration

        if self.goal_iteration is not None:
            print(f"  Goal Found Iteration: {self.goal_iteration}")
            print(f"  Goal path length: {self.goal_dist}")

        self.validate()

    def sample_batch(self):
        for _ in range(self.samples_per_batch):
            new_point = self.sample_point()
            self.samples[new_point] = Vertex(new_point)

    def draw_lines(self, lines: List[Line], color, *, linewidth=1.0):
        for line in lines:
            self.ax.plot([line.p1.x, line.p2.x], [line.p1.y,
                         line.p2.y], color, linewidth=linewidth)

    def draw_elipse(self, point1: Point, point2: Point, dist: float, color="r", linestyle='--'):
        len = point_dist(point1, point2)
        height = 2*sqrt((0.5*dist)**2 - (0.5*len)**2)
        xc, yc = (0.5*(point1.x + point2.x), 0.5*(point1.y + point2.y))
        c, s = ((point2.x - point1.x)/len, (point2.y - point1.y)/len)
        steps = 30
        xs, ys = ([], [])
        for i in range(steps+1):
            angle = 2*pi*i/steps
            x, y = (0.5*dist*cos(angle), 0.5*height*sin(angle))
            xs.append((c * x - s * y) + xc)
            ys.append((s * x + c * y) + yc)
        self.ax.plot(xs, ys, color=color, linestyle=linestyle)

    def draw_points(self, points: List[Point], color):
        if points:
            self.ax.plot(*zip(*points), color)

    def draw_tree(self, vertex: Vertex):
        for child in vertex.children:
            if child.point in self.new_vertexes:
                color = 'g'
                linewidth = 3.0
            else:
                color = 'k'
                linewidth = 1.0
            self.ax.plot([vertex.x, child.x], [
                         vertex.y, child.y],
                         color, linewidth=linewidth)
            self.draw_tree(child)

    def draw_goal_path(self):
        if self.goal.parent is not None:
            xs, ys = ([], [])
            vertex = self.goal
            while True:
                xs.append(vertex.point.x)
                ys.append(vertex.point.y)
                if vertex.parent is None:
                    break
                vertex = vertex.parent
            self.ax.plot(xs, ys, "c--", linewidth=4.0)

    def draw(self):
        self.fig = plt.figure("BIT-Star", figsize=(self.figsize, self.figsize))
        self.fig.clf()
        self.ax = self.fig.subplots()
        self.ax.cla()
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        for x, y, r in self.obstacles:
            self.ax.add_patch(plt.Circle((x, y), r, color='r', alpha=0.3))
        if self.goal_dist < float("+inf"):
            self.draw_elipse(self.start, self.goal, self.goal_dist, color='r')
        # when queue is non-empty, then draw elipse base on dist
        self.draw_elipse(self.start, self.goal, self.max_edge_queue_value, color='tab:orange')

        self.draw_goal_path()

        self.draw_points(self.new_sample_collisions, 'r.')
        self.draw_points(self.new_pruned_samples, 'm+')
        self.draw_points(self.new_pruned_vertexes, 'm.')
        self.draw_lines(self.new_pruned_edges, 'm-')
        self.draw_points(self.samples, 'b+')

        # draw edge queue
        for _, edge in self.edge_queue:
            self.ax.plot([edge.src.x, edge.dst.x],
                         [edge.src.y, edge.dst.y], '--', c=(0, 1, 0, 0.5))

        # draw vertex queue
        self.ax.plot([v.x for _, v in self.vertex_queue],
                     [v.y for _, v in self.vertex_queue],
                     'gx')

        for vertex in self.new_reduced_vertexes:
            self.ax.plot([vertex.x, vertex.parent.x], [vertex.y, vertex.parent.y], 'tab:orange', linewidth=3.0)

        self.draw_tree(self.start)

        # draw vertexes
        self.ax.plot([v.x for v in self.vertexes.keys()],
                     [v.y for v in self.vertexes.keys()],
                     'k.')

        self.ax.plot([v.x for _, v in self.vertex_queue],
                     [v.y for _, v in self.vertex_queue],
                     'g.', markersize=10)

        self.draw_lines(self.new_rewire_removed_edges, 'm-', linewidth=3.0)
        self.draw_lines(self.new_queue_removed_edges, 'm--')
        self.draw_lines(self.new_failed_edges, 'm--', linewidth=3.0)
        self.draw_lines(self.new_edge_collisions, 'r-', linewidth=3.0)

        self.ax.set_xlim((0, 1))
        self.ax.set_ylim((0, 1))
        self.ax.plot([self.goal.x], [self.goal.y], 'g*', label='goal')
        self.ax.plot([self.start.x], [self.start.y], 'r*', label='start')
        self.fig.tight_layout()

        self.new_vertexes = set()
        self.new_rewire_removed_edges = []
        self.new_queue_removed_edges = []
        self.new_reduced_vertexes = []
        self.new_failed_edges = []
        self.new_sample_collisions = []
        self.new_pruned_samples = []
        self.new_pruned_vertexes = []
        self.new_pruned_edges = []
        self.new_edge_collisions = []

    def plan_some(self, iterations=1, pause_iteration=None,
                  pause_at_goal=False):
        for _ in range(iterations):
            self.plan_one()
            if pause_at_goal and (self.iteration == self.goal_iteration):
                return
            if ((pause_iteration is not None) and (self.iteration >= pause_iteration)):
                return


def main():
    parser = argparse.ArgumentParser("rtt_cmp",
                                     description="Simulates different RRT algorithms running")
    parser.add_argument("--fig-size", type=float, default=6.0,
                        help="Figure window size")
    parser.add_argument("--save",
                        help="""Filename root for saving images as PNGs
                        Each image will be saved as name<iteration>.png""")
    parser.add_argument("--seed", "-s", type=int, default=0,
                        help="Random seed used for simulation")
    parser.add_argument("--single-step", "--ss", action="store_true",
                        help="""Stop running after each set of calculations.
                        and wait for figure to be closed""")
    parser.add_argument("--single-step-after", "--ssa", type=int, default=1000*1000*1000,
                        help="""Start single stepping after a certain iteration.""")
    parser.add_argument("--rate", type=float, default=10,
                        help="""Limits display update to this rate. Setting a
                        value of "inf" will allow display updates as fast as CPU allows""")
    parser.add_argument("--skip", type=int, default=1,
                        help="""Skip these many iterations before displaying changes""")
    parser.add_argument("--pause", action="append", nargs="+", type=int,
                        help="""Pause after these iterations and wait for window
                        to be closed""")
    parser.add_argument("--pause-at-goal", action="store_true",
                        help="Pause right after goal is reached")
    parser.add_argument("--obstacle-count", "-C", type=int,
                        default=7, help="""Count of randomly placed obstacles""")
    parser.add_argument("--k-nearest", "-K", type=int, default=4,
                        help="connect upto K-nearest neighbors when expanding a vertex")
    parser.add_argument("--samples-per-batch", "-spb", type=int, default=100,
                        help="New samples to add with every new batch")
    if False:
        parser.add_argument("--goal-resample-tolerance", "-grt", type=float, default=None,
                            help="""Trigger resampling when path distance to goal decreases by more than this ratio (0.0 to 1.0).
                            When a drastically improve route to goal is found, it can make sense to perform an expensive
                            pruning and resampling, because even though existing samples still have potential to improve
                            distance to goal, many of them probably will not""")
    args = parser.parse_args()

    random.seed(args.seed)

    obstacles = generate_obstacles(args.obstacle_count)
    start = find_free_point(obstacles)
    goal = find_free_point(obstacles)

    bitstar = BitStarCompare(
        obstacles,
        start,
        goal,
        figsize=args.fig_size,
        k_nearest=args.k_nearest,
        samples_per_batch=args.samples_per_batch
    )

    if args.pause is not None:
        pauses = sorted(sum(args.pause, []), reverse=True)
    else:
        defaut_final_pause = 1000
        print("Adding default final pause point at iteration", defaut_final_pause)
        pauses = [defaut_final_pause]
    period = 1.0 / args.rate
    while len(pauses) or (args.pause_at_goal and bitstar.goal_node is None):
        start_t = time.time()
        pause_iteration = pauses[-1] if pauses else None
        bitstar.plan_some(args.skip, pause_iteration, args.pause_at_goal)
        bitstar.draw()
        if (bitstar.iteration == pause_iteration) or args.single_step or (bitstar.iteration > args.single_step_after):
            print(f"  Paused at {bitstar.iteration} close figure to continue...")
            plt.ioff()
        elif args.pause_at_goal and (bitstar.goal_iteration == bitstar.iteration):
            print("  Paused after first reaching goal, close figure to continue...")
            plt.ioff()
        else:
            plt.ion()
        while pauses and (bitstar.iteration >= pauses[-1]):
            pauses.pop()
        plt.show()
        if args.save:
            fn = f"{args.save}{bitstar.iteration:04d}.png"
            print(f"  Saving image as {fn}")
            bitstar.fig.savefig(fn)
        # slow down if computation is faster than given rate
        stop_t = time.time()
        dt = stop_t - start_t
        if dt < period:
            plt.pause(period - dt)

    print("Done: exiting")


if __name__ == "__main__":
    main()
