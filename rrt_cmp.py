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
import collections
import matplotlib.pyplot as plt
from math import sqrt, pi, sin, cos
import random
import time
from typing import List, NamedTuple, Optional

Line = collections.namedtuple('Line', ['p1', 'p2'])
Circle = collections.namedtuple('Circle', ['x', 'y', 'radius'])


class Point(NamedTuple):
    x: float
    y: float

    def dist(self, other: 'Point') -> float:
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def __sub__(self, other: 'Point') -> 'Point':
        return Point(self.x - other.x, self.y - other.y)

    def __add__(self, other: 'Point') -> 'Point':
        return Point(self.x + other.x, self.y + other.y)

    def __mul__(self, scale: float) -> 'Point':
        return Point(self.x * scale, self.y * scale)


def point_dist(p1: Point, p2: Point):
    return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)


def is_point_in_collision(point: Point, obstacles: List[Circle]) -> bool:
    for x, y, r in obstacles:
        dsq = (x-point.x)**2 + (y-point.y)**2
        if dsq < r**2:
            return True
    return False


class Node:
    def __init__(self, point: Point, parent: Optional['Node'] = None):
        self.point = point
        self.dist: float = 0.0
        self.parent: Optional[Node] = parent
        self.children: List[Node] = []
        if parent is not None:
            assert isinstance(parent, Node)
            self.dist = parent.dist + point_dist(point, parent.point)
            parent.children.append(self)


class RRTCompare():
    def __init__(self,
                 obstacles: List[Circle],
                 start: Point,
                 goal: Point,
                 *,
                 informed: bool = False,
                 march: bool = False,
                 exploration_bias: float = 0.05,
                 step_size: float = 0.1,
                 neighborhood: float = 0.15,
                 figsize: float = 3.0
                 ):
        # environment
        self.obstacles = obstacles
        self.start = start
        self.goal = goal

        # params
        self.step_size = step_size
        self.neighborhood = neighborhood
        self.exploration_bias = exploration_bias
        self.informed = informed
        self.march = march
        self.figsize = figsize

        # stats
        self.iteration = 0
        self.point_collision_checks = 0
        self.edge_collision_checks = 0

        # will be set once goal is found
        self.goal_node: Optional[Node] = None
        self.goal_iteration: Optional[int] = None

        # in march mode will track what point to continue going towards
        self.march_point: Optional[Point] = None
        self.march_end_point: Optional[Point] = None

        # first node is start
        self.nodes: List[Node] = [Node(self.start)]

        # basically for debugging / visualization
        self.new_nodes: List[Node] = []
        self.sample_collisions: List[Point] = []
        self.new_line_collisions: List[Line] = []
        self.line_collisions: List[Line] = []
        self.new_reduced_lines: List[Line] = []
        self.modified_lines: List[Line] = []
        self.retarget_lines: List[Line] = []
        self.replaced_lines: List[Line] = []
        self.limited_lines: List[Line] = []

    def stop_march(self):
        self.march_point = None
        self.march_end_point = None

    def is_point_in_collision(self, point: Point) -> bool:
        """Return true if point is in collision with any obstacle"""
        self.point_collision_checks += 1
        return is_point_in_collision(point, self.obstacles)

    def is_edge_in_collision(self, p1: Point, p2: Point):
        self.edge_collision_checks += 1
        dx, dy = (p2.x - p1.x, p2.y - p1.y)
        d = sqrt(dx*dx + dy*dy)
        if d == 0.0:
            # pretend a zero length edge is in collision so it is not added
            return True
        c, s = (dx/d, dy/d)
        for x, y, r in self.obstacles:
            dx, dy = (x-p1.x, y-p1.y)
            xr, yr = (c * dx + s * dy, s * dx - c * dy)
            # don't check if points are in collision (they shouldn't be)
            if xr < 0:
                # circle is close to first pi
                if xr*xr + yr*yr < r*r:
                    return True
            elif xr > d:
                xr -= d
                if xr*xr + yr*yr < r*r:
                    return True
            else:
                if yr*yr < r*r:
                    return True
        return False

    def sample_point(self):
        if self.march and self.march_point:
            d = point_dist(self.march_end_point, self.march_point)
            if d > self.step_size:
                dx, dy = ((self.march_end_point.x - self.march_point.x),
                          self.march_end_point.y - self.march_point.y)
                scale = self.step_size / d
                self.march_point = Point(self.march_point.x + dx * scale,
                                         self.march_point.y + dy * scale)
                new_point = self.march_point
            else:
                new_point = self.march_end_point
                self.stop_march()
        else:
            new_point = Point(random.random(), random.random())
        return new_point

    def sample(self):
        if self.informed and self.goal_node is not None:
            # pick a point that is inside elipse
            while True:
                new_point = self.sample_point()
                d = new_point.dist(self.goal) + new_point.dist(self.start)
                if d <= self.goal_node.dist:
                    break
        else:
            new_point = self.sample_point()

        if False and self.is_point_in_collision(new_point):
            self.sample_collisions.append(new_point)
            self.stop_march()
            return
        self.connect(new_point)

    def connect(self, new_point):
        parent = None
        max_dist = float("+inf")
        for n in self.nodes:
            d = point_dist(n.point, new_point)
            if d < max_dist:
                max_dist = d
                parent = n
        if max_dist > self.step_size:
            self.limited_lines.append(Line(parent.point, new_point))
            # pull actual sampled point closer to parent
            scale = self.step_size / max_dist
            self.march_end_point = new_point
            new_point = parent.point + (new_point - parent.point) * scale
            self.march_point = new_point
            if self.is_point_in_collision(new_point):
                self.sample_collisions.append(new_point)
                self.stop_march()
                return

        neighbors = []
        if self.neighborhood > 0.0:
            best_parent = None
            best_dist = float("+inf")
            for n in self.nodes:
                d = point_dist(n.point, new_point)
                if d < self.neighborhood:
                    if not self.is_edge_in_collision(new_point, n.point):
                        neighbors.append(n)
                        if d + n.dist < best_dist:
                            best_dist = d + n.dist
                            best_parent = n
            if best_parent is not None:
                if best_parent is not parent:
                    self.retarget_lines.append(Line(new_point, parent.point))
                parent = best_parent

        if self.is_edge_in_collision(new_point, parent.point):
            self.new_line_collisions.append(Line(parent.point, new_point))
            self.stop_march()
            return

        new_node = Node(new_point, parent)  # todo compute distance
        self.nodes.append(new_node)
        self.new_nodes.append(new_node)

        for n in neighbors:
            d = point_dist(n.point, new_point) + new_node.dist
            if d < n.dist:
                d_reduction = n.dist - d
                self.replaced_lines.append(Line(n.parent.point, n.point))
                # update n's to point to this node as it parent
                # to do this, we must remove n as child from n's parent
                n.parent.children.remove(n)
                n.parent = new_node
                new_node.children.append(n)
                self.modified_lines.append(Line(n.parent.point, n.point))
                self.reduce_dist(n, d_reduction)
        # todo use KD tree

    def reduce_dist(self, node: Node, d_reduction: float):
        node.dist -= d_reduction
        for child in node.children:
            self.new_reduced_lines.append(Line(node.point, child.point))
            self.reduce_dist(child, d_reduction)

    def draw_tree(self, node: Node, color):
        for child in node.children:
            if child in self.new_nodes:
                color = 'g'
                linewidth = 3.0
            else:
                color = 'k'
                linewidth = 1.0
            self.ax.plot([node.point.x, child.point.x], [
                         node.point.y, child.point.y],
                         color, linewidth=linewidth)
            self.draw_tree(child, color)

    def draw_lines(self, lines: List[Line], color, *, linewidth=1.0):
        for line in lines:
            self.ax.plot([line.p1.x, line.p2.x], [line.p1.y,
                         line.p2.y], color, linewidth=linewidth)

    def draw_nodes(self, nodes, color):
        self.ax.plot([n.point.x for n in nodes], [
                     n.point.y for n in nodes], color)

    def draw_elipse(self, point1: Point, point2: Point, dist: float):
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
        self.ax.plot(xs, ys, 'r--')

    def draw(self):
        self.fig = plt.figure("RRT", figsize=(self.figsize, self.figsize))
        self.fig.clf()
        self.ax = self.fig.subplots()
        self.ax.cla()
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        for x, y, r in self.obstacles:
            self.ax.add_patch(plt.Circle((x, y), r, color='r', alpha=0.3))
        if self.goal_node is not None:
            xs, ys = ([], [])
            node = self.goal_node
            while True:
                xs.append(node.point.x)
                ys.append(node.point.y)
                if node.parent is None:
                    break
                node = node.parent
            self.ax.plot(xs, ys, "c--", linewidth=4.0)
            if self.informed:
                self.draw_elipse(self.start, self.goal, self.goal_node.dist)
        if self.sample_collisions:
            self.ax.plot(*zip(*self.sample_collisions),
                         'r.', label='sample collision')
        self.draw_lines(self.new_line_collisions, 'r-', linewidth=3.0)
        self.draw_lines(self.line_collisions, 'r-')
        self.draw_lines(self.modified_lines, 'c-', linewidth=3.0)
        self.draw_lines(self.replaced_lines, 'm-', linewidth=3.0)
        self.draw_lines(self.retarget_lines, 'r--', linewidth=3.0)
        self.draw_lines(self.limited_lines, 'y--')
        if self.march and self.march_point:
            self.ax.plot([self.march_end_point .x, self.march_point.x], [
                         self.march_end_point .y, self.march_point.y], 'y-')
        self.draw_lines(self.new_reduced_lines, 'tab:orange', linewidth=3.0)
        self.draw_tree(self.nodes[0], 'k-')
        self.new_reduced_lines = []
        self.modified_lines = []
        self.replaced_lines = []
        self.retarget_lines = []
        self.limited_lines = []
        self.line_collisions += self.new_line_collisions
        self.new_line_collisions = []
        self.draw_nodes(self.nodes, 'k.')
        self.draw_nodes(self.new_nodes, 'b*')
        self.new_nodes = []
        self.ax.set_xlim((0, 1))
        self.ax.set_ylim((0, 1))
        self.ax.plot([self.goal.x], [self.goal.y], 'g*', label='goal')
        self.ax.plot([self.start.x], [self.start.y], 'r*', label='start')
        self.fig.tight_layout()

    def sum_dist(self, node: Node):
        dist = 0.0
        while node.parent is not None:
            dist += point_dist(node.point, node.parent.point)
            node = node.parent
        return dist

    def validate_dist(self, node: Node):
        errmsgs = []
        for child in node.children:
            assert(isinstance(child, Node))
            expected_dist = node.dist + point_dist(child.point, node.point)
            if abs(child.dist - expected_dist) > 1e-3:
                msg = "For node at {child.point}, expected dist {expected_dist} have {child.dist}"
                errmsgs.append(msg)
        for child in node.children:
            errmsgs += self.validate_dist(child)
        return errmsgs

    def validation_connectivity(self, node: Node):
        errmsgs = []
        for child in node.children:
            if child.parent is not node:
                errmsg = f"For node at {child.point}, parent is not dist {node.point}"
                errmsgs.append(errmsg)
            self.validation_connectivity(child)
        return errmsgs

    def validate(self):
        errmsgs = self.validate_dist(self.nodes[0])
        errmsgs += self.validation_connectivity(self.nodes[0])
        for errmsg in errmsgs:
            print("ERROR : ", errmsg)

    def plan_some(self, iterations=1, pause_iteration=None,
                  pause_at_goal=False):
        for _ in range(iterations):
            self.iteration += 1
            print("### Iteration", self.iteration, "###")
            if random.random() < self.exploration_bias:
                self.connect(self.goal)
                if ((self.goal_node is None) and (self.nodes[-1].point == self.goal)):
                    self.goal_node = self.nodes[-1]
                    self.goal_iteration = self.iteration
            else:
                self.sample()
            self.validate()
            print(f"  Nodes {len(self.nodes)}")
            print(f"  Point Collision Checks : {self.point_collision_checks}")
            print(f"  Edge Collision Checks: {self.edge_collision_checks}")
            if self.goal_node:
                print(f"  Goal Found Iteration: {self.goal_iteration}")
                print(f"  Goal path length: {self.goal_node.dist}")
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
    parser.add_argument("--informed", '-I', action="store_true",
                        help="Run informed RRT*")
    parser.add_argument("--march", action="store_true", help="Run RRT-march")
    parser.add_argument("--exploration-bias", "-B", type=float, default=0.05,
                        help="""Exploration bias in range 0.0 to 1.0.
                        A larger value will mean planner has a higher chance
                        of choosing direction to the goal""")
    parser.add_argument("--step-size", type=float, default=0.1,
                        help="""Maximum step size for connecting new node""")
    parser.add_argument("--neighborhood", "-N", type=float, default=0.0,
                        help="""Neighborhood to rewire with RRT* Value of 0
                        will disable neighborhood and do just RRT""")
    parser.add_argument("--obstacle-count", "-C", type=int,
                        default=7, help="""Count of randomly placed obstacles""")
    args = parser.parse_args()

    random.seed(args.seed)

    obstacles: List[Circle] = []
    for _ in range(args.obstacle_count):
        obstacles.append(Circle(random.random(),
                                random.random(),
                                random.uniform(0.05, 0.1)))

    def find_free_point():
        while True:
            point = Point(random.random(), random.random())
            if not is_point_in_collision(point, obstacles):
                return point

    start = find_free_point()
    goal = find_free_point()

    rrt = RRTCompare(
        obstacles,
        start,
        goal,
        informed=bool(args.informed),
        march=bool(args.march),
        exploration_bias=args.exploration_bias,
        step_size=args.step_size,
        neighborhood=args.neighborhood,
        figsize=args.fig_size)

    if args.pause is not None:
        pauses = sorted(sum(args.pause, []), reverse=True)
    else:
        defaut_final_pause = 1000
        print("Adding default final pause point at iteration", defaut_final_pause)
        pauses = [defaut_final_pause]
    period = 1.0 / args.rate
    while len(pauses) or (args.pause_at_goal and rrt.goal_node is None):
        start_t = time.time()
        pause_iteration = pauses[-1] if pauses else None
        rrt.plan_some(args.skip, pause_iteration, args.pause_at_goal)
        rrt.draw()
        if (rrt.iteration == pause_iteration) or args.single_step:
            print(f"  Paused at {pause_iteration} close figure to continue...")
            plt.ioff()
        elif args.pause_at_goal and (rrt.goal_iteration == rrt.iteration):
            print("  Paused after first reaching goal, close figure to continue...")
            plt.ioff()
        else:
            plt.ion()
        while pauses and (rrt.iteration >= pauses[-1]):
            pauses.pop()
        plt.show()
        if args.save:
            fn = f"{args.save}{rrt.iteration:04d}.png"
            print(f"  Saving image as {fn}")
            rrt.fig.savefig(fn)
        # slow down if computation is faster than given rate
        stop_t = time.time()
        dt = stop_t - start_t
        if dt < period:
            plt.pause(period - dt)

    print("Done: exiting")


if __name__ == "__main__":
    main()
