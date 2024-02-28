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

import collections
import random
from typing import List, NamedTuple
from math import sqrt

Line = collections.namedtuple('Line', ['p1', 'p2'])
Circle = collections.namedtuple('Circle', ['x', 'y', 'radius'])


class Point(NamedTuple):
    x: float
    y: float

    def calc_dist(self, other: 'Point') -> float:
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


def is_edge_in_collision(p1: Point, p2: Point, obstacles: List[Circle]) -> bool:
    dx, dy = (p2.x - p1.x, p2.y - p1.y)
    d = sqrt(dx*dx + dy*dy)
    if d == 0.0:
        # pretend a zero length edge is in collision so it is not added
        print("WARN : Zero length edge checked for collision")
        return True
    c, s = (dx/d, dy/d)
    for x, y, r in obstacles:
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


def generate_obstacles(obstacle_count: int, min_radius=0.05, max_radius=0.1) -> List[Circle]:
    obstacles: List[Circle] = []
    for _ in range(obstacle_count):
        obstacles.append(Circle(random.random(),
                                random.random(),
                                random.uniform(min_radius, max_radius)))
    return obstacles


def find_free_point(obstacles: List[Circle]):
    while True:
        point = Point(random.random(), random.random())
        if not is_point_in_collision(point, obstacles):
            return point
