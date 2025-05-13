import numpy as np
import math
import matplotlib.pyplot as plt
import heapq
from map_2 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f


def get_action():
    return [
        [0, 1, 1],
        [1, 0, 1],
        [0, -1, 1],
        [-1, 0, 1],
        [1, 1, math.sqrt(2)],
        [1, -1, math.sqrt(2)],
        [-1, -1, math.sqrt(2)],
        [-1, 1, math.sqrt(2)]
    ]


def heuristic(pos, goal):
    # Euclidean distance
    return math.hypot(goal[0] - pos[0], goal[1] - pos[1])


def collision_check(omap, node_pos):
    for (ox, oy) in zip(omap[0], omap[1]):
        if node_pos[0] == ox and node_pos[1] == oy:
            return True
    return False


def reconstruct_path(current_node):
    path = []
    while current_node is not None:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]


def a_star(start, goal, map_obstacle, weight=1.0):
    start_node = Node(None, tuple(start))
    goal_node = Node(None, tuple(goal))

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, (start_node.f, start_node))

    while open_list:
        _, current_node = heapq.heappop(open_list)

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node == goal_node:
            return reconstruct_path(current_node)

        for action in get_action():
            new_pos = (current_node.position[0] + action[0],
                       current_node.position[1] + action[1])
            cost = action[2]

            if collision_check(map_obstacle, new_pos):
                continue
            if new_pos in closed_set:
                continue

            neighbor = Node(current_node, new_pos)
            neighbor.g = current_node.g + cost
            neighbor.h = heuristic(new_pos, goal_node.position)
            neighbor.f = neighbor.g + weight * neighbor.h

            heapq.heappush(open_list, (neighbor.f, neighbor))

        if show_animation and len(closed_set) % 100 == 0:
            plt.plot(current_node.position[0], current_node.position[1], 'yo', alpha=0.3)
            plt.pause(0.001)

    return None  # No path found


def main():
    start, goal, omap = map()

    if show_animation:
        plt.figure(figsize=(8, 8))
        plt.plot(start[0], start[1], 'bs', markersize=7)
        plt.text(start[0], start[1] + 0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs', markersize=7)
        plt.text(goal[0], goal[1] + 0.5, 'goal', fontsize=12)
        plt.plot(omap[0], omap[1], '.k', markersize=10)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("A* (Weighted: w=1.0)", fontsize=20)

    weight = 1.0  # 기본값: A*
    # weight = 1.5  # Weighted A*
    opt_path = a_star(start, goal, omap, weight)

    if opt_path is not None:
        print("Optimal path found!")
        opt_path = np.array(opt_path)
        if show_animation:
            plt.plot(opt_path[:, 0], opt_path[:, 1], "m.-")
            plt.show()
    else:
        print("No path found.")


if __name__ == "__main__":
    main()
