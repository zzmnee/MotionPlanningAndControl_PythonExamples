import numpy as np
import math
import matplotlib.pyplot as plt
from map_1 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def get_action():
    # action = [dx, dy, cost]
    action_set = [
        (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),         # 상하좌우
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),       # 대각선
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))
    ]
    return action_set


def collision_check(omap, node):
    x, y = node.position
    for (ox, oy) in zip(omap[0], omap[1]):
        if (x == ox) and (y == oy):
            return True
    return False


def dijkstra(start, goal, map_obstacle):
    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = []
    closed_list = []
    visited_positions = set()

    open_list.append(start_node)

    while open_list:
        # Find node with lowest cost
        cur_node = min(open_list, key=lambda node: node.f)
        open_list.remove(cur_node)
        closed_list.append(cur_node)
        visited_positions.add(cur_node.position)

        # If goal, return optimal path
        if cur_node == goal_node:
            path = []
            while cur_node is not None:
                path.append(cur_node.position)
                cur_node = cur_node.parent
            return path[::-1]

        # Generate child candidates
        action_set = get_action()
        for dx, dy, cost in action_set:
            new_pos = (cur_node.position[0] + dx, cur_node.position[1] + dy)
            child = Node(cur_node, new_pos)
            child.f = cur_node.f + cost

            if collision_check(map_obstacle, child):
                continue
            if child.position in visited_positions:
                continue

            existing = next((n for n in open_list if n == child), None)
            if existing is None:
                open_list.append(child)
            elif existing.f > child.f:
                open_list.remove(existing)
                open_list.append(child)

        # show graph
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 100 == 0:
                plt.pause(0.001)

    return []


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
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("Dijkstra algorithm", fontsize=20)

    opt_path = dijkstra(start, goal, omap)
    print("Optimal path found!")
    opt_path = np.array(opt_path)
    if show_animation:
        plt.plot(opt_path[:, 0], opt_path[:, 1], "m.-")
        plt.show()


if __name__ == "__main__":
    main()
