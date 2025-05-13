import numpy as np
import math
import matplotlib.pyplot as plt
import heapq
from map_3 import map  # 같은 폴더에 map_3.py가 있어야 함

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position  # [x, y, yaw]
        self.heading = 0.0
        self.f = 0
        self.g = 0
        self.h = 0

def isSamePosition(node_1, node_2, epsilon_position=0.3):
    dx = node_1.position[0] - node_2.position[0]
    dy = node_1.position[1] - node_2.position[1]
    return np.hypot(dx, dy) < epsilon_position

def isSameYaw(node_1, node_2, epsilon_yaw=0.2):
    return abs(node_1.position[2] - node_2.position[2]) < epsilon_yaw

def get_action(R, Vx, delta_time_step):
    yaw_rate = Vx / R
    distance_travel = Vx * delta_time_step
    return [
        [yaw_rate, delta_time_step, distance_travel],
        [-yaw_rate, delta_time_step, distance_travel],
        [yaw_rate / 2, delta_time_step, distance_travel],
        [-yaw_rate / 2, delta_time_step, distance_travel],
        [0.0, delta_time_step, distance_travel]
    ]

def vehicle_move(position_parent, yaw_rate, delta_time, Vx):
    x, y, yaw = position_parent

    if abs(yaw_rate) > 1e-5:
        R = Vx / yaw_rate
        cx = x - R * math.sin(yaw)
        cy = y + R * math.cos(yaw)
        dtheta = yaw_rate * delta_time
        yaw_child = (yaw + dtheta) % (2 * np.pi)
        x_child = cx + R * math.sin(yaw_child)
        y_child = cy - R * math.cos(yaw_child)
    else:
        x_child = x + Vx * delta_time * math.cos(yaw)
        y_child = y + Vx * delta_time * math.sin(yaw)
        yaw_child = yaw % (2 * np.pi)

    return [x_child, y_child, yaw_child]

def collision_check(position_parent, yaw_rate, delta_time_step, obstacle_list, Vx):
    x, y, _ = vehicle_move(position_parent, yaw_rate, delta_time_step, Vx)
    for (ox, oy, r) in obstacle_list:
        if np.hypot(ox - x, oy - y) <= r:
            return True
    return False

def isNotInSearchingSpace(position, space):
    x_min, x_max, y_min, y_max = space
    x, y = position[0], position[1]
    return not (x_min <= x <= x_max and y_min <= y <= y_max)

def heuristic(cur_node, goal_node):
    dx = cur_node.position[0] - goal_node.position[0]
    dy = cur_node.position[1] - goal_node.position[1]
    return np.hypot(dx, dy)

def reconstruct_path(current_node):
    path = []
    while current_node is not None:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]

def a_star(start, goal, space, obstacle_list, R, Vx, delta_time_step, weight):
    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = []
    closed_list = []

    heapq.heappush(open_list, (start_node.f, start_node))

    while open_list:
        _, cur_node = heapq.heappop(open_list)

        if isSamePosition(cur_node, goal_node, epsilon_position=0.6):
            return reconstruct_path(cur_node)

        # 이미 방문한 노드인지 확인
        skip = False
        for closed_node in closed_list:
            if isSamePosition(cur_node, closed_node) and isSameYaw(cur_node, closed_node):
                skip = True
                break
        if skip:
            continue

        closed_list.append(cur_node)

        for action in get_action(R, Vx, delta_time_step):
            yaw_rate, dt, cost = action
            child_pos = vehicle_move(cur_node.position, yaw_rate, dt, Vx)

            if isNotInSearchingSpace(child_pos, space):
                continue
            if collision_check(cur_node.position, yaw_rate, dt, obstacle_list, Vx):
                continue

            child_node = Node(cur_node, child_pos)
            child_node.g = cur_node.g + cost
            child_node.h = heuristic(child_node, goal_node)
            child_node.f = child_node.g + weight * child_node.h

            already_in_open = False
            for open_f, open_node in open_list:
                if isSamePosition(child_node, open_node) and isSameYaw(child_node, open_node):
                    if child_node.g < open_node.g:
                        open_node.g = child_node.g
                        open_node.f = child_node.f
                        open_node.parent = cur_node
                    already_in_open = True
                    break

            if not already_in_open:
                heapq.heappush(open_list, (child_node.f, child_node))

        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 100 == 0:
                plt.pause(0.01)

    return None

def main():
    start, goal, obstacle_list, space = map()

    if show_animation:
        theta_plot = np.linspace(0, 1, 101) * 2 * np.pi
        plt.figure(figsize=(8, 8))
        plt.plot(start[0], start[1], 'bs', markersize=7)
        plt.text(start[0], start[1]+0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs', markersize=7)
        plt.text(goal[0], goal[1]+0.5, 'goal', fontsize=12)
        for ox, oy, r in obstacle_list:
            x_obs = ox + r * np.cos(theta_plot)
            y_obs = oy + r * np.sin(theta_plot)
            plt.plot(x_obs, y_obs, 'k-')
        plt.axis(space)
        plt.grid(True)
        plt.title("Hybrid A* Path Planning", fontsize=20)

    path = a_star(start, goal, space, obstacle_list,
                  R=5.0, Vx=2.0, delta_time_step=0.5, weight=1.0)

    if path is not None:
        print("Optimal path found!")
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], "m.-")
        plt.show()
    else:
        print("Path not found.")

if __name__ == "__main__":
    main()
