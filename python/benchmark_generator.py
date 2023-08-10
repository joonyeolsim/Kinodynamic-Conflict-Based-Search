import math
import yaml
import random

import argparse

def check_collision_obstacles(x, y, r, center_x, center_y, width, height):
    rectLeft, rectRight = center_x - width/2, center_x + width/2
    rectTop, rectBottom = center_y - height/2, center_y + height/2

    if x > rectLeft and x < rectRight and y > rectTop and y < rectBottom:
        return True

    closestX = min(max(x, rectLeft), rectRight)
    closestY = min(max(y, rectTop), rectBottom)

    distX, distY = x - closestX, y - closestY

    return (distX * distX + distY * distY) <= (r * r)


def is_conflict_with_previous_positions(x, y, prev_positions, radius):
    for prev_x, prev_y in prev_positions:
        if math.hypot(prev_x - x, prev_y - y) < 2 * radius:
            return True
    return False


def generate_position(search_min_width, search_max_width, search_min_height, search_max_height, radius, prev_starts, prev_goals, rectangle_obstacles):
    while True:
        x = random.uniform(search_min_width, search_max_width)
        y = random.uniform(search_min_height, search_max_height)
        conflict = is_conflict_with_previous_positions(x, y, prev_starts + prev_goals, radius)

        for obstacle in rectangle_obstacles:
            if check_collision_obstacles(x, y, radius, *obstacle):
                conflict = True
                break

        if not conflict:
            return [x, y]

def main():
    parser = argparse.ArgumentParser(description='Process some integers.')

    parser.add_argument('-b', '--basename', type=str, help='The basename for the environment')
    parser.add_argument('-n', '--num_of_robot', type=str, help='The number of robots')
    parser.add_argument('-c', '--test_count', type=str, help='The number of test count')

    args = parser.parse_args()

    # Access the arguments
    basename = args.basename
    num_of_robot = args.num_of_robot
    test_count = args.test_count

    # Print the values
    print(f'basename: {basename}')
    print(f'num_of_robot: {num_of_robot}')

    # basename = "OpenEnv"
    # num_of_robot = "30"
    with open(f"../benchmark/{basename}/{basename}_{num_of_robot}.yaml", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    for i in range(int(test_count)):
        radius = config["robotRadii"][0] * 2.5
        robot_num = config["robotNum"]
        rectangle_obstacles = config["rectangleObstacles"]
        search_min_width, search_max_width = radius, config["spaceLimit"][0] - radius
        search_min_height, search_max_height = radius, config["spaceLimit"][1] - radius

        starts, goals = [], []
        for _ in range(robot_num):
            start = generate_position(search_min_width, search_max_width, search_min_height, search_max_height, radius, starts, goals, rectangle_obstacles)
            starts.append(start)
            goal = generate_position(search_min_width, search_max_width, search_min_height, search_max_height, radius, starts, goals, rectangle_obstacles)
            goals.append(goal)

        config["startPoints"] = starts
        config["goalPoints"] = goals
        if len(config["robotRadii"]) < robot_num:
            config["robotRadii"] = [config["robotRadii"][0]] * robot_num

        with open(f'../benchmark/{basename}/{basename}_{num_of_robot}_{i}.yaml', 'w') as f:
            yaml.dump(config, f)
            print(f"Dumped ../benchmark/{basename}/{basename}_{num_of_robot}_{i}.yaml")


if __name__ == '__main__':
    main()
