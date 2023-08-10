import math

import yaml
import random


def check_collision_obstacles(x, y, r, center_x, center_y, width, height):
    # 직사각형의 영역을 구한다
    rectLeft = center_x - width/2
    rectRight = center_x + width/2
    rectTop = center_y - height/2
    rectBottom = center_y + height/2

    # 원의 중심이 사각형 내부에 있는 경우, 충돌한다
    if x > rectLeft and x < rectRight and y > rectTop and y < rectBottom:
        return True

    # 원의 중심이 사각형 바깥에 있을 때, 사각형의 가장 가까운 경계와 원의 중심 사이의 거리를 계산한다
    closestX = rectLeft if x < rectLeft else rectRight if x > rectRight else x
    closestY = rectTop if y < rectTop else rectBottom if y > rectBottom else y

    distX = x - closestX
    distY = y - closestY

    # 거리와 원의 반지름을 비교한다
    return (distX * distX + distY * distY) <= (r * r)


if __name__ == '__main__':
    count = 1
    basename = "ClutteredEnv_10"
    with open(f"../benchmark/ClutteredEnv/{basename}.yaml", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    for i in range(count):
        radius = config["robotRadii"][0]
        robot_num = config["robotNum"]
        spaceLimit = config["spaceLimit"]
        rectangle_obstacles = config["rectangleObstacles"]

        if len(config["robotRadii"]) < robot_num:
            config["robotRadii"] = [radius] * robot_num

        search_min_width = radius
        search_min_height = radius
        search_max_width = config["spaceLimit"][0] - radius
        search_max_height = config["spaceLimit"][1] - radius

        def generate_start_and_goal():
            while True:
                conflict = False
                start = [random.uniform(search_min_width, search_max_width),
                         random.uniform(search_min_height, search_max_height)]
                goal = [random.uniform(search_min_width, search_max_width),
                        random.uniform(search_min_height, search_max_height)]
                for previous_start, previous_goal in zip(starts, goals):
                    if math.hypot(previous_start[0] - start[0], previous_start[1] - start[1]) < radius + radius or \
                            math.hypot(previous_goal[0] - start[0], previous_goal[1] - start[1]) < radius + radius:
                        conflict = True
                    if math.hypot(previous_start[0] - goal[0], previous_start[1] - goal[1]) < radius + radius or \
                            math.hypot(previous_goal[0] - goal[0], previous_goal[1] - goal[1]) < radius + radius:
                        conflict = True

                for obstacle in config["rectangleObstacles"]:
                    if check_collision_obstacles(start[0], start[1], radius, obstacle[0], obstacle[1], obstacle[2], obstacle[3]):
                        conflict = True
                    if check_collision_obstacles(goal[0], goal[1], radius, obstacle[0], obstacle[1], obstacle[2], obstacle[3]):
                        conflict = True

                if not conflict:
                    return start, goal

        # generate random starts and goals for each robot
        starts = []
        goals = []
        for _ in range(config["robotNum"]):
            start, goal = generate_start_and_goal()
            starts.append(start)
            goals.append(goal)

        config["startPoints"] = starts
        config["goalPoints"] = goals
        for start, goal in zip(starts, goals):
            for obstacle in config["rectangleObstacles"]:
                if check_collision_obstacles(start[0], start[1], radius, obstacle[0], obstacle[1], obstacle[2], obstacle[3]):
                    print("Collision")
                if check_collision_obstacles(goal[0], goal[1], radius, obstacle[0], obstacle[1], obstacle[2], obstacle[3]):
                    print("Collision")
        # dump benchmark
        with open(f'../benchmark/ClutteredEnv/{basename}_{i}.yaml', 'w') as f:
            yaml.dump(config, f)
            print(f"Dumped {basename}_{i}.yaml")