import matplotlib.pyplot as plt
import matplotlib.patches as patches
import yaml
import argparse

def visualize_environment(start_points, goal_points, obstacles):
    # Initialize the figure and axis
    fig, ax = plt.subplots(figsize=(12, 12))

    # Plot start points (triangle markers)
    for point in start_points:
        ax.plot(point[0], point[1], 'g^', markersize=10)

    # Plot goal points (circle markers)
    for point in goal_points:
        ax.plot(point[0], point[1], 'ro', markersize=10)

    # Plot obstacles (rectangles)
    for obstacle in obstacles:
        x, y, width, height = obstacle
        ax.add_patch(patches.Rectangle((x - (width / 2), y - (height / 2)), width, height, fill=True, color='gray'))

    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Environment Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()

parser = argparse.ArgumentParser(description='Process some integers.')

parser.add_argument('-b', '--basename', type=str, help='The basename for the environment')
parser.add_argument('-n', '--num_of_robot', type=str, help='The number of robots')
parser.add_argument('-c', '--test_count', type=str, help='The number of test count')

args = parser.parse_args()

# Access the arguments
basename = args.basename
num_of_robot = args.num_of_robot
test_count = args.test_count

with open(f"../benchmark/ClutteredEnv/{basename}_{num_of_robot}_{test_count}.yaml", 'r') as file:
    data = yaml.safe_load(file)

# Extract relevant data from the loaded YAML data
start_points = data['startPoints']
goal_points = data['goalPoints']
obstacles = data['rectangleObstacles']

# Visualize the environment
visualize_environment(start_points, goal_points, obstacles)
