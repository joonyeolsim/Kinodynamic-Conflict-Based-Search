import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import yaml

config_file = "benchmark/BaseEnv_4.yaml"

# benchmark 파일
with open(config_file, 'r') as stream:
    config = yaml.load(stream, Loader=yaml.FullLoader)

# 로봇의 경로
with open("paths/path.yaml", 'r') as stream:
    path = yaml.load(stream, Loader=yaml.FullLoader)

print(path)
# x, y, time 좌표 추출
x_coords = [point["x"] for point in path]
y_coords = [point["y"] for point in path]
time_coords = [point["time"] for point in path]

# 3D 그래프 설정
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 로봇의 경로 표시
ax.plot(x_coords, y_coords, time_coords, marker='o')

# 그래프의 축 설정
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')

def create_cube(center_x, center_y, width, height, depth):
    x, y = center_x - width / 2, center_y - height / 2

    vertices = np.array([
        [x, y, 0],
        [x + width, y, 0],
        [x + width, y + height, 0],
        [x, y + height, 0],
        [x, y, depth],
        [x + width, y, depth],
        [x + width, y + height, depth],
        [x, y + height, depth]
    ])

    faces = [
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[7], vertices[6], vertices[2], vertices[3]],
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[7], vertices[6], vertices[5], vertices[4]],
        [vertices[7], vertices[3], vertices[0], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]]
    ]

    return faces

# find max time in solution
max_time = len(path)
for rect_obstacle in config["rectangleObstacles"]:
    cube_faces = create_cube(rect_obstacle[0], rect_obstacle[1], rect_obstacle[2], rect_obstacle[3], max_time)
    face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.1, linewidths=1, edgecolors='k')
    ax.add_collection3d(face_collection)

# 그래프 출력
plt.show()
