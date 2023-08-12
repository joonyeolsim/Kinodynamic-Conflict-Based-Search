import subprocess
import multiprocessing


if __name__ == '__main__':
    basename_list = ["ConfinedEnv", "NarrowEnv", "OpenEnv", "ClutteredEnv"]
    robot_num_list = [
        [5],
        [2],
        [5, 10, 15, 20, 25, 30],
        [5, 10, 15, 20, 25, 30],
    ]
    count = 50
    for basename, robot_nums in zip(basename_list, robot_num_list):
        for robot_num in robot_nums:
            for i in range(count):
                benchmark_file = f"../benchmark/{basename}/{basename}_{robot_num}_{i}.yaml"

                try:
                    test = subprocess.Popen([f"../cmake-build-debug/ST_CBS", f"{basename}", f"{robot_num}", f"{i}"], stdout=subprocess.PIPE)
                except FileNotFoundError:
                    print(f'File not found {basename}_{robot_num}_{i}')
                    continue

                print(f'Test case : {basename}_{robot_num}_{i}')
                try:
                    out, err = test.communicate(timeout=300)
                except:
                    print(f'Timeout kill {basename}_{robot_num}_{i}')
                    test.kill()