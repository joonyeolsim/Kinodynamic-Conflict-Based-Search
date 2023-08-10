import subprocess
import multiprocessing


if __name__ == '__main__':
    basename_list = ["OpenEnv"]
    robot_num_list = [
        [5, 10, 15, 20, 25, 30],
        # [5, 10, 15, 20, 25, 30],
    ]
    count = 30
    for basename, robot_nums in zip(basename_list, robot_num_list):
        for robot_num in robot_nums:
            for i in range(count):
                benchmark_file = f"../benchmark/{basename}/{basename}_{robot_num}_{i}.yaml"

                test = subprocess.Popen([f"../cmake-build-debug/ST_CBS", f"../benchmark/{basename}/{basename}_{robot_num}_{i}"], stdout=subprocess.PIPE)
                print(f'Test case : {basename}_{robot_num}_{i}')
                try:
                    out, err = test.communicate(timeout=5)
                except:
                    print(f'Timeout kill {basename}_{robot_num}_{i}')
                    test.kill()