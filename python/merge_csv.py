if __name__ == '__main__':
    basename_list = ["ConfinedEnv", "NarrowEnv", "OpenEnv", "ClutteredEnv"]
    robot_num_list = [
        [5],
        [2],
        [5, 10, 15, 20, 25, 30],
        [5, 10, 15, 20, 25, 30],
	[15]
    ]
    count = 50
    for basename, robot_nums in zip(basename_list, robot_num_list):
        for robot_num in robot_nums:
            new_data = []
            for i in range(count):
                data_file = f"../raw_data/{basename}/{basename}_{robot_num}_{i}_data.csv"
                try:
                    with open(data_file, 'r') as f:
                        new_data.append(f.readline())
                except FileNotFoundError:
                    print(f'File not found {basename}_{robot_num}_{i}')
            with open(f"../test_result/{basename}/{basename}_{robot_num}.csv", 'a') as f:
                for new_d in new_data:
                    f.writelines(new_d + '\n')
