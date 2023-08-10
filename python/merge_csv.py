if __name__ == '__main__':
    basename_list = ["OpenEnv"]
    robot_num_list = [
        [5, 10, 15, 20, 30],
        # [5, 10, 15, 20, 30],
    ]
    count = 30
    for basename, robot_nums in zip(basename_list, robot_num_list):
        for robot_num in robot_nums:
            new_data = []
            for i in range(count):
                data_file = f"../raw_data/{basename}_{robot_num}_{i}_data.csv"
                with open(data_file, 'r') as f:
                    new_data.append(f.readline())
            with open(f"../test_result/{basename}_{robot_num}.csv", 'a') as f:
                for new_d in new_data:
                    f.writelines(new_d + '\n')
