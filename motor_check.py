import os
import pandas as pd
import shutil
import csv

def analyze_battery_data(filename):
    data = pd.read_csv(filename)
    current_values = data['current']
    timestamps = pd.to_datetime(data['timestamp'])

    # 初始阶段：前4秒
    initial_phase = current_values[timestamps <= timestamps.iloc[0] + pd.Timedelta(seconds=3)]
    # 稳定阶段：从第7秒到倒数第7秒
    steady_phase = current_values[(timestamps > timestamps.iloc[0] + pd.Timedelta(seconds=7)) & 
                                  (timestamps < timestamps.iloc[-1] - pd.Timedelta(seconds=7))]
    # 最终阶段：倒数后4秒
    final_phase = current_values[timestamps >= timestamps.iloc[-1] - pd.Timedelta(seconds=3)]

    # 这里的检查是基于60%推力情况下，推30秒的情况。
    initial_check = initial_phase.max() < 0.25
    steady_check = steady_phase.between(1, 1.25).all()
    final_check = final_phase.max() < 0.25

    overall_check = current_values.max() <= 1.25

    if initial_check and steady_check and final_check and overall_check:
        return 'good'
    else:
        return 'bad'

def process_folders(base_folder):
    good_folder = os.path.join(base_folder, 'good')
    bad_folder = os.path.join(base_folder, 'bad')

    if not os.path.exists(good_folder):
        os.makedirs(good_folder)
    if not os.path.exists(bad_folder):
        os.makedirs(bad_folder)

    bad_drones = []

    for folder in os.listdir(base_folder):
        folder_path = os.path.join(base_folder, folder)
        if os.path.isdir(folder_path) and folder not in ['good', 'bad']:
            for sub_folder in os.listdir(folder_path):
                sub_folder_path = os.path.join(folder_path, sub_folder)
                if os.path.isdir(sub_folder_path):
                    for file in os.listdir(sub_folder_path):
                        if file.startswith('battery_info') and file.endswith('.csv'):
                            file_path = os.path.join(sub_folder_path, file)
                            result = analyze_battery_data(file_path)
                            destination_folder = good_folder if result == 'good' else bad_folder
                            target_path = os.path.join(destination_folder, sub_folder)

                            # # 如果目标路径已存在，重命名目标路径
                            # if os.path.exists(target_path):
                            #     base_name = target_path
                            #     counter = 1
                            #     while os.path.exists(target_path):
                            #         target_path = f"{base_name}_{counter}"
                            #         counter += 1

                            shutil.move(sub_folder_path, target_path)

                            if result == 'bad':
                                drone_id = file.split('_')[1]
                                bad_drones.append((sub_folder, drone_id))

    if bad_drones:
        bad_drones_file = os.path.join(bad_folder, 'bad_drones.csv')
        with open(bad_drones_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['folder', 'drone_id'])
            for folder, drone_id in bad_drones:
                writer.writerow([folder, drone_id])

def main():
    current_folder = os.path.dirname(os.path.abspath(__file__))
    process_folders(current_folder)

if __name__ == '__main__':
    main()
