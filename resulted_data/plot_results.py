import json
import matplotlib.pyplot as plt
import os

# Define the directory where you want to save the figures
save_dir = "/home/yu/me326_ros2_ws/resulted_data/"

# Check if the directory exists, if not, create it
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Now, append your filenames to this directory when saving


def load_results(filename):
    # Construct an absolute path to the file
    base_dir = "/home/yu/me326_ros2_ws/resulted_data/"
    file_path = os.path.join(base_dir, filename)
    
    with open(file_path, 'r') as f:
        return json.load(f)

# Load the results
results_05 = load_results('results_kp_0.5.json')
results_1 = load_results('results_kp_1.0.json')
results_2 = load_results('results_kp_2.0.json')

import matplotlib.pyplot as plt

# Assuming results_05, results_1, results_2 are defined and loaded from your JSON files

# t vs. x
plt.figure(figsize=(10, 5))
plt.plot(results_05['time_stamps'], results_05['desired_x'], label='Ground Truth')
plt.plot(results_05['time_stamps'], results_05['actual_x'], label='Kp = 0.5')
plt.plot(results_1['time_stamps'], results_1['actual_x'], label='Kp = 1.0')
plt.plot(results_2['time_stamps'], results_2['actual_x'], label='Kp = 2.0')
plt.title('Actual x-coordinate Over Time for Different Kp Values')
plt.xlabel('Time (s)')
plt.ylabel('x-coordinate (m)')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_dir, 't_vs_x.png'))
plt.show()

# t vs. y
plt.figure(figsize=(10, 5))
plt.plot(results_05['time_stamps'], results_05['desired_y'], label='Ground Truth')
plt.plot(results_05['time_stamps'], results_05['actual_y'], label='Kp = 0.5')
plt.plot(results_1['time_stamps'], results_1['actual_y'], label='Kp = 1.0')
plt.plot(results_2['time_stamps'], results_2['actual_y'], label='Kp = 2.0')
plt.title('Actual y-coordinate Over Time for Different Kp Values')
plt.xlabel('Time (s)')
plt.ylabel('y-coordinate (m)')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_dir, 't_vs_y.png'))
plt.show()

# x vs. y
plt.figure(figsize=(10, 10))
plt.plot(results_05['desired_x'], results_05['desired_y'], label='Ground Truth')
plt.plot(results_05['actual_x'], results_05['actual_y'], label='Kp = 0.5')
plt.plot(results_1['actual_x'], results_1['actual_y'], label='Kp = 1.0')
plt.plot(results_2['actual_x'], results_2['actual_y'], label='Kp = 2.0')
plt.title('Trajectory for Different Kp Values')
plt.xlabel('x-coordinate (m)')
plt.ylabel('y-coordinate (m)')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_dir, 'x_vs_y.png'))
plt.show()


# import csv

# # Function to convert JSON file to CSV
# def convert_json_to_csv(json_file_path, csv_file_path):
#     # Open the JSON file and load the data
#     with open(json_file_path, 'r') as jfile:
#         data = json.load(jfile)
    
#     # Open the CSV file for writing
#     with open(csv_file_path, 'w', newline='') as cfile:
#         writer = csv.writer(cfile)
#         # Write the header
#         writer.writerow(['time_stamps', 'desired_xs','actual_xs','desired_y', 'actual_y'])
#         # Write the data
#         for time_stamp,desired_x, actual_x,desired_y, actual_y in zip(data['time_stamps'], data['desired_xs'],data['actual_xs'],data['desired_y'],data['actual_y']):
#             writer.writerow([time_stamp,desired_x, actual_x,desired_y,actual_y])

# # Example usage
# json_file_path = 'results_kp_2.0.json'  # Replace with your JSON file path
# csv_file_path = 'results_kp_2.0.csv'  # Replace with your desired CSV file path

# convert_json_to_csv(json_file_path, csv_file_path)
