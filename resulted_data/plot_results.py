import json
import matplotlib.pyplot as plt

def load_results(filename):
    with open(filename, 'r') as f:
        return json.load(f)

# Load the results
results_05 = load_results('results_kp_0.5.json')
results_1 = load_results('results_kp_1.0.json')
results_2 = load_results('results_kp_2.0.json')

# # t vs. x
# plt.figure(figsize=(10, 5))
# plt.plot(results_05['time_stamps'], results_05['desired_xs'],label='Ground Truth')
# # plt.plot(results_1['time_stamps'], results_1['desired_xs'],label='Ground Truth 1')
# # plt.plot(results_2['time_stamps'], results_2['desired_xs'],label='Ground Truth 2')
# plt.plot(results_05['time_stamps'], results_05['actual_xs'], label='Kp = 0.5')
# plt.plot(results_1['time_stamps'], results_1['actual_xs'], label='Kp = 1.0')
# plt.plot(results_2['time_stamps'], results_2['actual_xs'], label='Kp = 2.0')
# plt.title('Actual x-coordinate Over Time for Different Kp Values')
# plt.xlabel('t (s)')
# plt.ylabel('x (m)')
# plt.legend()
# plt.grid(True)
# plt.show()

# t vs. y
plt.figure(figsize=(10, 5))
plt.plot(results_05['time_stamps'], results_05['desired_y'],label='Ground Truth')
# plt.plot(results_1['time_stamps'], results_1['desired_xs'],label='Ground Truth 1')
# plt.plot(results_2['time_stamps'], results_2['desired_xs'],label='Ground Truth 2')
plt.plot(results_05['time_stamps'], results_05['actual_y'], label='Kp = 0.5')
plt.plot(results_1['time_stamps'], results_1['actual_y'], label='Kp = 1.0')
plt.plot(results_2['time_stamps'], results_2['actual_y'], label='Kp = 2.0')
plt.title('Actual y-coordinate Over Time for Different Kp Values')
plt.xlabel('t (s)')
plt.ylabel('y (m)')
plt.legend()
plt.grid(True)
plt.show()

# # x vs. y
# plt.figure(figsize=(10, 10))
# plt.plot(results_05['desired_xs'], results_05['desired_y'],label='Ground Truth')
# # plt.plot(results_1['time_stamps'], results_1['desired_xs'],label='Ground Truth 1')
# # plt.plot(results_2['time_stamps'], results_2['desired_xs'],label='Ground Truth 2')
# plt.plot(results_05['actual_xs'], results_05['actual_y'], label='Kp = 0.5')
# plt.plot(results_1['actual_xs'], results_1['actual_y'], label='Kp = 1.0')
# plt.plot(results_2['actual_xs'], results_2['actual_y'], label='Kp = 2.0')
# plt.title('Trajectory for Different Kp Values')
# plt.xlabel('t (s)')
# plt.ylabel('x (m)')
# plt.legend()
# plt.grid(True)
# plt.show()

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
