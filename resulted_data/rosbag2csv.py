import pandas as pd
import re
import matplotlib.pyplot as plt

# Function to parse the rosbag output from a CSV file and write to another CSV file
def parse_rosbag_csv(input_csv_file, output_csv_file):
    # Read the content from the input CSV file
    with open(input_csv_file, 'r') as file:
        rosbag_output = file.read()
    
    # The pattern matches the sec, nanosec and the x component of linear twist.
    pattern = re.compile(r"sec: (\d+)\s+nanosec: (\d+).*?position:\s+x: ([\d\.-]+)\s+y: ([\d\.-]+)", re.DOTALL)

    # Extract all matches
    matches = pattern.findall(rosbag_output)

    # Create a DataFrame from the matches
    df = pd.DataFrame(matches, columns=['sec', 'nanosec', 'pos_x','pos_y'])

    # Convert the columns to the appropriate data types
    df['sec'] = df['sec'].astype(int)
    df['nanosec'] = df['nanosec'].astype(int)
    df['pos_x'] = df['pos_x'].astype(float)
    df['pos_y'] = df['pos_y'].astype(float)

    # Combine 'sec' and 'nanosec' to a single 'timestamp' in seconds
    df['timestamp'] = df['sec'] + df['nanosec'] * 1e-9

    # Drop the old 'sec' and 'nanosec' columns
    df.drop(['sec', 'nanosec'], axis=1, inplace=True)
    
    # Write the DataFrame to a CSV file
    df.to_csv(output_csv_file, index=False)
    
    return df

# Function to plot the data and save the plot
def plot_data(df, plot_file_name):
    plt.figure(figsize=(10, 5))
    # Convert pandas series to numpy arrays before plotting
    timestamps = df['timestamp'].to_numpy()
    linear_xs = df['pos_x'].to_numpy()
    plt.plot(timestamps, linear_xs, marker='o')
    plt.title('Linear x-component vs. Timestamp')
    plt.xlabel('Timestamp (seconds)')
    plt.ylabel('Linear x-component')
    plt.grid(True)
    # Save the plot
    plt.savefig(plot_file_name)
    plt.close()  # Close the figure to free memory

# # File names
# input_csv_file = 'kp_05_data.csv'
# output_csv_file = 'kp_05_data_parsed.csv'
# input_csv_file_1 = 'kp_1_data.csv'
# output_csv_file_1 = 'kp_1_data_parsed.csv'
input_csv_file_2 = 'kp_2_data.csv'
output_csv_file_2 = 'kp_2_data_parsed.csv'


# Parse the CSV
# df = parse_rosbag_csv(input_csv_file, output_csv_file)
# df_1 = parse_rosbag_csv(input_csv_file_1, output_csv_file_1)
df_2 = parse_rosbag_csv(input_csv_file_2,output_csv_file_2)

# Construct plot file name by replacing .csv with .png
# plot_file_name = output_csv_file.replace('.csv', '.png')
# plot_file_name_1 = output_csv_file_1.replace('.csv', '.png')
plot_file_name_2 = output_csv_file_2.replace('.csv', '.png')
# Plot the data and save the plot
plot_data(df_2, plot_file_name_2)
# plot_data(df_1,plot_file_name_1)