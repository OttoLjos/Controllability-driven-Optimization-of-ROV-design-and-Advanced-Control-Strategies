# This code was created with the help of Chat GPT

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

# Configuration: Specify which controllers have been tested
# For example: ['PID', 'LQR', 'SMC'] or ['PID'] or ['SMC', 'LQR'], etc.

#  Translasjon
#tested_controllers = ['none', 'none', 'PID', 'LQR', 'SMC']
#time= 0.75
#file_path = '2024-05-20 15.31.02 sensor.log'

plot = 0

# Settpunkt
tested_controllers = ['PID', 'LQR', 'SMC']
time= 1.5
file_path = '2024-05-24 17.56.32 sensor.log'
colors = {
    'PID': 'red',
    'LQR': 'blue',
    'SMC': 'green'
}

# Sinus
"""
tested_controllers = ['PID', 'LQR', 'SMC']
time= 2
file_path = '2024-05-24 18.07.28 sensor.log'
colors = {
    'PID': 'red',
    'LQR': 'blue',
    'SMC': 'green'
}
"""

gyro_data = []
setpoint_data = []
times = []
setpoint_times = []

with open(file_path, 'r') as file:
    for line in file:
        if 'gyro' in line:
            # Extract the time
            time_match = re.match(r'(\d{4}-\d{2}-\d{2} (\d{2}:\d{2}:\d{2},\d{3}))', line)
            if time_match:
                # Extract only the time part
                times.append(pd.to_datetime(time_match.group(2), format='%H:%M:%S,%f'))
            
            # Extract the numeric values after 'gyro' using regex
            gyro_match = re.search(r"gyro': \[([-\d.,\s]+)\]", line)
            if gyro_match:
                values = gyro_match.group(1).split(',')
                gyro_data.append([float(value.strip()) for value in values[:3]])

        if 'settpunkt' in line:
            # Extract the time
            setpoint_time_match = re.match(r'(\d{4}-\d{2}-\d{2} (\d{2}:\d{2}:\d{2},\d{3}))', line)
            if setpoint_time_match:
                # Extract only the time part
                setpoint_times.append(pd.to_datetime(setpoint_time_match.group(2), format='%H:%M:%S,%f'))
            
            # Extract the numeric values after 'settpunkt' using regex
            setpoint_match = re.search(r"settpunkt': \[([-\d.,\s]+)\]", line)
            if setpoint_match:
                values = setpoint_match.group(1).split(',')
                setpoint_data.append([float(value.strip()) for value in values[:2]])

# Ensure the lengths match
if len(setpoint_times) > len(setpoint_data):
    setpoint_times = setpoint_times[:len(setpoint_data)]
elif len(setpoint_data) > len(setpoint_times):
    setpoint_data = setpoint_data[:len(setpoint_times)]

# Convert to DataFrame
gyro_df = pd.DataFrame(gyro_data, columns=['Depth (cm)', 'Roll (degrees)', 'Pitch (degrees)'], index=times)
setpoint_df = pd.DataFrame(setpoint_data, columns=['Key', 'Value'], index=setpoint_times)

# Split setpoint data into separate columns based on the Key
setpoint_df['Heave Setpoint'] = setpoint_df.apply(lambda row: row['Value'] if row['Key'] == 10.0 else np.nan, axis=1)
setpoint_df['Roll Setpoint'] = setpoint_df.apply(lambda row: row['Value'] if row['Key'] == 20.0 else np.nan, axis=1)
setpoint_df['Pitch Setpoint'] = setpoint_df.apply(lambda row: row['Value'] if row['Key'] == 30.0 else np.nan, axis=1)
setpoint_df['Controller Active'] = setpoint_df.apply(lambda row: row['Value'] if row['Key'] in [0.0, 1.0, 2.0, 3.0] else np.nan, axis=1)

# Forward fill the setpoint values to align them with the gyro data
setpoint_df_ffill = setpoint_df.ffill()

# Convert index to datetime
gyro_df.index = pd.to_datetime(gyro_df.index)
setpoint_df_ffill.index = pd.to_datetime(setpoint_df_ffill.index)

# Combine the data based on the index
combined_df = pd.merge_asof(gyro_df.sort_index(), setpoint_df_ffill[['Heave Setpoint', 'Roll Setpoint', 'Pitch Setpoint', 'Controller Active']].sort_index(), left_index=True, right_index=True)

# Convert Depth to m, Roll and Pitch to radians, invert Roll
combined_df['Depth (m)'] = combined_df['Depth (cm)'] / 100
combined_df['Roll (radians)'] = -np.deg2rad(combined_df['Roll (degrees)'])
combined_df['Pitch (radians)'] = np.deg2rad(combined_df['Pitch (degrees)'])

# Debug: Print out the initial processed data
print("Initial Processed Data:\n", combined_df.head(), "\n")

# Find the start times of each test by detecting the transition from 0.0 to 1.0 in Controller Active
test_starts = combined_df.index[(combined_df['Controller Active'] == 1) & (combined_df['Controller Active'].shift(1) == 0)].tolist()

# Ensure we have enough start times for the specified controllers
if len(test_starts) < len(tested_controllers):
    raise ValueError("Not enough test start times detected.")

# Define the duration for each test (e.g., 1 minute)
test_duration = pd.Timedelta(minutes=time)

# Extract data for each controller
controller_data = {}
for i, controller in enumerate(tested_controllers):
    start_time = test_starts[i]
    controller_data[controller] = combined_df[start_time:start_time + test_duration]


# Plot the data for each specified controller
for controller in tested_controllers:
    df = controller_data[controller]
    elapsed_time = (df.index - df.index[0]).total_seconds()
    color = colors.get(controller, 'black')

    plt.figure(figsize=(14, 10))

    # Plot Depth
    # Plot Depth
    plt.subplot(3, 1, 1)
    plt.plot(elapsed_time, df['Depth (m)'], label=f'{controller} Heave (m)', linewidth=2.0, color=color)
    plt.plot(elapsed_time, df['Heave Setpoint'], linestyle='--', label=f'{controller} Heave Setpoint', linewidth=2.0, color='black')
    plt.ylabel('Depth (m)', fontsize=14)
    plt.legend(fontsize=14)
    plt.grid(True)
    plt.gca().invert_yaxis()
    plt.tick_params(axis='both', which='major', labelsize=14)

    # Plot Roll
    plt.subplot(3, 1, 2)
    plt.plot(elapsed_time, df['Roll (radians)'], label=f'{controller} Roll (radians)', linewidth=2.0, color=color)
    plt.plot(elapsed_time, df['Roll Setpoint'], linestyle='--', label=f'{controller} Roll Setpoint', linewidth=2.0, color='black')
    plt.ylabel('Roll (radians)', fontsize=14)
    plt.legend(fontsize=14)
    plt.grid(True)
    plt.tick_params(axis='both', which='major', labelsize=14)

    # Plot Pitch
    plt.subplot(3, 1, 3)
    plt.plot(elapsed_time, df['Pitch (radians)'], label=f'{controller} Pitch (radians)', linewidth=2.0, color=color)
    plt.plot(elapsed_time, df['Pitch Setpoint'], linestyle='--', label=f'{controller} Pitch Setpoint', linewidth=2.0, color='black')
    plt.ylabel('Pitch (radians)', fontsize=14)
    plt.legend(fontsize=14)
    plt.grid(True)
    plt.tick_params(axis='both', which='major', labelsize=14)

    plt.xlabel('Time (s)', fontsize=18)
    plt.suptitle(f'{controller} Controller', fontsize=18)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()