# This code was created with the help of Chat GPT

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

# Configuration: Specify which controllers have been tested
tested_controllers = ['PID', 'LQR', 'SMC']
time= 2
file_path = '2024-05-24 18.07.28 sensor.log'
colors = {
    'PID': 'red',
    'LQR': 'blue',
    'SMC': 'green'
}

# Settpunkt
tested_controllers = ['PID', 'LQR', 'SMC']
time= 1.5
file_path = '2024-05-24 17.56.32 sensor.log'
colors = {
    'PID': 'red',
    'LQR': 'blue',
    'SMC': 'green'
}

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

# Function to create and save the animation for a controller
def create_animation(controller, df):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    lines = {}

    # Setup initial plots
    lines['Depth'], = axes[0].plot([], [], label=f'{controller} Depth (m)', color=colors[controller], linewidth=3)
    lines['Heave Setpoint'], = axes[0].plot([], [], linestyle='--', label=f'{controller} Heave Setpoint', color='black', linewidth=3)
    lines['Roll'], = axes[1].plot([], [], label=f'{controller} Roll (radians)', color=colors[controller], linewidth=3)
    lines['Roll Setpoint'], = axes[1].plot([], [], linestyle='--', label=f'{controller} Roll Setpoint', color='black', linewidth=3)
    lines['Pitch'], = axes[2].plot([], [], label=f'{controller} Pitch (radians)', color=colors[controller], linewidth=3)
    lines['Pitch Setpoint'], = axes[2].plot([], [], linestyle='--', label=f'{controller} Pitch Setpoint', color='black', linewidth=3)

    for ax in axes:
        ax.legend(fontsize=18)
        ax.grid(True)
        ax.tick_params(axis='both', which='major', labelsize=18)

    axes[0].set_ylabel('Depth (m)', fontsize=18)  # Increased font size
    axes[0].invert_yaxis()
    axes[1].set_ylabel('Roll (radians)', fontsize=18)  # Increased font size
    axes[2].set_ylabel('Pitch (radians)', fontsize=18)  # Increased font size
    axes[2].set_xlabel('Time (s)', fontsize=18)  # Increased font size

    def make_frame(t):
        frame = int(t * 10)  # Convert time in seconds to frame index

        if frame < len(df):
            current_time = df.index[:frame]
            if len(current_time) > 0:
                print(f"Time: {current_time[-1]}, Depth: {df['Depth (m)'][frame]}, Heave Setpoint: {df['Heave Setpoint'][frame]}, Roll: {df['Roll (radians)'][frame]}, Roll Setpoint: {df['Roll Setpoint'][frame]}, Pitch: {df['Pitch (radians)'][frame]}, Pitch Setpoint: {df['Pitch Setpoint'][frame]}")  # Debug

                lines['Depth'].set_data(current_time, df['Depth (m)'][:frame])
                lines['Heave Setpoint'].set_data(current_time, df['Heave Setpoint'][:frame])
                lines['Roll'].set_data(current_time, df['Roll (radians)'][:frame])
                lines['Roll Setpoint'].set_data(current_time, df['Roll Setpoint'][:frame])
                lines['Pitch'].set_data(current_time, df['Pitch (radians)'][:frame])
                lines['Pitch Setpoint'].set_data(current_time, df['Pitch Setpoint'][:frame])

                # Ensure the plot limits are updated
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view()

        return mplfig_to_npimage(fig)

    duration = len(df) / 10  # Assuming 10 frames per second
    animation = VideoClip(make_frame, duration=duration)

    # Save the animation as a video file
    animation.write_videofile(f'{controller}_test_animation.mp4', fps=10)

# Create and save animations for each tested controller
for controller in tested_controllers:
    print(f"Creating animation for {controller}...")
    create_animation(controller, controller_data[controller])
