import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import FontProperties

# List of CSV file paths
file_paths = [
    '/Users/kevinangers/Documents/RoboCulture/data/aug_28_exp/well_growth_2024-Aug-28-23:03:00/well_growth.csv',
    '/Users/kevinangers/Documents/RoboCulture/data/aug_28_exp/well_growth_2024-Aug-29-06:34:21/well_growth.csv',
    '/Users/kevinangers/Documents/RoboCulture/data/aug_28_exp/well_growth_2024-Aug-29-07:39:49/well_growth.csv',
    '/Users/kevinangers/Documents/RoboCulture/data/aug_28_exp/well_growth_2024-Aug-29-10:55:32/well_growth.csv',
    '/Users/kevinangers/Documents/RoboCulture/data/aug_28_exp/well_growth_2024-Aug-29-12:00:54/well_growth.csv',
]

# Function to load and clean data
def load_and_clean_data(file_paths):
    data_frames = []
    for path in file_paths:
        data = pd.read_csv(path)
        data.columns = data.columns.str.strip()  # Strip leading/trailing spaces from column names
        data_frames.append(data)
    return pd.concat(data_frames, ignore_index=True)

# Load and combine data from all CSV files
combined_data = load_and_clean_data(file_paths)

# write the combined data to a new CSV file
combined_data.to_csv('combined_data.csv', index=False)

# Define the groups of well numbers
groups = {
    '50M': [0, 1, 2, 3, 4, 5],
    '30M': [6, 7, 8, 9, 10, 11],
    '10M': [48, 49, 50, 51, 52, 53],
    'Blanks': [54, 55, 56, 57, 58, 59],
    'New 50M': [12, 13, 14, 15, 16, 17, 24, 25, 26, 27, 28, 29, 36, 37, 38, 39, 40, 41],
    'New 30M': [18, 19, 20, 21, 22, 23, 30, 31, 32, 33, 34, 35, 42, 43, 44, 45, 46, 47],
    'New 10M': [60, 61, 62, 63, 64, 65, 72, 73, 74, 75, 76, 77, 84, 85, 86, 87, 88, 89],
}

# Define the window size for smoothing
window_size = 5  # Adjust this to change the smoothing effect

# Function to apply a sliding window smoothing
def smooth_series(series, window_size):
    return series.rolling(window=window_size, center=True).mean()

# Define thresholds for groups (start_time, end_time)
thresholds = {
    '50M': (0, 6.9),
    '30M': (0, 8.05),
    '10M': (0, 11.71),
    'Blanks': (None, None),  # No threshold applied
    'New 50M': (7.51, None),
    'New 30M': (8.66, None),
    'New 10M': (12.32, None),
}

# Define custom colors for groups
group_colors = {
    '50M': '#1374EB',        
    'New 50M': '#1374EB',    
    '30M': '#14A509',        
    'New 30M': '#14A509',    
    '10M': '#E21919',       
    'New 10M': '#E21919',    
    'Blanks': '#7f7f7f',    
}

# Function to plot the data with thresholds applied
def plot_grouped_well_data_with_averaging(data, groups, window_size, thresholds=None, group_colors=None):
    if thresholds is None:
        thresholds = {}
    if group_colors is None:
        group_colors = {}

    # Create subplots for the main plot and derivative plot
    # fig, ax1 = plt.subplots(1, 1, figsize=(12, 7))
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 14))

    ax1.set_ylim(70, 170)  # Adjust as needed

    # Variables to store data for derivative computation
    derivative_relative_time = None
    derivative_group_average = None

    # Loop over each group and plot the wells in the specified color
    for group_name, well_nums in groups.items():
        group_color = group_colors.get(group_name, next(plt.gca()._get_lines.prop_cycler)['color'])
        smoothed_group_data = []
        relative_times = []

        # Get threshold times for this group
        start_time, end_time = thresholds.get(group_name, (None, None))

        for i, well_num in enumerate(well_nums):
            well_data = data[data['well_num'] == well_num]
            if well_data.empty:
                continue  # Skip if no data for this well

            # Calculate relative time in hours starting from 0
            relative_time = (well_data['time'] - well_data['time'].iloc[0]) / 3600

            # Apply start_time and end_time thresholds if necessary
            mask = np.ones(len(relative_time), dtype=bool)
            if start_time is not None:
                mask &= relative_time >= start_time
            if end_time is not None:
                mask &= relative_time <= end_time

            # Ensure mask is a pandas Series with the same index as well_data
            mask = pd.Series(mask, index=well_data.index)

            # Apply the mask to relative_time and well_data
            relative_time = relative_time[mask.values]
            well_data = well_data[mask.values]

            if len(relative_time) == 0:
                continue  # Skip if no data after applying thresholds

            # Calculate the smoothed data using the thresholded data
            smoothed_data = smooth_series(well_data['avg_v'], window_size)

            relative_times.append(relative_time.values)
            smoothed_group_data.append(smoothed_data.values)

            # Plot the original data with lower opacity
            ax1.plot(relative_time, well_data['avg_v'], color=group_color, alpha=0.1)

        # Proceed to calculate group average and plot if smoothed_group_data is not empty
        if smoothed_group_data:
            # Trim all smoothed data series and relative times to the length of the shortest one
            min_length = min(len(series) for series in smoothed_group_data)
            smoothed_group_data = [series[:min_length] for series in smoothed_group_data]
            relative_times = [rt[:min_length] for rt in relative_times]

            group_average = np.nanmean(smoothed_group_data, axis=0)

            # For plotting, use the relative_time from the first well (assuming they are similar)
            relative_time_to_plot = relative_times[0]

            # Plot the averaged smoothed data over the raw data
            if "New" in group_name:
                ax1.plot(relative_time_to_plot, group_average, color=group_color, linewidth=2, label=group_name, linestyle='-.')
            else:
                ax1.plot(relative_time_to_plot, group_average, color=group_color, linewidth=2, label=group_name)

            # Store data for derivative if group is '50M'
            if group_name == '30M':
                derivative_relative_time = relative_time_to_plot
                derivative_group_average = group_average

    # Set labels and titles for the main plot
    ax1.set_ylabel('Average Value of Well Images', fontsize=14, fontname='Arial', fontweight='bold')
    ax1.set_xlabel('Time (h)', fontsize=14, fontname='Arial', fontweight='bold')
    # ax1.set_title('Yeast Growth Data with Varying Initial Concentrations', fontsize=14)
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True)

    # Add shaded regions for transitions between old and new groups
    group_pairs = [('50M', 'New 50M'), ('30M', 'New 30M'), ('10M', 'New 10M')]
    ymin, ymax = ax1.get_ylim()
    y_position = ymax * 0.97  # Adjust as needed

    for old_group, new_group in group_pairs:
        old_end = thresholds.get(old_group, (None, None))[1]
        new_start = thresholds.get(new_group, (None, None))[0]
        if old_end is not None and new_start is not None:
            xmin = min(old_end, new_start)
            xmax = max(old_end, new_start)
            # Avoid zero-width regions
            if xmax > xmin:
                ax1.axvspan(xmin, xmax, facecolor='grey', alpha=0.2)
                # Place label in the middle of the shaded region
                x_text = (xmin + xmax) / 2
                label = f'Split {old_group} Group'
                ax1.text(x_text, y_position, label, rotation=90, verticalalignment='top',
                         horizontalalignment='center', color='grey', fontsize=13, fontweight='bold', fontname='Arial')

    # Now compute and plot the derivative for the '30M' group
    if derivative_group_average is not None and derivative_relative_time is not None:
        # Compute the derivative using numpy.gradient
        derivative = np.gradient(derivative_group_average, derivative_relative_time)

        # Plot the derivative in the second subplot
        ax2.plot(derivative_relative_time, derivative, color=group_colors['30M'], linewidth=2,
                 label='Derivative of 30M Group')
        ax2.set_xlabel('Time (h)', fontsize=12)
        ax2.set_ylabel('Derivative of avg_v', fontsize=12)
        ax2.set_title('Derivative of Smoothed Average for 30M Group', fontsize=14)
        font_properties = FontProperties(family='Arial', size=12)
        ax2.legend(fontsize=12, prop=font_properties)
        ax2.grid(True)

    plt.savefig('yeast_growth.pdf', format='pdf')

    plt.tight_layout()
    plt.show()

# Plot the data with custom colors and compute derivative
plot_grouped_well_data_with_averaging(
    combined_data,
    groups,
    window_size,
    thresholds=thresholds,
    group_colors=group_colors
)

