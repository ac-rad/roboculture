import argparse
import glob
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import FontProperties


def parse_args():
    parser = argparse.ArgumentParser(
        description='Process well growth CSVs from a RoboCulture experiment and generate growth curve plots.'
    )
    parser.add_argument(
        '--data-dir',
        default='data',
        help='Directory containing experiment CSV files (default: data)',
    )
    parser.add_argument(
        '--output-dir',
        default='output',
        help='Directory to save output CSV and figures (default: output)',
    )
    return parser.parse_args()


def load_and_clean_data(data_dir):
    """Discover and load all CSV files in data_dir."""
    pattern = os.path.join(data_dir, '*.csv')
    file_paths = sorted(glob.glob(pattern))
    if not file_paths:
        raise FileNotFoundError(f'No CSV files found in: {data_dir}')
    print(f'Found {len(file_paths)} CSV file(s):')
    for p in file_paths:
        print(f'  {p}')
    data_frames = []
    for path in file_paths:
        data = pd.read_csv(path)
        data.columns = data.columns.str.strip()
        data_frames.append(data)
    return pd.concat(data_frames, ignore_index=True)


# Define the groups of well numbers
groups = {
    '50M':     [0, 1, 2, 3, 4, 5],
    '30M':     [6, 7, 8, 9, 10, 11],
    '10M':     [48, 49, 50, 51, 52, 53],
    'Blanks':  [54, 55, 56, 57, 58, 59],
    'New 50M': [12, 13, 14, 15, 16, 17, 24, 25, 26, 27, 28, 29, 36, 37, 38, 39, 40, 41],
    'New 30M': [18, 19, 20, 21, 22, 23, 30, 31, 32, 33, 34, 35, 42, 43, 44, 45, 46, 47],
    'New 10M': [60, 61, 62, 63, 64, 65, 72, 73, 74, 75, 76, 77, 84, 85, 86, 87, 88, 89],
}

window_size = 5  # Adjust to change smoothing effect

# Time thresholds (hours) to trim each group's data: (start_time, end_time)
thresholds = {
    '50M':     (0,     6.9),
    '30M':     (0,     8.05),
    '10M':     (0,     11.71),
    'Blanks':  (None,  None),
    'New 50M': (7.51,  None),
    'New 30M': (8.66,  None),
    'New 10M': (12.32, None),
}

group_colors = {
    '50M':     '#1374EB',
    'New 50M': '#1374EB',
    '30M':     '#14A509',
    'New 30M': '#14A509',
    '10M':     '#E21919',
    'New 10M': '#E21919',
    'Blanks':  '#7f7f7f',
}


def smooth_series(series, window_size):
    return series.rolling(window=window_size, center=True).mean()


def plot_grouped_well_data(data, groups, window_size, output_dir, thresholds=None, group_colors=None):
    if thresholds is None:
        thresholds = {}
    if group_colors is None:
        group_colors = {}

    fig, ax1 = plt.subplots(figsize=(12, 7))
    ax1.set_ylim(70, 170)

    derivative_relative_time = None
    derivative_group_average = None

    for group_name, well_nums in groups.items():
        group_color = group_colors.get(group_name, next(iter(plt.rcParams['axes.prop_cycle']))['color'])
        smoothed_group_data = []
        relative_times = []

        start_time, end_time = thresholds.get(group_name, (None, None))

        for well_num in well_nums:
            well_data = data[data['well_num'] == well_num]
            if well_data.empty:
                continue

            relative_time = (well_data['time'] - well_data['time'].iloc[0]) / 3600

            mask = np.ones(len(relative_time), dtype=bool)
            if start_time is not None:
                mask &= relative_time >= start_time
            if end_time is not None:
                mask &= relative_time <= end_time
            mask = pd.Series(mask, index=well_data.index)

            relative_time = relative_time[mask.values]
            well_data = well_data[mask.values]

            if len(relative_time) == 0:
                continue

            smoothed_data = smooth_series(well_data['avg_v'], window_size)
            relative_times.append(relative_time.values)
            smoothed_group_data.append(smoothed_data.values)

            ax1.plot(relative_time, well_data['avg_v'], color=group_color, alpha=0.1)

        if smoothed_group_data:
            min_length = min(len(s) for s in smoothed_group_data)
            smoothed_group_data = [s[:min_length] for s in smoothed_group_data]
            relative_times = [rt[:min_length] for rt in relative_times]

            group_average = np.nanmean(smoothed_group_data, axis=0)
            relative_time_to_plot = relative_times[0]

            linestyle = '-.' if 'New' in group_name else '-'
            ax1.plot(relative_time_to_plot, group_average, color=group_color,
                     linewidth=2, label=group_name, linestyle=linestyle)

            if group_name == '30M':
                derivative_relative_time = relative_time_to_plot
                derivative_group_average = group_average

    ax1.set_ylabel('Average Value of Well Images', fontsize=14, fontname='Arial', fontweight='bold')
    ax1.set_xlabel('Time (h)', fontsize=14, fontname='Arial', fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True)

    group_pairs = [('50M', 'New 50M'), ('30M', 'New 30M'), ('10M', 'New 10M')]
    _, ymax = ax1.get_ylim()
    y_position = ymax * 0.97

    for old_group, new_group in group_pairs:
        old_end = thresholds.get(old_group, (None, None))[1]
        new_start = thresholds.get(new_group, (None, None))[0]
        if old_end is not None and new_start is not None:
            xmin = min(old_end, new_start)
            xmax = max(old_end, new_start)
            if xmax > xmin:
                ax1.axvspan(xmin, xmax, facecolor='grey', alpha=0.2)
                ax1.text(
                    (xmin + xmax) / 2,
                    y_position,
                    f'Split {old_group} Group',
                    rotation=90,
                    verticalalignment='top',
                    horizontalalignment='center',
                    color='grey',
                    fontsize=13,
                    fontweight='bold',
                    fontname='Arial',
                )

    fig.tight_layout()
    growth_path = os.path.join(output_dir, 'yeast_growth.pdf')
    fig.savefig(growth_path, format='pdf')
    print(f'Saved: {growth_path}')

    if derivative_group_average is not None and derivative_relative_time is not None:
        derivative = np.gradient(derivative_group_average, derivative_relative_time)
        fig2, ax2 = plt.subplots(figsize=(10, 6))
        ax2.plot(
            derivative_relative_time,
            derivative,
            color=group_colors.get('30M', 'C0'),
            linewidth=2,
            label='Derivative of 30M Group',
        )
        ax2.set_xlabel('Time (h)', fontsize=12)
        ax2.set_ylabel('Derivative of avg_v', fontsize=12)
        ax2.set_title('Derivative of Smoothed Average for 30M Group', fontsize=14)
        ax2.legend(fontsize=12, prop=FontProperties(family='Arial', size=12))
        ax2.grid(True)
        fig2.tight_layout()
        deriv_path = os.path.join(output_dir, 'yeast_growth_30M_derivative.pdf')
        fig2.savefig(deriv_path, format='pdf')
        print(f'Saved: {deriv_path}')

    plt.show()


def main():
    args = parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    combined_data = load_and_clean_data(args.data_dir)

    csv_path = os.path.join(args.output_dir, 'combined_data.csv')
    combined_data.to_csv(csv_path, index=False)
    print(f'Saved: {csv_path}')

    plot_grouped_well_data(
        combined_data,
        groups,
        window_size,
        output_dir=args.output_dir,
        thresholds=thresholds,
        group_colors=group_colors,
    )


if __name__ == '__main__':
    main()
