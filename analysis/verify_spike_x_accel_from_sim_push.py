import sys
import os
import argparse
import struct
import numpy as np


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("sim_data_file", type=str, help="This is the path to the recorded sim data file.")
    args = parser.parse_args()
    if not os.path.exists(args.sim_data_file):
        print(f"File {args.sim_data_file} does not exist.")
        sys.exit(1)

    # Define the format string used for packing the data
    format_string = "d f f f f f f f i f"
    # Calculate the size of each data packet
    packet_size = struct.calcsize(format_string)
    # Define the keys for the dictionary
    keys = [
        "timestamp",
        "velocity",
        "acceleration_x",
        "acceleration_y",
        "acceleration_z",
        "pitch",
        "roll",
        "yaw",
        "erpm",
        "motor_current",
    ]
    data_packets = []

    with open(args.sim_data_file, "rb") as f:
        while True:
            chunk = f.read(packet_size)
            if not chunk:
                break  # End of file reached
            unpacked_data = struct.unpack(format_string, chunk)
            data_packet = dict(zip(keys, unpacked_data))
            data_packets.append(data_packet)
    timestamps = [packet["timestamp"] for packet in data_packets]
    accelerations_x = [packet["acceleration_x"] for packet in data_packets]

    # Use the median absolute deviation method to find the number of spike groups in the x acceleration values

    # Calculate the median and MAD of the entire dataset
    median = np.median(accelerations_x)
    mad = np.median(np.abs(accelerations_x - median))

    # Define a threshold for what constitutes a spike
    # This threshold is the median plus a multiple of the MAD
    # Adjust the multiplier as needed
    multiplier = 10
    threshold = median + multiplier * mad

    # Identify data points above the threshold
    spikes = np.array(accelerations_x) > threshold

    spike_groups = []
    current_group = []

    for i, is_spike in enumerate(spikes):
        if is_spike:
            t = timestamps[i]
            current_group.append((t, i))
        else:
            if current_group:
                spike_groups.append(current_group)
                current_group = []
    if current_group:
        spike_groups.append(current_group)

    number_of_spikes = len(spike_groups)
    print(f"Number of spikes in x acceleration values: {number_of_spikes}")
