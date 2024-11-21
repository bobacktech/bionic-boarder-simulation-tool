import sys
import os
import argparse
import struct
import matplotlib.pyplot as plt

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
        "input_current",
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
    # Extract data from data_packets
    timestamps = [packet["timestamp"] * 1000 for packet in data_packets]  # Convert to milliseconds
    velocities = [packet["velocity"] for packet in data_packets]
    accelerations_x = [packet["acceleration_x"] for packet in data_packets]
    pitches = [packet["pitch"] for packet in data_packets]
    erpms = [packet["erpm"] for packet in data_packets]
    input_currents = [packet["input_current"] for packet in data_packets]

    # Create figure and subplots
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(10, 15), sharex=True)
    fig.suptitle("Vehicle Telemetry Data")

    # Plot 1: Velocity vs Time
    ax1.plot(timestamps, velocities, "b-")
    ax1.set_ylabel("Velocity (m/s)")
    ax1.grid(True)

    # Plot 2: Acceleration X vs Time
    ax2.plot(timestamps, accelerations_x, "r-")
    ax2.set_ylabel("Acceleration X (m/s²)")
    ax2.grid(True)

    # Plot 3: Pitch vs Time
    ax3.plot(timestamps, pitches, "g-")
    ax3.set_xlabel("Time (ms)")
    ax3.set_ylabel("Pitch (deg)")
    ax3.grid(True)

    # Plot 4: ERPM vs Time
    ax4.plot(timestamps, erpms, "m-")
    ax4.set_ylabel("ERPM")
    ax4.grid(True)

    # Plot 5: Input Current vs Time
    ax5.plot(timestamps, input_currents, "c-")
    ax5.set_xlabel("Time (ms)")
    ax5.set_ylabel("Input Current (A)")
    ax5.grid(True)

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Show the plot
    plt.show()
