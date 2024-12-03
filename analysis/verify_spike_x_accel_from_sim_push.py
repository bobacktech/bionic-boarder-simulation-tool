import sys
import os
import argparse
import struct


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
    accelerations_x = [packet["acceleration_x"] for packet in data_packets]
