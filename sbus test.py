import time
from pymavlink import mavutil

# Connect to the CubePilot Orange via MAVLink
# Replace 'udp:127.0.0.1:14550' with the appropriate connection string, like serial or UDP.
# For example, serial connection might look like 'COM3' or '/dev/ttyUSB0', 115200 baud
connection_string = 'udp:127.0.0.1:14550'  # Replace as needed
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()  # Wait until the CubePilot sends a heartbeat to confirm connection

print("Connected to CubePilot Orange.")

def read_rc_channels():
    """Read and display RC channels from CubePilot Orange via MAVLink."""
    while True:
        # Wait for an RC_CHANNELS message from the CubePilot
        msg = master.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
        
        if msg:
            # Extract the channel values (ranges typically 1000-2000)
            channels = [
                msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw
            ]

            # Print the RC channel values
            print("RC Channels:", channels)

        # Small delay to avoid excessive looping
        time.sleep(0.1)

try:
    read_rc_channels()
except KeyboardInterrupt:
    print("Disconnected from CubePilot.")
