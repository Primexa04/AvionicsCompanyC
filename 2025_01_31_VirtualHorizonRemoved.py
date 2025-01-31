import time
import csv
from pymavlink import mavutil
import tkinter as tk
from tkinter import PhotoImage
from PIL import Image, ImageTk
import base64
from io import BytesIO


# Function to establish MAVLink connection
def connect_mavlink():
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Adjust connection type as needed

# Initialize MAVLink connection
master = connect_mavlink()
print("Waiting for MAVLink heartbeat...")
#master.wait_heartbeat()
print("Heartbeat received. Connected to the CubePilot.")

# Function to request data streams from the autopilot
def request_data_stream():
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1  # 10 Hz, enable stream
    )
request_data_stream()

# Initialize CSV logging
log_file = "servo_attitude_log.csv"
with open(log_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Port Aileron", "Starboard Aileron", "Port Flap",
                     "Starboard Flap", "Elevator", "Rudder", "Roll", "Pitch", "Yaw"])

print(f"Logging to {log_file}")

# Function to calculate PWM from angle, centered at 1500
def angle_to_pwm(angle, min_angle, max_angle):
    pwm_center = 1500
    pwm_range = 1400  # 2200 - 800 = 1400
    angle_range = max_angle - min_angle
    pwm_per_degree = pwm_range / angle_range
    pwm_offset = angle * pwm_per_degree
    return int(pwm_center + pwm_offset)

# Function to send angle to the specified servo channel
def set_servo_angle(servo_n, angle, min_angle, max_angle):
    pwm_value = angle_to_pwm(angle, min_angle, max_angle)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, servo_n, pwm_value, 0, 0, 0, 0, 0
    )
    print(f"Servo {servo_n} set to {angle} degrees (PWM: {pwm_value})")

# Function to toggle the safety switch
def toggle_safety_switch():
    try:
        safety_state = safety_button["text"].startswith("Safety Enabled")
        if safety_state:
            master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, 0)
            print("Safety Disabled.")
            safety_button.config(text="Safety Disabled (Click to toggle)", bg="red")

        else:
            master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY, 1)
            print("Safety Enabled.")
            safety_button.config(text="Safety Enabled (Click to toggle)", bg="green")
    except Exception as e:
        print(f"Error toggling safety switch: {e}")

# Function to apply the selected flight mode
def apply_mode_change():
    try:
        mode = selected_mode.get()
        mode_mapping = {"MANUAL": 0, "STABILIZE": 2,}
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode]
        )
        print(f"Flight mode changed to {mode}.")
        mode_status_label.config(text=f"{mode}", bg="purple" if mode == "STABILIZE" else "blue")
    except Exception as e:
        print(f"Error applying mode change: {e}")




# Function to update RC channel 6 status
def update_rc_status():
    try:
        msg = master.recv_match(type='RC_CHANNELS', blocking=False)
        if msg:
            channel_6_pwm = msg.chan6_raw
            if channel_6_pwm < 1500:  # Example threshold for "RC Engaged"
                rc_status_label.config(text="ENGAGED", bg="green", fg="white")
            else:
                rc_status_label.config(text="DISENGAGED", bg="red", fg="white")
    except Exception as e:
        print(f"Error reading RC channel status: {e}")
    root.after(100, update_rc_status)

# Function to decode base64 image and return a resized PhotoImage object
def get_image_from_base64(base64_string, width=100, height=100):
    image_data = base64.b64decode(base64_string)
    image = Image.open(BytesIO(image_data))
    image = image.resize((width, height), Image.Resampling.LANCZOS)  # Resize the image to the desired dimensions
    return ImageTk.PhotoImage(image)


# Base64-encoded bitmask image
bitmask_base64 = "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"


# Update UI and logic for sliders and displays
root = tk.Tk()
root.title("Servo Angle Controller")

# Frames for better layout
header_frame = tk.Frame(root, padx=10, pady=10)
header_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)

display_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
display_frame.grid(row=2, column=0, sticky="nsew")  # Left column

control_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
control_frame.grid(row=2, column=1, sticky="nsew")  # Right column

attitude_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
attitude_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

# Adjust row and column weights for better resizing behavior
root.grid_rowconfigure(0, weight=1)  # Header row
root.grid_rowconfigure(1, weight=3)  # Main frames row
root.grid_rowconfigure(2, weight=3)  # Attitude frame row
root.grid_columnconfigure(0, weight=2)  # Display frame column
root.grid_columnconfigure(1, weight=1)  # Control frame column

# Add text label with 'CROPHEED COMMAND' to header_frame (aligned left but centered)
text_label = tk.Label(header_frame, text="CROPHEED COMMAND", font=("Helvetica", 25, "bold", "italic"))
text_label.grid(row=0, column=0, sticky="w", pady=10, padx=10)

# Decode and display the bitmask image in header_frame (aligned right but centered)
bitmask_image = get_image_from_base64(bitmask_base64)
image_label = tk.Label(header_frame, image=bitmask_image)
image_label.grid(row=0, column=2, sticky="e", padx=10, pady=5)

# Adjust the grid layout to ensure the text and image are centered
header_frame.grid_columnconfigure(0, weight=1)
header_frame.grid_columnconfigure(1, weight=3)  # Make the middle column take up most space
header_frame.grid_columnconfigure(2, weight=1)


# Label for Servo Control (row 0, above the sliders)
tk.Label(control_frame, text="Servo Control", font=("Helvetica", 25, "bold", "italic")).grid(row=0, column=0, columnspan=2, pady=5)

# Labels and sliders for control surfaces (same as before)
sliders = {
    "Port Aileron": tk.Scale(control_frame, from_=-30, to=30, orient=tk.HORIZONTAL,
                             command=lambda val: set_servo_angle(1, int(val), -30, 30)),
    "Starboard Aileron": tk.Scale(control_frame, from_=-30, to=30, orient=tk.HORIZONTAL,
                                  command=lambda val: set_servo_angle(8, int(val), -30, 30)),
    "Port Flap": tk.Scale(control_frame, from_=0, to=30, resolution=10, orient=tk.HORIZONTAL,
                          command=lambda val: set_servo_angle(7, int(val), 0, 30)),
    "Starboard Flap": tk.Scale(control_frame, from_=0, to=30, resolution=10, orient=tk.HORIZONTAL,
                               command=lambda val: set_servo_angle(8, int(val), 0, 30)),
    "Elevator": tk.Scale(control_frame, from_=-45, to=45, orient=tk.HORIZONTAL,
                         command=lambda val: set_servo_angle(2, int(val), -45, 45)),
    "Rudder": tk.Scale(control_frame, from_=-40, to=40, orient=tk.HORIZONTAL,
                       command=lambda val: set_servo_angle(4, int(val), -40, 40))
}

# Place the sliders below the label
for i, (label, slider) in enumerate(sliders.items(), start=1):
    tk.Label(control_frame, text=f"{label} ({slider['from']} to {slider['to']}):").grid(row=i, column=0, padx=5, pady=5)
    slider.grid(row=i, column=1)
 

# Label for "Control Surface Angles" (row 7, so it doesn't overlap)
tk.Label(control_frame, text="Control Surface Angles", font=("Helvetica", 14, "bold", "italic")).grid(row=7, column=0, columnspan=2, pady=10)

# Angle display section, starting from row 8 to avoid overlap
angle_vars = {key: tk.StringVar(value="0°") for key in sliders.keys()}
for i, (surface, var) in enumerate(angle_vars.items(), start=8):  # Start at row 8
    tk.Label(control_frame, text=surface, width=15, anchor="w").grid(row=i, column=0, padx=5, pady=2, sticky="w")
    tk.Label(control_frame, textvariable=var, anchor="e", width=10).grid(row=i, column=1, padx=5, pady=2, sticky="e")  

tk.Label(display_frame, text="Primary Flight Parameters", font=("Helvetica", 25, "bold", "italic")).pack(pady=5)
# Safety Switch button (same as before)
safety_button = tk.Button(display_frame, text="Safety Disabled (Click to toggle)", 
                          command=toggle_safety_switch, bg="red", fg="white")
safety_button.pack(pady=10)



# Flight mode dropdown menu (same as before)
flight_modes = ["STABILIZE", "MANUAL"]
selected_mode = tk.StringVar(value=flight_modes[0])
mode_menu = tk.OptionMenu(display_frame, selected_mode, *flight_modes)
mode_menu.pack(pady=10)

# Apply Mode Change button 
apply_mode_button = tk.Button(display_frame, text="Apply Mode Change", command=apply_mode_change, bg="blue", fg="white")
apply_mode_button.pack(pady=10)

# Mode Status Box
tk.Label(display_frame, text="Engaged Flight Mode", font=("Helvetica", 14, "bold")).pack(pady=5)
mode_status_label = tk.Label(display_frame, text="MANUAL", bg="blue", fg="white", font=("Helvetica", 12), width=20)
mode_status_label.pack(pady=10)

# RC Status Box (same as before)
tk.Label(display_frame, text="RC  Status", font=("Helvetica", 14, "bold")).pack(pady=5)
rc_status_label = tk.Label(display_frame, text="DISENGAGED", bg="red", fg="white", font=("Helvetica", 12), width=20)
rc_status_label.pack(pady=10)



# Logging data to CSV (same as before)
def log_data():
    current_time = time.time()
    data = [current_time]
    data += [slider.get() for slider in sliders.values()]
    msg = master.recv_match(type='ATTITUDE', blocking=False)
    if msg:
        data += [msg.roll * (180 / 3.14159), msg.pitch * (180 / 3.14159), msg.yaw * (180 / 3.14159)]
        attitude_vars["Roll"].set(f"{msg.roll * (180 / 3.14159):.1f}°")
        attitude_vars["Pitch"].set(f"{msg.pitch * (180 / 3.14159):.1f}°")
        attitude_vars["Yaw"].set(f"{msg.yaw * (180 / 3.14159):.1f}°")
    else:
        data += [None, None, None]
    with open(log_file, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(data)
    root.after(50, log_data)  # 50 ms = 20 Hz



# Attitude display section (same as before)

attitude_vars = {
    "Roll": tk.StringVar(value="0°"),
    "Pitch": tk.StringVar(value="0°"),
    "Yaw": tk.StringVar(value="0°"),
}
for attitude, var in attitude_vars.items():
    frame = tk.Frame(attitude_frame, padx=5, pady=2)
    frame.pack(fill="x")
    tk.Label(frame, text=attitude, width=15, anchor="w").pack(side="left")
    tk.Label(frame, textvariable=var, anchor="e", width=10).pack(side="right")







# Start logging and updating GUI
log_data()
update_rc_status()
root.mainloop()
