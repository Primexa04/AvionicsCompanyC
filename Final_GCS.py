import time
import csv
from pymavlink import mavutil
import tkinter as tk
from tkinter import StringVar
from tkinter import scrolledtext
from PIL import Image, ImageTk
import base64
from io import BytesIO
import numpy as np
import re 
import collections

######################################
# MAVLink Connection and Configuration
######################################

def connect_mavlink():
    """Establish a MAVLink connection."""
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Adjust connection type as needed

# Initialize MAVLink connection
master = connect_mavlink()
print("Waiting for MAVLink heartbeat...")
master.wait_heartbeat()
print("Heartbeat received. Connected to the CubePilot.")



def request_data_stream():
    """Request all MAVLink data streams at 10 Hz."""
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,20, 1  # 10 Hz, enable stream
    )
request_data_stream()



######################################
# Status Monitoring Functions
######################################

# Global flag to track whether an adjustment is in progress 
adjustment_in_progress = False




def get_rc6_status():
    """Get RC channel 6 status."""
    try:
        msg = master.recv_match(type='RC_CHANNELS', blocking=False)
        if msg:
            return msg.chan6_raw
        return None
    except Exception as e:
        print(f"Error getting RC channel 6 status: {e}")
        return None

def update_status_displays():
    """Update all status displays and indicators."""
    
    # Update RC status
    rc6_value = get_rc6_status()
    if rc6_value is not None:
        if rc6_value > 1500:
            rc_status_label.config(text="ENGAGED", bg="green", fg="white")
        else:
            rc_status_label.config(text="DISENGAGED", bg="red", fg="white")

    # Schedule next update
    root.after(100, update_status_displays)  # 10 Hz refresh rate


# Function to redirect print output to Text widget
class RedirectText:
    def __init__(self, widget):
        self.widget = widget
    
    def write(self, text):
        self.widget.insert(tk.END, text)
        self.widget.yview(tk.END)  # Scroll to the end



######################################
# Servo Calibration and Control Functions
######################################

# D


# List of control surfaces with parameters
control_surfaces = [
    {"label": "Port Aileron",      "servos": [1],    "min": -30, "max": 30},
    {"label": "Starboard Aileron", "servos": [5],    "min": -30, "max": 30},
    {"label": "Port Flap",         "servos": [8],    "min": 0,   "max": 30},
    {"label": "Starboard Flap",    "servos": [6,7],  "min": 0,   "max": 30},
    {"label": "Elevator",          "servos": [2],    "min": -45, "max": 45},
    {"label": "Rudder",            "servos": [4],    "min": -40, "max": 40},
]

import numpy as np

# Updated servo calibrations with multiple angle-PWM pairs
servo_calibrations = {
    1: {"calibration": [(-30, 1700),(-20, 1600), (0, 1500), (30, 1300)]},  # Port aileron
    2: {"calibration": [(-40, 710),(-30, 940), (-20, 1070), (-10, 1190), (0, 1300), (10, 1410), (20, 1520), (30, 1610), (40, 1700)]},  # Elevator should be reversed (positive up)
    4: {"calibration": [(-40, 1490), (-30, 1600), (-20, 1730), (-10, 1840), (0, 1985), (10, 2140), (20, 2280), (30, 2430), (40, 2585)]},  # Rudder (redo positive range)
    5: {"calibration": [(-30, 2000), (0, 1620), (10, 1510), (20, 1400), (30, 1280)]},  # Starboard aileron
    6: {"calibration": [(0, 1800), (30, 2600)]},  # Starboard Flap outoard
    7: {"calibration": [(0, 1175), (30, 375)]},  # Starboard Flap inboard
    8: {"calibration": [(0, 2500), (30, 500)]},  # Port Flap
}

def angle_to_pwm(servo_n, angle):
    """
    Convert a given angle to a PWM value using piecewise linear interpolation.
    """
    cal = servo_calibrations.get(servo_n)
    if cal is None or "calibration" not in cal:
        raise ValueError(f"Calibration data for servo {servo_n} not found.")
    
    # Extract angle-PWM pairs
    angles, pwms = zip(*sorted(cal["calibration"]))  # Ensure sorted order

    # Use numpy interpolation
    pwm_value = np.interp(angle, angles, pwms)
    
    return int(pwm_value)  # Ensure integer PWM output

def set_servo_angle(servo_n, angle):
    """Send a MAVLink command to set a servo to the specified angle."""
    
    pwm_value = angle_to_pwm(servo_n, angle)

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # Confirmation
        servo_n, pwm_value, 0, 0, 0, 0, 0
    )

    print(f"Servo {servo_n} set to {angle} degrees (PWM: {pwm_value})")


def update_servos(val, servo_ids):
    """
    Update a list of servos with the new angle value from a slider.
    """
    angle = int(val)
    for s_id in servo_ids:
        set_servo_angle(s_id, angle)


######################################
# MAVLink Data and Button State Functions
######################################

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

# Function to toggle the arming switch
def toggle_arming_switch():
    try:
        safety_state = arming_button["text"].startswith("Arming Enabled")
        if safety_state:
            # Disarm the vehicle
            master.mav.command_long_send(
                master.target_system,  # Target system ID
                master.target_component,  # Target component ID
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command
                0,  # Confirmation
                0,  # Arm (0 = disarm, 1 = arm)
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            print("Arming Disabled.")
            arming_button.config(text="Arming Disabled (Click to toggle)", bg="red")
            arming_status_label.config(text="DISARMED - NO LOGGING", bg="red", fg="white")

        else:
            # Arm the vehicle
            master.mav.command_long_send(
                master.target_system,  # Target system ID
                master.target_component,  # Target component ID
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command
                0,  # Confirmation
                1,  # Arm (0 = disarm, 1 = arm)
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            print("Arming Enabled.")
            arming_button.config(text="Arming Enabled (Click to toggle)", bg="green")
            arming_status_label.config(text="ARMED AND LOGGING", bg="green", fg="white")
    except Exception as e:
        print(f"Error toggling arming switch: {e}")

def apply_mode_change():
    try:
        mode = selected_mode.get()  # Get the selected mode from the dropdown
        mode_mapping = {"MANUAL": 0, "STABILIZE": 2}
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode]
        )
        print(f"Flight mode changed to {mode}.")
        mode_status_label.config(text=f"{mode}", bg="purple" if mode == "STABILIZE" else "blue")
    except Exception as e:
        print(f"Error applying mode change: {e}")

def initialise_defaults():
    apply_mode_change()
    update_status_displays()
    toggle_arming_switch()
    toggle_safety_switch()

######################################
# GUI Setup
######################################

# Create the main window
root = tk.Tk()
root.title("CROPHEED COMMAND")







# Create Frames
header_frame = tk.Frame(root, bd=2, relief="raised")
header_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

attitude_frame = tk.Frame(root, bd=2, relief="sunken")
attitude_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

display_frame = tk.Frame(root, bd=2, relief="sunken")  # Swapped to column 0
display_frame.grid(row=2, column=0, sticky="ew", padx=5, pady=5)

control_frame = tk.Frame(root, bd=2, relief="sunken")  # Swapped to column 1
control_frame.grid(row=2, column=1, sticky="ew", padx=5, pady=5)

specific_command_frame = tk.Frame(root, bd=2, relief="sunken")
specific_command_frame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)


# Create a message box frame at the bottom
message_frame = tk.Frame(root, bd=2, relief="sunken")
message_frame.grid(row=4, column=0, columnspan=3, sticky="ew", padx=5, pady=5)

# Create a scrolled text widget for the message box
message_box = scrolledtext.ScrolledText(message_frame, wrap=tk.WORD, height=5, width=170)
message_box.grid(row=0, column=2,columnspan=3, padx=5, pady=5)

# Redirect print statements to the message_box
redirect_output = RedirectText(message_box)
import sys
sys.stdout = redirect_output


# Make display frame expand
root.grid_columnconfigure(1, weight=1)
root.grid_rowconfigure(2, weight=1)

######################################
# Header Frame Widgets
######################################

# Add text label (aligned left)
text_label = tk.Label(header_frame, text="CROPHEED COMMAND", font=("Helvetica", 25, "bold", "italic"))
text_label.grid(row=0, column=0, sticky="w", pady=10, padx=10)

# Function to decode base64 image and return a resized PhotoImage
def get_image_from_base64(base64_string, width=100, height=100):
    image_data = base64.b64decode(base64_string)
    image = Image.open(BytesIO(image_data))
    image = image.resize((width, height), Image.Resampling.LANCZOS)
    return ImageTk.PhotoImage(image)

# Base64-encoded bitmask image should be defined here
bitmask_base64 = "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"


bitmask_image = get_image_from_base64(bitmask_base64)
image_label = tk.Label(header_frame, image=bitmask_image)
image_label.grid(row=0, column=2, sticky="e", padx=10, pady=5)

# Adjust grid layout for header_frame
header_frame.grid_columnconfigure(0, weight=1)
header_frame.grid_columnconfigure(1, weight=3)  # Middle column
header_frame.grid_columnconfigure(2, weight=1)

######################################
# Control Frame Widgets
######################################

# Add this in the Control Frame Widgets section:

def send_specific_angle():
    """Send a specific angle to the selected control surface gradually at 20 degrees per second."""
    global adjustment_in_progress

    # Prevent new adjustments while one is already in progress
    if adjustment_in_progress:
        print("Adjustment already in progress. Please wait.")
        return

    try:
        # Get the selected surface and angle
        surface = specific_surface_var.get()
        angle = float(angle_entry.get())

        # Map surface names to their configurations
        surface_mapping = {
            "Port Aileron": {"servos": [1], "min": -30, "max": 30},
            "Starboard Aileron": {"servos": [5], "min": -30, "max": 30},
            "Port Flap": {"servos": [8], "min": 0, "max": 30},
            "Starboard Flap": {"servos": [6, 7], "min": 0, "max": 30},
            "Elevator": {"servos": [2], "min": -45, "max": 45},
            "Rudder": {"servos": [4], "min": -40, "max": 40}
        }

        # Get the configuration for the selected surface
        config = surface_mapping.get(surface)
        if not config:
            print("Error: Invalid surface selected.")
            return

        # Validate angle is within limits
        if not (config["min"] <= angle <= config["max"]):
            print(f"Error: Angle must be between {config['min']} and {config['max']} degrees for {surface}")
            return

        # Get the current slider value for smooth transition
        current_value = sliders[surface].get()

        # Set the speed to 20 degrees per second
        speed = 60  # degrees per second
        interval = 5  # update every 100 milliseconds
        step_size = (speed * interval) / 1000  # degrees per update

        # Indicate that an adjustment is in progress
        adjustment_in_progress = True

        def update_specific_slider():
            """Update the slider incrementally."""
            nonlocal current_value

            # If the remaining distance is smaller than step_size, set to target value
            if abs(angle - current_value) < abs(step_size):
                current_value = angle
            else:
                # Update the slider by step_size
                current_value += step_size if angle > current_value else -step_size

            # Set the new slider value
            sliders[surface].set(current_value)

            # Continue updating until reaching the target angle
            if abs(current_value - angle) > abs(step_size) * 0.1:
                root.after(interval, update_specific_slider)
            else:
                sliders[surface].set(angle)  # Ensure exact final value
                global adjustment_in_progress
                adjustment_in_progress = False  # Mark adjustment as complete

        # Start the gradual update
        update_specific_slider()

    except ValueError:
        print("Error: Please enter a valid number for the angle.")
        adjustment_in_progress = False
    except Exception as e:
        print(f"Error: Failed to send command: {str(e)}")
        adjustment_in_progress = False

    
# Header for Specific Angle Command
tk.Label(specific_command_frame, text="Specific Angle Command", 
         font=("Helvetica", 15, "bold", "italic")).grid(row=0, column=0, 
         columnspan=3, pady=5)

# Create dropdown for surface selection
surface_options = ["Port Aileron", "Starboard Aileron", "Port Flap", 
                  "Starboard Flap", "Elevator", "Rudder"]
specific_surface_var = tk.StringVar(value=surface_options[0])
surface_dropdown = tk.OptionMenu(specific_command_frame, specific_surface_var, 
                               *surface_options)
surface_dropdown.grid(row=1, column=0, padx=5, pady=5)

# Create entry for angle
tk.Label(specific_command_frame, text="Angle (degrees):").grid(row=1, column=1, 
                                                              padx=5, pady=5)
angle_entry = tk.Entry(specific_command_frame, width=10)
angle_entry.grid(row=1, column=2, padx=5, pady=5)

# Create send button
send_button = tk.Button(specific_command_frame, text="Send Command", 
                       command=send_specific_angle, bg="blue", fg="black")
send_button.grid(row=1, column=3, padx=5, pady=5)

# Configure grid weights for specific_command_frame
for i in range(4):
    specific_command_frame.grid_columnconfigure(i, weight=1)


# Flight Configuration Presets Header
tk.Label(control_frame, text="Flight Configuration Presets", font=("Helvetica", 15, "bold", "italic")).grid(
    row=0, column=0, columnspan=3, pady=5
)

# Flaps Presets Header
tk.Label(control_frame, text="Flaps Presets", font=("Helvetica", 15, "bold", "italic")).grid(
    row=2, column=0, columnspan=3, pady=5
)


# Create Flap Preset Buttons
def create_flap_preset_buttons():
    flap_values = [5, 10, 15, 20, 25, 30]
    for i, value in enumerate(flap_values):
        row = 3 + i // 3  # First 3 buttons in one row, next 3 in the next
        col = i % 3        # 3 columns per row
        btn = tk.Button(
            control_frame,
            text=f"{value}°",
            command=lambda v=value: adjust_sliders(v, v),
            bg="orange",
            fg="black",
            font=("Helvetica", 10),
            width=4,
            height=1
        )
        btn.grid(row=row, column=col, padx=2, pady=2, sticky="nsew")
        
# Create a dictionary to store slider widgets (for later reference)
sliders = {}



# Create labels and sliders for control surfaces
start_row = 6  # Starting row index for sliders
for i, cs in enumerate(control_surfaces, start=start_row):
    label_text = f"{cs['label']} ({cs['min']} to {cs['max']}):"
    tk.Label(control_frame, text=label_text).grid(row=i, column=0, padx=10, pady=5)

    # Ensure min_angle, centre_angle, and max_angle exist
    min_angle = cs.get("min_angle", cs["min"])
    centre_angle = cs.get("centre_angle", (cs["min"] + cs["max"]) // 2)  # Default: midpoint
    max_angle = cs.get("max_angle", cs["max"])

    slider = tk.Scale(
        control_frame,
        from_=cs["min"],
        to=cs["max"],
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda val, s_ids=cs["servos"]: update_servos(val, s_ids)
    )
    
    slider.grid(row=i, column=1, sticky="ew")
    sliders[cs["label"]] = slider




tk.Label(control_frame, text="Deflection Command", font=("Helvetica", 14, "bold", "italic")).grid(
    row=5, column=0, columnspan=3, pady=10, sticky="ew"
)

# Global flag to track whether an adjustment is in progress 
adjustment_in_progress = False

# Replace the existing adjust_sliders function with this updated version

def adjust_sliders(target_port_flap, target_starboard_flap):
    """Smoothly adjust the Port and Starboard Flap sliders over 4 seconds, preventing overlapping calls."""
    global adjustment_in_progress

    if adjustment_in_progress:
        print("Adjustment already in progress. Please wait.")
        return  # Exit if another adjustment is running

    adjustment_in_progress = True  # Block further calls

    current_port_flap = sliders["Port Flap"].get()
    current_starboard_flap = sliders["Starboard Flap"].get()
    #try 80 steps and 25ms on root.after
    steps = 80  # 40 steps over 4 seconds (50 ms per step)
    port_step = (target_port_flap - current_port_flap) / steps
    starboard_step = (target_starboard_flap - current_starboard_flap) / steps

    def update_sliders(step=0):
        nonlocal current_port_flap, current_starboard_flap
        if step < steps:
            current_port_flap += port_step
            current_starboard_flap += starboard_step
            sliders["Port Flap"].set(current_port_flap)
            sliders["Starboard Flap"].set(current_starboard_flap)
            root.after(25, lambda: update_sliders(step + 1))
        else:
            global adjustment_in_progress
            adjustment_in_progress = False  # Release the block when done

    update_sliders()

        

def takeoff():
    adjust_sliders(31, 31)  # Example: Set flaps to 30° (adjust as needed)

def cruise():
    adjust_sliders(-1, -1)  # Example: Set flaps to 0° (adjust as needed)

def landing():
    adjust_sliders(31, 31)  # Example: Set flaps to 30° (adjust as needed)


# Buttons for Flight Configuration Presets
tk.Button(control_frame, text="Takeoff", command=takeoff, bg="green", fg="black").grid(
    row=1, column=0, padx=5, pady=5, sticky="ew")
tk.Button(control_frame, text="Cruise", command=cruise, bg="blue", fg="black").grid(
    row=1, column=1, padx=5, pady=5, sticky="ew")
tk.Button(control_frame, text="Landing", command=landing, bg="red", fg="black").grid(
    row=1, column=2, padx=5, pady=5, sticky="ew")

# Create flap preset buttons
create_flap_preset_buttons()

# Adjust grid layout for control_frame preset buttons (equal weight columns)
for i in range(3):
    control_frame.grid_columnconfigure(i, weight=1, uniform="flap_buttons")

######################################
# Display Frame Widgets
######################################

# Primary Flight Parameters Header
tk.Label(display_frame, text="Primary Flight Parameters", font=("Helvetica", 25, "bold", "italic")).pack(pady=5)

# Safety Button
safety_button = tk.Button(display_frame, text="Safety Disabled (Click to toggle)", command=toggle_safety_switch, bg="red", fg="black")
safety_button.pack(pady=10)

# Arming Button
arming_button = tk.Button(display_frame, text="Arming Enabled (Click to toggle)", command=toggle_arming_switch, bg="blue", fg="black")
arming_button.pack(pady=10)

# Flight Mode Dropdown and Apply Button
flight_modes = ["MANUAL", "STABILIZE"]
selected_mode = StringVar(value=flight_modes[0])  # Default to "MANUAL"
mode_menu = tk.OptionMenu(display_frame, selected_mode, *flight_modes)
mode_menu.pack(pady=10)
apply_mode_button = tk.Button(display_frame, text="Apply Mode Change", command=apply_mode_change, bg="blue", fg="black")
apply_mode_button.pack(pady=10)

# Mode Status Box
tk.Label(display_frame, text="Engaged Flight Mode", font=("Helvetica", 14, "bold")).pack(pady=5)
mode_status_label = tk.Label(display_frame, text="MANUAL", bg="blue", fg="white", font=("Helvetica", 12), width=20)
mode_status_label.pack(pady=10)

# Arming/Logging Status Box
tk.Label(display_frame, text="Arming/Logging Status", font=("Helvetica", 14, "bold")).pack(pady=5)
arming_status_label = tk.Label(display_frame, text="ARMED AND LOGGING", bg="green", fg="white", font=("Helvetica", 12), width=20)
arming_status_label.pack(pady=10)

# RC Status Box
tk.Label(display_frame, text="RC Status", font=("Helvetica", 14, "bold")).pack(pady=5)
rc_status_label = tk.Label(display_frame, text="DISENGAGED", bg="red", fg="white", font=("Helvetica", 12), width=20)
rc_status_label.pack(pady=10)


######################################
# Attitude Display and CSV Logging
######################################

# Create a CSV log file and write headers
log_file = "servo_attitude_log.csv"
with open(log_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Port Aileron", "Starboard Aileron", "Port Flap",
                     "Starboard Flap", "Elevator", "Rudder", "Roll", "Pitch", "Yaw"])
print(f"Logging to {log_file}")

# Attitude variables and display labels
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

angle_label = tk.Label(attitude_frame, text="Angle:", font=("Arial", 14))
angle_label = tk.Label(attitude_frame, text="Angle: ", font=("Arial", 14))
angle_label.pack()


angle_values = collections.deque(maxlen=5)  # Store the last 5 angle values
last_valid_angle = 0  # Default value in case of missing data

def read_mavlink():
    global last_valid_angle  # Keep track of the last valid angle

    try:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)  # Listen for STATUSTEXT messages
        if msg:
            text = msg.text.strip()  # Extract and clean the message text

            # Filter only messages containing "Angle: "
            match = re.search(r"Angle:\s([-+]?\d*\.\d+)", text)  # Extract numerical value
            if match:
                angle = float(match.group(1))  # Convert extracted angle to float
                angle_values.append(angle)  # Store latest angle
                last_valid_angle = angle  # Update last valid angle

    except Exception as e:
        print(f"Error reading MAVLink: {e}")  # Debugging print

    # Ensure we always have values for smoothing
    if len(angle_values) > 0:
        avg_angle = sum(angle_values) / len(angle_values)  # Compute average
        rounded_angle = round(avg_angle)  # Round to nearest integer
    else:
        rounded_angle = round(last_valid_angle)  # Use last valid angle

    # Update GUI
    angle_label.config(text=f"Flap Angle Sensor: {rounded_angle}°")

    # Keep UI responsive
    root.update_idletasks()  

    # Reschedule function
    root.after(100, read_mavlink)

# Start the MAVLink reading loop
root.after(100, read_mavlink)


def log_data():
    """Log servo positions and attitude data to CSV at 20 Hz."""
    current_time = time.time()
    data = [current_time]
    # Log current slider values in the order of control_surfaces
    data += [sliders[cs["label"]].get() for cs in control_surfaces]
    
    msg = master.recv_match(type='ATTITUDE', blocking=False)
    if msg:
        roll_deg = msg.roll * (180 / 3.14159)
        pitch_deg = msg.pitch * (180 / 3.14159)
        yaw_deg = msg.yaw * (180 / 3.14159)
        data += [roll_deg, pitch_deg, yaw_deg]
        attitude_vars["Roll"].set(f"{roll_deg:.1f}°")
        attitude_vars["Pitch"].set(f"{pitch_deg:.1f}°")
        attitude_vars["Yaw"].set(f"{yaw_deg:.1f}°")
    else:
        data += [None, None, None]
    
    with open(log_file, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(data)
    root.after(50, log_data)  # 20 Hz

######################################
# Start the GUI Loops
######################################
read_mavlink()
log_data()
initialise_defaults()
# Start MAVLink reading loop
root.mainloop()
