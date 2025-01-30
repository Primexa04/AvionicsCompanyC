import time
import csv
from pymavlink import mavutil
import tkinter as tk
from tkinter import filedialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

# Function to establish MAVLink connection
def connect_mavlink():
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Adjust connection type as needed

# Initialize MAVLink connection
master = connect_mavlink()
print("Waiting for MAVLink heartbeat...")
master.wait_heartbeat()
print("Heartbeat received. Connected to the CubePilot.")

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
    angle_range = 120  # 120 degrees corresponds to the full PWM range
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

# Function to arm/disarm the Cube
def toggle_arm_disarm():
    try:
        # Get the current arm state
        arm_state = master.motors_armed()
        # Toggle arm/disarm
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            not arm_state,  # 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0  # Unused parameters
        )
        state_str = "Armed" if not arm_state else "Disarmed"
        print(f"Cube {state_str}.")
        arm_button.config(text=f"{state_str} (Click to toggle)", bg=("green" if not arm_state else "red"))
    except Exception as e:
        print(f"Error toggling arm/disarm: {e}")

# Function to load and plot .bin file data
def load_and_plot_bin():
    file_path = filedialog.askopenfilename(filetypes=[("Binary files", "*.bin")])
    if file_path:
        data = np.fromfile(file_path, dtype=np.float32)
        plt.figure()
        plt.plot(data)
        plt.title("Data from .bin file")
        plt.xlabel("Sample")
        plt.ylabel("Value")
        plt.show()

# Update UI and logic for sliders and displays
root = tk.Tk()
root.title("Servo Angle Controller")

# Frames for better layout
control_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
control_frame.grid(row=0, column=0, sticky="nsew")
display_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
display_frame.grid(row=0, column=1, sticky="nsew")
attitude_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
attitude_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

# Labels and sliders for control surfaces
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

for i, (label, slider) in enumerate(sliders.items()):
    tk.Label(control_frame, text=f"{label} ({slider['from']} to {slider['to']}):").grid(row=i, column=0, padx=5, pady=5)
    slider.grid(row=i, column=1)

# Angle display section
tk.Label(display_frame, text="Control Surface Angles", font=("Helvetica", 14, "bold")).pack()
angle_vars = {key: tk.StringVar(value="0°") for key in sliders.keys()}
for surface, var in angle_vars.items():
    frame = tk.Frame(display_frame, padx=5, pady=2)
    frame.pack(fill="x")
    tk.Label(frame, text=surface, width=15, anchor="w").pack(side="left")
    tk.Label(frame, textvariable=var, anchor="e", width=10).pack(side="right")

# Add Arm/Disarm button under the live servo positions window
arm_button = tk.Button(display_frame, text="Disarmed (Click to toggle)", command=toggle_arm_disarm, bg="red", fg="white")
arm_button.pack(pady=10)  # Pack the button below live servo positions

# Attitude display section
tk.Label(attitude_frame, text="Attitude Data", font=("Helvetica", 14, "bold")).pack()
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

# Add a button to load and plot .bin files
bin_button = tk.Button(attitude_frame, text="Load and Plot .bin File", command=load_and_plot_bin)
bin_button.pack(pady=10)

# Initialize matplotlib figure for live plotting
fig, ax = plt.subplots()
lines = {key: ax.plot([], [], label=key)[0] for key in sliders.keys()}
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (°)")
ax.legend()
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(row=2, column=0, columnspan=2, sticky="nsew")

# Logging data to CSV and updating plot
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

    # Update plot
    for key, line in lines.items():
        x_data, y_data = line.get_xdata(), line.get_ydata()
        x_data = np.append(x_data, current_time)
        y_data = np.append(y_data, sliders[key].get())
        line.set_xdata(x_data)
        line.set_ydata(y_data)
    ax.relim()
    ax.autoscale_view()
    canvas.draw()

    root.after(50, log_data)  # 50 ms = 20 Hz

# Start logging and updating GUI
log_data()
root.mainloop()