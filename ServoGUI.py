import time
from pymavlink import mavutil
import tkinter as tk

# Function to establish MAVLink connection
def connect_mavlink():
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Adjust connection type as needed

# Initialize MAVLink connection
master = connect_mavlink()
print("Waiting for MAVLink heartbeat...")
#master.wait_heartbeat()
print("Heartbeat received. Connected to the CubePilot.")

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

# Update UI and logic for sliders and displays
root = tk.Tk()
root.title("Servo Angle Controller")

# Frames for better layout
control_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
control_frame.grid(row=0, column=0, sticky="nsew")
display_frame = tk.Frame(root, padx=10, pady=10, relief=tk.RAISED, borderwidth=2)
display_frame.grid(row=0, column=1, sticky="nsew")

# Labels and sliders for control surfaces
tk.Label(control_frame, text="Port Aileron (-30 to +30):").grid(row=0, column=0, padx=5, pady=5)
slider_port_aileron = tk.Scale(control_frame, from_=-30, to=30, orient=tk.HORIZONTAL,
                               command=lambda val: set_servo_angle(1, int(val), -30, 30))
slider_port_aileron.grid(row=0, column=1)

tk.Label(control_frame, text="Starboard Aileron (-30 to +30):").grid(row=1, column=0, padx=5, pady=5)
slider_starboard_aileron = tk.Scale(control_frame, from_=-30, to=30, orient=tk.HORIZONTAL,
                                    command=lambda val: set_servo_angle(2, int(val), -30, 30))
slider_starboard_aileron.grid(row=1, column=1)

tk.Label(control_frame, text="Port Flap (0 to +30):").grid(row=2, column=0, padx=5, pady=5)
slider_port_flap = tk.Scale(control_frame, from_=0, to=30, resolution=10, orient=tk.HORIZONTAL,
                            command=lambda val: set_servo_angle(3, int(val), 0, 30))
slider_port_flap.grid(row=2, column=1)

tk.Label(control_frame, text="Starboard Flap (0 to +30):").grid(row=3, column=0, padx=5, pady=5)
slider_starboard_flap = tk.Scale(control_frame, from_=0, to=30, resolution=10, orient=tk.HORIZONTAL,
                                 command=lambda val: set_servo_angle(4, int(val), 0, 30))
slider_starboard_flap.grid(row=3, column=1)

tk.Label(control_frame, text="Elevator (-45 to +45):").grid(row=4, column=0, padx=5, pady=5)
slider_elevator = tk.Scale(control_frame, from_=-45, to=45, orient=tk.HORIZONTAL,
                           command=lambda val: set_servo_angle(5, int(val), -45, 45))
slider_elevator.grid(row=4, column=1)

tk.Label(control_frame, text="Rudder (-40 to +40):").grid(row=5, column=0, padx=5, pady=5)
slider_rudder = tk.Scale(control_frame, from_=-40, to=40, orient=tk.HORIZONTAL,
                         command=lambda val: set_servo_angle(6, int(val), -40, 40))
slider_rudder.grid(row=5, column=1)

# Angle display section
tk.Label(display_frame, text="Control Surface Angles", font=("Helvetica", 14, "bold")).pack()
tk.Label(display_frame, text="").pack()  # Spacer

angle_vars = {
    "Port Aileron": tk.StringVar(value="0°"),
    "Starboard Aileron": tk.StringVar(value="0°"),
    "Port Flap": tk.StringVar(value="0°"),
    "Starboard Flap": tk.StringVar(value="0°"),
    "Elevator": tk.StringVar(value="0°"),
    "Rudder": tk.StringVar(value="0°"),
}

for surface, var in angle_vars.items():
    frame = tk.Frame(display_frame, padx=5, pady=2)
    frame.pack(fill="x")
    tk.Label(frame, text=surface, width=15, anchor="w").pack(side="left")
    tk.Label(frame, textvariable=var, anchor="e", width=10).pack(side="right")

def update_angles():
    angle_vars["Port Aileron"].set(f"{slider_port_aileron.get()}°")
    angle_vars["Starboard Aileron"].set(f"{slider_starboard_aileron.get()}°")
    angle_vars["Port Flap"].set(f"{slider_port_flap.get()}°")
    angle_vars["Starboard Flap"].set(f"{slider_starboard_flap.get()}°")
    angle_vars["Elevator"].set(f"{slider_elevator.get()}°")
    angle_vars["Rudder"].set(f"{slider_rudder.get()}°")
    root.after(100, update_angles)

# Start updating angles in the display
update_angles()

# Run the Tkinter main loop
root.mainloop()