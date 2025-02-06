import tkinter as tk
from pymavlink import mavutil

# Function to establish MAVLink connection
def connect_mavlink():
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')  # Adjust as needed

# Initialize MAVLink connection
master = connect_mavlink()
print("Waiting for MAVLink heartbeat...")
# master.wait_heartbeat()
print("Heartbeat received. Connected to the CubePilot.")

# Define individual PWM ranges for each servo
servo_pwm_ranges = {
    1: (800, 2200),
    2: (800, 2200),
    3: (800, 2200),
    4: (800, 2200),
    5: (800, 2200),
    6: (800, 2200),
    7: (800, 2200),
    8: (800, 2200),
}

# Function to send PWM command to the specified servo
def set_servo_pwm(servo_n, pwm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, servo_n, pwm, 0, 0, 0, 0, 0
    )
    pwm_vars[servo_n].set(f"{pwm} µs")

# Function to toggle safety switch
def toggle_safety():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0  # 21196 is the magic number for safety toggle
    )
    print("Safety switch toggled.")

# GUI setup
root = tk.Tk()
root.title("Servo PWM Controller")

# Create sliders for each servo
pwm_vars = {}  # Dictionary to store PWM value labels
for i, (servo, (pwm_min, pwm_max)) in enumerate(servo_pwm_ranges.items()):
    frame = tk.Frame(root, padx=10, pady=5)
    frame.pack()

    tk.Label(frame, text=f"Servo {servo}:").pack(side="left")

    pwm_vars[servo] = tk.StringVar(value=f"{(pwm_min + pwm_max) // 2} µs")
    tk.Label(frame, textvariable=pwm_vars[servo], width=10).pack(side="right")

    slider = tk.Scale(frame, from_=pwm_min, to=pwm_max, orient=tk.HORIZONTAL,
                      command=lambda val, s=servo: set_servo_pwm(s, int(val)))
    slider.set((pwm_min + pwm_max) // 2)  # Initialize at midpoint
    slider.pack()

# Safety switch button
safety_button = tk.Button(root, text="Toggle Safety Switch", command=toggle_safety, bg="red", fg="white")
safety_button.pack(pady=10)

# Run the GUI
root.mainloop()
