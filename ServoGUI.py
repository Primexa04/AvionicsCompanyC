import time
from pymavlink import mavutil
import tkinter as tk

# Function to establish MAVLink connection
def connect_mavlink():
    return mavutil.mavlink_connection('COM4')

# Initialize MAVLink connection
master = connect_mavlink()
print("Waiting for MAVLink heartbeat...")
master.wait_heartbeat()
print("Heartbeat received. Connected to the autopilot.")

# Function to convert angle to PWM (assuming 800-2200 range)
def angle_to_pwm(angle, min_angle, max_angle):
    min_pwm, max_pwm = 800, 2200
    return int(min_pwm + (angle - min_angle) * (max_pwm - min_pwm) / (max_angle - min_angle))

# Function to send angle to the specified servo channel
def set_servo_angle(servo_n, angle, min_angle, max_angle):
    pwm_value = angle_to_pwm(angle, min_angle, max_angle)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, servo_n, pwm_value, 0, 0, 0, 0, 0
    )
    print(f"Servo {servo_n} set to {angle} degrees (PWM: {pwm_value})")


# Set up Tkinter UI
root = tk.Tk()
root.title("Servo Angle Controller")


# Labels and Sliders for each servo with real-time updates
tk.Label(root, text="Port Flaps Angle (Open to Closed):").grid(row=2, column=0, padx=5, pady=5)
slider_port_flap = tk.Scale(root, from_=0, to=30, resolution=30, orient=tk.HORIZONTAL,
                            command=lambda val: set_servo_angle(2, int(val), 0, 30))
slider_port_flap.grid(row=2, column=1, padx=5, pady=5)

tk.Label(root, text="Starboard Flaps Angle (Open to Closed):").grid(row=3, column=0, padx=5, pady=5)
slider_starboard_flap = tk.Scale(root, from_=0, to=30, resolution=30, orient=tk.HORIZONTAL,
                                 command=lambda val: set_servo_angle(4, int(val), 0, 30))
slider_starboard_flap.grid(row=3, column=1, padx=5, pady=5)

tk.Label(root, text="Port Ailerons Angle (0-120):").grid(row=4, column=0, padx=5, pady=5)
slider_port_aileron = tk.Scale(root, from_=0, to=120, orient=tk.HORIZONTAL,
                               command=lambda val: set_servo_angle(1, int(val), 0, 120))
slider_port_aileron.grid(row=4, column=1, padx=5, pady=5)

tk.Label(root, text="Starboard Ailerons Angle (0-120):").grid(row=5, column=0, padx=5, pady=5)
slider_starboard_aileron = tk.Scale(root, from_=0, to=120, orient=tk.HORIZONTAL,
                                    command=lambda val: set_servo_angle(3, int(val), 0, 120))
slider_starboard_aileron.grid(row=5, column=1, padx=5, pady=5)

tk.Label(root, text="Elevators Angle (-45-+45):").grid(row=6, column=0, padx=5, pady=5)
slider_htp = tk.Scale(root, from_=-45, to=45, orient=tk.HORIZONTAL,
                      command=lambda val: set_servo_angle(5, int(val), -45, 45))
slider_htp.grid(row=6, column=1, padx=5, pady=5)

tk.Label(root, text="Rudder Angle (-40-+40):").grid(row=7, column=0, padx=5, pady=5)
slider_vtp = tk.Scale(root, from_=-40, to=40, orient=tk.HORIZONTAL,
                      command=lambda val: set_servo_angle(6, int(val), -40, 40))
slider_vtp.grid(row=7, column=1, padx=5, pady=5)

# Run the Tkinter main loop
root.mainloop()