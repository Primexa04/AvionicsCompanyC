import time
from pymavlink import mavutil
import tkinter as tk


# Function to establish MAVLink connection
def connect_mavlink():
    return mavutil.mavlink_connection('udpin:0.0.0.0:14550')

def arm(): 
    global arming_state
    if arming_state == False : 
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        print("Waiting for the vehicle to arm")
        master.motors_armed_wait()
        print('Armed!')
        arming_state =True
        
    elif arming_state==True:
        print("UAV already armed") 
         

def disarm():
    global arming_state
    if arming_state == True: 
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        master.motors_disarmed_wait()
        master.arming_state = False
        
    elif arming_state == False: 
        print("UAV not armed yet")  
    
    

# Initialize MAVLink connection
master = connect_mavlink()
print("Waiting for MAVLink heartbeat...")
#master.wait_heartbeat()
print("Heartbeat received. Connected to the autopilot.")

#Setting the UAV as disarmed 
arming_state = False

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

# Real-time update function for each slider
def update_servo(servo_n, angle, min_angle, max_angle):
    set_servo_angle(servo_n, angle, min_angle, max_angle)
    
# Set up Tkinter UI
root = tk.Tk()
root.title("Servo Angle Controller")

#Creating the Arm and Disarm buttons: 

tk.Button(root, text="Arm", command=arm, width=30, height=2).grid(row=0, column=0, columnspan=2)

tk.Button(root, text="Disarm", command=disarm, width=30, height=2).grid(row=1, column=0, columnspan=2)

## Labels and Sliders for each servo with real-time updates

# Port Flap
tk.Label(root, text="Port Flaps Angle (Open to Closed):").grid(row=2, column=0, padx=5, pady=5)
slider_port_flap = tk.Scale(root, from_=0, to=30, resolution=30, orient=tk.HORIZONTAL,
                            command=lambda val: update_servo(2, int(val), 0, 30))
slider_port_flap.grid(row=2, column=1, padx=5, pady=5)

# Starboard Flap
tk.Label(root, text="Starboard Flaps Angle (Open to closed):").grid(row=3, column=0, padx=5, pady=5)
slider_starboard_flap = tk.Scale(root, from_=0, to=30, resolution=30, orient=tk.HORIZONTAL,
                                 command=lambda val: update_servo(4, int(val), 0, 30))
slider_starboard_flap.grid(row=3, column=1, padx=5, pady=5)

# Port Ailerons
tk.Label(root, text="Port Ailerons Angle (0-120):").grid(row=4, column=0, padx=5, pady=5)
slider_port_aileron = tk.Scale(root, from_=0, to=120, orient=tk.HORIZONTAL,
                               command=lambda val: update_servo(1, int(val), 0, 60))
slider_port_aileron.grid(row=4, column=1, padx=5, pady=5)

# Starboard Ailerons
tk.Label(root, text="Starboard Ailerons Angle (0-120):").grid(row=5, column=0, padx=5, pady=5)
slider_starboard_aileron = tk.Scale(root, from_=0, to=120, orient=tk.HORIZONTAL,
                                    command=lambda val: update_servo(3, int(val), 0, 60))
slider_starboard_aileron.grid(row=5, column=1, padx=5, pady=5)

# HTP/Elevators
tk.Label(root, text="Elevators Angle (-45-+45):").grid(row=6, column=0, padx=5, pady=5)
slider_htp = tk.Scale(root, from_=-45, to=45, orient=tk.HORIZONTAL,
                      command=lambda val: update_servo(5, int(val), -45, 45))
slider_htp.grid(row=6, column=1, padx=5, pady=5)

# VTP/Rudder
tk.Label(root, text="VTP Angle (-40-+40):").grid(row=7, column=0, padx=5, pady=5)
slider_vtp = tk.Scale(root, from_=-40, to=40, orient=tk.HORIZONTAL,
                      command=lambda val: update_servo(6, int(val), -40, 40))
slider_vtp.grid(row=7, column=1, padx=5, pady=5)

# Run the Tkinter main loop
root.mainloop()