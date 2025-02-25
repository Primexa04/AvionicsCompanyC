import os
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import filedialog
import serial
import threading
import time

import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


PACKET_HEADER = b'PS'
MSG_TYPE_DATA = 0x01
MSG_TYPE_ACK  = 0x02
NUM_SENSORS = 12
PACKET_SIZE = 2 + 1 + (NUM_SENSORS * 2) 

class DataLoggerViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Arduino Data Logger Viewer")
        self.geometry("1200x800")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.serial_port = None
        self.read_thread = None
        self.running = False
        
        self.sensor_data_history = [[] for _ in range(NUM_SENSORS)]
        self.max_history = 100
        self.lines = []
        self.latest_sensor_values = [0] * NUM_SENSORS

        self.cache_duration_var = tk.IntVar(value=10)
        self.component1_var = tk.StringVar(value="A")
        self.component2_var = tk.StringVar(value="PW")
        self.angle_of_attack_var = tk.StringVar(value="0")
        self.flap_angle_var = tk.StringVar(value="0")
        self.wind_speed_var = tk.DoubleVar(value=0.0)
        self.route_directory_var = tk.StringVar(value="")

        self.colours = ['#e6194B', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#42d4f4', '#f032e6', '#bfef45', '#800000', '#469990', '#000075']
        
        self.create_widgets()

    def create_widgets(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        content_frame = ttk.Frame(self)
        content_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        content_frame.columnconfigure(0, weight=1, uniform="col")
        content_frame.columnconfigure(1, weight=1, uniform="col")
        content_frame.rowconfigure(0, weight=1)

        left_frame = ttk.Frame(content_frame)
        left_frame.grid(row=0, column=0, padx=(0, 5), pady=5, sticky="nsew")
        left_frame.columnconfigure(0, weight=1)

        serial_frame = ttk.LabelFrame(left_frame, text="Serial Connection")
        serial_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        serial_frame.columnconfigure(1, weight=1)

        ttk.Label(serial_frame, text="COM Port:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.port_var = tk.StringVar()
        self.update_serial_ports()
        self.port_menu = ttk.OptionMenu(serial_frame, self.port_var, self.available_ports[0], *self.available_ports)
        self.port_menu.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.port_menu["menu"].configure(postcommand=self.update_serial_ports)

        ttk.Label(serial_frame, text="Baud Rate:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.baud_var = tk.IntVar(value=9600)
        self.baud_entry = ttk.Entry(serial_frame, textvariable=self.baud_var, width=8)
        self.baud_entry.grid(row=0, column=3, padx=5, pady=5, sticky="ew")

        self.connect_btn = ttk.Button(serial_frame, text="Connect", command=self.connect_serial)
        self.connect_btn.grid(row=0, column=4, padx=5, pady=5, sticky="ew")
        self.disconnect_btn = ttk.Button(serial_frame, text="Disconnect", command=self.disconnect_serial, state="disabled")
        self.disconnect_btn.grid(row=0, column=5, padx=5, pady=5, sticky="ew")

        sensor_frame = ttk.LabelFrame(left_frame, text="Sensor Readings")
        sensor_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        for col in range(3):
            sensor_frame.columnconfigure(col, weight=1)

        self.sensor_labels = []
        for i in range(NUM_SENSORS):
            lbl = ttk.Label(sensor_frame, text=f"Sensor {i+1:2d}: {'N/A':>4}", anchor="center", padding=2)
            lbl.grid(row=i // 3, column=i % 3, padx=5, pady=2, sticky="ew")
            self.sensor_labels.append(lbl)

        command_frame = ttk.LabelFrame(left_frame, text="Commands")
        command_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        command_frame.columnconfigure(0, weight=1)

        self.zero_btn = ttk.Button(command_frame, text="Zero Sensors", command=self.send_zero_command, state="disabled")
        self.zero_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        ttk.Label(command_frame, text="Sample Rate (Hz):").grid(row=0, column=1, padx=5, pady=5, sticky="e")
        self.sample_rate_var = tk.IntVar(value=10)
        self.sample_rate_entry = ttk.Entry(command_frame, textvariable=self.sample_rate_var, width=8)
        self.sample_rate_entry.grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        
        self.sample_rate_btn = ttk.Button(command_frame, text="Set Rate", command=self.send_sample_rate_command, state="disabled")
        self.sample_rate_btn.grid(row=0, column=3, padx=5, pady=5, sticky="ew")

        history_frame = ttk.LabelFrame(left_frame, text="Sensor Data History")
        history_frame.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")
        self.history_frame = history_frame
        self.init_history_plot()

        bar_frame = ttk.LabelFrame(content_frame, text="Sensor Bar Plot")
        bar_frame.grid(row=0, column=1, padx=(5, 0), pady=10, sticky="nsew")
        self.bar_frame = bar_frame
        self.init_bar_plot()

        save_frame = ttk.LabelFrame(left_frame, text="Save Options")
        save_frame.grid(row=4, column=0, padx=5, pady=5, sticky="nsew")

        route_frame = ttk.Frame(save_frame)
        route_frame.grid(row=0, column=0, columnspan=4, padx=5, pady=5, sticky="ew")
        route_frame.columnconfigure(1, weight=1)
        ttk.Label(route_frame, text="Route Directory:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.route_dir_entry = ttk.Entry(route_frame, textvariable=self.route_directory_var, width=30)
        self.route_dir_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.set_route_btn = ttk.Button(route_frame, text="Set Directory", command=self.set_route_directory)
        self.set_route_btn.grid(row=0, column=2, padx=5, pady=5, sticky="e")

        ttk.Label(save_frame, text="Company:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        comp1_options = ("A", "B", "C")
        self.comp1_menu = ttk.OptionMenu(save_frame, self.component1_var, self.component1_var.get(), *comp1_options)
        self.comp1_menu.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(save_frame, text="Component:").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        comp2_options = ("PW", "SW", "FUS")
        self.comp2_menu = ttk.OptionMenu(save_frame, self.component2_var, self.component2_var.get(), *comp2_options)
        self.comp2_menu.grid(row=1, column=3, padx=5, pady=5, sticky="w")

        ttk.Label(save_frame, text="Angle of Attack:").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.aoa_entry = ttk.Entry(save_frame, textvariable=self.angle_of_attack_var, width=8)
        self.aoa_entry.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        ttk.Label(save_frame, text="Flap Angle:").grid(row=2, column=2, padx=5, pady=5, sticky="e")
        self.flap_entry = ttk.Entry(save_frame, textvariable=self.flap_angle_var, width=8)
        self.flap_entry.grid(row=2, column=3, padx=5, pady=5, sticky="w")

        ttk.Label(save_frame, text="Wind Speed (m/s):").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        self.wind_speed_entry = ttk.Entry(save_frame, textvariable=self.wind_speed_var, width=8)
        self.wind_speed_entry.grid(row=3, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(command_frame, text="Cache Duration (s):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.cache_duration_entry = ttk.Entry(command_frame, textvariable=self.cache_duration_var, width=8)
        self.cache_duration_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(command_frame, text="Set Cache", command=self.set_cache_length).grid(row=1, column=2, padx=5, pady=5)

        self.save_btn = ttk.Button(save_frame, text="Save Data", command=self.save_data)
        self.save_btn.grid(row=5, column=0, columnspan=4, padx=5, pady=10)

    def init_history_plot(self):
        self.figure = Figure(figsize=(4, 2), dpi=100)
        self.figure.patch.set_facecolor("#F0F0F0")
        self.ax = self.figure.add_subplot(111)
        self.ax.set_facecolor("none")
        self.ax.set_xlabel("Sample Number")
        self.ax.set_ylabel("Pressure (Pa)")
        self.ax.set_ylim(-2500, 2500)
        self.ax.set_xlim(0, self.max_history)
        self.ax.grid(True)
        self.lines = []
        for i in range(NUM_SENSORS):
            (line,) = self.ax.plot([], [], label=f"S{i+1}", color=self.colours[i])
            self.lines.append(line)
        self.history_canvas = FigureCanvasTkAgg(self.figure, master=self.history_frame)
        self.history_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def init_bar_plot(self):
        self.figure_bar = Figure(figsize=(4, 4), dpi=100)
        self.figure_bar.patch.set_facecolor("#F0F0F0")
        self.ax_bar = self.figure_bar.add_subplot(111)
        self.ax_bar.set_facecolor("none")
        self.ax_bar.set_xlabel("Sensor")
        self.ax_bar.set_ylabel("Pressure (Pa)")
        self.ax_bar.axhline(0, color="black", linewidth=1)
        self.ax_bar.set_ylim(-2500, 2500)
        self.ax.grid(True)
        self.ax_bar.spines['top'].set_visible(False)
        self.ax_bar.spines['right'].set_visible(False)
        self.bar_canvas = FigureCanvasTkAgg(self.figure_bar, master=self.bar_frame)
        self.bar_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def set_cache_length(self):
        try:
            duration_sec = int(self.cache_duration_var.get())
            sample_rate = int(self.sample_rate_var.get())
            new_length = duration_sec * sample_rate
            if new_length > 0:
                self.max_history = new_length
                for i in range(NUM_SENSORS):
                    if len(self.sensor_data_history[i]) > new_length:
                        self.sensor_data_history[i] = self.sensor_data_history[i][-new_length:]
                self.do_update_plot()
                print(f"Cache updated: duration {duration_sec} seconds equals {new_length} samples.")
            else:
                messagebox.showerror("Error", "Cache duration must be greater than 0.")
        except Exception as e:
            messagebox.showerror("Error", f"Invalid cache duration: {e}")

    def set_route_directory(self):
        directory = filedialog.askdirectory(title="Select Route Directory")
        if directory:
            self.route_directory_var.set(directory)
            print(f"Route directory set to: {directory}")

    def save_data(self):
        base_dir = self.route_directory_var.get()
        if not base_dir:
            messagebox.showerror("Error", "Please set the route directory before saving.")
            return

        comp1 = self.component1_var.get()
        comp2 = self.component2_var.get()
        aoa = self.angle_of_attack_var.get()
        flap = self.flap_angle_var.get()
        wind_speed = self.wind_speed_var.get()

        # Build directory path: base_dir/Component1/Component2/Wind_##/
        file_dir = os.path.join(base_dir, comp1, comp2, f"Wind_{int(wind_speed):02d}")
        os.makedirs(file_dir, exist_ok=True)
        # Build file name: AoA_YY-FLAP_ZZ.csv
        filename = f"AoA_{aoa}-FLAP_{flap}.csv"
        filepath = os.path.join(file_dir, filename)

        try:
            with open(filepath, "w") as f:
                header = ",".join([f"Sensor{i+1}" for i in range(NUM_SENSORS)])
                f.write(header + "\n")
                num_points = len(self.sensor_data_history[0])
                for i in range(num_points):
                    row = []
                    for sensor in range(NUM_SENSORS):
                        try:
                            row.append(str(self.sensor_data_history[sensor][i]))
                        except IndexError:
                            row.append("")
                    f.write(",".join(row) + "\n")
            messagebox.showinfo("Save Data", f"Data saved to:\n{filepath}")
            print(f"Data saved to: {filepath}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save data: {e}")

    def analog_to_pressure(self, analog_value):
        V_out = (analog_value / 1023.0) * 5.0
        pressure = (V_out - 2.5)*1000
        return pressure

    def connect_serial(self):
        port = self.port_var.get()
        baud = self.baud_var.get()
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open serial port: {e}")
            return

        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        self.connect_btn.config(state="disabled")
        self.disconnect_btn.config(state="normal")
        self.zero_btn.config(state="normal")
        self.sample_rate_btn.config(state="normal")

    def disconnect_serial(self):
        self.running = False
        time.sleep(0.2)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.zero_btn.config(state="disabled")
        self.sample_rate_btn.config(state="disabled")

    def read_serial(self):
        buffer = bytearray()
        while self.running:
            if self.serial_port.in_waiting:
                try:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                    while len(buffer) >= PACKET_SIZE:
                        if buffer[0:2] == PACKET_HEADER:
                            msg_type = buffer[2]
                            if msg_type == MSG_TYPE_DATA and len(buffer) >= PACKET_SIZE:
                                packet = buffer[0:PACKET_SIZE]
                                self.process_data_packet(packet)
                                buffer = buffer[PACKET_SIZE:]
                            elif msg_type == MSG_TYPE_ACK:
                                if len(buffer) >= 3:
                                    print("Received ACK from Arduino.")
                                    buffer = buffer[3:]
                                else:
                                    break
                            else:
                                buffer.pop(0)
                        else:
                            buffer.pop(0)
                except Exception as e:
                    print("Serial read error:", e)
            time.sleep(0.01)

    def process_data_packet(self, packet):
        raw_values = []
        index = 3
        for _ in range(NUM_SENSORS):
            value = (packet[index] << 8) | packet[index + 1]
            raw_values.append(value)
            index += 2
        pressure_values = [self.analog_to_pressure(v) for v in raw_values]
        self.latest_sensor_values = pressure_values
        self.after(0, self.update_sensor_labels, pressure_values)
        self.update_sensor_history(pressure_values)

        # Update charts only when new data arrives:
        self.after(0, self.update_history_plot)
        self.after(0, self.update_bar_plot, pressure_values)

    def update_sensor_labels(self, pressure_values):
        for i, pressure in enumerate(pressure_values):
            self.sensor_labels[i].config(text=f"Sensor {i+1:2d}: {pressure:6.2f} Pa")

    def update_sensor_history(self, pressure_values):
        for i, pressure in enumerate(pressure_values):
            self.sensor_data_history[i].append(pressure)
            if len(self.sensor_data_history[i]) > self.max_history:
                self.sensor_data_history[i].pop(0)

    def do_update_plot(self):
        self.update_history_plot()
        self.plot_update_pending = False

    def update_history_plot(self):
        for i, line in enumerate(self.lines):
            data = self.sensor_data_history[i]
            line.set_data(range(len(data)), data)
        
        self.ax.set_xlim(0, self.max_history)
        self.history_canvas.draw_idle()

    def update_bar_plot(self, pressure_values):
        self.ax_bar.cla()
        self.ax_bar.set_facecolor("none")
        sensors = [f"{i+1}" for i in range(NUM_SENSORS)]
        self.ax_bar.bar(sensors, pressure_values, color=self.colours)
        self.ax_bar.axhline(0, color="black", linewidth=1)
        self.ax_bar.set_ylim(-2500, 2500)
        self.ax_bar.set_xlabel("Sensor")
        self.ax_bar.set_ylabel("Pressure (Pa)")
        self.bar_canvas.draw_idle()

    def send_zero_command(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"ZERO\n")
                print("Sent ZERO command to Arduino.")
            except Exception as e:
                print("Error sending ZERO command:", e)

    def send_sample_rate_command(self):
        duration_sec = int(self.cache_duration_var.get())
        sample_rate = int(self.sample_rate_var.get())
        new_length = duration_sec * sample_rate
        if new_length > 0:
            self.max_history = new_length
            for i in range(NUM_SENSORS):
                if len(self.sensor_data_history[i]) > new_length:
                    self.sensor_data_history[i] = self.sensor_data_history[i][-new_length:]
            self.do_update_plot()
        if self.serial_port and self.serial_port.is_open:
            try:
                rate = self.sample_rate_var.get()
                cmd = f"RATE:{rate}\n".encode()
                self.serial_port.write(cmd)
                print(f"Sent sample rate command: {cmd}")
            except Exception as e:
                print("Error sending sample rate command:", e)

    def update_serial_ports(self):
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        self.available_ports = [port.device for port in ports]
        if not self.available_ports:
            self.available_ports = ["No Ports Found"]
        self.port_var.set(self.available_ports[0])
        if hasattr(self, 'port_menu'):
            menu = self.port_menu["menu"]
            menu.delete(0, "end")
            for port in self.available_ports:
                menu.add_command(label=port, command=lambda value=port: self.port_var.set(value))

    def on_close(self):
        self.plot_thread_running = False
        self.disconnect_serial()
        self.destroy()

if __name__ == "__main__":
    app = DataLoggerViewer()
    app.mainloop()