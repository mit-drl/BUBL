import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import queue
import sys

# V 1.1

# pip install pyserial numpy matplotlib

try:
    # change for correct serial port
    ser = serial.Serial('COM7', 921600, timeout=0.1)
except Exception as e:
    print("Error opening serial port:", e)
    exit(1)

# Create a queue to hold serial lines read from the port.
serial_queue = queue.Queue()

# -----------------------
# Serial Reading Thread
# -----------------------
def serial_reader():
    """Continuously read from the serial port and put lines into the queue."""
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                serial_queue.put(line)
        except PermissionError as e:
            print(f"Critical error reading from serial: {e}")
            ser.close()
            sys.exit(1)  # Exit on PermissionError
        except Exception as e:
            print(f"Error reading from serial: {e}")

reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

# -----------------------
# Matplotlib Setup for Visualization
# -----------------------
# Create a GridSpec layout with 4 rows and 2 columns.
# The height_ratios ensure that the LiDAR plots (in row 0) take up more space.
fig = plt.figure(figsize=(12, 10))
gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.4, wspace=0.3, height_ratios=[3, 1, 1, 1])

# --- LiDAR Axes (Row 0) ---
ax_lidar_left = fig.add_subplot(gs[0, 0])
ax_lidar_right = fig.add_subplot(gs[0, 2])
vmin, vmax = 0, 255
Z_left = np.zeros((4, 4))
Z_right = np.zeros((4, 4))
img_left = ax_lidar_left.imshow(Z_left, cmap='plasma', origin='lower', vmin=vmin, vmax=vmax)
img_right = ax_lidar_right.imshow(Z_right, cmap='plasma', origin='lower', vmin=vmin, vmax=vmax)
ax_lidar_left.set_title("Left LiDAR Scan", fontsize=12)
ax_lidar_right.set_title("Right LiDAR Scan", fontsize=12)
for ax in [ax_lidar_left, ax_lidar_right]:
    ax.set_xticks([])
    ax.set_yticks([])

# Add a colorbar for the LiDAR plots.
cbar_ax = fig.add_axes([0.92, 0.65, 0.02, 0.25])
cbar = fig.colorbar(img_left, cax=cbar_ax)
cbar.set_label("LiDAR Value", fontsize=12)

# --- Additional Sensor Axes ---
# Row 1: Depth and Foward
ax_depth = fig.add_subplot(gs[1, 0])
ax_fwd = fig.add_subplot(gs[1, 1])
ax_depth.set_title("Depth")
ax_fwd.set_title("Forward")
line_depth, = ax_depth.plot([], [], '-', color='blue')
line_fwd, = ax_fwd.plot([], [], '-', color='red')


# Row 2: Pitch and Yaw
ax_roll = fig.add_subplot(gs[2, 0])
ax_pitch = fig.add_subplot(gs[2, 1])
ax_yaw = fig.add_subplot(gs[2, 2])
ax_roll.set_title("Roll")
ax_pitch.set_title("Pitch")
ax_yaw.set_title("Yaw")
line_roll, = ax_roll.plot([], [], '-', color='blue')
line_pitch, = ax_pitch.plot([], [], '-', color='blue')
line_yaw, = ax_yaw.plot([], [], '-', color='blue')

# NEW: Autonomous sensor lines on Depth and Yaw axes.
line_autonomous_depth, = ax_depth.plot([], [], '-', color='red')
line_autonomous_yaw, = ax_yaw.plot([], [], '-', color='red')

# Row 3: Voltage and Current
ax_voltage = fig.add_subplot(gs[3, 0])
ax_current = fig.add_subplot(gs[3, 1])
ax_voltage.set_title("Voltage")
ax_current.set_title("Current")
line_voltage, = ax_voltage.plot([], [], '-', color='blue')
line_current, = ax_current.plot([], [], '-', color='blue')

# -----------------------
# History Buffers for Additional Data
# -----------------------
history_length = 50  # Number of samples to keep in history
history_depth   = []
history_roll    = []
history_pitch   = []
history_yaw     = []
history_voltage = []
history_current = []
# History buffers for autonomy commands
history_autonomous_fwd = []
history_autonomous_depth = []
history_autonomous_yaw   = []

# -----------------------
# Tkinter GUI Setup with Grid Layout
# -----------------------
root = tk.Tk()
root.title("LiDAR Command Interface")
root.geometry("1200x900")

# Use grid layout on the main window:
# Row 0: Canvas frame (plots)
# Row 1: Control frame (command text box & buttons)
root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=0)
root.columnconfigure(0, weight=1)

# --- Canvas Frame ---
canvas_frame = ttk.Frame(root)
canvas_frame.grid(row=0, column=0, sticky="nsew")

canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# --- Control Frame (Command text box and buttons) ---
control_frame = ttk.Frame(root)
control_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

# Command Entry subframe.
cmd_frame = ttk.Frame(control_frame)
cmd_frame.pack(side=tk.TOP, fill=tk.X, pady=2)

cmd_label = ttk.Label(cmd_frame, text="Command:")
cmd_label.pack(side=tk.LEFT, padx=5)

command_var = tk.StringVar()
cmd_entry = ttk.Entry(cmd_frame, textvariable=command_var, width=40)
cmd_entry.pack(side=tk.LEFT, padx=5)
cmd_entry.focus_set()  # Ensure the entry gets focus

# Command history variables.
command_history = []
history_index = -1

def send_command(cmd):
    """Send the command to the serial port and record it in history."""
    global command_history
    try:
        ser.write((cmd + "\n").encode('utf-8'))
        ser.flush()  # Ensure immediate transmission.
        command_history.append(cmd)
    except Exception as e:
        print("Error sending command:", e)

def on_return(event):
    """Handle pressing Return in the command entry."""
    global history_index
    cmd = cmd_entry.get()
    if cmd.strip():
        send_command(cmd.strip())
        cmd_entry.delete(0, tk.END)
        history_index = -1  # Reset history index after sending.
    return "break"

def on_up(event):
    """Handle the Up arrow to navigate command history."""
    global history_index, command_history
    if command_history:
        if history_index == -1:
            history_index = len(command_history) - 1
        else:
            history_index = max(0, history_index - 1)
        cmd_entry.delete(0, tk.END)
        cmd_entry.insert(0, command_history[history_index])
    return "break"

def on_down(event):
    """Handle the Down arrow to navigate command history."""
    global history_index, command_history
    if command_history:
        if history_index == -1:
            return "break"
        history_index += 1
        if history_index >= len(command_history):
            history_index = -1
            cmd_entry.delete(0, tk.END)
        else:
            cmd_entry.delete(0, tk.END)
            cmd_entry.insert(0, command_history[history_index])
    return "break"

# Bind key events to the command entry.
cmd_entry.bind("<Return>", on_return)
cmd_entry.bind("<KeyPress-Up>", on_up)
cmd_entry.bind("<KeyPress-Down>", on_down)
root.bind_all("<KeyPress-Up>", on_up)
root.bind_all("<KeyPress-Down>", on_down)

# -----------------------
# Common Command Buttons (arranged in two rows)
# -----------------------
button_frame = ttk.Frame(control_frame)
button_frame.pack(side=tk.TOP, fill=tk.X, pady=2)

button_groups = {
    "Power": [
        ("Enable", "[E]"),
        ("Disable", "[H]"),
    ],
    "Motion": [
            ("Surface", "[C,0,0,0]"),
            ("Dive", "[C,0,10,0]"),
            ("Forward", "[C,500,0,0]"),
            ("Reverse", "[C,-500,0,0]"),
            ("Rotate 180", "[C,0,0,180]"),
            ("Rotate 360", "[C,0,0,360]"),
        ],
    "Controller": [
        ("Quick Setup", "[H]\n[U,500,1000,100,100,500]\n[P,20,10,0,2,0,4,16,8]\n[F,0,400,0,0,0]\n[T,1,1,1,1]\n[L,0,0,0,0]\n[Y,0]"),
        ("Set Yaw", "[Y,0]"),
        ("Set PD", "[P,20,10,0,2,0,4,16,8]"),
        ("Set FF", "[F,0,400,0,0,0]"),
        ("Set Limits", "[U,500,1000,100,100,500]"),
        ("Set Filter", "[L,0,10,10,10]"),
    ],
    "Calibrations": [
        ("Gyro Cal", "[G]"),
        ("LiDAR Cal", "[V]"),
    ],
    "Data & Recording": [
        ("Stream", "[B,1]"),
        ("Stop Stream", "[B,0]"),
        ("Capture Image", "[O,1,1]"),
        ("Capture Audio", "[I,3]"),
        ("Record All Data", "[R,1,5]"),
        ("Record Basic Data", "[R,2,10]"),
        ("Record State Data", "[R,4,20]"),
        ("Record Roll", "[R,5,50]"),
        ("Record Pitch", "[R,6,50]"),
        ("Record Yaw", "[R,7,50]"),
        ("Stop Recording", "[R,0,1]"),
        ("Note Start", "[N,start]"),
        ("Note Stop", "[N,stop]"),
    ],
    "Autonomy": [
        ("STOP", "[A,0]"),
        ("CA1", "[A,1,200,50,0,0]"),
        ("CA2", "[A,2,200,0,200,0]"),
        ("CA3", "[A,3]"),
    ],

    "Experiments": [
        ("dC", "[dC,200,5,5]"),
        ("CA4","[A,4,200,3,10,100,0]"),
        ("RED","[A,5,0.3,80,100,10,1000,1,0.1,0,0]")
    ]
}

# A, 5, {alpha}, {centerX}, {yaw_kp}, {yaw_kd}, {desired_pixels}, {fwd_kp}, {fwd_kd}, {fwd_ff}, {depth}

for col, (group_name, items) in enumerate(button_groups.items()):
    lf = ttk.LabelFrame(button_frame, text=group_name)
    lf.grid(row=0, column=col, padx=5, pady=2, sticky="n")
    for i, (label, cmd_code) in enumerate(items):
        btn = ttk.Button(lf, text=label,
                         command=lambda c=cmd_code: send_command(c))
        btn.grid(row=i//2, column=i%2, padx=2, pady=2, sticky="ew")

def button_command(cmd_code):
    send_command(cmd_code)

# -----------------------
# Update Plot Function
# -----------------------
def update_plot():
    updated = False
    while not serial_queue.empty():
        line = serial_queue.get()
        if line.startswith("data:"):
            data_str = line[5:].strip()
            tokens = data_str.replace(",", " ").split()
            if len(tokens) == 42:
                try:
                    # Parse first 32 tokens as integers (LiDAR data)
                    lidar_values = [int(token) for token in tokens[:32]]
                    # Parse next 6 tokens as floats (sensor data)
                    sensor_values = [float(token) for token in tokens[32:38]]
                    # Parse autonomous tokens.
                    autonomous_fwd      = float(tokens[38])
                    autonomous_depth    = float(tokens[39])
                    autonomous_yaw      = float(tokens[40])

                    debug_1 = float(tokens[41])

                except ValueError as e:
                    print(f"Error parsing data from line:\n  {line}\n{e}")
                    continue

                left_scan = np.array(lidar_values[:16]).reshape((4, 4))
                right_scan = np.array(lidar_values[16:]).reshape((4, 4))
                depth_val   = sensor_values[0]
                roll_val    = sensor_values[1]
                pitch_val   = sensor_values[2]
                yaw_val     = sensor_values[3]
                voltage_val = sensor_values[4]
                current_val = sensor_values[5]

                autonomous_yaw = yaw_val + autonomous_yaw

                img_left.set_data(left_scan)
                img_right.set_data(right_scan)

                # Append new sensor values to history buffers.
                history_depth.append(depth_val)
                history_roll.append(roll_val)
                history_pitch.append(pitch_val)
                history_yaw.append(yaw_val)
                history_voltage.append(voltage_val)
                history_current.append(current_val)
                history_autonomous_fwd.append(autonomous_fwd)
                history_autonomous_depth.append(autonomous_depth)
                history_autonomous_yaw.append(autonomous_yaw)


                # Limit history length.
                if len(history_depth) > history_length:
                    history_depth.pop(0)
                    history_roll.pop(0)
                    history_pitch.pop(0)
                    history_yaw.pop(0)
                    history_voltage.pop(0)
                    history_current.pop(0)
                    history_autonomous_fwd.pop(0)
                    history_autonomous_depth.pop(0)
                    history_autonomous_yaw.pop(0)

                # Update Depth plot.
                x_vals = list(range(len(history_depth)))
                line_depth.set_data(x_vals, history_depth)
                line_autonomous_depth.set_data(x_vals, history_autonomous_depth)
                ax_depth.set_xlim(0, history_length)
                ax_depth.relim()
                ax_depth.autoscale_view()

                # Update Forward plot.
                x_vals = list(range(len(history_autonomous_fwd)))
                line_fwd.set_data(x_vals, history_autonomous_fwd)
                ax_fwd.set_xlim(0, history_length)
                ax_fwd.relim()
                ax_fwd.autoscale_view()

                # Update Roll plot.
                x_vals = list(range(len(history_roll)))
                line_roll.set_data(x_vals, history_roll)
                ax_roll.set_xlim(0, history_length)
                ax_roll.relim()
                ax_roll.autoscale_view()

                # Update Pitch plot.
                x_vals = list(range(len(history_pitch)))
                line_pitch.set_data(x_vals, history_pitch)
                ax_pitch.set_xlim(0, history_length)
                ax_pitch.relim()
                ax_pitch.autoscale_view()

                # Update Yaw plot.
                x_vals = list(range(len(history_yaw)))
                line_yaw.set_data(x_vals, history_yaw)
                line_autonomous_yaw.set_data(x_vals, history_autonomous_yaw)
                ax_yaw.set_xlim(0, history_length)
                ax_yaw.relim()
                ax_yaw.autoscale_view()

                # Update Voltage plot.
                x_vals = list(range(len(history_voltage)))
                line_voltage.set_data(x_vals, history_voltage)
                ax_voltage.set_xlim(0, history_length)
                ax_voltage.relim()
                ax_voltage.autoscale_view()

                # Update Current plot.
                x_vals = list(range(len(history_current)))
                line_current.set_data(x_vals, history_current)
                ax_current.set_xlim(0, history_length)
                ax_current.relim()
                ax_current.autoscale_view()

                updated = True

            else:
                print(f"Unexpected number of tokens ({len(tokens)}) in line:\n  {line}")
        else:
            print(line)
    if updated:
        canvas.draw_idle()
        print(debug_1)
    root.after(50, update_plot)

root.after(50, update_plot)
root.mainloop()
ser.close()
