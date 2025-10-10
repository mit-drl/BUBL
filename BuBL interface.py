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
import os
from matplotlib.patches import Circle

import time

# V 1.2  (adds 4-bar thrust visualization)

# pip install pyserial numpy matplotlib

try:
    # change for correct serial port
    ser = serial.Serial('COM4', 921600, timeout=0.1)
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
fig = plt.figure(figsize=(14, 10))
gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.4, wspace=0.3, height_ratios=[3, 1, 1, 1])

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
# Row 1: Depth, Forward, and NEW Thrust Bars
ax_depth = fig.add_subplot(gs[1, 0])
ax_fwd   = fig.add_subplot(gs[1, 1])
ax_depth.set_title("Depth")
ax_fwd.set_title("Forward")
line_depth, = ax_depth.plot([], [], '-', color='blue')
line_fwd,   = ax_fwd.plot([], [], '-', color='red')

# NEW: Thrust bar chart (Rows 1–2, Col 2)
ax_thrust = fig.add_subplot(gs[1:3, 2])   # spans row 1 and 2
ax_thrust.set_title("Thrusts")
bar_positions = np.arange(4)
bar_labels = ["T1", "T2", "T3", "T4"]
bar_container = ax_thrust.bar(bar_positions, [0, 0, 0, 0])
ax_thrust.set_xticks(bar_positions, bar_labels)
ax_thrust.axhline(0, linewidth=1)
ax_thrust.set_ylabel("Thrust (0–800)")
ax_thrust.set_ylim(0, 800)

# --- Robot top-down diagram (Rows 1–2, Col 3) ---
ax_robot = fig.add_subplot(gs[1:3, 3])
ax_robot.set_title("Thruster Layout (Top-down)")
ax_robot.set_aspect('equal')
ax_robot.set_xlim(-1.1, 1.1)
ax_robot.set_ylim(-1.1, 1.1)
ax_robot.axis('off')

# Body outline (remove if using a detailed background image)
body = Circle((0, 0), 0.9, fill=False, lw=1.5)
ax_robot.add_patch(body)

T_POS = {
    "T1": (-0.6,  0.6),  # top-left
    "T3": ( 0.6,  0.6),  # top-right
    "T4": ( 0.6, -0.6),  # bottom-right
    "T2": (-0.6, -0.6),  # bottom-left
}
ORDER = ["T1", "T3", "T4", "T2"]  # enforce visual order

thruster_nodes = {}
thruster_arrows = {}

for name in ORDER:
    x, y = T_POS[name]
    dot = Circle((x, y), 0.06, color='tab:blue', alpha=0.85, zorder=2)
    ax_robot.add_patch(dot)
    thruster_nodes[name] = dot
    arr = ax_robot.arrow(x, y, 0, 0, width=0.0, head_width=0.12, head_length=0.12,
                         length_includes_head=True, color='tab:orange', alpha=0.9, zorder=3)
    thruster_arrows[name] = arr
    if name == "T2" or name == "T4":
        ax_robot.text(x, y+0.12, name, ha='center', va='bottom', fontsize=9, zorder=4)
    else:
        ax_robot.text(x, y-0.12, name, ha='center', va='top', fontsize=9, zorder=4)

# Row 2: Roll, Pitch, Yaw
ax_roll  = fig.add_subplot(gs[2, 0])
ax_pitch = fig.add_subplot(gs[2, 1])
ax_yaw   = fig.add_subplot(gs[3, 2])
ax_roll.set_title("Roll")
ax_pitch.set_title("Pitch")
ax_yaw.set_title("Yaw")
line_roll,  = ax_roll.plot([], [], '-', color='blue')
line_pitch, = ax_pitch.plot([], [], '-', color='blue')
line_yaw,   = ax_yaw.plot([], [], '-', color='blue')

# NEW: Autonomous sensor lines on Depth and Yaw axes.
line_autonomous_depth, = ax_depth.plot([], [], '-', color='red')
line_autonomous_yaw,   = ax_yaw.plot([], [], '-', color='red')

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
history_autonomous_fwd   = []
history_autonomous_depth = []
history_autonomous_yaw   = []

# Keep recent thrusts for simple smoothing/limit logic (optional)
last_thrusts = [0.0, 0.0, 0.0, 0.0]

# -----------------------
# Tkinter GUI Setup with Grid Layout
# -----------------------
root = tk.Tk()
root.title("LiDAR Command Interface")
root.geometry("1600x900")

root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=0)
root.columnconfigure(0, weight=1)

# --- Canvas Frame ---
canvas_frame = ttk.Frame(root)
canvas_frame.grid(row=0, column=0, sticky="nsew")

canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# --- Control Frame ---
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
# Common Command Buttons
# -----------------------
button_frame = ttk.Frame(control_frame)
button_frame.pack(side=tk.TOP, fill=tk.X, pady=2)

button_groups = {
    "Connection": [
        ("NEPTUNE", "!NEPTUNE"),
        ("POSEIDON", "!POSEIDON"),
        ("TRITON", "!TRITON"),
        ("NAUTILUS", "!NAUTILUS"),
        ("OCEANUS", "!OCEANUS"),
    ],
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
        ("Quick Setup", "[H]\n[U,1000,1000,100,100,500]\n[F,0,400,0,0,0]\n[T,1,1,1,1]\n[L,0,10,10,10]\n[W,200,200,200,200]\n[Y,0]"),
        ("Set Yaw", "[Y,0]"),
        ("Set PID Depth", "[Pn,1,20,1,5]"),
        ("Zero PID Depth", "[Pn,1,0,0,0]"),
        ("Set PID Roll", "[Pn,2,0,0,2]"),
        ("Zero PID Roll", "[Pn,2,0,0,0]"),
        ("Set PID Pitch", "[Pn,3,0,0,2]"),
        ("Zero PID Pitch", "[Pn,3,0,0,0]"),
        ("Set PID Yaw", "[Pn,4,4,1,2]"),
        ("Zero PID Yaw", "[Pn,4,0,0,0]"),
        ("Set Windup", "[W,200,200,200,200]"),
        ("Set FF", "[F,0,400,0,0,0]"),
        ("Set Limits", "[U,1000,1000,100,100,500]"),
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
        ("Record Power Data", "[R,3,20]"),
        ("Record Roll", "[R,5,50]"),
        ("Record Pitch", "[R,6,50]"),
        ("Record Yaw", "[R,7,50]"),
        ("Record Thrust", "[R,8,20]"),
        ("Stop Recording", "[R,0,1]"),
        ("Note Start", "[N,start]"),
        ("Note Stop", "[N,stop]"),
    ],
    # "Autonomy": [
    #     ("STOP", "[A,0]"),
    #     ("CA1", "[A,1,200,50,0,0]"),
    #     ("CA2", "[A,2,200,0,200,0]"),
    #     ("CA3", "[A,3]"),
    #     ("TRACK", "[A,5,0.5,80,-0.4,-0.05,1000,0,0,0,0]"),
    # ],

    "Thrust Stand Experiments": [
        ("35% Forward", "[eC,280,0,280,0]"),
        ("40% Forward", "[eC,320,0,320,0]"),
        ("45% Forward", "[eC,360,0,360,0]"),
        ("50% Forward", "[eC,400,0,400,0]"),
        ("55% Forward", "[eC,440,0,440,0]"),
        ("60% Forward", "[eC,480,0,480,0]"),
        ("65% Forward", "[eC,520,0,520,0]"),
        ("70% Forward", "[eC,560,0,560,0]"),
        ("75% Forward", "[eC,600,0,600,0]"),
        ("80% Forward", "[eC,640,0,640,0]"),
        ("85% Forward", "[eC,680,0,680,0]"),
        ("90% Forward", "[eC,720,0,720,0]"),
        ("95% Forward", "[eC,760,0,760,0]"),
        ("100% Forward", "[eC,800,0,800,0]"),
    ],

# "Thrust Stand Experiments reverse": [
#         ("35% Forward", "[eC,0,280,0,280]"),
#         ("40% Forward", "[eC,0,320,0,320]"),
#         ("45% Forward", "[eC,0,360,0,360]"),
#         ("50% Forward", "[eC,0,400,0,400]"),
#         ("55% Forward", "[eC,0,440,0,440]"),
#         ("60% Forward", "[eC,0,480,0,480]"),
#         ("65% Forward", "[eC,0,520,0,520]"),
#         ("70% Forward", "[eC,0,560,0,560]"),
#         ("75% Forward", "[eC,0,600,0,600]"),
#         ("80% Forward", "[eC,0,640,0,640]"),
#         ("85% Forward", "[eC,0,680,0,680]"),
#         ("90% Forward", "[eC,0,720,0,720]"),
#         ("95% Forward", "[eC,0,760,0,760]"),
#         ("100% Forward", "[eC,0,800,0,800]"),
#     ],

    #[A,6,{mode},{amplitude\degrees},{frequency\Hz},{phase/radians},{forward_bias},{desired_depth/cm}]
    "Misc Experiments": [
        ("Wiggle Setup", "[U,1000,1000,100,100,500]\n[F,0,800,0,0,0]"),
        ("Wiggle Surface", "[A,6,2,1000,1,90,500,0]"),
        ("Wiggle Depth", "[A,6,2,1000,1,90,500,10]"),
        ("Square", "[A,7,90,5,600,0"),
        ("STOP", "[A,0]\n[C,0,0,0]\n[H]"),
    ]

    # "Experiments": [
    #     ("dC", "[dC,200,5,5]"),
    #     ("CA4", "[A,4,200,3,10,100,0]"),
    #     ("RED", "[A,5,0.5,80,-0.4,-0.05,1000,0,0,0,0]"),
    # ],

    # "Experiments - Swarm": [
    #     ("Enable", "!NEPTUNE#[E]#!POSEIDON#[E]#!TRITON#[E]#!NAUTILUS#[E]#!OCEANUS#[E]"),
    #     ("Disable", "!NEPTUNE#[H]#!POSEIDON#[H]#!TRITON#[H]#!NAUTILUS#[H]#!OCEANUS#[H]"),
    #     ("Set Yaw", "!NEPTUNE#[Y,0]#!POSEIDON#[Y,0]#!TRITON#[Y,0]#!NAUTILUS#[Y,0]#!OCEANUS#[Y,0]"),
    #     ("Record", "!NEPTUNE#[R,3,20]#!POSEIDON#[R,3,20]#!TRITON#[R,3,20]#!NAUTILUS#[R,3,20]#!OCEANUS#[R,3,20]"),
    #     ("Stop Record", "!NEPTUNE#[R,0,10]#!POSEIDON#[R,0,10]"),
    #     ("Note Start Test", "!NEPTUNE#[N,Start]#!POSEIDON#[N,Start]#!TRITON#[N,Start]#!NAUTILUS#[N,Start]#!OCEANUS#[N,Start]"),
    #     ("Note Stop Test", "!NEPTUNE#[N,Stop]#!POSEIDON#[N,Stop]#!TRITON#[N,Stop]#!NAUTILUS#[N,Stop]#!OCEANUS#[N,Stop]"),
    #     ("Note Good", "!NEPTUNE#[N,good]#!POSEIDON#[N,good]#!TRITON#[N,good]#!NAUTILUS#[N,good]#!OCEANUS#[N,good]"),
    #     ("Note Bad", "!NEPTUNE#[N,Bad]#!POSEIDON#[N,Bad]#!TRITON#[N,Bad]#!NAUTILUS#[N,Bad]#!OCEANUS#[N,Bad]"),
    #     ("Forward", "!NEPTUNE#[C,1000,0,0]#!POSEIDON#[C,1000,0,0]#!TRITON#[C,1000,0,0]#!NAUTILUS#[C,1000,0,0]#!OCEANUS#[C,1000,0,0]"),
    #     ("Attach", "!NEPTUNE#[C,1000,0,0]#!POSEIDON#[C,1000,0,0]"),
    #     ("Detach", "!NEPTUNE#[C,-1000,0,1000]#!POSEIDON#[C,-1000,0,-1000]"),
    # ]
}

for col, (group_name, items) in enumerate(button_groups.items()):
    lf = ttk.LabelFrame(button_frame, text=group_name)
    lf.grid(row=0, column=col, padx=5, pady=2, sticky="n")

    if group_name == "Connection":
        names = [label for (label, _cmd) in items]
        conn_var = tk.StringVar()
        combo = ttk.Combobox(
            lf, textvariable=conn_var, values=names, state="readonly", width=14
        )
        combo.grid(row=0, column=0, padx=2, pady=2, sticky="ew", columnspan=2)

        if names:
            conn_var.set(names[0])
            combo.current(0)

        def on_select(_evt=None):
            target = combo.get()
            cmd = f"!{target}"
            print("Connecting with command:", cmd)
            send_command(cmd)

        combo.bind("<<ComboboxSelected>>", on_select)
        continue

    for i, (label, cmd_code) in enumerate(items):
        btn = ttk.Button(lf, text=label, command=lambda c=cmd_code: send_command(c))
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
        # print(line)
        if line.startswith("$"):
            status_str = line[1:].strip()
        elif line.startswith("data:"):
            data_str = line[5:].strip()
            tokens = data_str.replace(",", " ").split()
            if len(tokens) == 46:
                try:
                    # Parse first 32 tokens as integers (LiDAR data)
                    lidar_values = [int(token) for token in tokens[:32]]
                    # Parse next 6 tokens as floats (sensor data)
                    sensor_values = [float(token) for token in tokens[32:38]]
                    # Parse autonomous tokens.
                    autonomous_fwd      = float(tokens[38])
                    autonomous_depth    = float(tokens[39])
                    autonomous_yaw      = float(tokens[40])
                    thrust_1            = float(tokens[41])
                    thrust_2            = float(tokens[42])
                    thrust_3            = float(tokens[43])
                    thrust_4            = float(tokens[44])

                    debug_1 = float(tokens[45])

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

                # -------- NEW: Update Thrust Bars --------
                thrusts = [thrust_1, thrust_2, thrust_3, thrust_4]
                for i, b in enumerate(bar_container):
                    b.set_height(thrusts[i])
                ax_thrust.set_ylim(0, 800)
                # ----------------------------------------

                # ---- Update top-down diagram (CCW order T1,T2,T4,T3) ----
                thr_vals = {"T1": thrust_1, "T2": thrust_2, "T4": thrust_4, "T3": thrust_3}
                MAX_THRUST = 800.0
                ARROW_BOOST = 2.3  # extend more per unit thrust
                MARGIN = 0.04

                ylow, yhigh = ax_robot.get_ylim()

                # Directions: up for top pair, down for bottom pair
                dir_map = {
                    "T1": np.array([0.0, 1.0]),
                    "T2": np.array([0.0, -1.0]),
                    "T4": np.array([0.0, -1.0]),
                    "T3": np.array([0.0, 1.0]),
                }

                for name in ORDER:
                    x, y = T_POS[name]
                    val = thr_vals[name]
                    direction = dir_map[name]

                    prev = thruster_arrows.get(name)
                    if prev is not None:
                        try:
                            prev.remove()
                        except Exception:
                            pass

                    # available room to the edge in the arrow's direction
                    max_len = (yhigh - MARGIN - y) if direction[1] > 0 else (y - (ylow + MARGIN))
                    max_len = max(0.0, max_len)

                    # boosted proportional length, clamped to available room
                    raw_len = (max(0.0, min(val, MAX_THRUST)) / MAX_THRUST) * max_len * ARROW_BOOST
                    length = min(raw_len, max_len)

                    dx, dy = direction * length
                    thruster_arrows[name] = ax_robot.arrow(
                        x, y, dx, dy,
                        width=0.0, head_width=0.12, head_length=0.12,
                        length_includes_head=True, color='tab:orange', alpha=0.9, zorder=3
                    )

                    # optional: dot grows with thrust
                    base_r = 0.06
                    thruster_nodes[name].set_radius(base_r + 0.06 * (val / MAX_THRUST))
                # ----------------------------------------------------------

                updated = True

            else:
                print(f"Unexpected number of tokens ({len(tokens)}) in line:\n  {line}")
        else:
            print(line)
    if updated:
        canvas.draw_idle()
        # Optional debug output:
        # print(debug_1)
        # print(*last_thrusts)
    root.after(50, update_plot)

root.after(50, update_plot)
root.mainloop()
ser.close()
