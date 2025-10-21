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
from matplotlib.patches import Circle, FancyArrowPatch

import time

# V 1.2  (adds 4-bar thrust visualization)

# pip install pyserial numpy matplotlib

try:
    # change for correct serial port
    ser = serial.Serial('COM14', 921600, timeout=0.1)
except Exception as e:
    print("Error opening serial port:", e)
    exit(1)

# Create a queue to hold serial lines read from the port.
serial_queue = queue.Queue()

# -----------------------
# Serial Reading Thread
# -----------------------
def serial_reader():
    """Read raw bytes, assemble complete '\n'-terminated lines, push only full lines."""
    buf = bytearray()
    MAX_BUF = 1_000_000  # safety cap to avoid runaway memory on garbage input

    while True:
        try:
            # Read whatever is available; if nothing, yield briefly
            chunk = ser.read(4096)
            if not chunk:
                time.sleep(0.005)
                continue

            buf.extend(chunk)

            # Process all complete lines currently in the buffer
            while True:
                nl = buf.find(b'\n')
                if nl == -1:
                    break  # no complete line yet
                line = buf[:nl]                   # up to '\n' (excluded)
                del buf[:nl+1]                    # drop consumed bytes including newline
                s = line.strip().decode('utf-8', errors='ignore')
                if s:                              # skip empty lines
                    serial_queue.put(s)

            # Safety: if buffer gets absurdly large without newlines, reset it
            if len(buf) > MAX_BUF:
                buf.clear()

        except PermissionError as e:
            print(f"Critical error reading from serial: {e}")
            ser.close()
            sys.exit(1)
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(0.01)

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

# NEW: centroid markers (drawn over the 4x4 images)
centroid_left_pt, = ax_lidar_left.plot([], [], marker='o', linestyle='None',
                                       markerfacecolor='white', markeredgecolor='black',
                                       markersize=8, zorder=5)
centroid_right_pt, = ax_lidar_right.plot([], [], marker='o', linestyle='None',
                                         markerfacecolor='white', markeredgecolor='black',
                                         markersize=8, zorder=5)

centroid_left_pt.set_markerfacecolor('black')
centroid_left_pt.set_markeredgecolor('black')
centroid_right_pt.set_markerfacecolor('black')
centroid_right_pt.set_markeredgecolor('black')

# NEW: distance readouts just below each LiDAR plot
txt_lidar_left_dist = ax_lidar_left.text(0.5, -0.12, "", transform=ax_lidar_left.transAxes,
                                         ha='center', va='top', fontsize=10)
txt_lidar_right_dist = ax_lidar_right.text(0.5, -0.12, "", transform=ax_lidar_right.transAxes,
                                           ha='center', va='top', fontsize=10)

# Add a colorbar for the LiDAR plots.
cbar_ax = fig.add_axes([0.92, 0.65, 0.02, 0.25])
cbar = fig.colorbar(img_left, cax=cbar_ax)
cbar.set_label("LiDAR Value", fontsize=12)

# --- Additional Sensor Axes ---
# Row 1: Depth, Forward, and NEW Thrust Bars
ax_fwd   = fig.add_subplot(gs[1, 0])
ax_depth = fig.add_subplot(gs[1, 1])
ax_depth.set_title("Depth")
ax_fwd.set_title("Thrust")
line_depth, = ax_depth.plot([], [], '-', color='blue')
line_fwd,   = ax_fwd.plot([], [], '-', color='red')

# NEW: Thrust bar chart (Rows 1–2, Col 2)
ax_thrust = fig.add_subplot(gs[2:4, 2])   # spans row 1 and 2
ax_thrust.set_title("Motors")
bar_positions = np.arange(4)
bar_labels = ["T1", "T2", "T3", "T4"]
bar_container = ax_thrust.bar(bar_positions, [0, 0, 0, 0])
ax_thrust.set_xticks(bar_positions, bar_labels)
ax_thrust.axhline(0, linewidth=1)
ax_thrust.set_ylim(0, 800)

# --- Fluid temperature (Row 4, Col 3) ---
ax_temperature = fig.add_subplot(gs[3, 3])
ax_temperature.set_title("Temperature")
line_temperature, = ax_temperature.plot([], [], '-', color='blue')

# --- Robot top-down diagram (Rows 1–2, Col 3) ---
ax_robot = fig.add_subplot(gs[1:3, 3])
ax_robot.set_title("Thruster Layout (Top-down)")
ax_robot.set_aspect('equal')
ax_robot.set_xlim(-1.1, 1.1)
ax_robot.set_ylim(-1.1, 1.1)
ax_robot.axis('off')

# Body outline
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
arrow_patches = {}

# Create once; update later
for name in ORDER:
    x, y = T_POS[name]
    dot = Circle((x, y), 0.06, color='tab:blue', alpha=0.85, zorder=2)
    ax_robot.add_patch(dot)
    thruster_nodes[name] = dot

    arr = FancyArrowPatch((x, y), (x, y),
                          arrowstyle='-|>',
                          mutation_scale=18,
                          color='tab:orange', alpha=0.9, lw=1.2, zorder=3)
    ax_robot.add_patch(arr)
    arrow_patches[name] = arr

    if name == "T2" or name == "T4":
        ax_robot.text(x, y+0.12, name, ha='center', va='bottom', fontsize=9, zorder=4)
    else:
        ax_robot.text(x, y-0.12, name, ha='center', va='top', fontsize=9, zorder=4)

# Row 2: Roll, Pitch, Yaw
ax_roll  = fig.add_subplot(gs[2, 0])
ax_pitch = fig.add_subplot(gs[2, 1])
ax_yaw   = fig.add_subplot(gs[1, 2])
ax_roll.set_title("Roll")
ax_pitch.set_title("Pitch")
ax_yaw.set_title("Yaw")
line_roll,  = ax_roll.plot([], [], '-', color='blue')
line_pitch, = ax_pitch.plot([], [], '-', color='blue')
line_yaw,   = ax_yaw.plot([], [], '-', color='blue')

# Autonomous sensor lines on Depth and Yaw axes.
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
# History Buffers
# -----------------------
history_length = 50  # Number of samples to keep in history
history_depth   = []
history_roll    = []
history_pitch   = []
history_yaw     = []
history_voltage = []
history_current = []
history_temperature = []
history_autonomous_fwd   = []
history_autonomous_depth = []
history_autonomous_yaw   = []

# -----------------------
# Tkinter GUI
# -----------------------
root = tk.Tk()
root.title("LiDAR Command Interface")
root.geometry("1600x1200")

root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=0)
root.columnconfigure(0, weight=1)

# --- Canvas Frame ---
canvas_frame = ttk.Frame(root)
canvas_frame.grid(row=0, column=0, sticky="nsew")

canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill=tk.BOTH, expand=True)

# --- Control Frame ---
control_frame = ttk.Frame(root)
control_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

# -----------------------
# FPS Counter (top-left label)
# -----------------------
fps_label = ttk.Label(root, text="FPS: ", font=("Segoe UI", 10))
fps_label.place(x=10, y=0)  # top-left corner of window

_fps_data = {"frames": 0, "t0": time.perf_counter()}

def update_fps():
    _fps_data["frames"] += 1
    now = time.perf_counter()
    dt = now - _fps_data["t0"]
    if dt >= 1.0:
        fps = _fps_data["frames"] / dt
        fps_label.config(text=f"FPS: {fps:.1f}")
        _fps_data["frames"] = 0
        _fps_data["t0"] = now

# ---- Fixed limits (remove autoscale cost) ----
ax_fwd.set_ylim(-1000, 1000)
ax_depth.set_ylim(-5, 30)
ax_roll.set_ylim(-30, 30)
ax_pitch.set_ylim(-30, 30)
ax_yaw.set_ylim(-200, 200)
ax_voltage.set_ylim(0, 5)
ax_current.set_ylim(0, 5)
ax_temperature.set_ylim(0, 40)
for a in (ax_fwd, ax_depth, ax_roll, ax_pitch, ax_yaw, ax_voltage, ax_current, ax_temperature):
    a.set_xlim(0, history_length)


# -----------------------
# Numeric overlays (solid boxes, shifted down/right)
# -----------------------
def make_value_text(ax):
    return ax.text(
        0.025, 0.95, "",                    # moved right and down
        transform=ax.transAxes,
        ha="left", va="top",
        fontsize=10,
        bbox=dict(
            facecolor="white",
            alpha=1.0,                     # fully opaque
            edgecolor="black",
            linewidth=0.5,
        ),
        zorder=10
    )

txt_fwd         = make_value_text(ax_fwd)
txt_depth       = make_value_text(ax_depth)
txt_roll        = make_value_text(ax_roll)
txt_pitch       = make_value_text(ax_pitch)
txt_yaw         = make_value_text(ax_yaw)
txt_voltage     = make_value_text(ax_voltage)
txt_current     = make_value_text(ax_current)
txt_temperature = make_value_text(ax_temperature)

def update_value_texts():
    # fixed-width formats to keep box size steady
    if history_autonomous_fwd:
        txt_fwd.set_text(f"{history_autonomous_fwd[-1]:7.1f}")
    if history_depth:
        txt_depth.set_text(f"{history_depth[-1]:7.2f}")
    if history_roll:
        txt_roll.set_text(f"{history_roll[-1]:7.2f}")
    if history_pitch:
        txt_pitch.set_text(f"{history_pitch[-1]:7.2f}")
    if history_yaw:
        txt_yaw.set_text(f"{history_yaw[-1]:7.1f}")
    if history_voltage:
        txt_voltage.set_text(f"{history_voltage[-1]:7.2f}")
    if history_current:
        txt_current.set_text(f"{history_current[-1]:7.2f}")
    if history_temperature:
        txt_temperature.set_text(f"{history_temperature[-1]:7.2f}")

# -----------------------
# Figure-level blitting (resize-safe)
# -----------------------
USE_BLIT = True

# Collect all animated artists (lines, images, bars, arrows, dots, texts)
ANIMATED = [
    # lines
    line_depth, line_autonomous_depth, line_fwd,
    line_roll, line_pitch, line_yaw, line_autonomous_yaw,
    line_voltage, line_current, line_temperature,
    # images
    img_left, img_right,
    # NEW: lidar centroid markers and distance texts
    centroid_left_pt, centroid_right_pt, txt_lidar_left_dist, txt_lidar_right_dist,
    # thrust bars
    *list(bar_container),
    # top-down robot arrows + dots
    *list(arrow_patches.values()),
    *list(thruster_nodes.values()),
    # numeric overlays
    txt_fwd, txt_depth, txt_roll, txt_pitch, txt_yaw, txt_voltage, txt_current, txt_temperature
]
for a in ANIMATED:
    a.set_animated(True)

_bg = {"img": None}
_last_wh = [canvas_widget.winfo_width(), canvas_widget.winfo_height()]

def _on_draw(_event=None):
    # Capture full-figure background whenever a full draw happens (incl. resize)
    _bg["img"] = fig.canvas.copy_from_bbox(fig.bbox)

fig.canvas.mpl_connect("draw_event", _on_draw)

def fast_draw():
    # If background not ready (first frame / after resize), do a full draw once.
    if not USE_BLIT or _bg["img"] is None:
        fig.canvas.draw()
        fig.canvas.flush_events()
        return

    # Restore saved full-figure background
    fig.canvas.restore_region(_bg["img"])

    # Redraw animated artists
    for a in ANIMATED:
        fig.draw_artist(a)

    # Push to GUI
    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()
    update_fps()  # refresh FPS counter once per draw

def _on_resize(_evt=None):
    # Force a full draw on next tick; draw_event will recache the background
    _bg["img"] = None

# Bind both Tk and Matplotlib resize notifications
canvas_widget.bind("<Configure>", _on_resize)
fig.canvas.mpl_connect("resize_event", _on_resize)

# Seed backgrounds
fig.canvas.draw()   # triggers draw_event -> caches _bg["img"]

# -----------------------
# Command Entry
# -----------------------
cmd_frame = ttk.Frame(control_frame)
cmd_frame.pack(side=tk.TOP, fill=tk.X, pady=2)

cmd_label = ttk.Label(cmd_frame, text="Command:")
cmd_label.pack(side=tk.LEFT, padx=5)

command_var = tk.StringVar()
cmd_entry = ttk.Entry(cmd_frame, textvariable=command_var, width=40)
cmd_entry.pack(side=tk.LEFT, padx=5)
cmd_entry.focus_set()

# Command history
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
        history_index = -1
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

# Bind keys
cmd_entry.bind("<Return>", on_return)
cmd_entry.bind("<KeyPress-Up>", on_up)
cmd_entry.bind("<KeyPress-Down>", on_down)
root.bind_all("<KeyPress-Up>", on_up)
root.bind_all("<KeyPress-Down>", on_down)

# -----------------------
# Buttons (unchanged)
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
        ("Quick Setup", "[H]\n[U,1000,1000,100,100,500]\n[F,0,800,0,0,0]\n[T,1,1,1,1]\n[L,0,10,10,10]\n[W,200,200,200,200]\n[Y,0]"),
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
        ("Stop Recording", "[R,0,1]"),
        ("Record All", "[R,1,10]"),
        ("Record Basic", "[R,2,20]"),
        ("Record Power", "[R,3,50]"),
        ("Record RPY", "[R,4,50]"),
        ("Record Thrust", "[R,5,50]"),
        ("Note Start", "[N,start]"),
        ("Note Stop", "[N,stop]"),
    ],
    "Thrust - Triton": [
        ("Connect", "!TRITON"),
        ("Setup", "[M,280,900]"),
        ("Disable", "[H]"),
        ("40% Forward", "[eC,320,0,320,0]"),
        ("50% Forward", "[eC,400,0,400,0]"),
        ("60% Forward", "[eC,480,0,480,0]"),
        ("70% Forward", "[eC,560,0,560,0]"),
        ("80% Forward", "[eC,640,0,640,0]"),
        ("90% Forward", "[eC,720,0,720,0]"),
        ("100% Forward", "[eC,800,0,800,0]"),
    ],
    "Thrust - Neptune": [
        ("Connect", "!NEPTUNE"),
        ("Setup", "[M,280,900]"),
        ("Disable", "[H]"),
        ("40% Forward", "[eC,320,0,320,0]"),
        ("50% Forward", "[eC,400,0,400,0]"),
        ("60% Forward", "[eC,480,0,480,0]"),
        ("70% Forward", "[eC,560,0,560,0]"),
        ("80% Forward", "[eC,640,0,640,0]"),
        ("90% Forward", "[eC,720,0,720,0]"),
        ("100% Forward", "[eC,800,0,800,0]"),
    ],
    "Misc Experiments": [
        ("Wiggle Setup", "[U,1000,1000,100,100,500]\n[F,0,800,0,0,0]"),
        ("Wiggle Surface", "[A,6,2,1000,1,90,500,0]"),
        ("Wiggle Depth", "[A,6,2,1000,1,90,500,10]"),
        ("Square", "[A,7,90,5,600,0"),
        ("STOP", "[A,0]\n[C,0,0,0]\n[H]"),
    ]
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
# FAST Update Plot Function
# -----------------------
TARGET_DRAW_HZ  = 15.0
DRAW_INTERVAL_S = 1.0 / TARGET_DRAW_HZ
_next_deadline  = time.perf_counter()

def update_plot():
    global _next_deadline
    updated = False

    # Drain multiple lines per tick (reduces overhead)
    MAX_DRAIN = 250
    drained = 0

    while drained < MAX_DRAIN and not serial_queue.empty():
        line = serial_queue.get()
        drained += 1

        # Print any non-data message to the console
        if not line.startswith("data:"):
            print(line)
            continue

        data_str = line[5:].strip()
        tokens = data_str.replace(",", " ").split()
        # UPDATED: now expecting 52 tokens (added 6 centroid/distance values at end)
        if len(tokens) != 52:
            continue

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
            temperature_val     = float(tokens[45])

            # NEW: centroid + distance values (Left then Right)
            centroid_left_row   = float(tokens[46])
            centroid_left_col   = float(tokens[47])
            distance_left       = float(tokens[48])
            centroid_right_row  = float(tokens[49])
            centroid_right_col  = float(tokens[50])
            distance_right      = float(tokens[51])
        except ValueError:
            continue

        left_scan = np.array(lidar_values[:16]).reshape((4, 4))
        right_scan = np.array(lidar_values[16:]).reshape((4, 4))
        depth_val   = sensor_values[0]
        roll_val    = sensor_values[1]
        pitch_val   = sensor_values[2]
        yaw_val     = sensor_values[3]
        voltage_val = sensor_values[4]
        current_val = sensor_values[5]

        # LiDAR images
        img_left.set_data(left_scan)
        img_right.set_data(right_scan)

        # NEW: update centroid markers (imshow origin='lower' => (x=col, y=row))
        centroid_left_pt.set_data([centroid_left_col], [centroid_left_row])
        centroid_right_pt.set_data([centroid_right_col], [centroid_right_row])


        # NEW: update distance text below each LiDAR
        txt_lidar_left_dist.set_text(f"{distance_left:.2f}")
        txt_lidar_right_dist.set_text(f"{distance_right:.2f}")

        centroid_left_pt.set_visible(distance_left < 250)
        centroid_right_pt.set_visible(distance_right < 250)

        # Append new sensor values to history buffers.
        history_depth.append(depth_val)
        history_roll.append(roll_val)
        history_pitch.append(pitch_val)
        history_yaw.append(yaw_val)
        history_voltage.append(voltage_val)
        history_current.append(current_val)
        history_temperature.append(temperature_val)
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
            history_temperature.pop(0)
            history_autonomous_fwd.pop(0)
            history_autonomous_depth.pop(0)
            history_autonomous_yaw.pop(0)

        # Update lines (no relim/autoscale; xlim is fixed)
        x_vals = range(len(history_depth))
        line_depth.set_data(x_vals, history_depth)
        line_autonomous_depth.set_data(x_vals, history_autonomous_depth)

        x_vals = range(len(history_autonomous_fwd))
        line_fwd.set_data(x_vals, history_autonomous_fwd)

        x_vals = range(len(history_roll))
        line_roll.set_data(x_vals, history_roll)

        x_vals = range(len(history_pitch))
        line_pitch.set_data(x_vals, history_pitch)

        x_vals = range(len(history_yaw))
        line_yaw.set_data(x_vals, history_yaw)
        x_vals = range(len(history_autonomous_yaw))
        line_autonomous_yaw.set_data(x_vals, history_autonomous_yaw)

        x_vals = range(len(history_voltage))
        line_voltage.set_data(x_vals, history_voltage)
        x_vals = range(len(history_current))
        line_current.set_data(x_vals, history_current)
        x_vals = range(len(history_temperature))
        line_temperature.set_data(x_vals, history_temperature)

        # Update overlay numbers
        update_value_texts()

        # -------- Update Thrust Bars --------
        thrusts = [thrust_1, thrust_2, thrust_3, thrust_4]
        for i, b in enumerate(bar_container):
            b.set_height(thrusts[i])
        # -----------------------------------

        # ---- Update top-down arrows IN PLACE ----
        thr_vals = {"T1": thrust_1, "T2": thrust_2, "T4": thrust_4, "T3": thrust_3}
        MAX_THRUST = 800.0
        ARROW_BOOST = 2.3
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
            x0, y0 = T_POS[name]
            val = thr_vals[name]
            direction = dir_map[name]

            # available room to the edge in the arrow's direction
            max_len = (yhigh - MARGIN - y0) if direction[1] > 0 else (y0 - (ylow + MARGIN))
            max_len = max(0.0, max_len)

            # boosted proportional length, clamped to available room
            raw_len = (max(0.0, min(val, MAX_THRUST)) / MAX_THRUST) * max_len * ARROW_BOOST
            length = min(raw_len, max_len)

            dx, dy = direction * length
            arrow_patches[name].set_positions((x0, y0), (x0 + dx, y0 + dy))

            # optional: dot grows with thrust
            base_r = 0.06
            thruster_nodes[name].set_radius(base_r + 0.06 * (val / MAX_THRUST))
        # -----------------------------------------

        updated = True

    # throttle actual draw calls
    now = time.perf_counter()
    if updated and now >= _next_deadline:
        # handle geometry changes: force full draw once if size changed
        w = canvas_widget.winfo_width()
        h = canvas_widget.winfo_height()
        if w != _last_wh[0] or h != _last_wh[1]:
            fig.canvas.draw()               # triggers draw_event -> recache background
            _last_wh[0], _last_wh[1] = w, h
        else:
            fast_draw()
        _next_deadline = now + DRAW_INTERVAL_S

    # UI tick cadenceF
    root.after(10, update_plot)

root.after(50, update_plot)
root.mainloop()
ser.close()
