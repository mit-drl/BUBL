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
import glob
import matplotlib.image as mpimg
import time

# -----------------------
# Serial Port
# -----------------------
try:
    # change for correct serial port
    ser = serial.Serial('COM14', 921600, timeout=0.1)
except Exception as e:
    print("Error opening serial port:", e)
    sys.exit(1)

serial_queue = queue.Queue()

# -----------------------
# Serial Reader Thread
# -----------------------
def serial_reader():
    """Read raw bytes, assemble complete '\\n'-terminated lines, push only full lines."""
    buf = bytearray()
    MAX_BUF = 1_000_000
    while True:
        try:
            chunk = ser.read(4096)
            if not chunk:
                time.sleep(0.005)
                continue
            buf.extend(chunk)

            while True:
                nl = buf.find(b'\n')
                if nl == -1:
                    break
                line = buf[:nl]
                del buf[:nl+1]
                s = line.strip().decode('utf-8', errors='ignore')
                if s:
                    serial_queue.put(s)

            if len(buf) > MAX_BUF:
                buf.clear()

        except PermissionError as e:
            print(f"Critical error reading from serial: {e}")
            ser.close()
            sys.exit(1)
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(0.01)

threading.Thread(target=serial_reader, daemon=True).start()

# -----------------------
# Figure & Layout
# -----------------------
fig = plt.figure(figsize=(16, 11))
# 4 rows, 4 cols. Row 0 tall for LiDARs; rows 1–2 for sensors; row 3 for power/audio
gs = gridspec.GridSpec(
    4, 4, figure=fig,
    hspace=0.35, wspace=0.25,
    height_ratios=[3, 1, 1, 1]
)

# Row 0 (top): LiDARs; col 1 intentionally EMPTY; image sits at col 3
ax_lidar_left  = fig.add_subplot(gs[0, 0])
# gs[0, 1] intentionally EMPTY
ax_lidar_right = fig.add_subplot(gs[0, 2])
ax_cam         = fig.add_subplot(gs[0, 3])   # PNG immediately to the right of LiDAR plots

vmin, vmax = 0, 255
Z_left = np.zeros((4, 4))
Z_right = np.zeros((4, 4))
img_left  = ax_lidar_left.imshow(Z_left,  cmap='plasma', origin='lower', vmin=vmin, vmax=vmax)
img_right = ax_lidar_right.imshow(Z_right, cmap='plasma', origin='lower', vmin=vmin, vmax=vmax)
ax_lidar_left.set_title("Left LiDAR Scan", fontsize=12)
ax_lidar_right.set_title("Right LiDAR Scan", fontsize=12)
for ax in (ax_lidar_left, ax_lidar_right):
    ax.set_xticks([]); ax.set_yticks([])

# Centroid dots & distance labels for LiDARs
centroid_left_pt, = ax_lidar_left.plot([], [], marker='o', linestyle='None',
                                       markerfacecolor='black', markeredgecolor='black',
                                       markersize=8, zorder=5)
centroid_right_pt, = ax_lidar_right.plot([], [], marker='o', linestyle='None',
                                         markerfacecolor='black', markeredgecolor='black',
                                         markersize=8, zorder=5)
txt_lidar_left_dist = ax_lidar_left.text(0.5, -0.12, "", transform=ax_lidar_left.transAxes,
                                         ha='center', va='top', fontsize=10)
txt_lidar_right_dist = ax_lidar_right.text(0.5, -0.12, "", transform=ax_lidar_right.transAxes,
                                           ha='center', va='top', fontsize=10)

# Colorbar for LiDARs
cbar_ax = fig.add_axes([0.70, 0.60, 0.01, 0.25])
cbar = fig.colorbar(img_left, cax=cbar_ax)

# Load interface_image (same directory as script), keep aspect equal
ax_cam.set_axis_off()
ax_cam.margins(0, 0)
def _load_interface_image():
    base_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
    candidates = []
    root = os.path.join(base_dir, "interface_image")
    if os.path.isfile(root):
        candidates.append(root)
    else:
        for ext in (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"):
            p = root + ext
            if os.path.isfile(p):
                candidates.append(p)
        if not candidates:
            candidates = glob.glob(os.path.join(base_dir, "interface_image.*"))
    if candidates:
        try:
            img = mpimg.imread(candidates[0])
            ax_cam.imshow(img, aspect='equal')  # preserve aspect
            ax_cam.set_axis_off()
            return True
        except Exception as e:
            ax_cam.text(0.5, 0.5, f"Failed to load image:\n{e}", ha='center', va='center', fontsize=10)
            return False
    else:
        ax_cam.text(0.5, 0.5, "interface_image not found", ha='center', va='center', fontsize=12)
        return False
_load_interface_image()

# Row 1: Thrust (fwd), Depth, Yaw, Motors (top half)
ax_fwd   = fig.add_subplot(gs[1, 0])
ax_depth = fig.add_subplot(gs[1, 1])
ax_yaw   = fig.add_subplot(gs[1, 2])
ax_thrust = fig.add_subplot(gs[1:3, 3])   # double tall immediately below image (rows 1–2)

ax_fwd.set_title("Thrust")
ax_depth.set_title("Depth")
ax_yaw.set_title("Yaw")
ax_thrust.set_title("Motors")

# Row 2: Roll, Pitch, Temperature (below Yaw)
ax_roll        = fig.add_subplot(gs[2, 0])
ax_pitch       = fig.add_subplot(gs[2, 1])
ax_temperature = fig.add_subplot(gs[2, 2])  # directly under yaw

ax_roll.set_title("Roll")
ax_pitch.set_title("Pitch")
ax_temperature.set_title("Temperature")

# Row 3: Voltage, Current, Audio Level; col 3 intentionally EMPTY
ax_voltage = fig.add_subplot(gs[3, 0])
ax_current = fig.add_subplot(gs[3, 1])
ax_audio   = fig.add_subplot(gs[3, 2])  # NEW: right below Temperature
ax_voltage.set_title("Voltage")
ax_current.set_title("Current")
ax_audio.set_title("Filtered Audio Level")

# -----------------------
# Plot Artists
# -----------------------
line_depth,      = ax_depth.plot([], [], '-', color='blue')
line_autonomous_depth, = ax_depth.plot([], [], '-', color='red')
line_fwd,        = ax_fwd.plot([], [], '-', color='red')
line_yaw,        = ax_yaw.plot([], [], '-', color='blue')
line_autonomous_yaw, = ax_yaw.plot([], [], '-', color='red')
line_roll,       = ax_roll.plot([], [], '-', color='blue')
line_pitch,      = ax_pitch.plot([], [], '-', color='blue')
line_voltage,    = ax_voltage.plot([], [], '-', color='blue')
line_current,    = ax_current.plot([], [], '-', color='blue')
line_temperature,= ax_temperature.plot([], [], '-', color='blue')
line_audio,      = ax_audio.plot([], [], '-', color='blue')  # NEW

bar_positions = np.arange(4)
bar_labels    = ["T1", "T2", "T3", "T4"]
bar_container = ax_thrust.bar(bar_positions, [0, 0, 0, 0])
ax_thrust.set_xticks(bar_positions, bar_labels)
ax_thrust.axhline(0, linewidth=1)
ax_thrust.set_ylim(0, 800)

# -----------------------
# History Buffers
# -----------------------
history_length = 50
history_depth   = []
history_roll    = []
history_pitch   = []
history_yaw     = []
history_voltage = []
history_current = []
history_temperature = []
history_audio_level = []          # NEW
history_autonomous_fwd   = []
history_autonomous_depth = []
history_autonomous_yaw   = []

# -----------------------
# Tkinter GUI
# -----------------------
root = tk.Tk()
root.title("BuBL Command Interface")
root.geometry("1720x1250")

root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=0)
root.columnconfigure(0, weight=1)

# Canvas Frame
canvas_frame = ttk.Frame(root)
canvas_frame.grid(row=0, column=0, sticky="nsew")

canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill=tk.BOTH, expand=True)
canvas_widget.pack_configure(pady=0)

# Control Frame
control_frame = ttk.Frame(root)
control_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

# FPS Label
fps_label = ttk.Label(root, text="FPS: ", font=("Segoe UI", 10))
fps_label.place(x=2, y=0)
_fps = {"frames": 0, "t0": time.perf_counter()}

def update_fps():
    _fps["frames"] += 1
    dt = time.perf_counter() - _fps["t0"]
    if dt >= 1.0:
        fps = _fps["frames"] / dt
        fps_label.config(text=f"FPS: {fps:.1f}")
        _fps["frames"] = 0
        _fps["t0"] = time.perf_counter()

# Fixed limits
ax_fwd.set_ylim(-1000, 1000)
ax_depth.set_ylim(-5, 30)
ax_roll.set_ylim(-30, 30)
ax_pitch.set_ylim(-30, 30)
ax_yaw.set_ylim(-200, 200)
ax_voltage.set_ylim(0, 5)
ax_current.set_ylim(0, 5)
ax_temperature.set_ylim(0, 40)
ax_audio.set_ylim(0, 100)  # NEW: 0–100 as requested
for a in (ax_fwd, ax_depth, ax_roll, ax_pitch, ax_yaw, ax_voltage, ax_current, ax_temperature, ax_audio):
    a.set_xlim(0, history_length)

# -----------------------
# Numeric Overlays
# -----------------------
def make_value_text(ax):
    return ax.text(
        0.025, 0.95, "",
        transform=ax.transAxes,
        ha="left", va="top",
        fontsize=10,
        bbox=dict(facecolor="white", alpha=1.0, edgecolor="black", linewidth=0.5),
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
txt_audio       = make_value_text(ax_audio)   # NEW

def update_value_texts():
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
    if history_audio_level:
        txt_audio.set_text(f"{history_audio_level[-1]:7.1f}")  # NEW

# -----------------------
# Command Entry & Buttons
# -----------------------
cmd_frame = ttk.Frame(control_frame)
cmd_frame.pack(side=tk.TOP, fill=tk.X, pady=2)

ttk.Label(cmd_frame, text="Command:").pack(side=tk.LEFT, padx=5)
command_var = tk.StringVar()
cmd_entry = ttk.Entry(cmd_frame, textvariable=command_var, width=40)
cmd_entry.pack(side=tk.LEFT, padx=5)
cmd_entry.focus_set()

command_history = []
history_index = -1

def send_command(cmd):
    global command_history
    try:
        ser.write((cmd + "\n").encode('utf-8'))
        ser.flush()
        command_history.append(cmd)
    except Exception as e:
        print("Error sending command:", e)

def on_return(event):
    global history_index
    cmd = cmd_entry.get()
    if cmd.strip():
        send_command(cmd.strip())
        cmd_entry.delete(0, tk.END)
        history_index = -1
    return "break"

def on_up(event):
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

cmd_entry.bind("<Return>", on_return)
cmd_entry.bind("<KeyPress-Up>", on_up)
cmd_entry.bind("<KeyPress-Down>", on_down)
root.bind_all("<KeyPress-Up>", on_up)
root.bind_all("<KeyPress-Down>", on_down)

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
        combo = ttk.Combobox(lf, textvariable=conn_var, values=names, state="readonly", width=14)
        combo.grid(row=0, column=0, padx=2, pady=2, sticky="ew", columnspan=2)
        if names:
            conn_var.set(names[0]); combo.current(0)
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
# Blitting Setup
# -----------------------
USE_BLIT = True
_bg = {"img": None}

def _on_draw(_event=None):
    _bg["img"] = fig.canvas.copy_from_bbox(fig.bbox)
fig.canvas.mpl_connect("draw_event", _on_draw)

# Animated artists
ANIMATED = [
    # lines
    line_depth, line_autonomous_depth, line_fwd,
    line_roll, line_pitch, line_yaw, line_autonomous_yaw,
    line_voltage, line_current, line_temperature, line_audio,  # NEW includes audio
    # lidar images & annotations
    img_left, img_right, centroid_left_pt, centroid_right_pt,
    txt_lidar_left_dist, txt_lidar_right_dist,
    # bars
    *list(bar_container),
    # numeric overlays
    txt_fwd, txt_depth, txt_roll, txt_pitch, txt_yaw, txt_voltage, txt_current, txt_temperature, txt_audio  # NEW
]
for a in ANIMATED:
    a.set_animated(True)

def fast_draw():
    if not USE_BLIT or _bg["img"] is None:
        fig.canvas.draw(); fig.canvas.flush_events(); return
    fig.canvas.restore_region(_bg["img"])
    for a in ANIMATED:
        fig.draw_artist(a)
    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()
    update_fps()

def _on_resize(_evt=None):
    _bg["img"] = None

canvas_widget.bind("<Configure>", _on_resize)
fig.canvas.mpl_connect("resize_event", _on_resize)
fig.canvas.draw()   # seed background

# -----------------------
# Update Loop
# -----------------------
TARGET_DRAW_HZ  = 15.0
DRAW_INTERVAL_S = 1.0 / TARGET_DRAW_HZ
_next_deadline  = time.perf_counter()
_last_wh = [canvas_widget.winfo_width(), canvas_widget.winfo_height()]

def update_plot():
    global _next_deadline
    updated = False

    MAX_DRAIN = 250
    drained = 0
    while drained < MAX_DRAIN and not serial_queue.empty():
        line = serial_queue.get()
        drained += 1

        if not line.startswith("data:"):
            print(line)
            continue

        tokens = line[5:].strip().replace(",", " ").split()
        # Expecting 53 tokens total (ADDED audio_level after temperature)
        if len(tokens) != 53:
            continue

        try:
            # 0..31 LiDAR ints
            lidar_values = [int(t) for t in tokens[:32]]
            # 32..37 sensors: depth, roll, pitch, yaw, voltage, current
            sensor_values = [float(t) for t in tokens[32:38]]
            autonomous_fwd      = float(tokens[38])
            autonomous_depth    = float(tokens[39])
            autonomous_yaw      = float(tokens[40])
            thrust_1            = float(tokens[41])
            thrust_2            = float(tokens[42])
            thrust_3            = float(tokens[43])
            thrust_4            = float(tokens[44])
            temperature_val     = float(tokens[45])
            audio_level         = float(tokens[46])  # NEW
            # centroid/distance values shift by +1
            centroid_left_row   = float(tokens[47])
            centroid_left_col   = float(tokens[48])
            distance_left       = float(tokens[49])
            centroid_right_row  = float(tokens[50])
            centroid_right_col  = float(tokens[51])
            distance_right      = float(tokens[52])
        except ValueError:
            continue

        left_scan  = np.array(lidar_values[:16]).reshape((4, 4))
        right_scan = np.array(lidar_values[16:]).reshape((4, 4))
        depth_val, roll_val, pitch_val, yaw_val, voltage_val, current_val = sensor_values

        # LiDAR updates
        img_left.set_data(left_scan)
        img_right.set_data(right_scan)
        centroid_left_pt.set_data([centroid_left_col], [centroid_left_row])
        centroid_right_pt.set_data([centroid_right_col], [centroid_right_row])
        txt_lidar_left_dist.set_text(f"{distance_left:.2f}")
        txt_lidar_right_dist.set_text(f"{distance_right:.2f}")
        centroid_left_pt.set_visible(distance_left < 250)
        centroid_right_pt.set_visible(distance_right < 250)

        # Append histories
        history_depth.append(depth_val)
        history_roll.append(roll_val)
        history_pitch.append(pitch_val)
        history_yaw.append(yaw_val)
        history_voltage.append(voltage_val)
        history_current.append(current_val)
        history_temperature.append(temperature_val)
        history_audio_level.append(audio_level)  # NEW
        history_autonomous_fwd.append(autonomous_fwd)
        history_autonomous_depth.append(autonomous_depth)
        history_autonomous_yaw.append(autonomous_yaw)

        # Trim
        if len(history_depth) > history_length:
            history_depth.pop(0); history_roll.pop(0); history_pitch.pop(0); history_yaw.pop(0)
            history_voltage.pop(0); history_current.pop(0); history_temperature.pop(0); history_audio_level.pop(0)
            history_autonomous_fwd.pop(0); history_autonomous_depth.pop(0); history_autonomous_yaw.pop(0)

        # Set data
        line_depth.set_data(range(len(history_depth)), history_depth)
        line_autonomous_depth.set_data(range(len(history_autonomous_depth)), history_autonomous_depth)
        line_fwd.set_data(range(len(history_autonomous_fwd)), history_autonomous_fwd)
        line_roll.set_data(range(len(history_roll)), history_roll)
        line_pitch.set_data(range(len(history_pitch)), history_pitch)
        line_yaw.set_data(range(len(history_yaw)), history_yaw)
        line_autonomous_yaw.set_data(range(len(history_autonomous_yaw)), history_autonomous_yaw)
        line_voltage.set_data(range(len(history_voltage)), history_voltage)
        line_current.set_data(range(len(history_current)), history_current)
        line_temperature.set_data(range(len(history_temperature)), history_temperature)
        line_audio.set_data(range(len(history_audio_level)), history_audio_level)  # NEW

        # Overlays
        update_value_texts()

        # Thrust bars
        thrusts = [thrust_1, thrust_2, thrust_3, thrust_4]
        for i, b in enumerate(bar_container):
            b.set_height(thrusts[i])

        updated = True

    now = time.perf_counter()
    if updated and now >= _next_deadline:
        w = canvas_widget.winfo_width()
        h = canvas_widget.winfo_height()
        if w != _last_wh[0] or h != _last_wh[1]:
            fig.canvas.draw()               # recache background on size change
            _last_wh[0], _last_wh[1] = w, h
        else:
            fast_draw()
        _next_deadline = now + DRAW_INTERVAL_S

    root.after(10, update_plot)

# Kick off UI / draw loop
root.after(50, update_plot)
root.mainloop()
ser.close()
