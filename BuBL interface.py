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
import matplotlib as mpl

mpl.rcParams.update({
    "font.size": 8,
    "axes.titlesize": 9,
    "axes.labelsize": 8,
    "lines.linewidth": 0.8,
})

# Control interface for commanding a BuBL over bluetooth low energy
# -----------------------
# Serial Port
# -----------------------
try:
    # change for correct serial port
    ser = serial.Serial('COM7', 921600, timeout=0.1)
except Exception as e:
    print("Error opening serial port:", e)
    sys.exit(1)

serial_queue = queue.Queue()

# -----------------------
# Serial Reader Thread
# -----------------------
def serial_reader():
    """Read raw bytes, assemble complete '\n'-terminated lines, push only full lines."""
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
fig = plt.figure(figsize=(16, 20))
# 4 rows, 4 cols. Row 0 tall for LiDARs; rows 1–3 for sensors
gs = gridspec.GridSpec(
    4, 4, figure=fig,
    hspace=0.25, wspace=0.25,
    height_ratios=[3, 1, 1, 1]
)
gs.update(left=0.05, right=0.95, top=0.96, bottom=0.22, wspace=0.17, hspace=0.20)

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
ax_lidar_left.set_title("Left LiDAR Scan", fontsize=9)
ax_lidar_right.set_title("Right LiDAR Scan", fontsize=9)
for ax in (ax_lidar_left, ax_lidar_right):
    ax.set_xticks([]); ax.set_yticks([])

# Centroid dots & distance labels for LiDARs
centroid_left_pt, = ax_lidar_left.plot([], [], marker='o', linestyle='None',
                                       markerfacecolor='black', markeredgecolor='black',
                                       markersize=8, zorder=5)
centroid_right_pt, = ax_lidar_right.plot([], [], marker='o', linestyle='None',
                                         markerfacecolor='black', markeredgecolor='black',
                                         markersize=8, zorder=5)
txt_lidar_left_dist = ax_lidar_left.text(0.98, -0.03, "", transform=ax_lidar_left.transAxes,
                                         ha='center', va='top', fontsize=8)
txt_lidar_right_dist = ax_lidar_right.text(0.98, -0.03, "", transform=ax_lidar_right.transAxes,
                                           ha='center', va='top', fontsize=8)

# Colorbar for LiDARs
cbar_ax = fig.add_axes([0.72, 0.65, 0.01, 0.30])
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

# -----------------------
# Axes (Rows 1–3) per your requested layout
# -----------------------
# Row 1: Forward Thrust, Lateral Thrust, Depth; Motors spans rows 1–2 (col3)
ax_fwd    = fig.add_subplot(gs[1, 0])
ax_lat    = fig.add_subplot(gs[1, 1])
ax_depth  = fig.add_subplot(gs[1, 2])
ax_thrust = fig.add_subplot(gs[1:3, 3])      # 2 tall: rows 1–2

ax_fwd.set_title("Forward Thrust")
ax_lat.set_title("Lateral Thrust")
ax_depth.set_title("Depth")
ax_thrust.set_title("Motors")

# Row 2: Roll, Pitch, Yaw
ax_roll  = fig.add_subplot(gs[2, 0])
ax_pitch = fig.add_subplot(gs[2, 1])
ax_yaw   = fig.add_subplot(gs[2, 2])

ax_roll.set_title("Roll")
ax_pitch.set_title("Pitch")
ax_yaw.set_title("Yaw")

# Row 3: Voltage, Signal Strength, FREE plot; Current MUST stay below the thrust plot (col3)
ax_voltage = fig.add_subplot(gs[3, 0])
ax_rssi    = fig.add_subplot(gs[3, 1])
ax_free    = fig.add_subplot(gs[3, 2])
ax_current = fig.add_subplot(gs[3, 3])       # stays below thrust plot (col3)

ax_voltage.set_title("Voltage")
ax_rssi.set_title("Signal Strength")
ax_free.set_title("FREE")
ax_current.set_title("Current")

# -----------------------
# Plot Artists
# -----------------------
line_depth,      = ax_depth.plot([], [], '-', color='blue')
line_autonomous_depth, = ax_depth.plot([], [], '-', color='red')

line_fwd,        = ax_fwd.plot([], [], '-', color='red')
line_lat,        = ax_lat.plot([], [], '-', color='red')

line_roll,       = ax_roll.plot([], [], '-', color='blue')
line_pitch,      = ax_pitch.plot([], [], '-', color='blue')
line_yaw,        = ax_yaw.plot([], [], '-', color='blue')
line_autonomous_yaw, = ax_yaw.plot([], [], '-', color='red')  # kept (parsed), optional display

line_voltage,    = ax_voltage.plot([], [], '-', color='blue')
line_current,    = ax_current.plot([], [], '-', color='blue')
line_rssi,       = ax_rssi.plot([], [], '-', color='blue')

line_free,       = ax_free.plot([], [], '-', color='blue')    # selectable signal

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
history_audio_level = []
history_rssi    = []
history_autonomous_fwd   = []
history_autonomous_lat   = []
history_autonomous_depth = []
history_autonomous_yaw   = []
history_omega            = []   # FC_omega renamed to omega

# -----------------------
# Tkinter GUI
# -----------------------
root = tk.Tk()
root.title("BuBL Command Interface")
root.geometry("1300x800")
free_var = tk.StringVar(master=root, value="temperature")

root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=0)
root.columnconfigure(0, weight=1)

# Canvas Frame
canvas_frame = ttk.Frame(root)

# Fix the drawing area so it never grows when data/text appears
canvas_w, canvas_h = 1000, 600
canvas_frame.configure(width=canvas_w, height=canvas_h)
canvas_frame.pack_propagate(False)
canvas_frame.grid(row=0, column=0, sticky="nsew")

canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill=tk.BOTH, expand=True)
canvas_widget.pack_configure(pady=0)

# --- FIX FIGURE SIZE (final) ---
def freeze_figure_size(event=None):
    """Lock figure size to the current canvas pixel size."""
    w_px = canvas_widget.winfo_width()
    h_px = canvas_widget.winfo_height()
    dpi = fig.get_dpi()
    if w_px > 0 and h_px > 0:
        fig.set_size_inches(w_px / dpi, h_px / dpi, forward=True)
        fig.canvas.draw_idle()

canvas_widget.bind("<Configure>", freeze_figure_size)
freeze_figure_size()

# -----------------------
# Hide ALL x-axis ticks/labels
# -----------------------
for ax in (
    ax_fwd, ax_lat, ax_depth,
    ax_roll, ax_pitch, ax_yaw,
    ax_voltage, ax_current, ax_rssi, ax_free
):
    ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    ax.set_xlabel('')

# -----------------------
# Freeze layout (prevents subplot size shifts)
# -----------------------
def freeze_layout(fig):
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    for ax in fig.axes:
        pos = ax.get_position()
        ax.set_position(pos)
        ax.set_autoscale_on(False)
freeze_layout(fig)

# -----------------------
# Control Frame
# -----------------------
control_frame = ttk.Frame(root)
control_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

# FPS (kept as-is)
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

# -----------------------
# Fixed limits
# -----------------------
ax_fwd.set_ylim(-1000, 1000)
ax_lat.set_ylim(-1000, 1000)
ax_depth.set_ylim(-5, 30)

ax_roll.set_ylim(-30, 30)
ax_pitch.set_ylim(-30, 30)
ax_yaw.set_ylim(-200, 200)

ax_voltage.set_ylim(0, 5)
ax_current.set_ylim(0, 3)
ax_rssi.set_ylim(-150, 0)

# FREE plot limits are set dynamically based on selection
ax_free.set_ylim(10, 35)

for a in (ax_fwd, ax_lat, ax_depth, ax_roll, ax_pitch, ax_yaw, ax_voltage, ax_current, ax_rssi, ax_free):
    a.set_xlim(0, history_length)

# -----------------------
# Numeric Overlays
# -----------------------
def make_value_text(ax):
    return ax.text(
        -0.01, 0.96, "",
        transform=ax.transAxes,
        ha="left", va="top",
        fontsize=8,
        bbox=None,
        zorder=10
    )

txt_fwd         = make_value_text(ax_fwd)
txt_lat         = make_value_text(ax_lat)
txt_depth       = make_value_text(ax_depth)
txt_roll        = make_value_text(ax_roll)
txt_pitch       = make_value_text(ax_pitch)
txt_yaw         = make_value_text(ax_yaw)
txt_voltage     = make_value_text(ax_voltage)
txt_current     = make_value_text(ax_current)
txt_rssi        = make_value_text(ax_rssi)
txt_free        = make_value_text(ax_free)

# --- MCU time + FC state overlay (anchored to the robot PNG axes) ---
x0 = 0.10   # left edge anchor
y0 = -0.30  # bottom line baseline
dy = 0.11   # vertical spacing between lines

txt_fcstate = ax_cam.text(
    x0, y0, "",
    transform=ax_cam.transAxes,
    ha="left", va="bottom", fontsize=9,
    zorder=50
)

txt_mcutime = ax_cam.text(
    x0, y0 + dy, "",
    transform=ax_cam.transAxes,
    ha="left", va="bottom", fontsize=9,
    zorder=50
)

FC_STATE_MAP = {
    0: "not connected",
    1: "absolute",
    2: "yaw rate",
    3: "direct",
}



def _apply_free_ylim_and_title():
    sel = free_var.get()
    if sel == "temperature":
        ax_free.set_title("Temperature")
        ax_free.set_ylim(10, 35)
    elif sel == "audio level":
        ax_free.set_title("Audio Level")
        ax_free.set_ylim(0, 110)
    else:
        ax_free.set_title("Angular Rate")
        ax_free.set_ylim(-0.5, 3.5)

def update_value_texts():
    if history_autonomous_fwd:
        txt_fwd.set_text(f"{history_autonomous_fwd[-1]:7.1f}")
    if history_autonomous_lat:
        txt_lat.set_text(f"{history_autonomous_lat[-1]:7.1f}")
    if history_depth:
        txt_depth.set_text(f"{history_depth[-1]:7.1f}")
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
    if history_rssi:
        txt_rssi.set_text(f"{history_rssi[-1]:7.1f}")

    sel = free_var.get()
    if sel == "temperature" and history_temperature:
        txt_free.set_text(f"{history_temperature[-1]:7.1f}")
    elif sel == "audio level" and history_audio_level:
        txt_free.set_text(f"{history_audio_level[-1]:7.1f}")
    elif sel == "omega" and history_omega:
        txt_free.set_text(f"{history_omega[-1]:7.2f}")

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

# --- FREE plot dropdown moved to the RIGHT of the command line (as requested) ---
free_combo = ttk.Combobox(
    cmd_frame,
    textvariable=free_var,
    values=["temperature", "audio level", "omega"],
    state="readonly",
    width=14
)
free_combo.pack(side=tk.LEFT)
free_combo.current(0)

def refresh_free_plot_from_history():
    """Immediately rebind the FREE plot line to the selected history buffer."""
    sel = free_var.get()

    if sel == "temperature":
        y = history_temperature
    elif sel == "audio level":
        y = history_audio_level
    else:
        y = history_omega

    line_free.set_data(range(len(y)), y)

    # Update numeric overlay for FREE plot too
    update_value_texts()

def on_free_select(_evt=None):
    _apply_free_ylim_and_title()
    refresh_free_plot_from_history()

    # IMPORTANT: titles/ylims are not blitted artists -> force full redraw + recache background
    _bg["img"] = None
    fig.canvas.draw()

free_combo.bind("<<ComboboxSelected>>", on_free_select)

# Also catch programmatic changes reliably
def _free_trace(*_args):
    on_free_select()

free_var.trace_add("write", _free_trace)

_apply_free_ylim_and_title()
refresh_free_plot_from_history()

command_history = []
history_index = -1

def build_multi_target_cmd(cmd: str) -> str:
    """
    If one or more robot toggles are ON, wrap a simple command so that it is
    sent to each selected robot as:
        !ROBOT1#<cmd>#!ROBOT2#<cmd>!...
    If no toggles are on, or if the command already contains '!' or '#'
    (i.e., it is already preformatted for routing / connection), return cmd unchanged.
    """
    if '!' in cmd or '#' in cmd:
        return cmd

    active = [name for name, var in multi_target_vars.items() if var.get()]
    if not active:
        return cmd

    pieces = [f"!{name}#{cmd}" for name in active]
    return "#".join(pieces)

def send_command(cmd):
    global command_history
    try:
        final_cmd = build_multi_target_cmd(cmd)
        ser.write((final_cmd + "\n").encode('utf-8'))
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
        ("CETUS", "!CETUS"),
        ("LEVIATHAN", "!LEVIATHAN"),
        ("KRAKEN", "!KRAKEN"),
    ],
    "Power": [
        ("Enable", "[E]"),
        ("Disable", "[H]"),
    ],
    "Motion & Calibration": [
        ("Forward", "[C,500,0,0,0]"),
        ("Reverse", "[C,-500,0,0,0]"),
        ("Left", "[C,0,100,0,0]"),
        ("Right", "[C,0,-100,0,0]"),
        ("Surface", "[C,0,0,0,0]"),
        ("Dive", "[C,0,0,10,0]"),
        ("Rotate Left", "[C,0,0,0,-360]"),
        ("Rotate Right", "[C,0,0,0,360]"),
        ("Gyro Cal.", "[G]"),
        ("LiDAR Cal.", "[V]"),
    ],
    "Controller": [
        ("Quick Setup", "[H]\n[U,1000,1000,1000,100,100,500]\n[F,0,0,800,0,0,0]\n[T,1,1,1,1]\n[L,0,10,10,10]\n[W,200,200,200,200]\n[M,280,900]"),
        ("Set Yaw", "[Y,0]"),
        ("Set PID Depth", "[Pn,1,20,1,5]"),
        ("Zero PID Depth", "[Pn,1,0,0,0]"),
        ("Set PID Roll", "[Pn,2,0,0,2]"),
        ("Zero PID Roll", "[Pn,2,0,0,0]"),
        ("Set PID Pitch", "[Pn,3,0,0,2]"),
        ("Zero PID Pitch", "[Pn,3,0,0,0]"),
        ("Set PID Yaw", "[Pn,4,4,0,2]"),
        ("Zero PID Yaw", "[Pn,4,0,0,0]"),
    ],
    "Data & Recording": [
        ("Stream", "[B,1]"),
        ("Stop Stream", "[B,0]"),
        ("Collect Data", "[Q,1]"),
        ("Stop Collect", "[Q,0]"),
        ("Capture Image", "[O,8,1,1,2]"),
        ("Capture Audio", "[I,3]"),
        ("Record All", "[R,1,20]"),
        ("Stop Recording", "[R,0,1]"),
        ("Record Basic", "[R,2,20]"),
        ("Record Power", "[R,3,50]"),
    ],
    "Imaging Experiments": [
        ("10 Image HQ", "[O,8,1,10,0.1]"),
        ("One Image", "[O,8,1,1,0.5]"),
        ("Record Gyro", "[R,6,50]"),
        ("Stop Recording", "[R,0,1]"),
        ("Collect Data", "[Q,1]"),
        ("Stop Collect", "[Q,0]"),
        ("Slow Rotate", "[dC,0,0,0,0.1]"),
    ],
    "Chain Experiments": [
        ("Steer Front", "[A,7,-5,0,0]"),
        ("Steer Rear", "[A,7,5,0,0]"),
        ("Zero PID Yaw", "[Pn,4,0,0,0]"),
        ("Only D Yaw", "[Pn,4,0,0,2]"),
        ("Set Yaw 0", "[Y,0]"),
        ("Set Yaw 90", "[Y,90]"),
        ("Set Yaw -90", "[Y,-90]"),
        ("Stop Program", "[A,0]\n[C,0,0,0]\n[H]"),
    ],
    "LiDAR": [
        ("Wonder", "[A,3,400,1,1.0,1.0,1.0,800,150,600,400,2,600,0]"),
        ("Wall Follow", "[A,4,400,1,1.0,1.0,1.0,800,150,600,400,0.5,0.25,600,0]"),
        ("Stop Program", "[A,0]\n[C,0,0,0]\n[H]"),
    ],
    "Direct Thrust": [
        ("Reset Thrust", "[T,0,0,0,0 ]"),
        ("40% Forward", "[eC,320,0,320,0]"),
        ("50% Forward", "[eC,400,0,400,0]"),
        ("60% Forward", "[eC,480,0,480,0]"),
        ("70% Forward", "[eC,560,0,560,0]"),
        ("80% Forward", "[eC,640,0,640,0]"),
        ("90% Forward", "[eC,720,0,720,0]"),
        ("100% Forward","[eC,800,0,800,0]"),
        ("40% Reverse", "[eC,0,320,0,320]"),
        ("50% Reverse", "[eC,0,400,0,400]"),
        ("60% Reverse", "[eC,0,480,0,480]"),
        ("70% Reverse", "[eC,0,560,0,560]"),
        ("80% Reverse", "[eC,0,640,0,640]"),
        ("90% Reverse", "[eC,0,720,0,720]"),
        ("100% Reverse","[eC,0,800,0,800]"),
    ],
    "Tail": [
        ("Stop Program", "[A,0]\n[C,0,0,0]\n[H]"),
        ("Increase Torque", "[U,1000,1000,1000,100,100,1000]"),
        ("Decrease Torque", "[U,1000,1000,1000,100,100,500]"),
        ("Wiggle", "[A,6,2,1000,1,0,0,0]"),
        ("Wiggle Fwd S", "[A,6,2,1000,1,0,500,0]"),
        ("Wiggle Dive", "[A,6,2,1000,1,0,0,10]"),
        ("Wiggle Fwd D", "[A,6,2,1000,1,0,500,10]"),
        ("Square", "[A,7,90,5,600,0]"),
        ("Record Power", "[R,3,50]\n[N,Begin Tail Experiment]"),
        ("Stop Record", "[R,0,1]\n[N,End Tail Experiment]"),
    ],

    "Disassembly": [
        ("Increase Torque", "[U,1000,1000,1000,100,100,1000]"),
        ("Disassemble - Direct","!NEPTUNE#[eC,720,0,0,720]#!POSEIDON#[eC,0,720,720,0]#!TRITON#[eC,720,0,0,720]#!NAUTILUS#[eC,0,720,720,0]#!OCEANUS#[eC,720,0,0,720]#!CETUS#[eC,0,720,720,0]#!LEVIATHAN#[eC,720,0,0,720]#!KRAKEN#[eC,0,720,720,0]"),
    ],

    "Mapping": [
        ("Unique Imaging", "!NEPTUNE#[O,8,0.2,150,0.5]#!TRITON#[O,8,0.2,150,0.4]#!NAUTILUS#[O,8,0.2,150,0.3]#!OCEANUS#[O,8,0.2,150,0.2]#!CETUS#[O,8,0.2,150,0.1]#!LEVIATHAN#[O,8,0.2,150,0.3]#!KRAKEN#[O,8,0.2,150,0.2]"),
        ("Wonder 15cm", "[A,3,400,1,1.0,1.0,1.0,800,150,600,400,2,600,15]"),
        ("Wonder 20cm", "[A,3,400,1,1.0,1.0,1.0,800,150,600,400,2,600,20]"),
        ("Wonder 25cm", "[A,3,400,1,1.0,1.0,1.0,800,150,600,400,2,600,25]"),
        ("Wonder 30cm", "[A,3,400,1,1.0,1.0,1.0,800,150,600,400,2,600,30]"),
        ("HQ Imaging", "[O,8,0.5,500,0.1]"),
        ("Mapping", "[A,2,400,1,1.0,1.0,1.0,800,150,600,400,2,30,12,10,0.5,300,0]"),
    ],

    "ICRA26": [
        ("Forward 300", "[C,300,0,0,0]"),
        ("Forward 500", "[C,500,0,0,0]"),
        ("Forward 700", "[C,700,0,0,0]"),
        ("1 robot", "[N,1 robot]"),
        ("2 robot", "[N,2 robot]"),
        ("3 robot", "[N,3 robot]"),
        ("4 robot", "[N,4 robot]"),
        ("5 robot", "[N,5 robot]"),
        ("6 robot", "[N,6 robot]"),
        ("7 robot", "[N,7 robot]"),
        ("8 robot", "[N,8 robot]"),

    ],

    "Vision": [
        ("Stop Program", "[A,0]\n[C,0,0,0]\n[H]"),
        ("Hit Red", "[A,5,0.5,80,-0.4,-0.05,0,0,0,300,10,1]"),
        ("Hit Green", "[A,5,0.5,80,-0.4,-0.05,0,0,0,300,10,2]"),
        ("Hit Yellow", "[A,5,0.5,80,-0.4,-0.05,0,0,0,300,10,4]"),
        ("Capture Image", "[O,8,1,1]"),
    ],

    "Controllers": [
        ("Base PID", "[Pn,1,20,1,5]#[Pn,2,0,0,2]#[Pn,3,0,0,2]#[Pn,4,4,1,2]"),
        ("High PD", "[Pn,1,20,0,5]#[Pn,2,0,0,4]#[Pn,3,0,0,4]#[Pn,4,20,0,10]"),
        ("Thrust - all", "[T,1,1,1,1]"),
        ("Thrust - variable", "[T,0,0,0,0]"),
        ("Thrust - rear only", "[T,1,-1,1,-1]"),
        ("Thrust - front only", "[T,-1,1,-1,1]"),
        ("FF 400", "[F,0,0,400,0,0,0]"),
        ("FF 600", "[F,0,0,600,0,0,0]"),
        ("FF 800", "[F,0,0,800,0,0,0]"),
    ],

    "Shapes": [
        ("- Small", "[A,1,180,5,500,0]"),
        ("- Large", "[A,1,180,10,800,0]"),
        ("△ Small", "[A,1,120,5,500,0]"),
        ("△ Large", "[A,1,120,10,800,0]"),
        ("□ Small", "[A,1,90,5,500,0]"),
        ("□ Large", "[A,1,90,10,800,0]"),
        ("Stop Program", "[A,0]\n[C,0,0,0]\n[H]"),
    ]
}

# -----------------------
# Multi-robot broadcast support
# -----------------------
multi_target_vars = {}
for label, _cmd in button_groups["Connection"]:
    multi_target_vars[label] = tk.BooleanVar(master=root, value=False)

MODE_GROUPS = ["Imaging Experiments","LiDAR", "Vision", "Mapping", "Disassembly",
               "Direct Thrust", "Tail", "Controllers", "Shapes", "Chain Experiments", "ICRA26"]

right_side_started = False
current_col = 0  # track which column we're placing into on row=0

def _place_two_per_row_buttons(parent, items):
    for i, (label, cmd_code) in enumerate(items):
        r, c = divmod(i, 2)
        btn = ttk.Button(parent, text=label, command=lambda c=cmd_code: send_command(c))
        btn.grid(row=r, column=c, padx=2, pady=2, sticky="ew")

# 1) Place all groups up through and including "Data & Recording"
for group_name, items in button_groups.items():
    if group_name in MODE_GROUPS:
        continue

    lf = ttk.LabelFrame(button_frame, text=group_name)
    lf.grid(row=0, column=current_col, padx=5, pady=2, sticky="n")

    if group_name == "Connection":
        names = [label for (label, _cmd) in items]
        conn_var = tk.StringVar()
        combo = ttk.Combobox(lf, textvariable=conn_var, values=names,
                             state="readonly", width=14)
        combo.grid(row=0, column=0, padx=2, pady=2,
                   sticky="ew", columnspan=2)
        if names:
            conn_var.set(names[0])
            combo.current(0)

        def on_select(_evt=None):
            target = combo.get()
            cmd = f"!{target}"
            send_command(cmd)

        combo.bind("<<ComboboxSelected>>", on_select)

        toggles_frame = ttk.Frame(lf)
        toggles_frame.grid(row=1, column=0, columnspan=2,
                           sticky="w", padx=2, pady=(2, 0))

        PER_ROW = 2
        for i, name in enumerate(names):
            var = multi_target_vars[name]
            r, c = divmod(i, PER_ROW)
            chk = ttk.Checkbutton(toggles_frame, text=name, variable=var)
            chk.grid(row=r, column=c, padx=2, pady=1, sticky="w")

    elif group_name == "Power":
        for i, (label, cmd_code) in enumerate(items):
            btn = ttk.Button(lf, text=label,
                             command=lambda c=cmd_code: send_command(c))
            btn.grid(row=i, column=0, padx=2, pady=2, sticky="ew")

    else:
        _place_two_per_row_buttons(lf, items)

    current_col += 1
    if group_name == "Data & Recording":
        right_side_started = True
        break

# 2) Build the mode pane in the next column to the right
modes_lf = ttk.LabelFrame(button_frame, text="Select")
modes_lf.grid(row=0, column=current_col, padx=5, pady=2, sticky="n")

stack = ttk.Frame(modes_lf)
stack.grid(row=0, column=0, sticky="nsew")

mode_select_frame = ttk.Frame(stack)
mode_select_frame.grid(row=0, column=0, sticky="nsew")

mode_frames = {}
for mode_name in MODE_GROUPS:
    f = ttk.Frame(stack)
    f.grid(row=0, column=0, sticky="nsew")
    mode_frames[mode_name] = f

def show_mode(name):
    if name == "__select__":
        mode_select_frame.tkraise()
    else:
        mode_frames[name].tkraise()

MAX_PER_ROW = 4

for i, mode_name in enumerate(MODE_GROUPS):
    r = i // MAX_PER_ROW
    c = i % MAX_PER_ROW
    btn = ttk.Button(mode_select_frame, text=mode_name,
                     command=lambda n=mode_name: show_mode(n))
    btn.grid(row=r, column=c, padx=2, pady=2, sticky="ew")

for c in range(MAX_PER_ROW):
    mode_select_frame.columnconfigure(c, weight=1)

for mode_name in MODE_GROUPS:
    items = button_groups.get(mode_name, [])

    back_btn = ttk.Button(mode_frames[mode_name], text="⟵ Back",
                          command=lambda: show_mode("__select__"))
    back_btn.grid(row=0, column=0, columnspan=MAX_PER_ROW,
                  padx=2, pady=(2, 6), sticky="ew")

    for c in range(MAX_PER_ROW):
        mode_frames[mode_name].columnconfigure(c, weight=1)

    for i, (label, cmd_code) in enumerate(items):
        r = 1 + (i // MAX_PER_ROW)
        c = i % MAX_PER_ROW
        btn = ttk.Button(mode_frames[mode_name], text=label,
                         command=lambda c=cmd_code: send_command(c))
        btn.grid(row=r, column=c, padx=2, pady=2, sticky="ew")

show_mode("__select__")

# -----------------------
# Blitting Setup
# -----------------------
USE_BLIT = True
_bg = {"img": None}

def _on_draw(_event=None):
    _bg["img"] = fig.canvas.copy_from_bbox(fig.bbox)
fig.canvas.mpl_connect("draw_event", _on_draw)

ANIMATED = [
    line_depth, line_autonomous_depth, line_fwd, line_lat,
    line_roll, line_pitch, line_yaw, line_autonomous_yaw,
    line_voltage, line_current, line_rssi, line_free,
    img_left, img_right, centroid_left_pt, centroid_right_pt,
    txt_lidar_left_dist, txt_lidar_right_dist,
    *list(bar_container),
    txt_fwd, txt_lat, txt_depth, txt_roll, txt_pitch, txt_yaw,
    txt_voltage, txt_current, txt_rssi, txt_free,
    txt_mcutime, txt_fcstate,
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
            if line.startswith("[INFO]"):
                print(line)
            elif line.startswith("[SUCCESS]"):
                print("\033[32m" + line + "\033[0m")
            elif line.startswith("[WARNING]"):
                print("\033[33m" + line + "\033[0m")
            elif line.startswith("[ERROR]"):
                print("\033[31m" + line + "\033[0m")
            else:
                print(line)
            continue

        tokens = line[5:].strip().replace(",", " ").split()

        # Transmitter now prints:
        # 32 lidar
        # depth roll pitch yaw voltage current
        # a_fwd a_lat a_depth a_yaw
        # t1 t2 t3 t4
        # temp sound
        # cL_r cL_c Ldis cR_r cR_c Rdis
        # MCUtime FC_state FC_omega
        # RSSI_filtered
        #
        # total = 58 tokens
        if len(tokens) != 58:
            continue

        try:
            # 0..31 LiDAR ints
            lidar_values = [int(t) for t in tokens[:32]]

            # 32..37 sensors
            depth_val   = float(tokens[32])
            roll_val    = float(tokens[33])
            pitch_val   = float(tokens[34])
            yaw_val     = float(tokens[35])
            voltage_val = float(tokens[36])
            current_val = float(tokens[37])

            # autonomy
            autonomous_fwd   = float(tokens[38])
            autonomous_lat   = float(tokens[39])  # NEW: a_lat -> autonomous_lat
            autonomous_depth = float(tokens[40])
            autonomous_yaw   = float(tokens[41])

            # motors
            thrust_1 = float(tokens[42])
            thrust_2 = float(tokens[43])
            thrust_3 = float(tokens[44])
            thrust_4 = float(tokens[45])

            # free signals
            temperature_val = float(tokens[46])
            audio_level     = float(tokens[47])

            # lidar centroid/dists
            centroid_left_row   = float(tokens[48])
            centroid_left_col   = float(tokens[49])
            distance_left       = float(tokens[50])
            centroid_right_row  = float(tokens[51])
            centroid_right_col  = float(tokens[52])
            distance_right      = float(tokens[53])

            # overlays
            mcu_time_min  = float(tokens[54])
            fc_state_raw  = float(tokens[55])
            omega         = float(tokens[56])     # NEW: FC_omega -> omega

            # tx-side rssi
            rssi_filtered = float(tokens[57])

        except ValueError:
            continue

        left_scan  = np.array(lidar_values[:16]).reshape((4, 4))
        right_scan = np.array(lidar_values[16:]).reshape((4, 4))

        # LiDAR updates
        img_left.set_data(left_scan)
        img_right.set_data(right_scan)
        centroid_left_pt.set_data([centroid_left_col], [centroid_left_row])
        centroid_right_pt.set_data([centroid_right_col], [centroid_right_row])
        txt_lidar_left_dist.set_text(f"{int(distance_left)}")
        txt_lidar_right_dist.set_text(f"{int(distance_right)}")
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
        history_audio_level.append(audio_level)
        history_omega.append(omega)

        history_rssi.append(rssi_filtered)

        history_autonomous_fwd.append(autonomous_fwd)
        history_autonomous_lat.append(autonomous_lat)
        history_autonomous_depth.append(autonomous_depth)
        history_autonomous_yaw.append(autonomous_yaw)

        # Trim
        if len(history_depth) > history_length:
            history_depth.pop(0); history_roll.pop(0); history_pitch.pop(0); history_yaw.pop(0)
            history_voltage.pop(0); history_current.pop(0)
            history_temperature.pop(0); history_audio_level.pop(0); history_omega.pop(0)
            history_rssi.pop(0)
            history_autonomous_fwd.pop(0); history_autonomous_lat.pop(0)
            history_autonomous_depth.pop(0); history_autonomous_yaw.pop(0)

        # Set data
        line_depth.set_data(range(len(history_depth)), history_depth)
        line_autonomous_depth.set_data(range(len(history_autonomous_depth)), history_autonomous_depth)

        line_fwd.set_data(range(len(history_autonomous_fwd)), history_autonomous_fwd)
        line_lat.set_data(range(len(history_autonomous_lat)), history_autonomous_lat)

        line_roll.set_data(range(len(history_roll)), history_roll)
        line_pitch.set_data(range(len(history_pitch)), history_pitch)
        line_yaw.set_data(range(len(history_yaw)), history_yaw)
        line_autonomous_yaw.set_data(range(len(history_autonomous_yaw)), history_autonomous_yaw)

        line_voltage.set_data(range(len(history_voltage)), history_voltage)
        line_current.set_data(range(len(history_current)), history_current)
        line_rssi.set_data(range(len(history_rssi)), history_rssi)

        # FREE plot (selectable)
        sel = free_var.get()
        if sel == "temperature":
            line_free.set_data(range(len(history_temperature)), history_temperature)
        elif sel == "audio level":
            line_free.set_data(range(len(history_audio_level)), history_audio_level)
        else:
            line_free.set_data(range(len(history_omega)), history_omega)

        # Overlays
        update_value_texts()

        # MCU time + FC state overlay
        _fc_state_i = int(round(fc_state_raw))
        _fc_state_s = FC_STATE_MAP.get(_fc_state_i, str(_fc_state_i))
        txt_mcutime.set_text(f"Operating Time:  {int(round(mcu_time_min))} mins")
        txt_fcstate.set_text(f"Controller Mode: {_fc_state_s}")

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
