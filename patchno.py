import tkinter as tk
root = tk.Tk()
print("Tcl version:", root.tk.call("info", "tclversion"))
print("Tk version :", root.tk.call("info", "patchlevel"))
root.destroy()