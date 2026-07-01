import os
import threading
import time
import tkinter as tk
from tkinter import ttk

import mujoco

try:
    import mujoco.viewer as mujoco_viewer
except Exception:
    try:
        import mujoco_viewer
    except Exception:
        mujoco_viewer = None

MODEL_FILENAME = "knee_joint_mechanism.xml"

JOINT_SUMMARY = [
    {"ID": "1", "Joint Name": "joint1", "Type": "hinge", "Axis": "[0 1 0]", "Function": "Base rotation"},
    {"ID": "2", "Joint Name": "joint2", "Type": "hinge", "Axis": "[0 1 0]", "Function": "Shaft rotation"},
    {"ID": "3", "Joint Name": "joint3", "Type": "slide", "Axis": "[0 0 1]", "Function": "Coupler slider"},
    {"ID": "4", "Joint Name": "joint4", "Type": "hinge", "Axis": "[0 1 0]", "Function": "Rod rotation"},
    {"ID": "5", "Joint Name": "joint5", "Type": "hinge", "Axis": "[0 1 0]", "Function": "Knee (crank)"},
    {"ID": "6", "Joint Name": "joint6", "Type": "hinge", "Axis": "[0 1 0]", "Function": "Shank rotation"},
]

KEY_PARAMETERS = [
    ("Timestep", "0.001 s"),
    ("Gravity", "0 0 -9.81 m/s²"),
    ("Coupler Slide Range", "-0.02 to 0.02"),
    ("Joint Damping (default)", "10.0"),
    ("Actuator", "Motor on joint5 (knee)")
]

ACTUATION_NOTES = [
    "motor_name='motor_knee' joint='joint5' gear='30'",
    "Controls knee flexion/extension",
    "Positive command → extension"
]

COMPONENTS = [
    "Base Link", "Shaft (Bearing)", "Coupler (Slider)",
    "Connecting Rod", "Crank (Knee Plate)", "Shank (Lower Link)"
]

JOINT_TYPES = [
    ("hinge", "Rotational joint around one axis"),
    ("slide", "Prismatic joint along one axis")
]


def read_xml_snippet(xml_path, lines=40):
    if not os.path.exists(xml_path):
        return "<XML file not found>"

    with open(xml_path, "r", encoding="utf-8") as f:
        xml_lines = f.readlines()

    return "".join(xml_lines[:lines])


def run_viewer(model, data):
    if mujoco_viewer is None:
        print("MuJoCo viewer is unavailable in this Python environment.")
        print("Install the viewer package or run the simulation in an environment with MuJoCo GUI support.")
        return

    if hasattr(mujoco_viewer, "launch_passive"):
        with mujoco_viewer.launch_passive(model, data) as viewer:
            viewer.cam.azimuth = 160
            viewer.cam.elevation = -20
            viewer.cam.distance = 0.5
            viewer.cam.lookat = (0.0, 0.0, -0.05)

            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.001)

    elif hasattr(mujoco_viewer, "MjViewer"):
        viewer = mujoco_viewer.MjViewer(model, data)
        viewer.cam.azimuth = 160
        viewer.cam.elevation = -20
        viewer.cam.distance = 0.5
        viewer.cam.lookat = (0.0, 0.0, -0.05)

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.render()
            time.sleep(0.001)

    else:
        print("MuJoCo viewer module found, but it does not support launch_passive or MjViewer.")


def create_table(frame, columns, rows, heading=""):
    table = ttk.Treeview(frame, columns=columns, show="headings", height=len(rows))
    for col in columns:
        table.heading(col, text=col)
        table.column(col, width=120, anchor="center")
    for row in rows:
        table.insert("", "end", values=row)
    return table


def build_ui(root, xml_snippet):
    root.title("MuJoCo Knee Joint Mechanism UI")
    root.geometry("1200x800")

    top_frame = ttk.Frame(root)
    top_frame.pack(fill="both", expand=True, padx=10, pady=10)

    left_frame = ttk.LabelFrame(top_frame, text="XML (mjCF model)")
    left_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))

    xml_text = tk.Text(left_frame, wrap="none", font=("Courier", 10), bg="#111111", fg="#d0d0d0")
    xml_text.insert("1.0", xml_snippet)
    xml_text.config(state="disabled")
    xml_text.pack(fill="both", expand=True, padx=5, pady=5)

    right_frame = ttk.Frame(top_frame)
    right_frame.pack(side="left", fill="y", ipadx=10)

    summary_frame = ttk.LabelFrame(right_frame, text="Joint Summary")
    summary_frame.pack(fill="x", pady=(0, 10))
    summary_table = create_table(
        summary_frame,
        ["ID", "Joint Name", "Type", "Axis", "Function"],
        [[item[c] for c in ["ID", "Joint Name", "Type", "Axis", "Function"]] for item in JOINT_SUMMARY],
    )
    summary_table.pack(fill="x", padx=5, pady=5)

    param_frame = ttk.LabelFrame(right_frame, text="Key Parameters")
    param_frame.pack(fill="x", pady=(0, 10))
    for name, value in KEY_PARAMETERS:
        row = ttk.Frame(param_frame)
        row.pack(fill="x", pady=2, padx=5)
        ttk.Label(row, text=f"{name}:", width=22, anchor="w").pack(side="left")
        ttk.Label(row, text=value, anchor="w").pack(side="left")

    act_frame = ttk.LabelFrame(right_frame, text="Actuation")
    act_frame.pack(fill="x", pady=(0, 10))
    for note in ACTUATION_NOTES:
        ttk.Label(act_frame, text=note, wraplength=260, anchor="w", justify="left").pack(fill="x", padx=5, pady=2)

    bottom_frame = ttk.LabelFrame(root, text="Component Geoms & Joint Types")
    bottom_frame.pack(fill="both", expand=False, padx=10, pady=(0, 10))

    left_bottom = ttk.Frame(bottom_frame)
    left_bottom.pack(side="left", fill="both", expand=True, padx=(0, 10))
    ttk.Label(left_bottom, text="Component Geometries", font=("Arial", 10, "bold")).pack(anchor="w", pady=(5, 2))
    for idx, component in enumerate(COMPONENTS, 1):
        ttk.Label(left_bottom, text=f"{idx}. {component}", anchor="w").pack(fill="x", padx=5, pady=2)

    right_bottom = ttk.Frame(bottom_frame)
    right_bottom.pack(side="left", fill="both", expand=True)
    ttk.Label(right_bottom, text="Joint Types", font=("Arial", 10, "bold")).pack(anchor="w", pady=(5, 2))
    for joint_type, desc in JOINT_TYPES:
        ttk.Label(right_bottom, text=f"{joint_type}: {desc}", anchor="w").pack(fill="x", padx=5, pady=2)

    launch_frame = ttk.Frame(root)
    launch_frame.pack(fill="x", padx=10, pady=(0, 10))
    if mujoco_viewer is None:
        ttk.Label(launch_frame, text="MuJoCo viewer unavailable in this environment.").pack(side="left", padx=5)
    else:
        ttk.Label(launch_frame, text="3D MuJoCo viewer should open automatically in a separate window.").pack(side="left", padx=5)


def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, MODEL_FILENAME)

    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"Cannot find model XML: {xml_path}")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    xml_snippet = read_xml_snippet(xml_path)

    ui_thread = None
    if mujoco_viewer is not None:
        ui_thread = threading.Thread(target=run_viewer, args=(model, data), daemon=True)
        ui_thread.start()

    root = tk.Tk()
    build_ui(root, xml_snippet)
    root.mainloop()

    # When Tk window closes, exit cleanly
    if ui_thread is not None and ui_thread.is_alive():
        print("Closing MuJoCo viewer...")


if __name__ == "__main__":
    main()
