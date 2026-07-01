#!/home/quang/ARMI/spacemouse/sm/bin/python
"""
Simulate and control a lower limb exoskeleton in MuJoCo.
- Coordinated walking/stepping gait (Automatic Mode).
- Keyboard & GUI Interactive Joint control (Manual Mode).
  - Press 'M' to toggle between Gait Mode and Manual Mode.
  - Press 'S' or SPACEBAR to toggle pelvis suspension.
  - In Manual Mode, press 0-9 to select a joint, and Arrow Keys (or '[' and ']') to control it.
"""

import os
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer

def main():
    # 1. Load the scene
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, "scene.xml")
    
    if not os.path.exists(xml_path):
        print(f"Error: {xml_path} does not exist!")
        sys.exit(1)
        
    print(f"Loading MuJoCo model from: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # Find Pelvis Weld constraint ID
    weld_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "pelvis_weld")
    if weld_id == -1:
        print("Warning: 'pelvis_weld' equality constraint not found in scene.xml!")

    # Find Actuator IDs
    actuator_names = [
        "left_hip_yaw_act",     # 0
        "left_hip_roll_act",    # 1
        "left_hip_pitch_act",   # 2
        "left_knee_pitch_act",  # 3
        "left_ankle_pitch_act", # 4
        "right_hip_yaw_act",    # 5
        "right_hip_roll_act",   # 6
        "right_hip_pitch_act",  # 7
        "right_knee_pitch_act", # 8
        "right_ankle_pitch_act" # 9
    ]
    actuator_ids = {}
    for name in actuator_names:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id == -1:
            print(f"Error: Actuator '{name}' not found!")
            sys.exit(1)
        actuator_ids[name] = act_id

    # 2. Simulation State & Parameters
    frequency = 0.8        # Hz (frequency of walking steps)
    hip_amplitude = 0.35   # radians (hip flexion/extension)
    knee_amplitude = 0.5   # radians (knee flexion)
    ankle_amplitude = 0.15 # radians (ankle dorsi/plantar flexion)
    
    suspended = True
    current_suspension_state = True
    
    gait_mode = True
    selected_actuator_idx = 2  # Default to Left Hip Pitch
    manual_targets = np.zeros(len(actuator_names))

    # Keyboard Callback for Interactive Control
    def key_callback(keycode):
        nonlocal suspended, gait_mode, selected_actuator_idx, manual_targets
        try:
            key_char = chr(keycode).lower()
        except ValueError:
            key_char = None
            
        # Toggle suspension
        if key_char == ' ' or key_char == 's':
            suspended = not suspended
            
        # Toggle control modes
        elif key_char == 'm':
            gait_mode = not gait_mode
            if not gait_mode:
                # Store current commands to prevent sudden jerking
                for i, name in enumerate(actuator_names):
                    manual_targets[i] = data.ctrl[actuator_ids[name]]
            print(f"\n[MODE] Changed to: {'GAIT (AUTOMATIC)' if gait_mode else 'MANUAL (KEYBOARD/GUI)'}")
            
        # Reset manual joint targets to 0
        elif key_char == 'r':
            manual_targets.fill(0.0)
            print("\n[RESET] All manual joint commands reset to 0.0 rad")
            
        # Select joint index 0-9
        elif key_char in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            selected_actuator_idx = int(key_char)
            print(f"\n[SELECT] Selected joint: [{selected_actuator_idx}] {actuator_names[selected_actuator_idx]}")
            
        # Increment selected joint target (']' or UP/RIGHT arrow keys)
        elif key_char == ']' or keycode == 265 or keycode == 262:
            act_name = actuator_names[selected_actuator_idx]
            act_id = actuator_ids[act_name]
            ctrl_range = model.actuator_ctrlrange[act_id]
            manual_targets[selected_actuator_idx] = np.clip(
                manual_targets[selected_actuator_idx] + 0.05,
                ctrl_range[0],
                ctrl_range[1]
            )
            
        # Decrement selected joint target ('[' or DOWN/LEFT arrow keys)
        elif key_char == '[' or keycode == 264 or keycode == 263:
            act_name = actuator_names[selected_actuator_idx]
            act_id = actuator_ids[act_name]
            ctrl_range = model.actuator_ctrlrange[act_id]
            manual_targets[selected_actuator_idx] = np.clip(
                manual_targets[selected_actuator_idx] - 0.05,
                ctrl_range[0],
                ctrl_range[1]
            )

    print("\n" + "="*70)
    print("Lower Limb Exoskeleton Simulation Running!")
    print("General Controls:")
    print(" - Press SPACEBAR or 'S': Toggle pelvis suspension (lock/unlock base)")
    print(" - Press 'M': Toggle between AUTOMATIC GAIT and MANUAL CONTROL")
    print(" - Double-click on any part to select; hold CTRL + Drag mouse to apply force")
    print("\nManual Control Mode (Press 'M' to activate):")
    print(" - Press keys '0' to '9' to select a joint to control:")
    for idx, name in enumerate(actuator_names):
        print(f"   [{idx}] {name.replace('_act','')}")
    print(" - Press UP/DOWN Arrow keys or '[' and ']' to move the selected joint")
    print(" - Press 'R' to reset all manual joint targets to 0")
    print(" - OR: use the built-in 'Control' panel sliders in the MuJoCo GUI")
    print("="*70 + "\n")

    # 3. Launch simulation viewer
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        # Default camera position
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -15
        viewer.cam.distance = 2.6
        viewer.cam.lookat = np.array([0, 0, 1.5])

        # Step loop
        while viewer.is_running():
            step_start = time.time()

            # Handle suspension toggle from key press
            if suspended != current_suspension_state:
                current_suspension_state = suspended
                if weld_id != -1:
                    with viewer.lock():
                        data.eq_active[weld_id] = 1 if current_suspension_state else 0
                print(f"\n[TOGGLE] Pelvis suspension: {'SUSPENDED' if current_suspension_state else 'FREE'}")

            # Coordinated walking gait trajectory generation (Gait Mode)
            if gait_mode:
                t = data.time
                omega = 2.0 * np.pi * frequency
                
                # Left leg phase and targets
                phase_l = omega * t
                left_hip_pitch_target = hip_amplitude * np.sin(phase_l)
                left_knee_pitch_target = knee_amplitude * (np.sin(phase_l - np.pi/2) + 1.0)
                left_ankle_pitch_target = ankle_amplitude * np.sin(phase_l + 0.5)
                left_hip_roll_target = 0.05 * np.cos(phase_l)
                left_hip_yaw_target = 0.0

                # Right leg phase and targets (180 degrees out of phase)
                phase_r = phase_l + np.pi
                right_hip_pitch_target = hip_amplitude * np.sin(phase_r)
                right_knee_pitch_target = knee_amplitude * (np.sin(phase_r - np.pi/2) + 1.0)
                right_ankle_pitch_target = ankle_amplitude * np.sin(phase_r + 0.5)
                right_hip_roll_target = 0.05 * np.cos(phase_r)
                right_hip_yaw_target = 0.0

                # Apply command values to actuators
                data.ctrl[actuator_ids["left_hip_yaw_act"]] = left_hip_yaw_target
                data.ctrl[actuator_ids["left_hip_roll_act"]] = left_hip_roll_target
                data.ctrl[actuator_ids["left_hip_pitch_act"]] = left_hip_pitch_target
                data.ctrl[actuator_ids["left_knee_pitch_act"]] = left_knee_pitch_target
                data.ctrl[actuator_ids["left_ankle_pitch_act"]] = left_ankle_pitch_target

                data.ctrl[actuator_ids["right_hip_yaw_act"]] = right_hip_yaw_target
                data.ctrl[actuator_ids["right_hip_roll_act"]] = right_hip_roll_target
                data.ctrl[actuator_ids["right_hip_pitch_act"]] = right_hip_pitch_target
                data.ctrl[actuator_ids["right_knee_pitch_act"]] = right_knee_pitch_target
                data.ctrl[actuator_ids["right_ankle_pitch_act"]] = right_ankle_pitch_target
            else:
                # Manual Control Mode:
                # 1. First, check if the user is dragging the sliders in the MuJoCo GUI.
                #    If they are, we update manual_targets to stay in sync with the GUI sliders.
                #    (In MuJoCo passive viewer, dragging a GUI slider updates data.ctrl directly).
                # 2. Write manual_targets to data.ctrl to apply our keyboard commands.
                for i, name in enumerate(actuator_names):
                    act_id = actuator_ids[name]
                    # If we are not updating via keyboard in this step, let the GUI slider take priority
                    # by checking if data.ctrl differs from our last manual target.
                    if abs(data.ctrl[act_id] - manual_targets[i]) > 1e-4:
                        manual_targets[i] = data.ctrl[act_id]
                    
                    data.ctrl[act_id] = manual_targets[i]

            # Step simulation
            mujoco.mj_step(model, data)
            viewer.sync()

            # Live status console log
            mode_str = "GAIT (Auto)" if gait_mode else "MANUAL (Key/GUI)"
            susp_str = "SUSPENDED" if current_suspension_state else "FREE"
            if not gait_mode:
                sel_name = actuator_names[selected_actuator_idx].replace('_act','')
                sel_val = manual_targets[selected_actuator_idx]
                sys.stdout.write(
                    f"\rMode: {mode_str} | Pelvis: {susp_str} | Selected: {sel_name}={sel_val:+.2f} rad | Use Arrow keys / [ ]"
                )
            else:
                sys.stdout.write(
                    f"\rMode: {mode_str} | Pelvis: {susp_str} | Time: {data.time:.2f}s | Press 'M' for Manual Mode"
                )
            sys.stdout.flush()

            # Maintain real-time simulation step
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
