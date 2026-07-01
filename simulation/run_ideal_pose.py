import argparse
import json
import os
import sys
import time

import mujoco
import mujoco.viewer


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the exoskeleton scene with an ideal joint pose loaded from JSON."
    )
    parser.add_argument(
        "--pose-file",
        default=None,
        help="Path to a JSON file mapping joint names to target angles in radians."
    )
    return parser.parse_args()


def load_pose_file(pose_path):
    if not os.path.exists(pose_path):
        raise FileNotFoundError(f"Pose file not found: {pose_path}")

    with open(pose_path, "r", encoding="utf-8") as f:
        pose = json.load(f)

    if not isinstance(pose, dict):
        raise ValueError("Pose file must contain a JSON object mapping joint names to angles.")

    return pose


def apply_joint_pose(model, data, pose):
    for joint_name, angle in pose.items():
        if not isinstance(angle, (int, float)):
            raise ValueError(f"Angle for joint '{joint_name}' must be numeric.")

        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id == -1:
            raise ValueError(f"Joint '{joint_name}' not found in the model.")

        data.qpos[joint_id] = float(angle)

        actuator_name = f"{joint_name}_act"
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if actuator_id != -1:
            data.ctrl[actuator_id] = float(angle)

    mujoco.mj_forward(model, data)


def main():
    args = parse_args()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, "scene.xml")

    if not os.path.exists(xml_path):
        print(f"Error: scene file not found: {xml_path}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    pose_file = args.pose_file or os.path.join(current_dir, "ideal_pose.json")
    if os.path.exists(pose_file):
        try:
            pose = load_pose_file(pose_file)
            apply_joint_pose(model, data, pose)
            print(f"Loaded ideal pose from: {pose_file}")
            for joint_name, angle in pose.items():
                print(f"  {joint_name}: {angle:+.3f} rad")
        except Exception as exc:
            print(f"Warning: failed to load ideal pose '{pose_file}': {exc}")
            print("Continuing with the default initial pose.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)


if __name__ == "__main__":
    main()
