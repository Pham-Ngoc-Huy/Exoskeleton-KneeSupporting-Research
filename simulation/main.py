import mujoco
import mujoco.viewer
import time

# Load model
model = mujoco.MjModel.from_xml_path("knee_joint_mechanism.xml")
data = mujoco.MjData(model)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # Run one simulation step
        mujoco.mj_step(model, data)

        # Update viewer
        viewer.sync()

        time.sleep(0.001)