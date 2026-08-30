import mujoco
import matplotlib.pyplot as plt
from pathlib import Path

TO_DEG = 180.0 / 3.141592653589793

xml_path = Path("examples/mujoco/simplified_balance_bot.xml")
xml_path_str = str(xml_path.resolve())

model = mujoco.MjModel.from_xml_path(xml_path_str)
data  = mujoco.MjData(model)

duration = 10.0
framerate = 60
frames = []
gyrodata = []
acceldata = []
tilt = []
times = []
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    while data.time < duration:
        mujoco.mj_step(model, data)
        gyrodata.append(data.sensor('imu_gyro').data.copy())
        acceldata.append(data.sensor('imu_acc').data.copy())
        times.append(data.time)
        if len(frames) < data.time * framerate:
            renderer.update_scene(data)
            frames.append(renderer.render())
prev_time = times[0]
for i, g in enumerate(gyrodata):
    tilt_angle = g[0] * (times[i] - prev_time) * TO_DEG
    tilt.append(tilt_angle)
    prev_time = times[i]
plt.plot(times, tilt, label='Tilt Angle')
plt.show()