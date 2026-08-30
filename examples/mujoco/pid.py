import mujoco
import matplotlib.pyplot as plt
from pathlib import Path

RAD_TO_DEG = 180.0 / 3.141592653589793

KP = 1.0
KI = 0.0
KD = 0.0

def compute_control_law(tilt_speed: float) -> float:

    # tilt_speed is in radians per second
    # we need to convert it to meters per second for the wheel speed
    # one revolution is 6.28 radians, and the wheel circumference is 0.05 meters
    # so radians per second * (0.05 / 6.28) = meters per second
    error = tilt_speed * (0.05 / 6.28)

    # proportional term
    p_term = KP * error

    # integral term

    # derivative term

    control = p_term

    return -control

xml_path = Path("examples/mujoco/simplified_balance_bot.xml")
xml_path_str = str(xml_path.resolve())

model = mujoco.MjModel.from_xml_path(xml_path_str)
data  = mujoco.MjData(model)

duration = 10.0
framerate = 60
frames = []
tilt = []
times = []
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    prev_time = 0.0
    while data.time < duration:
        mujoco.mj_step(model, data)
        gyrodata = data.sensor('imu_gyro').data.copy()
        times.append(data.time)
        tilt_speed = gyrodata[0] # radians per second
        tilt.append(tilt_speed)
        wheel_speed = compute_control_law(tilt_speed)
        data.ctrl[0] = wheel_speed
        data.ctrl[1] = wheel_speed
        prev_time = data.time
        if len(frames) < data.time * framerate:
            renderer.update_scene(data)
            frames.append(renderer.render())
plt.plot(times, tilt, label='Tilt Angle')
plt.show()