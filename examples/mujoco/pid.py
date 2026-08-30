import mujoco
import mediapy as media
import matplotlib.pyplot as plt
from pathlib import Path

KP = 20.0
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
    if control > 0.1:
        control = 0.1
    elif control < -0.1:
        control = -0.1

    return -control

xml_path = Path("examples/mujoco/simplified_balance_bot.xml")
xml_path_str = str(xml_path.resolve())

model = mujoco.MjModel.from_xml_path(xml_path_str)
data  = mujoco.MjData(model)

duration = 4
framerate = 60
frames = []
tilt = []
controls = []
times = []
model.opt.timestep = 0.01
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    prev_time = 0.0
    while data.time < duration:
        mujoco.mj_step(model, data)
        gyrodata = data.sensor('imu_gyro').data.copy()
        times.append(data.time)
        tilt_speed = gyrodata[0] # radians per second
        if tilt_speed > 3:
            tilt_speed = 3
        elif tilt_speed < -3:
            tilt_speed = -3
        tilt.append(tilt_speed)
        wheel_speed = compute_control_law(tilt_speed)
        controls.append(wheel_speed)
        data.ctrl[0] = wheel_speed
        data.ctrl[1] = wheel_speed
        prev_time = data.time
        with open("pid_log.txt", "a") as log_file:
            log_file.write(f"Time: {data.time:.2f} s, Tilt Speed: {data.sensor('imu_gyro').data[0]:.4f} rad/s, Control Signal: {wheel_speed:.4f}\n")
        if len(frames) < data.time * framerate:
            renderer.update_scene(data)
            frames.append(renderer.render())
media.write_video('pid.mp4', frames, fps=framerate)
plt.plot(times, tilt, label='Tilt Angle')
plt.plot(times, controls, label='Control Signal')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.legend()
plt.show()