import mujoco
import mediapy as media
import matplotlib.pyplot as plt
from pathlib import Path

KP = 1.0
KI = 0.0
KD = 0.1

def compute_control_law(pitch_rate: float, pitch: float) -> float:


    # proportional term
    p_term = KP * pitch

    # integral term

    # derivative term
    d_term = KD * pitch_rate

    control = p_term + d_term
#   if control > 10:
#       control = 10
#   elif control < -10:
#       control = -10

    return control

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
pitch_angle = 0.0
model.opt.timestep = 0.002
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    prev_time = 0.0
    while data.time < duration:
        mujoco.mj_step(model, data)
        gyrodata = data.sensor('imu_gyro').data.copy()
        times.append(data.time)
        tilt_speed = gyrodata[0] # radians per second
        pitch_angle += tilt_speed * (data.time - prev_time) # integrate to get angle
        tilt.append(pitch_angle)
        wheel_speed = compute_control_law(gyrodata[0], pitch_angle)
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