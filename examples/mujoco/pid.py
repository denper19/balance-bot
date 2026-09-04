import mujoco
import mediapy as media
import matplotlib.pyplot as plt
from pathlib import Path

def compute_pitch_control(ref: float, pitch_angle: float, pitch_rate: float) -> float:

    KP = 1.5
    KI = 0.0
    KD = 0.2

    # proportional term
    p_term = KP * (pitch_angle - ref)

    # derivative term
    d_term = KD * pitch_rate

    control = p_term + d_term

    return control

def compute_position_control(position_rate: float, position: float, pitch: float, pitch_rate: float) -> float:
    
    KP = 2.0
    KI = 0.0
    KD = 0.3

    # proportional term
    p_term = KP * position

    # derivative term
    d_term = KD * position_rate

    tilt = -(p_term + d_term)
    tilt = max(min(tilt, 0.08), -0.08)  # limit tilt to [-0.5, 0.5] radians

    control = compute_pitch_control(tilt,pitch,pitch_rate)
    return control

xml_path = Path("examples/mujoco/simplified_balance_bot.xml")
xml_path_str = str(xml_path.resolve())

model = mujoco.MjModel.from_xml_path(xml_path_str)
data  = mujoco.MjData(model)

# compute the volume of all the geoms in the model
for i in range(model.ngeom):
    volume = 0.0
    if model.geom(i).type == mujoco.mjtGeom.mjGEOM_BOX:
        size = model.geom(i).size
        volume = 8 * size[0] * size[1] * size[2]
    elif model.geom(i).type == mujoco.mjtGeom.mjGEOM_SPHERE:
        radius = model.geom(i).size[0]
        volume = (4/3) * 3.14159 * radius**3
    elif model.geom(i).type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        radius = model.geom(i).size[0]
        length = 2 * model.geom(i).size[1]
        volume = 3.14159 * radius**2 * length
    print(f"Geom: {model.geom(i).name}, Mass: {volume * 1000:.4f} kg")

duration = 15
framerate = 60
frames = []
tilt = []
position = []
controls = []
times = []
pitch_angle = 0.0
position = 0.0
model.opt.timestep = 0.002
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    prev_time = 0.0
    while data.time < duration:
        mujoco.mj_step(model, data)
        gyrodata = data.sensor('imu_gyro').data.copy()
        times.append(data.time)

        tilt_speed = gyrodata[0] # radians per second
        speed = gyrodata[1]

        wheel_l = data.sensor('wheel_l_pos').data[0]
        wheel_r = data.sensor('wheel_r_pos').data[0]
        x = 0.5 * (wheel_l + wheel_r) * 0.05      # radius = 0.05

        vel_l = data.sensor('wheel_l_vel').data[0]
        vel_r = data.sensor('wheel_r_vel').data[0]
        x_dot = 0.5 * (vel_l + vel_r) * 0.05

        pitch_angle += tilt_speed * (data.time - prev_time) # integrate to get angle
        tilt.append(pitch_angle)

        wheel_speed = compute_position_control(x_dot, x, pitch_angle, gyrodata[0])

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