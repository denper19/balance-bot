import mujoco
import mediapy as media
import pathlib
import matplotlib.pyplot as plt
import os

xml_path = pathlib.Path("examples/mujoco/pendulum.xml")

model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
data  = mujoco.MjData(model)

#mujoco.mj_resetData(model, data)
#with mujoco.Renderer(model) as renderer:
#    mujoco.mj_forward(model, data)
#    renderer.update_scene(data)
#    plt.imshow(renderer.render())
#    plt.show()

duration = 20.0
framerate = 60
sensordata = []
times = []
frames = []
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    while data.time < duration:
        mujoco.mj_step(model, data)
        if len(frames) < data.time * framerate:
            renderer.update_scene(data, camera="closeup")
            frames.append(renderer.render())
            sensordata.append(data.sensor('tip_pos').data.copy())
            times.append(data.time)

print(sensordata[0])
media.write_video('output.mp4', frames, fps=60)

x = [s[0] for s in sensordata]
y = [s[1] for s in sensordata]
plt.plot(x, y)
plt.show()