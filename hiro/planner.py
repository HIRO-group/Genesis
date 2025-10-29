import numpy as np
import genesis as gs
import time

########################## init ##########################
gs.init(seed=0, backend=gs.gpu)


########################## create a scene ##########################
scene = gs.Scene(
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (3, -2, 1.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 30,
        max_FPS       = 60,
    ),
    show_viewer = True,
)

cam = scene.add_camera(
    res    = (1280, 960),
    pos    = (3, -2, 1.5),
    lookat = (0, 0, 0.5),
    fov    = 30,
    GUI    = False,
)


########################## entities ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)

franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

box = scene.add_entity(
    gs.morphs.URDF(
    file='../MorphIt-1/results/urdfs/pink_box_10.urdf',
    pos = (0.5, 0, 0),
    fixed = True,
    ),
)

scene.build()

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
)


# get the end-effector link
end_effector = franka.get_link('hand')

# move to pre-grasp pose
# print("HERE")

qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.25]),
    quat = np.array([0, 1, 0, 0]),
)
# gripper open pos
qpos[-2:] = 0.04


# avg_time = []
# for _ in range (10):
#     start_time = time.time()
path = franka.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2s duration
)
#     end_time = time.time()
#     avg_time.append(end_time - start_time)


# avg_time = np.array(avg_time)
# mean_time = np.mean(avg_time)
# median_time = np.median(avg_time)
# std_time = np.std(avg_time, ddof=1)
# print(f"Average: {mean_time:.4f} s")
# print(f"Median:  {median_time:.4f} s")
# print(f"Std Dev: {std_time:.4f} s")

# 20 spheres
# Average: 3.7786 s
# Median:  3.8106 s
# Std Dev: 0.1954 s

# 10 spheres
# Average: 3.3805 s
# Median:  3.3552 s
# Std Dev: 0.2343 s

# 5 spheres
# Average: 3.2832 s
# Median:  3.2962 s
# Std Dev: 0.1782 s



# execute the planned path
# cam.start_recording()

for waypoint in path:
    franka.control_dofs_position(waypoint)
    scene.step()

    cam.render()

# cam.stop_recording(save_to_filename='video10.mp4', fps=60)