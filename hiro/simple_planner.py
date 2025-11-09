import numpy as np
import genesis as gs

########################## init ##########################
gs.init(backend=gs.cpu)

########################## create a scene ##########################
scene = gs.Scene(
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (3, -1, 1.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 30,
        max_FPS       = 60,
    ),
    show_viewer = True,
)

########################## entities ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)

# Genesis Box object
genesis_cube = scene.add_entity(
    gs.morphs.Box(
        size = (0.04, 0.04, 0.04), # meters
        pos  = (0.65, 0.0, 0.02),
    )
)

# Define different cube entities with URDFs that were generated from MorphIt
pink_box_urdf20 = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/src/results/urdfs/pink_box_20.urdf',
        pos  = (0.65, 0.0, 0.02),
   ),
)

pink_box_urdf10 = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/src/results/urdfs/pink_box_10.urdf',
        pos  = (0.65, 0.0, 0.02),
        ),
)

pink_box_urdf5 = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/src/results/urdfs/pink_box_5.urdf',
        pos  = (0.65, 0.0, 0.02),
        ),
)

pink_box_urdf = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/assets/urdfs/pink_box/pink_box.urdf',
        pos  = (0.65, 0.0, 0.02),
        ),
)

# Add all representations of the box to a list for easy access
box_entitites = []
box_entitites.append(genesis_cube)
box_entitites.append(pink_box_urdf20)
box_entitites.append(pink_box_urdf10)
box_entitites.append(pink_box_urdf5)
box_entitites.append(pink_box_urdf)

print(f"Scene entities: {scene.entities}")

# Robotic arm (not created with MorphIt spheres)
franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)
########################## build ##########################
scene.build()

# TODO: At this point, entities cannot be added to the scene, it is unclear if they can
# be enabled/disabled dynamically to allow us to switch between different representations 


########################## control ##########################

# TODO: Add step to identify which URDFs should be active for initial planning
# scene.entities = select_urdf_entities(scene, task_goal='pick_and_place', candidates=box_entitites)

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

# set control gains
# Note: the following values are tuned for achieving best behavior with Franka
# Typically, each new robot would have a different set of parameters.
# Sometimes high-quality URDF or XML file would also provide this and will be parsed.
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
qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.25]),
    quat = np.array([0, 1, 0, 0]),
)
# gripper open pos
qpos[-2:] = 0.04
path = franka.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2s duration
)
# execute the planned path
for waypoint in path:
    franka.control_dofs_position(waypoint)
    scene.step()

# allow robot to reach the last waypoint
for i in range(100):
    scene.step()

# TODO: Add step to identify which URDFs should be active for initial planning
# scene.entities = select_urdf_entities(scene, task_goal='pick_and_place', candidates=box_entitites)

# reach
qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.130]),
    quat = np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(100):
    scene.step()

# TODO: Add step to identify which URDFs should be active for initial planning
# scene.entities = select_urdf_entities(scene, task_goal='pick_and_place', candidates=box_entitites)

# grasp
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)

for i in range(100):
    scene.step()

# TODO: Add step to identify which URDFs should be active for initial planning
# scene.entities = select_urdf_entities(scene, task_goal='pick_and_place', candidates=box_entitites)

# lift
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.28]),
    quat=np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(200):
    scene.step()