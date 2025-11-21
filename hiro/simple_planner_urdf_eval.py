
"""
Script for evaluating the computational cost of different URDF representations
(Work in progress)
"""

import numpy as np
import genesis as gs
import time 

# Init
gs.init(backend=gs.cpu)


# Dictionary of each urdf to evaluate
urdf_entities = [
    {
        'urdf_name': 'none',
        'entity_name': 'genesis_cube',
        'total_simulation_time': 0.0,
        'total_planning_time': 0.0,
        'urdf_path': None
    },
    {
        'urdf_name': 'pink_box_20',
        'entity_name': 'genesis_cube',
        'total_simulation_time': 0.0,
        'total_planning_time': 0.0,
        'urdf_path': '../MorphIt-1/src/results/urdfs/pink_box_20.urdf'
    },
    {
        'urdf_name': 'pink_box_urdf10',
        'entity_name': 'genesis_cube',
        'total_simulation_time': 0.0,
        'total_planning_time': 0.0,
        'urdf_path': '../MorphIt-1/src/results/urdfs/pink_box_10.urdf'
    },
    {
        'urdf_name': 'pink_box_urdf5',
        'entity_name': 'genesis_cube',
        'total_simulation_time': 0.0,
        'total_planning_time': 0.0,
        'urdf_path': '../MorphIt-1/src/results/urdfs/pink_box_5.urdf'
    },
    {
        'urdf_name': 'pink_box_urdf',
        'entity_name': 'genesis_cube',
        'total_simulation_time': 0.0,
        'total_planning_time': 0.0,
        'urdf_path': '../MorphIt-1/assets/urdfs/pink_box/pink_box.urdf'
    },
]


# Main evaluation loop
for urdf_description in urdf_entities:

    # Initialize scene
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
        show_viewer = False,
    )

    # Add ground plane
    plane = scene.add_entity(
        gs.morphs.Plane(),
    )

    # Add cube entity using the URDF description

    if urdf_description['urdf_path'] is not None:
        entity = scene.add_entity(
            gs.morphs.URDF(
                file=urdf_description['urdf_path'],
                pos  = (0.65, 0.0, 0.02),
            ),
        )
        entity.name = urdf_description['entity_name'] + "_" + urdf_description['urdf_name']
        urdf_description['entity'] = entity
    else:
        # If path not defined, use Genesis cube object
        genesis_cube = scene.add_entity(
            gs.morphs.Box(
                size = (0.04, 0.04, 0.04), # meters
                pos  = (0.65, 0.0, 0.02),
            )
        )

    # Add robotic arm (not created with MorphIt spheres)
    franka = scene.add_entity(
        gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
    )

    # TODO: Should this go before or after scene.build()?
    sim_start_time = time.time()

    # Build the scene
    scene.build()

    # Control
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

    # reach
    qpos = franka.inverse_kinematics(
        link = end_effector,
        pos  = np.array([0.65, 0.0, 0.130]),
        quat = np.array([0, 1, 0, 0]),
    )
    franka.control_dofs_position(qpos[:-2], motors_dof)
    for i in range(100):
        scene.step()


    # grasp
    franka.control_dofs_position(qpos[:-2], motors_dof)
    franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)

    for i in range(100):
        scene.step()

    # lift
    qpos = franka.inverse_kinematics(
        link=end_effector,
        pos=np.array([0.65, 0.0, 0.28]),
        quat=np.array([0, 1, 0, 0]),
    )
    franka.control_dofs_position(qpos[:-2], motors_dof)
    for i in range(200):
        scene.step()

    end_state = scene.get_state()

    print(f"End state: {end_state.__repr__()}")

    sim_end_time = time.time()

    urdf_description['total_simulation_time'] = sim_end_time - sim_start_time

    # Clean up
    print(f"Completed evaluation for URDF: {urdf_description['urdf_name']}")
    del scene

    # TODO: What are "environments" in Genesis?

# Print results
for urdf_description in urdf_entities:
    print(f"URDF: {urdf_description['urdf_name']}, Total Simulation Time: {urdf_description['total_simulation_time']:.4f} seconds")