import numpy as np
import genesis as gs
from google import genai
import time 
import json

########################## init ##########################
gs.init(backend=gs.cpu)

# Initialize client for GenAI
client = genai.Client(api_key="AIzaSyCUdzu7AVnOfgyUsAXbW9YxzvDqmhAfdJI")


def select_urdf_entities(scene_description:dict, task_goal:str, entity_name:str, candidates:list):
    """
    Select URDF entities based on the task goal using GenAI.
    Args:
        scene: Dict or structured summary of the simulation scene.
        task_goal: String describing the task goal (e.g., 'pick_and_place')
        entity_name: String name of the entity to select the URDF for
        candidates: List of candidate URDF entities (each a dict with fidelity and features).
    Returns:
        Selected URDF name
    """

    # Serialize scene and candidates to structured text
    scene_str = json.dumps(scene_description, indent=2)
    candidates_str = json.dumps(candidates, indent=2)

    # TODO: We probably need to iterate on this prompt, what are the things the LLM should balance in its selection?
    # Does it matter if we have some collisions? Or do we need to be as accurate as possible?
    # What task_goal options should we support? etc.
    prompt = f"""
    You are an expert in robotics and simulation tasked with configuring a simulation 
    environment for efficient and effective robot planning. A URDF file describes
    the physical and visual properties of objects in a simulation and are used for rendering and
    for collision checking when planning and simlating.
    Given the task goal: "{task_goal}" and scene description:
    {scene_str}
    Select the most suitable URDF representation for the entity:
    {entity_name} 
    from the available candidates below:
    {candidates_str}
    The goal is to complete the task accurately and quickly.
    Select URDF representations that provide enough fidelity for successful planning and execution,
    while minimizing computational cost and simulation time. Use simpler models where they are sufficient, 
    and higher-fidelity models only where necessary.

    Provide a single URDF selection entities and a brief justification for your selection. 
    Also tell provide what information in the scene is most important for your decision.
    """

    print(" > Prompt to Gemini >>>>")
    print(prompt)

    # Query Gemini
    response = client.models.generate_content(
        model="gemini-2.5-flash",
        contents=prompt
    )

    print(" > Gemini Response >>>>")
    print(response.text)

    # Parse and return results
    # There should only be one selected entity but make sure that we capture all in case multiple are provided
    selected_entities = []
    for entity in candidates:
        if entity['urdf_name'] in response.text:
            selected_entities.append(entity)

    return selected_entities


def summarize_scene(scene) -> str:
    """
    Converts a Genesis scene object into a structured dictionary that summarizes
    key properties of each entity for LLM input.

    Args:
        scene: Genesis scene object

    Returns:
        dict: keys are entity names or IDs, values are dictionaries of properties
    """
    scene_summary = {}

    for i, entity in enumerate(scene.entities):
        entity_info = {}

        # Use short name if it exsits, otherwise use class name with index
        name_attr = getattr(entity, 'name', None)
        name = name_attr if name_attr else f"{entity.__class__.__name__}_{i}"

        def safe_to_tuple(val):
            # Handles torch tensors
            try:
                import torch
                if isinstance(val, torch.Tensor):
                    # If it's a single value, use item; otherwise, use tolist()
                    if val.dim() == 0:
                        return val.item()
                    else:
                        return tuple(val.tolist())
            except ImportError:
                pass
            # Add numpy handling here if needed
            return tuple(val) if isinstance(val, (list, tuple)) else val


        # Basic properties
        entity_info['type'] = entity.morph.__class__.__name__ if hasattr(entity, 'morph') else 'Unknown'
        entity_info['position'] = safe_to_tuple(entity.get_pos()) if hasattr(entity, 'get_pos') else None
        entity_info['orientation'] = safe_to_tuple(entity.get_quat()) if hasattr(entity, 'get_quat') else None

        # If available, add size or scale info
        # This depends on morph type and may require custom extraction per morph type
        if hasattr(entity.morph, 'size'):
            entity_info['size'] = safe_to_tuple(entity.morph.size)
        elif hasattr(entity.morph, 'scale'):
            entity_info['scale'] = safe_to_tuple(entity.morph.scale) if isinstance(entity.morph.scale, (list, tuple)) else entity.morph.scale

        # Optionally add URDF filename if applicable
        if hasattr(entity.morph, 'file') and entity.morph.file:
            entity_info['file'] = entity.morph.file

        scene_summary[name] = entity_info
        print(f" > Entity Summary >>>> \n{name}: \n{entity_info}")
    # print("> Scene Summary >>>>")
    # print(scene_summary)

    return scene_summary


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
plane.name = "plane"

# Genesis Box object
genesis_cube = scene.add_entity(
    gs.morphs.Box(
        size = (0.04, 0.04, 0.04), # meters
        pos  = (0.65, 0.0, 0.02),
    )
)
genesis_cube.name = "genesis_cube"

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

# Add an entity that uses the URDF that was used as the input to MorphIt
pink_box_urdf = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/assets/urdfs/pink_box/pink_box.urdf',
        pos  = (0.65, 0.0, 0.02),
        ),
)

# Add all representations of the box to a list for easy access
# TODO: Commented out for now until we figure out how to switch between entities after scene.build()
# box_entitites = []
# box_entitites.append(genesis_cube)
# box_entitites.append(pink_box_urdf20)
# box_entitites.append(pink_box_urdf10)
# box_entitites.append(pink_box_urdf5)
# box_entitites.append(pink_box_urdf)

print(f"Scene entities: {scene.entities}")


# Dictionary description of the candidate representations of entities
# Instead of passing the URDFs directly to the LLM, we provide these descriptions so there may be 
# issues with this implementation
cube_candidates = [
    {
        # Use the default representation of the cube
        'urdf_name': 'none',
        # Name of the entity object that this representation corresponds to
        'entity_name': 'genesis_cube',
        'fidelity': 'none',
        'features': 'simple box primitive, no spheres, default Genesis object'
    },
    {
        'urdf_name': 'pink_box_urdf20',
        # Name of the entity object that this representation corresponds to
        'entity_name': 'genesis_cube',
        'fidelity': 'high',
        'features': 'URDF with 20 spheres, articulated joints, high accuracy'
    },
    {
        'urdf_name': 'pink_box_urdf10',
        # Name of the entity object that this representation corresponds to
        'entity_name': 'genesis_cube',
        'fidelity': 'medium',
        'features': 'URDF with 10 spheres, basic articulation, moderate accuracy'
    },
    {
        'urdf_name': 'pink_box_urdf5',
        # Name of the entity object that this representation corresponds to
        'entity_name': 'genesis_cube',
        'fidelity': 'low',
        'features': 'URDF with 5 spheres, limited articulation, reduced accuracy but faster simulation'
    },
    {
        'urdf_name': 'pink_box_urdf',
        # Name of the entity object that this representation corresponds to
        'entity_name': 'genesis_cube',
        'fidelity': 'none',
        'features': 'default pink box URDF, no spheres, used to generate the other URDFs'
    },
]



# Robotic arm (not created with MorphIt spheres)
franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)
franka.name = "franka"
########################## build ##########################
# TODO: In this scenario, ALL the URDFs have been used to create entities so the scene contains information for 
# each of them (so there's 5 entity objects that all represent the same cube in the scene). 
# This may not be necessary in practice. We probably only want 1 entity for the cube and to switch between the URDFs that 
# are being used for that entity.
scene.build()

# TODO: At this point, entities cannot be added to the scene, it is unclear if they can
# be enabled/disabled dynamically to allow us to switch between different representations 

# TODO: Add step to identify which URDFs should be active for initial planning and switch the entity to 
# whatever URDF is selected
scene_summary_dict = summarize_scene(scene)

time_start = time.time()
selected = select_urdf_entities(scene_summary_dict, task_goal='pick_and_place', entity_name='genesis_cube', candidates=cube_candidates)
time_end = time.time()
response_time = time_end - time_start

print(f" > Selected entities for planning: {selected}")
print(f" > Entity selection response time: {response_time}")

########################## control ##########################
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

# TODO: Add step to identify which URDFs should be active for next phase
# and switch to those URDFs
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

# TODO: Add step to identify which URDFs should be active for next phase
# and switch to those URDFs
# scene.entities = select_urdf_entities(scene, task_goal='pick_and_place', candidates=box_entitites)

# grasp
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)

for i in range(100):
    scene.step()

# TODO: Add step to identify which URDFs should be active for next phase
# and switch to those URDFs
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