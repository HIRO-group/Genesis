import genesis as gs
import time
import numpy as np
gs.init(backend=gs.cpu)

scene = gs.Scene(show_viewer=True,
                 viewer_options=gs.options.ViewerOptions(
                     camera_pos    = (2, -2, 2),
                     camera_lookat = (0.0, 0.0, 0.5),
                     camera_fov    = 30,
                     max_FPS       = 60,
                    )
)

plane = scene.add_entity(gs.morphs.Plane())
cam = scene.add_camera(
    res    = (1280, 960),
    pos    = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov    = 30,
    GUI    = True
)

pink_box_spherical5 = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/src/results/urdfs/pink_box_5.urdf',
        pos = (0, 1/2, 0),
        ),
)

pink_box_spherical10 = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/src/results/urdfs/pink_box_10.urdf',
        pos = (1/2, 0, 0),
        ),
)

pink_box_spherical20 = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/src/results/urdfs/pink_box_20.urdf',
        pos = (-1/2, 0, 0),
        ),
)

# Input URDF that MorphIt used to generate the other URDFs
pink_box = scene.add_entity(
    gs.morphs.URDF(
        file='../MorphIt-1/assets/urdfs/pink_box/pink_box.urdf',
        pos = (0, -1/2, 0),
        ),
)

start = time.time()
scene.build()
# end = time.time()

# print("Build time:", (end - start))

# 4 x sph5: 41.2, 15.3, 15.7 15.3 15.8
# 4 x sph10: 39.95 15.97 15.63 12.26 15.79

for i in range(1000):
    scene.step()

    if i % 100 == 0:
        cam.set_pose(
            pos    = (3.0 * np.sin(i / 60), 3.0 * np.cos(i / 60), 2.5),
            lookat = (0, 0, 0.5),
        )
        print(f">>>>> pink_box20: \n{pink_box_spherical20.__repr__()}")
        cam.render(segmentation=True, depth=True, rgb=True)