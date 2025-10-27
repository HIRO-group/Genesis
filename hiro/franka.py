import genesis as gs
import time
gs.init(backend=gs.cpu)

scene = gs.Scene(show_viewer=True)
plane = scene.add_entity(gs.morphs.Plane())

for i in range(1, 3):
    pink_box_spherical5 = scene.add_entity(
        gs.morphs.URDF(
            file='../MorphIt-1/results/urdfs/pink_box_5.urdf',
            pos = (0, i/2, 0),
            ),
    )

for i in range(1, 3):
    pink_box_spherical10 = scene.add_entity(
        gs.morphs.URDF(
            file='../MorphIt-1/results/urdfs/pink_box_10.urdf',
            pos = (i/2, 0, 0),
            ),
    )

for i in range(1, 3):
    pink_box_spherical20 = scene.add_entity(
        gs.morphs.URDF(
            file='../MorphIt-1/results/urdfs/pink_box_20.urdf',
            pos = (-i/2, 0, 0),
            ),
    )


for i in range(1, 3):
    pink_box = scene.add_entity(
        gs.morphs.URDF(
            file='../assets/urdfs/pink_box/pink_box.urdf',
            pos = (0, -i/2, 0),
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