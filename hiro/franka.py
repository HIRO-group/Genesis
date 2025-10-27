import genesis as gs
import time
gs.init(backend=gs.cpu)

scene = gs.Scene(show_viewer=True)
plane = scene.add_entity(gs.morphs.Plane())

for i in range(4):
    pink_box_spherical5 = scene.add_entity(
        gs.morphs.URDF(
            file='/home/ava/Research/Codes/fall25/MorphIt-1/results/urdfs/pink_box_5.urdf',
            pos = (0, 0, 0),
            ),
    )

for i in range(4):
    pink_box_spherical10 = scene.add_entity(
        gs.morphs.URDF(
            file='/home/ava/Research/Codes/fall25/MorphIt-1/results/urdfs/pink_box_10.urdf',
            pos = (0.5, 0, 0),
            ),
    )



start = time.time()
scene.build()
end = time.time()

print("Build time:", (end - start))

# 4 x sph5: 41.2, 15.3, 15.7 15.3 15.8
# 4 x sph10: 39.95 15.97 15.63 12.26 15.79
