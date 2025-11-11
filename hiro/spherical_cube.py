import argparse
import os

import numpy as np

import genesis as gs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vis", action="store_true", default=True)
    parser.add_argument("-c", "--cpu", action="store_true", default=False)
    args = parser.parse_args()

    ########################## init ##########################
    gs.init(backend=gs.cpu if args.cpu else gs.gpu, logging_level="debug")

    ########################## create a scene ##########################

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            substeps=10,
            gravity=(0, 0, -9.8),
        ),
        pbd_options=gs.options.PBDOptions(
            max_stretch_solver_iterations=0,
            max_bending_solver_iterations=0,
            max_volume_solver_iterations=0,
            max_density_solver_iterations=0,
            max_viscosity_solver_iterations=0,
            # particle_size=2*1e-2,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(2, 2, 1.5),
            camera_lookat=(0, 0, 0.5),
            camera_up=(0, 0, 1),
        ),
        show_viewer=args.vis,
    )

    mat_elastic = gs.materials.PBD.Particle()

    ########################## entities ##########################
    plane = scene.add_entity(
        gs.morphs.Plane(),
    )

    cube = scene.add_entity(
        material=mat_elastic,
        morph=gs.morphs.Box(
            size=(0.04, 0.04, 0.04),
            pos=(0.65, 0.0, 0.02),
            # euler=(0, 45, 0),
        )
    )

    scene.build()

    horizon = 1000 if "PYTEST_VERSION" not in os.environ else 5
    # forward pass
    for i in range(horizon):
        scene.step()


if __name__ == "__main__":
    main()
