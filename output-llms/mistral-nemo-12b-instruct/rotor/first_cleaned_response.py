import pychrono as pc
import numpy as np


pc.set_log_level(pc.LogLevel.INFO)
sim = pc.Simulation()
scene = pc.Scene()
sim.set_active_scene(scene)


beam = pc.IGABeam(
    name="beam",
    points=np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]]),
    cross_sections=[pc.CrossSection.rectangular(0.1, 0.05)],
    material=pc.Material(YoungModulus=1e6, PoissonRatio=0.3, Density=1000),
)
scene.add(beam)


flywheel = pc.Sphere(
    name="flywheel",
    radius=0.2,
    material=pc.Material(YoungModulus=1e6, PoissonRatio=0.3, Density=1000),
    pos=[1.5, 0, 0],
)
beam.attach(flywheel, [1.5, 0, 0], [0, 0, 0])


motor = pc.Motor(
    name="motor",
    pos=[0, 0, 0],
    axis=[1, 0, 0],
    max_torque=100,
    target_omega=10,
)
scene.add(motor)
motor.connect(beam, [0, 0, 0], [1, 0, 0])


beam.set_default_parameters(
    gravity=[0, -9.81, 0],
    time_step=0.01,
    max_steps=1000,
)


fem_vis = pc.FEMVisualizer(beam)
scene.add(fem_vis)


irrlicht = pc.IrrlichtVisualizer()
scene.add(irrlicht)


sim.run()


sim.save("jeffcott_rotor.pcd")