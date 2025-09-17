import pychrono as pc
import numpy as np
import matplotlib.pyplot as plt

# Initialize PyChrono environment
pc.init()

# Set up solver configuration
solver_config = pc.SolverConfig()
solver_config.set_tolerance(1e-6)
pc.solve_config(solver_config)

# Create the Jeffcott rotor beam
beam = pc.iga_beam(pc.IGABeamProperties(length=4.0, cross_section=pc.CROSS_SECTION_SQUARE(0.2), density=8000, young_modulus=210e9))

# Create and attach the flywheel
flywheel = pc.rigid_body(pc.MASS(1.0), pc.VECTOR3D(0, 0, 0))
beam.rigid_body.set_center_of_mass(pc.VECTOR3D(2, 0, 0))
pc.attach(flywheel, beam.rigid_body, pc.VECTOR3D(0, 0, 0))

# Create the rotational motor
motor = pc.motor_rotational(pc.MASS(0.01), pc.VECTOR3D(0, 0, 0))
beam.rigid_body.set_joint(motor, pc.VECTOR3D(0, 0, 0))
motor.set_inertia(pc.MATRIX3x3(0.01, 0, 0, 0, 0, 0, 0, 0, 0), pc.VECTOR3D(0, 0, 0))
motor.set_force(pc.VECTOR3D(0, 0, 0), pc.VECTOR3D(0, 10, 0))
motor.set_ang_vel(pc.VECTOR3D(0, 100, 0))

# Apply gravity
pc.gravity(pc.VECTOR3D(0, -9.81, 0))

# Set up visualization
vis = pc.Irrlicht()
vis.set_background_color(pc.Vec3(0.0, 0.0, 0.0))
vis.set_camera(pc.Cam(10, 10, 10, 0, 0, -1, 0, 0, 0))
vis.add_light(pc.Vec3(0, 0, 10))

# Add visualization for the beam
beam.add_mesh_points(pc.MeshPoints(0.01, 0.01, 0.01), pc.Colors(1, 1, 1), pc.SHADER_FILLED)
vis.render()

# Run the simulation
t = 0.0
h = 0.001
n = 10000
for i in range(n):
    pc.step()
    t += h

plt.show()