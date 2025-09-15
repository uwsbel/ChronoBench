import pychrono as pc
import numpy as np
import matplotlib.pyplot as plt

# Initialize PyChrono environment
pc.init()

# Create a coordinate system
s = pc.SimulationSetup()
s.set_gd(1, 1, 1)

# Create the beam
beam = pc.IGABeam(s, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
L = 4  # Length of the beam
A = 0.2  # Cross-sectional area
I_beam = 1/12 * A * L**4  # Moment of inertia of the beam
beam.set_properties(
    length=L,
    cross_section_area=A,
    material_name="Steel",
    young_modulus=200000,
    poisson_ratio=0.2,
    density=7850,
    moment_inertia=I_beam,
    shear_inertia=0.1*A*L**3
)

# Attach the flywheel
flywheel = pc.DiskFlywheel(beam, beam.get_point(beam.size))
R_fly = 0.5  # Radius of the flywheel
thickness_fly = 0.1  # Thickness of the flywheel
density_fly = 7850  # Density of the flywheel
mass_fly = (np.pi * R_fly**2 * thickness_fly) * density_fly
flywheel.set_properties(
    radius=R_fly,
    thickness=thickness_fly,
    material_name="Steel",
    density=density_fly,
    mass=mass_fly,
    moment_inertia=mass_fly * R_fly**2
)

# Attach the motor
motor = pc.RevoluteMotor(flywheel, flywheel.get_point(pc.vector3(0, 1, 0)))
motor.set_properties(
    inertia=motor.get_point(pc.vector3(0, 1, 0)).length()**2 * 1e-4,
    damping=0.1,
    initial_torque=0
)

# Define the system
system = pc.System(s)
system.add(beam)
system.add(flywheel)
system.add(motor)

# Add forces and interactions
# Elastic forces in the beam
spring = pc.Spring(beam, beam.get_point(pc.vector3(-1, 0, 0)), 0.1, 1e8)
system.add(spring)

# Rotational inertia of the flywheel
system.add(pc.Damping(flywheel, flywheel.get_point(pc.vector3(0, 1, 0)), 0.1))

# Motor torque
def motor_torque(t):
    return 0.5 * (1 - np.exp(-t/0.1))  # Linear acceleration to speed up

system.add(pc.Torque(motor, motor.get_point(pc.vector3(0, 1, 0)), motor_torque))

# Visualization setup
window = pc.IrrlichtWindow(800, 600, 0, 0, 0, False)
window.add(beam)
window.add(flywheel)
window.add(pc.StressVisualizer(beam, 0, 0, 0, "Stress", 0.5, 0, 0, 0, 0, 0))
window.add(pc.DeformationVisualizer(beam, 0, 0, 0, "Deformation", 0.5, 0, 0, 0, 0, 0))

# Simulation setup
sim = pc.Simulation(s, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
sim.set_contact=False
sim.set_gravity=False

# Add damping forces
damping = pc.DampingForces()
damping.add(pc.DampingForce(beam, beam.get_point(pc.vector3(0, -1, 0)), 0.1, 0.5))
system.add(damping)

# Add gravity
g = pc.GravityEngine(s, pc.vector3(0, -9.81, 0))
system.add(g)

# Add simulation steps
t_final = 5
dt = 0.01
n_steps = int((t_final / dt) + 1)
sim.set_timestep(dt)
sim.set_max_steps(n_steps)

# Add motor torque as a function of time
def update_torque(t):
    return motor_torque(t)
sim.add_user_function(update_torque)

# Run the simulation
sim.run()

# Animate the results
plt.figure(figsize=(10, 8))
t = np.linspace(0, t_final, n_steps)
plt.plot(t, np.array([sim.get_tip_deflection(0, 0, 0)[0] for t in t]))
plt.title('Tip Deflection vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Deflection (m)')
plt.colorbar()
plt.show()

window.close()
pc.finalize()