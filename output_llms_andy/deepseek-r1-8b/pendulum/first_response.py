import pychrono as pc
import pychrono.trajectory as pt
import pychrono.visualization.irrlicht as vis
import numpy as np
import time

# Initialize PyChrono and visualization
pc.init()
vis.init()

# Create simulation
sim = pc.Simulation()
sim.setVisualStyle("Pendulum")

# Create ground body
ground = pc.Body("ground", pc.Shape.createStaticSphere(1, 1, 1))
sim.addBody(ground)

# Create pendulum body
mass = 1  # kg
length = 5  # m
moment_inertia = mass * (length**2)  # I = m * L^2 for a point mass
pendulum = pc.Body("pendulum", pc.Shape.createSphere(0.5, 0.5, 0.5))
pendulum.setMass(mass)
pendulum.setMomentOfInertia(moment_inertia, pc.g, pc.g)
# Position pendulum above ground
pendulum.setPosition([0, 0, 5])

# Create revolute joint between pendulum and ground
joint = pc.RevoluteJoint("pendulum_joint", pendulum, ground, pc.Vec3(0, 0, 0), pc.Vec3(0, 0, 1))
sim.addJoint(joint)

# Set up visualization
renderer = vis.IrrlichtRenderer()
renderer.setCameraDistance(10)
renderer.setCameraLookAt([0, 0, 10])
renderer.setLightDirection([0, 1, 0])
renderer.setColorBackground([0.1, 0.1, 0.1])

# Add pendulum to renderer
renderer.add(pendulum, "Pendulum", 1, 1, 1, True)

# Define forces and interactions
# Add a spring force to simulate the pendulum's natural movement
spring = pc.HarmonicPotentialSpring("spring", pendulum, 0.5, 0.5, pc.Vec3(0, 0, -gravity), 0.1)
sim.addForce(spring)

# Logger for position and velocity
logger = pc.Logger("PendulumData", pc.Logger.BOTH_FILE_AND_SCREEN, 100, 0.1)
logger.add(pendulum, "position", pc.Logger.POSITION, 0.1)
logger.add(pendulum, "velocity", pc.Logger.VELOCITY, 0.1)

# Set simulation parameters
sim.setGravity(pc.Vec3(0, -9.81, 0))
sim.setTimeStep(0.001)
sim.setRunTime(1000)  # Run for 1000 steps

# Run simulation
sim.run()

# Close visualization
vis.close()
pc.shutdown()