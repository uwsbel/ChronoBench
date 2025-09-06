import chrono
import numpy as np
import math
import os
import sys
import time

# Initialize PyChrono environment
try:
    chrono = chrono.ChSystem()
    chrono.SetChronoDataPath('/path/to/data')
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    sys.exit(1)

# Define the simulation parameters
num_particles = 100
particle_radius = 0.1
particle_mass = 1.0
gravity = 0.0
particle_velocity = np.array([0.0, 0.0, 0.0])
particle_orientation = np.array([0.0, 0.0, 0.0])
time_step = 0.01

# Create the system
my_system = chrono.ChSystem()

# Set default collision parameters (simple example)
my_system.SetDefaultSuggestedEnvelope(0.005)

# Create a simple particle emitter
emitter = chrono.ChParticleEmitter()
emitter.SetParticleType(chrono.ChParticleType_Sphere)
emitter.SetParticleRadius(particle_radius)
emitter.SetParticleMass(particle_mass)
emitter.SetVelocity(particle_velocity)
emitter.SetOrientation(particle_orientation)
emitter.SetForce(gravity)
emitter.SetCollision(True)
emitter.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
emitter.SetCollisionType(chrono.ChCollisionType_NoCollision)
emitter.SetInitialPosition(chrono.ChVector3d(0, 0, 0))
emitter.SetInitialVelocity(chrono.ChVector3d(0, 0, 0))
emitter.SetInitialOrientation(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))


# Create a visualization system
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Simulation')

# Create a camera
camera = chrono.ChCamera()
camera.SetPosition(chrono.ChVector3d(0, 0, 0))
camera.SetRotation(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))

# Run the simulation
try:
    while vis.Run():
        # Update the simulation
        sys.DoStepDynamics(time_step)

        # Update the visualization
        vis.Render()

        # Print simulation status
        print(f"Time: {time.time()}")

        # Optional: Print particle positions
        # for i in range(num_particles):
        #     print(f"Particle {i}: {particle_velocity[i]}")

        # Print orientation
        print(f"Particle Orientation: {particle_orientation[i]}")

        # Optional: Print particle velocity
        # for i in range(num_particles):
        #     print(f"Particle {i}: {particle_velocity[i]}")

        time.sleep(0.01)  # Adjust for simulation speed

except KeyboardInterrupt:
    print("Simulation interrupted by user.")
    sys.exit(0)