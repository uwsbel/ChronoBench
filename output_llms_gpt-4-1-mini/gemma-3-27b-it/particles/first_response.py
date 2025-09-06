import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# --- Simulation Setup ---

# Set the path to the Chrono data directory.  Adjust this if necessary.
chrono.SetChronoDataPath("../chrono/data/")

# Create a physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Visualization Setup ---

# Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 3, -6))
vis.AddTypicalLights()
vis.AddSkyBox()

# --- Particle Emitter ---

class ParticleEmitter:
    def __init__(self, system, num_particles, min_radius, max_radius, min_mass, max_mass):
        self.system = system
        self.num_particles = num_particles
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.min_mass = min_mass
        self.max_mass = max_mass

    def create_particles(self):
        for i in range(self.num_particles):
            # Random particle properties
            radius = np.random.uniform(self.min_radius, self.max_radius)
            mass = np.random.uniform(self.min_mass, self.max_mass)

            # Random position
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(0, 5)
            z = np.random.uniform(-5, 5)
            pos = chrono.ChVector3d(x, y, z)

            # Random velocity
            vx = np.random.uniform(-1, 1)
            vy = np.random.uniform(-1, 1)
            vz = np.random.uniform(-1, 1)
            vel = chrono.ChVector3d(vx, vy, vz)

            # Create a sphere body
            particle = chrono.ChBodyEasySphere(radius, mass, True, True)
            particle.SetPos(pos)
            particle.SetPos_DT(vel)  # Set initial velocity
            
            # Add the particle to the system
            self.system.Add(particle)

# --- Gravitational Force ---

gravity_constant = 1.0  # Adjust this value to control the strength of gravity

def apply_gravity(system, time_step):
    for i in range(system.GetNbBody()):
        body1 = system.Get_Body(i)
        for j in range(i + 1, system.GetNbBody()):
            body2 = system.Get_Body(j)

            # Calculate distance vector
            dist_vec = body2.GetPos() - body1.GetPos()
            distance = dist_vec.Norm()

            # Avoid division by zero and very close particles
            if distance < 0.1:
                continue

            # Calculate gravitational force magnitude
            force_magnitude = (gravity_constant * body1.GetMass() * body2.GetMass()) / (distance * distance)

            # Calculate force vector
            force_vec = force_magnitude * dist_vec.Normalize()

            # Apply force to both bodies (Newton's third law)
            body1.ApplyForce(force_vec)
            body2.ApplyForce(-force_vec)

# --- Main Simulation Loop ---

emitter = ParticleEmitter(sys, 50, 0.2, 0.5, 0.1, 0.5)
emitter.create_particles()

time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Apply gravity
    apply_gravity(sys, time_step)

    # Advance the simulation
    sys.DoStepDynamics(time_step)