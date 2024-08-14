import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import random
import math

# Initialize the PyChrono system
chrono.ChSystemNSC()
system = chrono.ChSystemNSC()

# Set up the Irrlicht visualization
application = chronoirr.ChIrrApp(system, "Particle Emitter Simulation", chronoirr.dimension2du(800, 600))
application.SetSkyBox()
application.AddTypicalLights()
application.SetCamera(chrono.ChVectorD(0, 0, 10))

# Particle parameters
num_particles = 100
particle_radius = 0.1
particle_mass = 1.0
gravitational_constant = 1.0

# Create a list to hold particles
particles = []

# Function to create a random particle
def create_random_particle():
    # Random position
    position = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
    # Random velocity
    velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    # Create a particle body
    particle = chrono.ChBody()
    particle.SetMass(particle_mass)
    particle.SetPos(position)
    particle.SetPos_dt(velocity)
    particle.GetCollisionModel().ClearModel()
    particle.GetCollisionModel().AddSphere(particle_radius)
    particle.GetCollisionModel().BuildModel()
    particle.SetCollide(true)
    system.Add(particle)
    return particle

# Create particles
for _ in range(num_particles):
    particle = create_random_particle()
    particles.append(particle)

# Function to apply gravitational force between particles
def apply_gravitational_forces():
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            pos_i = particles[i].GetPos()
            pos_j = particles[j].GetPos()
            distance = (pos_j - pos_i).Length()
            if distance > 0:
                force_magnitude = gravitational_constant * (particle_mass ** 2) / (distance ** 2)
                direction = (pos_j - pos_i).GetNormalized()
                force = direction * force_magnitude
                particles[i].AccumulateForce(-force)  # Apply force on particle i
                particles[j].AccumulateForce(force)    # Apply force on particle j

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Apply gravitational forces
    apply_gravitational_forces()
    
    # Perform the simulation step
    system.DoStepDynamics(0.01)
    
    application.EndScene()

# Clean up
application.GetDevice().drop()