import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math

# Initialize the Chrono system
chrono.SetChronoDataPath('path/to/chrono/data')
my_system = chrono.ChSystemNSC()

# Set up the visualization with Irrlicht
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Gravitational Attraction Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 10, -20), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Define a custom force function for gravitational attraction
def gravitational_force(particle1, particle2):
    G = 6.67430e-11  # Gravitational constant
    m1 = particle1.GetMass()
    m2 = particle2.GetMass()
    pos1 = particle1.GetPos()
    pos2 = particle2.GetPos()
    r = (pos2 - pos1).Length()
    if r < 1e-5:
        return chrono.ChVectorD(0, 0, 0)
    return G * m1 * m2 / r**2 * (pos2 - pos1).GetNormalized()

# Create a list to hold particles
particles = []

# Function to emit a particle with random properties
def emit_particle():
    # Random shape: for simplicity, we'll use spheres with random radii
    radius = random.uniform(0.1, 0.5)
    density = 1000  # kg/m^3
    mass = (4/3) * math.pi * radius**3 * density

    # Random position within a certain range
    position = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(0, 10), random.uniform(-5, 5))

    # Random velocity
    velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

    # Create the particle
    particle = chrono.ChBodyEasySphere(radius, density, True, True)
    particle.SetPos(position)
    particle.SetPos_dt(velocity)

    # Add the particle to the system
    my_system.Add(particle)
    particles.append(particle)

# Emit a few particles to start the simulation
for _ in range(10):
    emit_particle()

# Define a custom timestepper
class CustomTimestepper(chrono.ChTimestepper):
    def Advance(self, step):
        super().Advance(step)
        # Apply gravitational forces between particles
        for i, particle1 in enumerate(particles):
            force = chrono.ChVectorD(0, 0, 0)
            for j, particle2 in enumerate(particles):
                if i != j:
                    force += gravitational_force(particle1, particle2)
            particle1.Accumulate_force(force, particle1.GetPos(), False)

# Set the custom timestepper
my_system.SetTimestepper(CustomTimestepper())

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    my_system.DoStepDynamics(0.01)

    # Emit a new particle periodically
    if my_system.GetChTime() % 1 < 0.01:
        emit_particle()

vis.GetDevice().closeDevice()