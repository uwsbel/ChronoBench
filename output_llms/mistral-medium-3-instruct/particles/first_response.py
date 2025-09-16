import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # Set the path to Chrono data directory

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity

# 2. Add the required physical systems and objects
# Create a container for the particles
container = chrono.ChBody()
container.SetBodyFixed(True)
container.SetCollide(False)
system.Add(container)

# Particle emitter settings
num_particles = 50
particle_mass = 0.1
particle_radius = 0.1
emission_radius = 5.0

# Custom gravitational force callback
class ParticleGravitationalForce(chrono.ChForce):
    def __init__(self, G=1.0):
        super().__init__()
        self.G = G  # Gravitational constant

    def ComputeForce(self, body, time):
        force = chrono.ChVectorD(0, 0, 0)
        for other_body in system.Get_bodylist():
            if other_body != body and other_body.GetIdentifier() > 0:  # Skip container and itself
                # Calculate distance between particles
                delta = other_body.GetPos() - body.GetPos()
                distance = delta.Length()

                if distance > 0:
                    # Calculate gravitational force (F = G * m1 * m2 / r^2)
                    force_direction = delta / distance
                    force_magnitude = self.G * body.GetMass() * other_body.GetMass() / (distance * distance)
                    force += force_direction * force_magnitude

        body.Force_Accumulate(force)

# Add particles with random properties
for i in range(num_particles):
    # Random shape selection
    shape_type = random.choice(['sphere', 'box', 'ellipsoid'])

    # Create particle body
    particle = chrono.ChBody()
    particle.SetMass(particle_mass)

    # Set random position within emission radius
    angle1 = random.uniform(0, 2*np.pi)
    angle2 = random.uniform(0, np.pi)
    r = random.uniform(0, emission_radius)
    x = r * np.sin(angle2) * np.cos(angle1)
    y = r * np.sin(angle2) * np.sin(angle1)
    z = r * np.cos(angle2)
    particle.SetPos(chrono.ChVectorD(x, y, z))

    # Set random velocity
    particle.SetPos_dt(chrono.ChVectorD(
        random.uniform(-0.5, 0.5),
        random.uniform(-0.5, 0.5),
        random.uniform(-0.5, 0.5)
    ))

    # Set random orientation
    particle.SetRot(chrono.Q_from_AngAxis(random.uniform(0, 2*np.pi), chrono.ChVectorD(1, 0, 0)))
    particle.SetRot_dt(chrono.ChVectorD(
        random.uniform(-0.1, 0.1),
        random.uniform(-0.1, 0.1),
        random.uniform(-0.1, 0.1)
    ))

    # Add visual and collision geometry based on shape
    if shape_type == 'sphere':
        particle.GetVisualModel().AddSphere(particle_radius)
        particle.GetCollisionModel().AddSphere(particle_radius)
    elif shape_type == 'box':
        size = chrono.ChVectorD(particle_radius, particle_radius, particle_radius)
        particle.GetVisualModel().AddBox(size)
        particle.GetCollisionModel().AddBox(size)
    else:  # ellipsoid
        size = chrono.ChVectorD(particle_radius*1.5, particle_radius, particle_radius*0.7)
        particle.GetVisualModel().AddEllipsoid(size)
        particle.GetCollisionModel().AddEllipsoid(size)

    # Add gravitational force to the particle
    grav_force = ParticleGravitationalForce(G=0.1)
    particle.AddForce(grav_force)

    system.Add(particle)

# 3. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravitational Attraction')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalSky()
vis.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by 0.01 seconds
    system.DoStepDynamics(0.01)