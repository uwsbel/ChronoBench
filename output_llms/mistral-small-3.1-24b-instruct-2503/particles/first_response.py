import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math

# Initialize the Chrono::Engine
chrono.SetChronoDataPath('path_to_chrono_data')

# Create the system
system = chrono.ChSystemNSC()

# Create the Irrlicht visualization
visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('PyChrono Particle Simulation')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'))
visualization.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 1, 1, 50, 50, 50)
visualization.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))

# Create a particle emitter
class ParticleEmitter:
    def __init__(self, system):
        self.system = system

    def emit_particle(self):
        # Random position
        pos = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0, 2))

        # Random velocity
        vel = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

        # Random orientation
        quat = chrono.ChQuaternionD(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        quat.Normalize()

        # Create a particle
        particle = chrono.ChParticle()
        particle.SetPos(pos)
        particle.SetMass(1)
        particle.SetPos_dt(vel)
        particle.SetRot(quat)

        # Add particle to the system
        system.Add(particle)

        # Create a visual shape
        shape = chrono.ChSphereShape()
        shape.GetRadius() = random.uniform(0.1, 0.5)
        visual = chrono.ChVisualShapeSphere()
        visual.GetSphereGeometry().rad = shape.GetRadius()
        visual.SetColor(chrono.ChColor(1, 0, 0))
        particle.AddAsset(visual)

# Create the particle emitter
emitter = ParticleEmitter(system)

# Emit particles
for _ in range(100):
    emitter.emit_particle()

# Custom gravitational force function
def custom_gravity(particle, system):
    force = chrono.ChVectorD(0, 0, 0)
    for other in system.Get_physics_items():
        if isinstance(other, chrono.ChParticle) and other != particle:
            distance = (other.GetPos() - particle.GetPos()).Length()
            if distance > 0:
                direction = (other.GetPos() - particle.GetPos()).Normalize()
                force += direction * (1 / distance ** 2)  # Simple inverse square law
    particle.ApplyForce(force)

# Apply custom gravitational forces
def apply_custom_forces(system):
    for item in system.Get_physics_items():
        if isinstance(item, chrono.ChParticle):
            custom_gravity(item, system)

# Simulation loop
while visualization.Run():
    system.DoStepDynamics(0.01)  # Step size of 0.01 seconds
    apply_custom_forces(system)
    visualization.Render()

# Clean up
visualization.Close()