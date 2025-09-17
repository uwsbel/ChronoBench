import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particlefactory as chrono_particlefactory
import random

# Initialize the PyChrono environment
chrono.SetChronoDataPath("/path/to/chrono/data/")

# Create a Chrono system
system = chrono.ChSystemSMC()

# Set default collision and contact parameters
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a particle factory
particle_factory = chrono_particlefactory.ChParticleFactory()

# Create a particle emitter
particle_emitter = chrono_particlefactory.ChParticleEmitter(particle_factory)

# Define a callback to create particles with random shapes
class RandomParticleCreator(chrono_particlefactory.ChRandomParticleCreator):
    def __init__(self):
        super().__init__()

    def CreateParticle(self, position, velocity):
        particle = chrono_particlefactory.ChParticle()
        particle.SetMass(1.0)
        particle.SetPos(position)
        particle.SetPos_dt(velocity)
        particle.SetRot(chrono.ChQuaternionD(random.random(), random.random(), random.random(), random.random()))
        particle.SetRot_dt(chrono.ChQuaternionD(random.random(), random.random(), random.random(), random.random()))
        
        # Randomly choose a shape for the particle
        shape_type = random.randint(0, 2)
        if shape_type == 0:
            particle.AddShape(chrono.ChSphereShape(0.1))
        elif shape_type == 1:
            particle.AddShape(chrono.ChBoxShape(0.1, 0.1, 0.1))
        else:
            particle.AddShape(chrono.ChEllipsoidShape(0.1, 0.2, 0.3))
        
        return particle

# Create an instance of the callback
particle_creator = RandomParticleCreator()
particle_emitter.SetParticleCreator(particle_creator)

# Create a custom gravitational force
class CustomGravity(chrono.ChForce):
    def __init__(self, body1, body2):
        super().__init__()
        self.body1 = body1
        self.body2 = body2
        self.G = 6.674 * (10**-11)

    def Update(self, step):
        r = self.body2.GetPos() - self.body1.GetPos()
        distance = r.Length()
        if distance > 0:
            force = self.G * (self.body1.GetMass() * self.body2.GetMass()) / (distance**3) * r
            self.body1.AddForce(force)
            self.body2.AddForce(-force)

# Create two bodies to attract particles
body1 = chrono.ChBody()
body1.SetMass(1000.0)
body1.SetPos(chrono.ChVectorD(0, 0, 0))
body1.SetBodyFixed(True)
system.AddBody(body1)

body2 = chrono.ChBody()
body2.SetMass(1000.0)
body2.SetPos(chrono.ChVectorD(5, 0, 0))
body2.SetBodyFixed(True)
system.AddBody(body2)

# Create a custom gravitational force between the two bodies
custom_gravity = CustomGravity(body1, body2)
system.AddForce(custom_gravity)

# Create an Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Attraction Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    # Emit particles
    if random.random() < 0.1:
        particle_emitter.EmitParticles(system, 1, chrono.ChCoordsysD(chrono.ChVectorD(random.uniform(-5, 5), 5, random.uniform(-5, 5))))

    # Update the custom gravitational force for each particle
    for body in system.Get_bodylist():
        if body != body1 and body != body2:
            custom_gravity = CustomGravity(body1, body)
            custom_gravity.Update(0)
            custom_gravity = CustomGravity(body2, body)
            custom_gravity.Update(0)

    vis.EndScene()
    system.DoStepDynamics(0.01)

# No need to put anything here as the code is already given above inside the triple backticks is not required.

import pychrono as chrono
# ... rest of the code