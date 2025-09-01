import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particlefactory as chrono_particlefactory
import random

# Initialize the PyChrono environment
chrono.SetChronoDataPath("/path/to/chrono/data/")

# Create a PyChrono system
sys = chrono.ChSystemSMC()

# Create a particle factory
factory = chrono_particlefactory.ChParticleFactory()

# Create a particle emitter
emitter = chrono_particlefactory.ChParticleEmitter()

# Set the particle factory's particle type to be random
factory.SetParticleCreator(chrono_particlefactory.ChRandomParticleCreator(
    chrono_particlefactory.ChRandomShapeCreator(
        chrono_particlefactory.ChRandomParticleCreator.ParticleType(chrono_particlefactory.ChRandomParticleCreator.ParticleType_SPHERE, 
                                                                    chrono_particlefactory.ChRandomParticleCreator.ParticleType_BOX, 
                                                                    chrono_particlefactory.ChRandomParticleCreator.ParticleType_ELLIPSOID),
        [0.4, 0.2, 0.1],  # scale
        chrono.ChVector3(0.2, 0.2, 0.2),  # min size
        chrono.ChVector3(0.5, 0.5, 0.5)   # max size
    ),
    chrono_particlefactory.ChRandomPositionCreator(
        chrono.ChCoordsys(chrono.ChVector3(0, 0, 0), chrono.QUNIT),
        chrono.ChCoordsys(chrono.ChVector3(1, 1, 1), chrono.QUNIT)
    ),
    chrono_particlefactory.ChRandomVelocityCreator(
        chrono.ChVector3(-0.5, -0.5, -0.5),
        chrono.ChVector3(0.5, 0.5, 0.5)
    )
))

# Set the emitter's particle factory
emitter.SetParticleFactory(factory)

# Add the emitter to the system
sys.Add(emitter)

# Create a custom gravitational force
class CustomGravity(chrono.ChForce):
    def __init__(self):
        super().__init__()

    def Update(self, time):
        for body in sys.Get_bodylist():
            if isinstance(body, chrono.ChBody):
                # Apply a gravitational force towards the origin
                force = -body.GetPos() * 0.1
                body.AddForce(force)

# Create an instance of the custom gravitational force
custom_gravity = CustomGravity()

# Add the custom gravitational force to the system
sys.AddForce(custom_gravity)

# Create an Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Emitter Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3(0, 2, -4))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    emitter.EmitParticles(0.01)