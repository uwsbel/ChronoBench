import pychrono as chrono
import pychrono.irrlicht as irr
import random

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create an Irrlicht application to visualize the simulation
my_application = irr.ChIrrApp(my_system, 'Particle Attraction Simulation', irr.dimension2du(1024, 768))
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalCamera(irr.vector3df(0, 3, -6))
my_application.AddTypicalLights()

# Create a particle emitter
emitter_position = chrono.ChVectorD(0, 3, 0)
emitter_velocity = chrono.ChVectorD(0, 0, 0)

# 2. Add the required physical systems and objects as specified
def create_particle():
    # Randomly choose a shape
    shape_choice = random.choice([chrono.ChBodyEasyBox, chrono.ChBodyEasySphere, chrono.ChBodyEasyCylinder])
    particle = shape_choice(1, 1, 1, 1000)

    # Set random position, velocity, and orientation
    particle.SetPos(emitter_position + chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
    particle.SetPos_dt(emitter_velocity + chrono.ChVectorD(random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1)))
    particle.SetRot(chrono.ChQuaternionD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)).GetNormalized())

    # Add the particle to the system
    my_system.Add(particle)

# 3. Set necessary default parameters such as positions, forces, and interactions
# Custom gravitational force
def custom_gravity(body, time, force):
    force += body.GetMass() * 0.5 * chrono.ChVectorD(0, -9.81, 0)

# Add custom gravity force to all particles
my_system.AddForceField(custom_gravity)

# Main simulation loop
while my_application.GetDevice().run():
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()

    # Create a new particle every frame
    create_particle()

    my_application.EndScene()