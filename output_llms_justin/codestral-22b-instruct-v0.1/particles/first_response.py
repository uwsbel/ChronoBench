import pychrono as chrono
import pychrono.irrlicht as irr
import random

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('path_to_chrono_data')
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# 2. Add the required physical systems and objects as specified
# Define the particle emitter
def emit_particle():
    # Randomly select a shape
    shape = random.choice([chrono.ChSphereShape(), chrono.ChBoxShape(0.1, 0.1, 0.1), chrono.ChCylinderShape(0.05, 0.2)])
    # Create a body with the selected shape
    body = chrono.ChBody()
    body.SetCollide(True)
    body.AddAsset(shape)
    # Set random position, velocity, and orientation
    body.SetPos(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(0, 2), random.uniform(-1, 1)))
    body.SetPos_dt(chrono.ChVectorD(random.uniform(-0.1, 0.1), 0, random.uniform(-0.1, 0.1)))
    body.SetRot(chrono.ChQuaternionD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)).GetNormalized())
    # Add the body to the system
    system.Add(body)

# Emit 100 particles
for _ in range(100):
    emit_particle()

# 3. Set necessary default parameters such as positions, forces, and interactions
# Apply custom gravitational forces to simulate particle attraction dynamically
def apply_gravity(body):
    force = chrono.ChVectorD(0, -1, 0) * body.GetMass()
    body.Empty_forces_accumulators()
    body.AccumulateForce(force, body.GetPos(), True)

system.AddCustomGravityForceFunction(apply_gravity)

# Visualize the simulation using Irrlicht
application = irr.ChIrrApp(system, 'Particle Attraction Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(irr.vector3df(0, 2, -3), irr.vector3df(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()