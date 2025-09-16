import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize Chrono
chrono.SetChronoDataPath("data/")  
sys = chrono.ChSystemNSC()

# Create a scene with Irrlicht visualization
vis = chronoirr.ChIrrApp(sys)
vis.SetWindowSize(1280, 720)
vis.SetTerrain(chrono.ChColor(0.5, 0.5, 0.5))
vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
vis.AddLight(chrono.ChVectorD(1, 1, 1))

# Particle emitter settings
emitter_pos = chrono.ChVectorD(0, 0, 1)
particle_count = 100
particle_mass = 0.1
particle_radius = 0.05

# Create particle emitter
emitter = chrono.ChParticleEmitter(particle_count, particle_mass, particle_radius)
emitter.SetPos(emitter_pos)

# Define custom gravitational force function
def custom_gravity(particle, other_particle):
    distance = chrono.ChVectorD.Distance(particle.GetPos(), other_particle.GetPos())
    force = chrono.ChVectorD(0, 0, -1) * 10 / (distance**2)
    return force

# Initialize particle system
for i in range(particle_count):
    particle = chrono.ChParticle(particle_mass)
    particle.SetPos(chrono.ChVectorD(chrono.RandomDouble(-1, 1), chrono.RandomDouble(-1, 1), chrono.RandomDouble(0, 1)))
    particle.SetVel(chrono.ChVectorD(chrono.RandomDouble(-1, 1), chrono.RandomDouble(-1, 1), chrono.RandomDouble(-1, 1)))
    particle.SetRot(chrono.Q_from_Euler(chrono.RandomDouble(0, 360), chrono.RandomDouble(0, 360), chrono.RandomDouble(0, 360)))
    sys.Add(particle)
    
# Set up particle interactions
for particle in sys.GetBodyList():
    particle.SetBodyType(chrono.ChBody.Type.BodyFixed)
    particle.SetCollide(True)
    particle.SetMaterial(chrono.ChMaterialSurfaceNSC())
    particle.SetCustomGravityFunction(custom_gravity)

# Run the simulation
vis.Start()
while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.Render()