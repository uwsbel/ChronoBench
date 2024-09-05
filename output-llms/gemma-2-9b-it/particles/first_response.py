import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize PyChrono environment
chrono.SetChronoDataPath("path/to/chrono/data")  # Replace with your Chrono data path
system = chrono.ChSystemNSC()

# Create Irrlicht visualization
vis = chronoirr.ChIrrApp(system)
vis.AddCamera(chrono.ChVectorD(5, 5, 5))  # Set initial camera position
vis.SetWindowSize(1280, 720)
vis.SetRenderMode(chronoirr.CH_WIREFRAME)  # Set rendering mode (optional)
vis.Start()

# Particle emitter settings
emitter_pos = chrono.ChVectorD(0, 0, 0)
particle_count = 100
particle_mass = 0.1
particle_radius = 0.1

# Create particle emitter
emitter = chrono.ChParticleEmitter()
emitter.SetPos(emitter_pos)
emitter.SetNumParticles(particle_count)
emitter.SetParticleMass(particle_mass)
emitter.SetParticleRadius(particle_radius)

# Define particle shape (random)
particle_shape = chrono.ChBodyEasyBox(particle_radius, particle_radius, particle_radius)

# Set initial particle properties
for i in range(particle_count):
    # Random position within a sphere
    particle_pos = chrono.ChVectorD(
        chrono.RandomDouble(-1, 1), chrono.RandomDouble(-1, 1), chrono.RandomDouble(-1, 1)
    )
    # Random velocity
    particle_vel = chrono.ChVectorD(
        chrono.RandomDouble(-1, 1), chrono.RandomDouble(-1, 1), chrono.RandomDouble(-1, 1)
    )
    # Random orientation
    particle_quat = chrono.ChQuaternionD.FromEulerZYX(chrono.RandomDouble(0, 2 * 3.14159), chrono.RandomDouble(0, 2 * 3.14159), chrono.RandomDouble(0, 2 * 3.14159))

    # Create particle body
    particle = chrono.ChBodyEasy(particle_mass)
    particle.SetPos(particle_pos)
    particle.SetRot(particle_quat)
    particle.SetBody(particle_shape)
    particle.SetBodyFixed(False)
    particle.SetLinearVelocity(particle_vel)

    # Add particle to the system
    system.AddBody(particle)

# Custom gravitational force function
def apply_gravitational_force(particle1, particle2):
    # Calculate distance between particles
    distance = particle1.GetPos() - particle2.GetPos()
    distance_magnitude = distance.Length()

    # Apply gravitational force
    force_magnitude = -G * particle1.GetMass() * particle2.GetMass() / (distance_magnitude ** 2)
    force_direction = distance / distance_magnitude
    force = force_magnitude * force_direction

    # Apply force to particles
    particle1.AddForce(force)
    particle2.AddForce(-force)

# Simulation loop
G = 6.67430e-11  # Gravitational constant
dt = 0.01  # Time step

while vis.Run():
    system.DoStepDynamics(dt)

    # Apply custom gravitational forces between all particles
    for i in range(system.GetBodyCount()):
        for j in range(i + 1, system.GetBodyCount()):
            apply_gravitational_force(system.GetBody(i), system.GetBody(j))

    vis.Render()

# Cleanup
vis.Close()