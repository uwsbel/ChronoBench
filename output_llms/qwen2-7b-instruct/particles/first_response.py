import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize PyChrono environment
chrono.Ch纪元().Initialize()

# Create a new system
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create Irrlicht visualization
vis = chronoirr.ChIrrApp(system, "PyChrono Particle Attraction Simulation", chronoirr.dimension2du(800, 600))
vis.AddTypicalCamera(chrono.ChVectorD(0, 10, 10))

# Add particles with random shapes, positions, velocities, and orientations
num_particles = 100
particle_radius = 0.1
particle_mass = 1.0
particle_density = 1000.0

for i in range(num_particles):
    # Random position
    pos = chrono.ChVectorD(chrono.GetRandom(-10, 10), chrono.GetRandom(-10, 10), chrono.GetRandom(-10, 10))
    
    # Random velocity
    vel = chrono.ChVectorD(chrono.GetRandom(-5, 5), chrono.GetRandom(-5, 5), chrono.GetRandom(-5, 5))
    
    # Random orientation
    rot = chrono.ChQuaternionD()
    rot.SetFromEuler(chrono.GetRandom(-chrono.CH_C_PI, chrono.CH_C_PI), 
                     chrono.GetRandom(-chrono.CH_C_PI, chrono.CH_C_PI), 
                     chrono.GetRandom(-chrono.CH_C_PI, chrono.CH_C_PI))
    
    # Create particle shape
    shape = chrono.ChCylinderShape()
    shape.GetCylinderGeometry().rad = particle_radius
    shape.GetCylinderGeometry().len = 1.0
    
    # Create particle
    particle = chrono.ChBody()
    particle.SetIdentifier(i)
    particle.SetBodyFixed(False)
    particle.SetMass(particle_mass)
    particle.SetPos(pos)
    particle.SetRot(rot)
    particle.AddAsset(shape)
    particle.SetCollide(True)
    particle.SetInertiaXX(chrono.ChVectorD(particle_mass * particle_radius ** 2 / 12, 
                                           particle_mass * particle_radius ** 2 / 12, 
                                           particle_mass * particle_radius ** 2 / 4))
    system.Add(particle)

    # Apply gravity to particle
    particle.AddForce(chrono.ChVectorD(0, -particle_mass * particle_density * 9.81, 0), True)

    # Add custom gravitational force
    def custom_gravity(particle1, particle2):
        force = chrono.ChVectorD(0, 0, 0)
        if particle1.GetIdentifier() != particle2.GetIdentifier():
            distance = (particle1.GetPos() - particle2.GetPos()).Length()
            if distance > 0:
                force = (particle1.GetPos() - particle2.GetPos()) * (particle_density * 9.81 / distance ** 2)
        return force

    system.AddForceField(custom_gravity)

# Run the simulation
vis.AddTypicalSky()
vis.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddTypicalLights()
vis.AddTypicalCamera(chrono.ChVectorD(0, 10, 10))
vis.AddParticles(system.GetBodies(), 1.0)
vis.ExecuteAndStep(1000, 1)