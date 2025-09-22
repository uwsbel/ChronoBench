import pychrono.core as chrono
import pychrono.irrlicht as irr
import random

# Initialize the Chrono system
system = chrono.ChSystemNSC()

# Set gravity to zero initially, as we'll implement custom gravity
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Create Irrlicht visualization
application = irr.ChIrrApp(system, "Particle Attraction Simulation", irr.dimension2du(800, 600))
application.SetSkyBox()
application.SetLights()
application.SetCamera(chrono.ChVectorD(0, 10, -20), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()

# Parameters for particle emitter
num_particles = 50
spawn_area = 10  # particles spawn within a cube of size spawn_area
max_initial_speed = 2
particle_radius_range = (0.1, 0.3)

# List to hold particles
particles = []

# Function to create a particle with random properties
def create_particle():
    radius = random.uniform(*particle_radius_range)
    shape_type = random.choice(['sphere', 'box'])

    # Create a rigid body for the particle
    body = chrono.ChBodyEasySphere(radius, 1000, True, True) if shape_type == 'sphere' else None
    if shape_type == 'sphere':
        body = chrono.ChBodyEasySphere(radius, 1000, True, True)
    else:
        # For box shape, create a box shape
        size = chrono.ChVectorD(radius, radius, radius)
        body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True)

    # Random position within spawn area
    pos = chrono.ChVectorD(
        random.uniform(-spawn_area/2, spawn_area/2),
        random.uniform(-spawn_area/2, spawn_area/2),
        random.uniform(-spawn_area/2, spawn_area/2)
    )
    body.SetPos(pos)

    # Random initial velocity
    vel = chrono.ChVectorD(
        random.uniform(-max_initial_speed, max_initial_speed),
        random.uniform(-max_initial_speed, max_initial_speed),
        random.uniform(-max_initial_speed, max_initial_speed)
    )
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChMatrix33D(chrono.ChVectorD(0.4 * radius * radius, 0, 0),
                                           chrono.ChVectorD(0, 0.4 * radius * radius, 0),
                                           chrono.ChVectorD(0, 0, 0.4 * radius * radius)))
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetPos(pos)
    body.SetVel(vel)

    system.Add(body)
    return body

# Generate particles
for _ in range(num_particles):
    particle = create_particle()
    particles.append(particle)

# Define gravitational constant for the custom force
G = 1.0  # Adjust as needed for desired attraction strength

# Custom force function to apply pairwise gravitational attraction
def apply_gravitational_forces():
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            bodyA = particles[i]
            bodyB = particles[j]
            posA = bodyA.GetPos()
            posB = bodyB.GetPos()
            delta = posB - posA
            distance = delta.Length()
            if distance > 1e-3:
                force_magnitude = G * bodyA.GetMass() * bodyB.GetMass() / (distance * distance)
                force_dir = delta / distance
                force = force_dir * force_magnitude
                bodyA.ApplyForce(force, bodyA.GetFrame().GetPos())
                bodyB.ApplyForce(-force, bodyB.GetFrame().GetPos())

# Main simulation loop
while application.GetDevice().run():
    # Apply custom gravitational attraction
    apply_gravitational_forces()

    # Optional: add some damping or other forces if desired
    # For now, just proceed

    # Begin scene rendering
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    # Advance the simulation
    system.DoStepDynamics(1/60.0)