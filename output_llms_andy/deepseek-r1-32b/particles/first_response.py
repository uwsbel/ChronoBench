importchrono
import math
import random
from chrono import ChVector, ChQuaternion, ChSystem, ChIrrApp, ChIrrGui, ChCollisionModel, ChBody, ChSphere, ChBox, ChCylinder

# Initialize the Chrono system
system = ChSystem()
system.Set_G_acc(ChVector(0, 0, 0))  # Gravity will be handled by custom forces

# Set up Irrlicht visualization
application = ChIrrApp(system, "Particle Attraction Simulation", ChVector(1024, 768, 0))
application.AddTypicalLights()
application.AddTypicalLogo()
application.SetCameraPosition(ChVector(0, 50, 50))
application.SetCameraRotation(ChQuaternion(0.7071, 0, 0.7071, 0))

# Particle parameters
num_particles = 50
particle_mass = 1.0
gravitational_constant = 0.001  # Adjust this value for different attraction strength

def create_particle(shape_type):
    """Create a particle with random properties"""
    particle = ChBody()
    system.AddBody(particle)
    particle.SetMass(particle_mass)
    
    # Random position around origin
    x = random.uniform(-5, 5)
    y = random.uniform(-5, 5)
    z = random.uniform(-5, 5)
    particle.SetPos(ChVector(x, y, z))
    
    # Random velocity
    vx = random.uniform(-1, 1)
    vy = random.uniform(-1, 1)
    vz = random.uniform(-1, 1)
    particle.SetPos_dt(ChVector(vx, vy, vz))
    
    # Random orientation
    angle = random.uniform(0, 2 * math.pi)
    axis = ChVector(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    particle.SetRot(ChQuaternion(angle, axis))
    
    # Create geometry based on shape type
    if shape_type == 'sphere':
        sphere = ChSphere()
        sphere.SetRadius(random.uniform(0.1, 0.5))
        particle.AddGeometry(sphere)
    elif shape_type == 'box':
        box = ChBox()
        box.SetDimensions(ChVector(random.uniform(0.2, 1), random.uniform(0.2, 1), random.uniform(0.2, 1)))
        particle.AddGeometry(box)
    elif shape_type == 'cylinder':
        cylinder = ChCylinder()
        cylinder.SetHeight(random.uniform(0.5, 2))
        cylinder.SetRadius(random.uniform(0.1, 0.5))
        particle.AddGeometry(cylinder)
    
    particle.GetGeometry().SetMaterialColor(random.uniform(0,1), random.uniform(0,1), random.uniform(0,1))
    particle.SetCollide(False)  # Disable collision for simplicity
    
    return particle

# Create particles with random shapes
particles = []
for _ in range(num_particles):
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    particles.append(create_particle(shape_type))

def apply_gravitational_forces():
    """Apply gravitational forces between all particle pairs"""
    for i in range(len(particles)):
        for j in range(i+1, len(particles)):
            # Calculate distance between particles
            pos_i = particles[i].GetPos()
            pos_j = particles[j].GetPos()
            delta = pos_j - pos_i
            distance = delta.Length()
            
            if distance > 0:
                # Calculate gravitational force
                force_magnitude = gravitational_constant * (particle_mass * particle_mass) / (distance * distance)
                force_direction = delta.Normalize()
                
                # Apply forces to both particles
                particles[i].AddForce(force_direction * force_magnitude)
                particles[j].AddForce(-force_direction * force_magnitude)

# Simulation loop
application.Begin()
while application.GetDevice().Run():
    # Clear forces
    for particle in particles:
        particle.ClearForces()
    
    # Apply gravitational forces
    apply_gravitational_forces()
    
    # Add damping to prevent excessive movement
    for particle in particles:
        velocity = particle.GetPos_dt()
        damping_force = -0.1 * velocity
        particle.AddForce(damping_force)
    
    # Update simulation
    system.DoStepDynamics(0.005)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

application.End()