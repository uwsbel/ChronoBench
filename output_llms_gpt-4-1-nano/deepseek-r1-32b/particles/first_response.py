importchrono
import random
import math

# Initialize the Chrono system
system =chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolverType_SOR)
system.SetTimeStep(0.001)

# Create a particle emitter class
class ParticleEmitter:
    def __init__(self, system):
        self.system = system
        self.particles = []

    def emit_particle(self):
        # Random position within a cube
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        z = random.uniform(-5, 5)
        
        # Random velocity
        vx = random.uniform(-1, 1)
        vy = random.uniform(-1, 1)
        vz = random.uniform(-1, 1)
        
        # Random orientation
        rx = random.uniform(0, 2 * math.pi)
        ry = random.uniform(0, 2 * math.pi)
        rz = random.uniform(0, 2 * math.pi)
        
        # Create a particle with random shape
        if random.random() < 0.33:
            # Sphere
            particle =chrono.ChBody()
            particle.SetMass(1)
            particle.SetPos(chrono.ChVectorD(x, y, z))
            particle.SetRot(chrono.ChQuaternionD(rx, ry, rz))
            particle.SetVelocity(chrono.ChVectorD(vx, vy, vz))
            sphere =chrono.ChSphereShape()
            sphere.SetRadius(random.uniform(0.1, 0.5))
            particle.AddVisualShape(sphere)
        elif random.random() < 0.66:
            # Box
            particle =chrono.ChBody()
            particle.SetMass(1)
            particle.SetPos(chrono.ChVectorD(x, y, z))
            particle.SetRot(chrono.ChQuaternionD(rx, ry, rz))
            particle.SetVelocity(chrono.ChVectorD(vx, vy, vz))
            box =chrono.ChBoxShape()
            box.SetDimensions(chrono.ChVectorD(random.uniform(0.2, 0.8), random.uniform(0.2, 0.8), random.uniform(0.2, 0.8)))
            particle.AddVisualShape(box)
        else:
            # Cylinder
            particle =chrono.ChBody()
            particle.SetMass(1)
            particle.SetPos(chrono.ChVectorD(x, y, z))
            particle.SetRot(chrono.ChQuaternionD(rx, ry, rz))
            particle.SetVelocity(chrono.ChVectorD(vx, vy, vz))
            cylinder =chrono.ChCylinderShape()
            cylinder.SetHeight(random.uniform(0.4, 1.2))
            cylinder.SetRadius(random.uniform(0.1, 0.3))
            particle.AddVisualShape(cylinder)
        
        # Random color
        r = random.uniform(0, 1)
        g = random.uniform(0, 1)
        b = random.uniform(0, 1)
        particle.GetVisualShape(0).SetColor(chrono.ChColor(r, g, b))
        
        self.system.AddBody(particle)
        self.particles.append(particle)
        return particle

    def apply_gravitational_forces(self):
        # Apply gravitational forces between all pairs of particles
        for i in range(len(self.particles)):
            for j in range(i + 1, len(self.particles)):
                p1 = self.particles[i]
                p2 = self.particles[j]
                
                # Calculate distance between particles
                dx = p2.GetPos().x - p1.GetPos().x
                dy = p2.GetPos().y - p1.GetPos().y
                dz = p2.GetPos().z - p1.GetPos().z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
                # Calculate gravitational force
                G = 0.1  # Gravitational constant
                force_magnitude = G * p1.GetMass() * p2.GetMass() / (distance**2)
                
                # Calculate force vector
                force = chrono.ChVectorD(
                    force_magnitude * dx / distance,
                    force_magnitude * dy / distance,
                    force_magnitude * dz / distance
                )
                
                # Apply forces
                p1.AddForce(force)
                p2.AddForce(-force)

# Create Irrlicht application
application =chrono.ChIrrApp(system, "Particle Gravitation Simulation", chrono.CH.dimension2D(800, 600))
application.AddTypicalLights()
application.SetCameraPosition(chrono.ChVectorD(10, 10, 10))
application.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))

# Create particle emitter
emitter = ParticleEmitter(system)

# Main simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Handle user input
    if application.GetDevice().isKeyDown(chrono.CH_KEY_SPACE):
        emitter.emit_particle()
    
    if application.GetDevice().isKeyDown(chrono.CH_KEY_R):
        for particle in emitter.particles:
            system.Remove(particle)
        emitter.particles.clear()
    
    # Apply gravitational forces
    emitter.apply_gravitational_forces()
    
    # Update the system
    system.DoStepDynamics()
    
    application.EndScene()

# Reset simulation
def reset_simulation():
    for particle in emitter.particles:
        system.Remove(particle)
    emitter.particles.clear()

# Run the simulation
application.Run()