import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math

# Initialize the PyChrono environment
chrono.SetChronoDataPath("./")

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity

# Simulation parameters
GRAVITATIONAL_CONSTANT = 0.1
PARTICLE_MASS = 1.0
EMISSION_RATE = 0.5  # particles per second
MAX_PARTICLES = 50
SIMULATION_DOMAIN = 20.0

class ParticleEmitter:
    def __init__(self, system):
        self.system = system
        self.particles = []
        self.last_emission_time = 0
        
    def emit_particle(self, current_time):
        """Emit a new particle with random properties"""
        if len(self.particles) >= MAX_PARTICLES:
            return
            
        # Create particle body
        particle = chrono.ChBodyEasySphere(
            random.uniform(0.3, 0.8),  # Random radius
            PARTICLE_MASS,
            True,  # Enable collision
            True   # Enable visualization
        )
        
        # Random position within domain
        pos_x = random.uniform(-SIMULATION_DOMAIN/2, SIMULATION_DOMAIN/2)
        pos_y = random.uniform(-SIMULATION_DOMAIN/2, SIMULATION_DOMAIN/2)
        pos_z = random.uniform(-SIMULATION_DOMAIN/2, SIMULATION_DOMAIN/2)
        particle.SetPos(chrono.ChVectorD(pos_x, pos_y, pos_z))
        
        # Random initial velocity
        vel_x = random.uniform(-2, 2)
        vel_y = random.uniform(-2, 2)
        vel_z = random.uniform(-2, 2)
        particle.SetPos_dt(chrono.ChVectorD(vel_x, vel_y, vel_z))
        
        # Random orientation
        q1, q2, q3, q4 = [random.uniform(-1, 1) for _ in range(4)]
        norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        particle.SetRot(chrono.ChQuaternionD(q1/norm, q2/norm, q3/norm, q4/norm))
        
        # Random angular velocity
        omega_x = random.uniform(-1, 1)
        omega_y = random.uniform(-1, 1)
        omega_z = random.uniform(-1, 1)
        particle.SetWvel_par(chrono.ChVectorD(omega_x, omega_y, omega_z))
        
        # Random color for visualization
        color = chrono.ChVectorF(random.random(), random.random(), random.random())
        particle.GetVisualShape(0).SetColor(color)
        
        # Set material properties
        particle.GetMaterialSurface().SetFriction(0.3)
        particle.GetMaterialSurface().SetRestitution(0.8)
        
        # Add to system
        self.system.Add(particle)
        self.particles.append(particle)
        
    def update(self, current_time):
        """Update particle emission"""
        if current_time - self.last_emission_time > 1.0/EMISSION_RATE:
            self.emit_particle(current_time)
            self.last_emission_time = current_time
            
        # Remove particles that are too far from origin
        particles_to_remove = []
        for particle in self.particles:
            pos = particle.GetPos()
            distance = math.sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z)
            if distance > SIMULATION_DOMAIN:
                particles_to_remove.append(particle)
                
        for particle in particles_to_remove:
            self.system.Remove(particle)
            self.particles.remove(particle)

class GravitationalForceCalculator:
    def __init__(self, system):
        self.system = system
        
    def apply_gravitational_forces(self, particles):
        """Apply gravitational forces between all particle pairs"""
        for i in range(len(particles)):
            for j in range(i + 1, len(particles)):
                particle1 = particles[i]
                particle2 = particles[j]
                
                # Get positions
                pos1 = particle1.GetPos()
                pos2 = particle2.GetPos()
                
                # Calculate distance vector
                dx = pos2.x - pos1.x
                dy = pos2.y - pos1.y
                dz = pos2.z - pos1.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Avoid division by zero and extremely large forces
                if distance < 0.5:
                    distance = 0.5
                    
                # Calculate gravitational force magnitude
                force_magnitude = (GRAVITATIONAL_CONSTANT * PARTICLE_MASS * PARTICLE_MASS) / (distance * distance)
                
                # Calculate unit vector
                unit_x = dx / distance
                unit_y = dy / distance
                unit_z = dz / distance
                
                # Calculate force vectors
                force1_x = force_magnitude * unit_x
                force1_y = force_magnitude * unit_y
                force1_z = force_magnitude * unit_z
                
                # Apply forces (Newton's third law)
                particle1.Empty_forces_accumulators()
                particle2.Empty_forces_accumulators()
                
                particle1.Accumulate_force(chrono.ChVectorD(force1_x, force1_y, force1_z), pos1, False)
                particle2.Accumulate_force(chrono.ChVectorD(-force1_x, -force1_y, -force1_z), pos2, False)

# Create particle emitter and force calculator
emitter = ParticleEmitter(system)
gravity_calculator = GravitationalForceCalculator(system)

# Create some initial particles
for _ in range(5):
    emitter.emit_particle(0)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gravitational Particle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 30))
vis.AddTypicalLights()

# Set up lighting
vis.GetActiveCamera().SetLocation(chrono.ChVectorD(0, 0, 25))
vis.GetActiveCamera().SetTarget(chrono.ChVectorD(0, 0, 0))

# Add coordinate system for reference
coord_sys = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, False, False)
coord_sys.SetBodyFixed(True)
coord_sys.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(coord_sys)

# Simulation parameters
time_step = 0.01
simulation_time = 0

# Simulation loop
print("Starting gravitational particle simulation...")
print("Controls: Mouse to rotate view, Mouse wheel to zoom")
print(f"Particles will be attracted to each other with G = {GRAVITATIONAL_CONSTANT}")

while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    # Update particle emission
    emitter.update(simulation_time)
    
    # Apply gravitational forces
    gravity_calculator.apply_gravitational_forces(emitter.particles)
    
    # Step the simulation
    system.DoStepDynamics(time_step)
    simulation_time += time_step
    
    # Display information
    if int(simulation_time * 10) % 10 == 0:  # Every second
        vis.GetGUIEnvironment().addStaticText(
            f"Time: {simulation_time:.1f}s | Particles: {len(emitter.particles)}",
            chronoirr.recti(10, 10, 300, 30),
            False, True, None, -1, True
        )
    
    vis.EndScene()

print("Simulation completed.")