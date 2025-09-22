import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


chrono.SetChronoDataPath("./")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


GRAVITATIONAL_CONSTANT = 0.1
PARTICLE_MASS = 1.0
EMISSION_RATE = 0.5  
MAX_PARTICLES = 50
SIMULATION_DOMAIN = 20.0

class ParticleEmitter:
    def __init__(self, system):
        self.system = system
        self.particles = []
        self.last_emission_time = 0
        
    def emit_particle(self, current_time):
        
        if len(self.particles) >= MAX_PARTICLES:
            return
            
        
        particle = chrono.ChBodyEasySphere(
            random.uniform(0.3, 0.8),  
            PARTICLE_MASS,
            True,  
            True   
        )
        
        
        pos_x = random.uniform(-SIMULATION_DOMAIN/2, SIMULATION_DOMAIN/2)
        pos_y = random.uniform(-SIMULATION_DOMAIN/2, SIMULATION_DOMAIN/2)
        pos_z = random.uniform(-SIMULATION_DOMAIN/2, SIMULATION_DOMAIN/2)
        particle.SetPos(chrono.ChVectorD(pos_x, pos_y, pos_z))
        
        
        vel_x = random.uniform(-2, 2)
        vel_y = random.uniform(-2, 2)
        vel_z = random.uniform(-2, 2)
        particle.SetPos_dt(chrono.ChVectorD(vel_x, vel_y, vel_z))
        
        
        q1, q2, q3, q4 = [random.uniform(-1, 1) for _ in range(4)]
        norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        particle.SetRot(chrono.ChQuaternionD(q1/norm, q2/norm, q3/norm, q4/norm))
        
        
        omega_x = random.uniform(-1, 1)
        omega_y = random.uniform(-1, 1)
        omega_z = random.uniform(-1, 1)
        particle.SetWvel_par(chrono.ChVectorD(omega_x, omega_y, omega_z))
        
        
        color = chrono.ChVectorF(random.random(), random.random(), random.random())
        particle.GetVisualShape(0).SetColor(color)
        
        
        particle.GetMaterialSurface().SetFriction(0.3)
        particle.GetMaterialSurface().SetRestitution(0.8)
        
        
        self.system.Add(particle)
        self.particles.append(particle)
        
    def update(self, current_time):
        
        if current_time - self.last_emission_time > 1.0/EMISSION_RATE:
            self.emit_particle(current_time)
            self.last_emission_time = current_time
            
        
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
        
        for i in range(len(particles)):
            for j in range(i + 1, len(particles)):
                particle1 = particles[i]
                particle2 = particles[j]
                
                
                pos1 = particle1.GetPos()
                pos2 = particle2.GetPos()
                
                
                dx = pos2.x - pos1.x
                dy = pos2.y - pos1.y
                dz = pos2.z - pos1.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                
                if distance < 0.5:
                    distance = 0.5
                    
                
                force_magnitude = (GRAVITATIONAL_CONSTANT * PARTICLE_MASS * PARTICLE_MASS) / (distance * distance)
                
                
                unit_x = dx / distance
                unit_y = dy / distance
                unit_z = dz / distance
                
                
                force1_x = force_magnitude * unit_x
                force1_y = force_magnitude * unit_y
                force1_z = force_magnitude * unit_z
                
                
                particle1.Empty_forces_accumulators()
                particle2.Empty_forces_accumulators()
                
                particle1.Accumulate_force(chrono.ChVectorD(force1_x, force1_y, force1_z), pos1, False)
                particle2.Accumulate_force(chrono.ChVectorD(-force1_x, -force1_y, -force1_z), pos2, False)


emitter = ParticleEmitter(system)
gravity_calculator = GravitationalForceCalculator(system)


for _ in range(5):
    emitter.emit_particle(0)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gravitational Particle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 30))
vis.AddTypicalLights()


vis.GetActiveCamera().SetLocation(chrono.ChVectorD(0, 0, 25))
vis.GetActiveCamera().SetTarget(chrono.ChVectorD(0, 0, 0))


coord_sys = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, False, False)
coord_sys.SetBodyFixed(True)
coord_sys.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(coord_sys)


time_step = 0.01
simulation_time = 0


print("Starting gravitational particle simulation...")
print("Controls: Mouse to rotate view, Mouse wheel to zoom")
print(f"Particles will be attracted to each other with G = {GRAVITATIONAL_CONSTANT}")

while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    emitter.update(simulation_time)
    
    
    gravity_calculator.apply_gravitational_forces(emitter.particles)
    
    
    system.DoStepDynamics(time_step)
    simulation_time += time_step
    
    
    if int(simulation_time * 10) % 10 == 0:  
        vis.GetGUIEnvironment().addStaticText(
            f"Time: {simulation_time:.1f}s | Particles: {len(emitter.particles)}",
            chronoirr.recti(10, 10, 300, 30),
            False, True, None, -1, True
        )
    
    vis.EndScene()

print("Simulation completed.")