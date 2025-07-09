import math
import numpy as np
import pychrono as chrono
import pychrono.robot as chronorobot
import pychrono.irrlicht as chronoirr


def initialize_simulation():
    
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVectorD(0, -9.81, 0))
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Particle Attraction Simulation")
    vis.Initialize()
    vis.AddCamera(chrono.ChVector3D(0, 5, 10))
    vis.AddTypicalLights()
    
    return system, vis


class ParticleEmitter:
    def __init__(self, system, vis, num_particles=100):
        self.system = system
        self.vis = vis
        self.num_particles = num_particles
        self.particles = []
        
    def create_particle(self, position=None):
        if position is None:
            position = chrono.ChVectorD(
                np.random.uniform(-5, 5),
                np.random.uniform(-5, 5),
                np.random.uniform(-5, 5)
            )
        
        
        particle = chrono.ChBodyEasySphere(
            radius=np.random.uniform(0.1, 0.5),
            density=1000,
            visualization=True,
            collision=True,
            material=None
        )
        
        
        particle.SetPos(position)
        particle.SetMass(np.random.uniform(1, 10))
        particle.SetInertiaXX(chrono.ChVectorD(1,1,1))
        particle.SetRandomVelocity(np.random.uniform(0, 5))
        particle.SetRandomRotation(np.random.uniform(0, 2*math.pi))
        
        
        color = chrono.ChColor(
            np.random.uniform(0,1),
            np.random.uniform(0,1),
            np.random.uniform(0,1)
        )
        particle.AddVisualShape(
            chrono.ChVisualShapeSphere(particle.GetSphereGeometry().GetRadius()),
            color
        )
        
        
        self.system.Add(particle)
        self.particles.append(particle)
        
        return particle
    
    def emit_particles(self):
        for _ in range(self.num_particles):
            self.create_particle()


def apply_gravitational_force(particle, system):
    
    pos = particle.GetPos()
    distance = pos.Length()
    
    
    force_magnitude = 1000000 / (distance**2) if distance != 0 else 0
    
    
    force = chrono.ChVectorD(
        -force_magnitude * pos.x / distance,
        -force_magnitude * pos.y / distance,
        -force_magnitude * pos.z / distance
    )
    
    particle.AddForce(force)


def run_simulation():
    system, vis = initialize_simulation()
    emitter = ParticleEmitter(system, vis)
    emitter.emit_particles()
    
    
    def on_contact(pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
        print(f"Contact detected between particles! Force: {cforce}")
        
    system.GetContactContainer().RegisterAddContactCallback(on_contact)
    
    
    step_count = 1000
    time_step = 0.001
    
    for step in range(step_count):
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        for particle in emitter.particles:
            apply_gravitational_force(particle, system)
        
        
        system.DoStepDynamics(time_step)
        
        
        if vis.Run():
            break
    
    
    system.RemoveAll()
    vis.Close()


if __name__ == "__main__":
    run_simulation()