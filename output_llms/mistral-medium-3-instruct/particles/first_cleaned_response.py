import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import random


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle("Particle Attraction Simulation")
visualization.Initialize()
visualization.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))
visualization.AddSkyBox()
visualization.AddLogo()
visualization.SetCameraVertical(chrono.ChCameraVerticalDir_Z)


class ParticleAttractionForce(chrono.ChForce):
    def __init__(self, G=1.0):
        super().__init__()
        self.G = G  

    def ComputeForce(self, time, body):
        force = chrono.ChVectorD(0, 0, 0)

        
        bodies = system.Get_bodylist()

        for other_body in bodies:
            if other_body != body:
                
                delta_pos = other_body.GetPos() - body.GetPos()
                distance = delta_pos.Length()

                if distance > 0.1:  
                    
                    direction = delta_pos.GetNormalized()
                    force += direction * (self.G * body.GetMass() * other_body.GetMass() / (distance * distance))

        body.Force_Accumulate(force)


class ParticleEmitter:
    def __init__(self, system, rate=5, max_particles=100):
        self.system = system
        self.rate = rate  
        self.max_particles = max_particles
        self.time_accum = 0
        self.particle_count = 0

    def emit_particles(self, time_step):
        self.time_accum += time_step

        
        particles_to_emit = int(self.time_accum * self.rate)
        self.time_accum -= particles_to_emit / self.rate

        if self.particle_count + particles_to_emit > self.max_particles:
            particles_to_emit = max(0, self.max_particles - self.particle_count)

        for _ in range(particles_to_emit):
            
            shape_type = random.choice(['sphere', 'box'])

            
            particle = chrono.ChBody()
            particle.SetMass(random.uniform(0.1, 1.0))
            particle.SetInertiaXX(chrono.ChVectorD(particle.GetMass(), particle.GetMass(), particle.GetMass()))

            
            theta = random.uniform(0, 2 * np.pi)
            phi = np.arccos(2 * random.uniform(0, 1) - 1)
            r = random.uniform(0, 2)
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi)
            particle.SetPos(chrono.ChVectorD(x, y, z))

            
            particle.SetPos_dt(chrono.ChVectorD(
                random.uniform(-0.5, 0.5),
                random.uniform(-0.5, 0.5),
                random.uniform(-0.5, 0.5)
            ))

            
            particle.SetRot(chrono.Q_from_AngAxis(random.uniform(0, 2*np.pi),
                                               chrono.ChVectorD(random.random(),
                                                               random.random(),
                                                               random.random()).GetNormalized()))

            
            if shape_type == 'sphere':
                sphere_shape = chrono.ChSphereShape()
                sphere_shape.GetSphereGeometry().rad = random.uniform(0.1, 0.3)
                particle.AddAsset(sphere_shape)
                particle.GetVisualModel().AddSphere(chrono.ChVectorD(0, 0, 0),
                                                   sphere_shape.GetSphereGeometry().rad)
            else:
                box_shape = chrono.ChBoxShape()
                box_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(
                    random.uniform(0.1, 0.3),
                    random.uniform(0.1, 0.3),
                    random.uniform(0.1, 0.3)
                ))
                particle.AddAsset(box_shape)
                particle.GetVisualModel().AddBox(
                    -box_shape.GetBoxGeometry().Size * 0.5,
                    box_shape.GetBoxGeometry().Size,
                    chrono.ChColor(random.random(), random.random(), random.random())
                )

            
            self.system.Add(particle)
            self.particle_count += 1


grav_force = ParticleAttractionForce(G=10.0)
system.AddForce(grav_force)


emitter = ParticleEmitter(system, rate=10, max_particles=50)


time_step = 0.01
simulation_time = 20.0
current_time = 0.0


while visualization.Run() and current_time < simulation_time:
    
    emitter.emit_particles(time_step)

    
    system.DoStepDynamics(time_step)

    
    visualization.BeginScene()
    visualization.Render()
    visualization.EndScene()

    current_time += time_step
    print(f"Simulation time: {current_time:.2f}s, Particles: {emitter.particle_count}")

print("Simulation complete")