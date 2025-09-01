import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystem()


sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddCamera(chrono.ChVectorD(0, 0, -5))
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Gravitational Attraction between Particles")

class ParticleEmitter:
    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.particles = []

    def emit_particles(self):
        for _ in range(self.num_particles):
            
            shape_type = np.random.choice(["sphere", "box", "cylinder"])

            
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(-5, 5)
            z = np.random.uniform(-5, 5)
            position = chrono.ChVectorD(x, y, z)

            
            vx = np.random.uniform(-1, 1)
            vy = np.random.uniform(-1, 1)
            vz = np.random.uniform(-1, 1)
            velocity = chrono.ChVectorD(vx, vy, vz)

            
            rx = np.random.uniform(0, np.pi)
            ry = np.random.uniform(0, np.pi)
            rz = np.random.uniform(0, np.pi)
            rotation = chrono.ChQuaternionD(rx, ry, rz)

            
            if shape_type == "sphere":
                particle = chrono.ChBodyEasySphere(sys, 0.1, 1000, True)
            elif shape_type == "box":
                particle = chrono.ChBodyEasyBox(sys, chrono.ChVectorD(0.1, 0.1, 0.1), 1000, True)
            elif shape_type == "cylinder":
                particle = chrono.ChBodyEasyCylinder(sys, chrono.ChVectorD(0.1, 0.1, 0.1), 1000, True)

            
            particle.SetPos(position)
            particle.SetPos_dt(velocity)
            particle.SetRot(rotation)

            
            sys.Add(particle)

            
            self.particles.append(particle)


emitter = ParticleEmitter(100)
emitter.emit_particles()

class CustomGravitationalForce(chrono.ChForce):
    def __init__(self, particle):
        super().__init__()
        self.particle = particle
        self.G = 0.1  

    def ComputeF(self, xnew, vnew, xold, vold, time):
        force = chrono.ChVectorD(0, 0, 0)
        for other_particle in emitter.particles:
            if other_particle != self.particle:
                distance = (other_particle.GetPos() - self.particle.GetPos()).Length()
                if distance > 0:
                    force += (other_particle.GetPos() - self.particle.GetPos()) / distance * self.G / distance**2
        return force


for particle in emitter.particles:
    force = CustomGravitationalForce(particle)
    particle.AddForce(force)


time_step = 0.01
total_time = 10


while sys.GetChTime() < total_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()