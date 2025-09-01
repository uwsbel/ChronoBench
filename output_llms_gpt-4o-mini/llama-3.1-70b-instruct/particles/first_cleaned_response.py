import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('./')


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


class ParticleEmitter:
    def __init__(self, system):
        self.system = system
        self.particle_count = 0

    def emit_particle(self):
        
        shape_type = np.random.choice(['sphere', 'box'])

        if shape_type == 'sphere':
            
            radius = np.random.uniform(0.1, 0.5)
            
            particle = chrono.ChBodyEasySphere(self.system, radius, 1000, True, True)
        else:
            
            length = np.random.uniform(0.1, 0.5)
            width = np.random.uniform(0.1, 0.5)
            height = np.random.uniform(0.1, 0.5)
            
            particle = chrono.ChBodyEasyBox(self.system, length, width, height, 1000, True, True)

        
        x = np.random.uniform(-5, 5)
        y = np.random.uniform(5, 10)
        z = np.random.uniform(-5, 5)
        particle.SetPos(chrono.ChVectorD(x, y, z))

        
        vx = np.random.uniform(-1, 1)
        vy = np.random.uniform(-1, 1)
        vz = np.random.uniform(-1, 1)
        particle.SetVel(chrono.ChVectorD(vx, vy, vz))

        
        angle = np.random.uniform(0, 2 * np.pi)
        axis = chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
        axis.Normalize()
        particle.SetRot(chrono.Q_from_AngAxis(angle, axis))

        
        self.system.Add(particle)
        self.particle_count += 1


emitter = ParticleEmitter(mysystem)


for _ in range(100):
    emitter.emit_particle()


myapplication = chronoirr.ChVisualSystemIrrlicht()


myapplication.AddCamera(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 1, 0))


myapplication.SetWindowSize(800, 600)
myapplication.SetWindowTitle('PyChrono Simulation')
myapplication.SetCameraDistance(10)
myapplication.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))


class CustomGravitationalForce(chrono.ChForce):
    def __init__(self, system):
        super().__init__()
        self.system = system

    def ComputeF(self, time):
        
        for particle in self.system.Get_bodies():
            
            force = chrono.ChVectorD(0, 0, 0)
            for other_particle in self.system.Get_bodies():
                if particle != other_particle:
                    distance = (particle.GetPos() - other_particle.GetPos()).Length()
                    force += (other_particle.GetPos() - particle.GetPos()) / distance * 0.1 / distance**2
            
            particle.AddForce(force)


custom_force = CustomGravitationalForce(mysystem)
mysystem.Add(custom_force)


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()
    mysystem.DoStepDynamics(0.01)
    myapplication.Synchronize(True)