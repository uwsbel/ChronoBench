import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


class ParticleEmitter:
    def __init__(self):
        self.shapes = [chrono.ChBodyEasySphere(1, 1000), chrono.ChBodyEasyCylinder(1, 1, 1000), chrono.ChBodyEasyBox(1, 1, 1, 1000)]

    def emit_particle(self):
        particle = self.shapes[np.random.randint(0, len(self.shapes))].copy()
        particle.SetPos(chrono.ChVectorD(np.random.uniform(-5, 5), 10, np.random.uniform(-5, 5)))
        particle.SetRot(chrono.ChQuaternionD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)).GetNormalized())
        particle.SetPos_dt(chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 0), np.random.uniform(-1, 1)))
        particle.SetWvel_loc(chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
        my_system.Add(particle)


emitter = ParticleEmitter()


myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono Particle Attraction', chronoirr.dimension2du(800, 600))


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    emitter.emit_particle()  
    myapplication.EndScene()


for body in my_system.Get_bodylist():
    for other_body in my_system.Get_bodylist():
        if body != other_body:
            force = chrono.ChVectorD.Zero()
            distance = body.GetPos() - other_body.GetPos()
            force.x = distance.x * 10000
            force.y = distance.y * 10000
            force.z = distance.z * 10000
            body.Accumulate_force(force, body.GetPos(), True)

myapplication.Close()