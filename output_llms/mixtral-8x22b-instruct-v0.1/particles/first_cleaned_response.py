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


def custom_gravitational_force(body1, body2):
    force = chrono.ChVectorD(0, -6.67430e-11 * body1.GetMass() * body2.GetMass() / (body1.GetPos() - body2.GetPos()).LengthSquared(), 0)
    body1.Accumulate_force(force, body1.GetPos(), True)
    body2.Accumulate_force(-force, body2.GetPos(), True)


visualizer = chronoirr.ChIrrApp(my_system, 'PyChrono Simulation', chronoirr.dimension2du(800, 600))
visualizer.AddTypicalSky()
visualizer.AddTypicalLogo()
visualizer.AddTypicalCamera(chronoirr.vector3df(0, 4, -3))
visualizer.AddLightWithShadow(chronoirr.vector3df(2, 4, -2), chronoirr.vector3df(2, 2, -2), 3, 2, 20, 50, 100)


while visualizer.Run():
    my_system.DoStepDynamics(0.01)
    visualizer.BeginScene()
    visualizer.DrawAll()
    visualizer.EndScene()

    
    if np.random.rand() < 0.01:
        emitter.emit_particle()

    
    for body1 in my_system.Get_bodylist():
        for body2 in my_system.Get_bodylist():
            if body1 != body2:
                custom_gravitational_force(body1, body2)


visualizer.Close()