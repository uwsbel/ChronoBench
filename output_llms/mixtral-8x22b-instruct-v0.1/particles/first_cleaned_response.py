import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data")  
chrono.SetChronoDataPath("/path/to/irrlicht/data")  

my_system = chrono.ChSystemNSC()


class ParticleEmitter:
    def __init__(self):
        self.shapes = [chrono.ChBodyEasySphere(1), chrono.ChBodyEasyBox(1, 1, 1), chrono.ChBodyEasyCylinder(1, 1)]

    def emit_particle(self):
        shape = np.random.choice(self.shapes)
        shape.SetPos(chrono.ChVectorD(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(-10, 10)))
        shape.SetPos_dt(chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
        shape.SetWvel_par(chrono.ChVectorD(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(-10, 10)))
        my_system.Add(shape)

emitter = ParticleEmitter()


def apply_gravitational_forces(my_system):
    for body in my_system.Get_bodylist():
        for other_body in my_system.Get_bodylist():
            if body != other_body:
                force = chrono.ChVectorD(0, 0, 0)
                distance = body.GetPos() - other_body.GetPos()
                force_magnitude = 10 * body.GetMass() * other_body.GetMass() / np.linalg.norm(distance) ** 2
                force = force_magnitude * distance.GetNormalized()
                body.Accumulate_force(force, body.GetPos(), False)


application = chronoirr.ChIrrApp(my_system, "PyChrono Particle Simulation", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, -10))
application.AddLightWithShadow(chronoirr.vector3df(10, 10, 10), chronoirr.vector3df(0, 0, 0), 1, 10, 50, 32)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    if application.GetTimer() > 1:
        emitter.emit_particle()
        application.GetTimer() = 0

    
    apply_gravitational_forces(my_system)

    my_system.DoStepDynamics(0.01)
    application.EndScene()

chrono.ChEnd()