import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


def create_particle(sys):
    
    shape = np.random.choice(['Sphere', 'Box', 'Cylinder'])
    pos = np.random.uniform(-1, 1, size=3)
    vel = np.random.uniform(-1, 1, size=3)
    rot = chrono.ChQuaternionD(np.random.uniform(-1, 1, size=4))

    
    if shape == 'Sphere':
        particle = chrono.ChBodyEasySphere(sys, 0.1, 1.0, pos)
    elif shape == 'Box':
        particle = chrono.ChBodyEasyBox(sys, 0.1, 0.1, 0.1, 1.0, pos)
    elif shape == 'Cylinder':
        particle = chrono.ChBodyEasyCylinder(sys, 0.1, 0.1, 1.0, pos)

    
    particle.SetPos(chrono.ChVectorD(pos))
    particle.SetVel(chrono.ChVectorD(vel))
    particle.SetRot(rot)

    
    sys.AddBody(particle)


for _ in range(10):
    create_particle(sys)


class GravitationalForce(chrono.ChForce):
    def __init__(self, sys):
        super().__init__()
        self.sys = sys

    def Update(self, time, dt):
        for i in range(self.sys.GetNbodies()):
            for j in range(i+1, self.sys.GetNbodies()):
                body_i = self.sys.GetBody(i)
                body_j = self.sys.GetBody(j)
                pos_i = body_i.GetPos()
                pos_j = body_j.GetPos()
                dist = (pos_i - pos_j).Length()
                force = 0.1 * (pos_j - pos_i) / dist**3
                body_i.AddForce(force)
                body_j.AddForce(-force)


sys.AddForce(GravitationalForce(sys))


app = chronoirr.ChIrrApp(sys, 'Gravitational Attraction', chronoirr.dimension2du(800, 600))


app.AddTypicalSky()
app.AddTypicalCamera(chronoirr.vector3df(0, 0, -3))
app.AddLightWithShadow(chronoirr.vector3df(2, 4, 2), chronoirr.vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512)


app.GetDevice().run()