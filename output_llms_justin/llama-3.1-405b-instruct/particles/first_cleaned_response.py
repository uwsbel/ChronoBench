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
        particle = chrono.ChBodyEasySphere(sys, 0.1, 1000, True, True)
    elif shape == 'Box':
        particle = chrono.ChBodyEasyBox(sys, 0.1, 0.1, 0.1, 1000, True, True)
    elif shape == 'Cylinder':
        particle = chrono.ChBodyEasyCylinder(sys, 0.1, 0.1, 1000, True, True)

    
    particle.SetPos(chrono.ChVectorD(*pos))
    particle.SetVel(chrono.ChVectorD(*vel))
    particle.SetRot(rot)

    
    sys.Add(particle)


for _ in range(10):
    create_particle(sys)


def gravitational_force(p1, p2):
    G = 6.67430e-11  
    m1 = p1.GetMass()
    m2 = p2.GetMass()
    r = (p2.GetPos() - p1.GetPos()).Length()
    F = G * m1 * m2 / r**2
    return chrono.ChVectorD(F * (p2.GetPos() - p1.GetPos()).Normalized())


for i in range(sys.GetNbodies()):
    for j in range(i+1, sys.GetNbodies()):
        p1 = sys.GetBody(i)
        p2 = sys.GetBody(j)
        force = gravitational_force(p1, p2)
        p1.AddForce(force)
        p2.AddForce(-force)


app = chronoirr.ChIrrApp(sys, "PyChrono Simulation", chronoirr.dimension2du(800, 600))


app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(0, 0, -3))


app.GetDevice().run()


app.GetDevice().closeDevice()