import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/your/chrono/data")  
chrono.SetChronoDataPath("/path/to/your/irrlicht/data")  

my_system = chrono.ChSystemNSC()



ground = chrono.ChBodyEasyBox(2, 2, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))
my_system.Add(ground)


mass = 1
inertia = chrono.ChVectorD(1/12 * mass * (0.1**2 + 0.5**2), 1/12 * mass * (0.1**2 + 0.5**2), 1/12 * mass * (0.5**2))
pendulum = chrono.ChBodyEasyCylinder(0.05, 0.5, mass, True, True, inertia)
pendulum.SetPos(chrono.ChVectorD(0, -0.5, 0))
my_system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
my_system.Add(joint)


pendulum.SetPos_dt(chrono.ChVectorD(0, 0.1, 0))
pendulum.SetWvel_par(chrono.ChVectorD(0, 0, 1))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -1.5))
vis.AddTypicalLights()


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    print("Pendulum position: ", pendulum.GetPos())
    print("Pendulum velocity: ", pendulum.GetPos_dt())