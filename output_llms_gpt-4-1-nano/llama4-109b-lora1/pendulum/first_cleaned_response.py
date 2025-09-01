import pychrono as chrono
import pychrono.irrlicht as irr
import math


chrono.SetChairsThreading(chrono.GetNumProcs())
chrono.SetChronoDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
system.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVector3d(1, 1, 0))
system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
system.Add(joint)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.SetWindowSize(800, 600)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 5), chrono.ChVector3d(0, 0, 0))
vis.AddLightDirectional()
vis.AddLightPoint(chrono.ChVector3d(0, 0, 5), chrono.ChVector3d(0, 0, 0), 500, 500, 500)


while vis.Run() :
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)
    print('Pendulum position: ', pendulum.GetPos())
    print('Pendulum velocity: ', pendulum.GetPos_dt())