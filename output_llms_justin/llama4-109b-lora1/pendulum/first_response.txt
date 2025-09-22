import pychrono as chrono
import pychrono.irrlicht as irr
import math

# Initialize PyChrono environment and core components
chrono.SetChairsThreading(chrono.GetNumProcs())
chrono.SetChronoDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create the simulation system and add all components
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
system.Add(ground)

# Create the pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVector3d(1, 1, 0))
system.Add(pendulum)

# Create a revolute joint between the ground and pendulum
joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
system.Add(joint)

# Create the Irrlicht application
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

# Simulation loop
while vis.Run() :
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)
    print('Pendulum position: ', pendulum.GetPos())
    print('Pendulum velocity: ', pendulum.GetPos_dt())