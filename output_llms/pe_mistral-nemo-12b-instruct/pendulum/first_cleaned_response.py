import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', '..', '..', '..', 'data'))


sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, -5))
vis.AddTypicalLights()


material = chrono.ChContactMaterial()
material.SetFriction(0.4)
material.SetDampingF(0.1)
material.SetCompliance(0.01)
sys.AddContactMaterial(material)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 10, 10))
ground.AddVisualShape(ground_shape)
sys.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.01))
pendulum.SetPos(chrono.ChVector3d(0, 1, 0))
pendulum_shape = chrono.ChVisualShapeSphere(0.5)
pendulum.AddVisualShape(pendulum_shape, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))
sys.Add(joint)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(0.01)
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetPos_dt()
    print(f'Pendulum position: {pendulum_pos}, Velocity: {pendulum_vel}')

    
    if pendulum_pos.y < 0.5:
        break


vis.GetDevice().closeDevice()