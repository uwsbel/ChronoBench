import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
pendulum.SetPos(chrono.ChVectorD(0, -1, 0))  
pendulum.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
sys.Add(pendulum)


rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.AddLink(rev_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, -3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)  
    
    
    if sys.GetChTime() % 0.1 < 0.01:  
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {sys.GetChTime():.2f}, Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")

    
    if vis.GetDevice().isWindowActive():
        if vis.GetDevice().getEventReceiver().KeyIsPressed(irrlicht.EKEY_ESCAPE):
            break