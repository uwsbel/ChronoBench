import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath("/path/to/chrono/data/")  


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))
pendulum.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(pendulum)


rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
sys.Add(rev_joint)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if sys.GetChTime() % 0.1 < 0.01:
        print(f"Time: {sys.GetChTime()}, Pendulum Pos: {pendulum.GetPos()}, Vel: {pendulum.GetPos_dt()}")
    
    sys.DoStepDynamics(0.01)