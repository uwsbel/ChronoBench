import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.SetSymbolScale(0.01)
vis.SetShowInfos(True)
vis.SetCamera(chronoirr.ChCameraSettings(chrono.ChVectorD(0, 3, 5), chrono.ChVectorD(0, 0, 0)))
vis.SetCameraRotation(chrono.Q_from_Euler123(chrono.ChVectorD(0, math.pi/4, 0)))
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(ground)


ground_shape = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(ground_shape)


pendulum_length = 2.0  
pendulum_radius = 0.1  
desired_mass = 1.0     


volume = (4.0/3.0) * math.pi * (pendulum_radius)**3
density = desired_mass / volume


pendulum = chrono.ChBodyEasySphere(pendulum_radius, density, True, True)
pendulum.SetPos(chrono.ChVectorD(0.5, -pendulum_length, 0))  
pendulum.SetMaterialSurface(chrono.ChMaterialSurface())  
sys.Add(pendulum)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(rev_joint)


time_step = 0.001  
log_interval = 0.1  

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(time_step)
    
    current_time = sys.GetChTime()
    if current_time % log_interval < time_step:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {current_time:.2f}s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")


vis.Run()