import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




ground = chrono.ChBodyEasyBox(4, 0.2, 4, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = chrono.ChBodyEasySphere(0.2, 1, True, True)  
pendulum.SetPos(chrono.ChVectorD(0, 1.5, 0))
system.Add(pendulum)


revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0)))
system.Add(revolute)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.5, 2))
vis.AddTypicalLights()
vis.AssetBindAll()
vis.AssetUpdateAll()


time_step = 0.01
sim_time = 10.0
output_step = 0.1  


current_time = 0
output_time = 0
while current_time < sim_time:
    
    system.DoStepDynamics(time_step)
    current_time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if current_time >= output_time:
        
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()

        
        angle = math.atan2(pos.x, 1.5 - pos.y)  

        print(f"Time: {current_time:.2f}s | Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | "
              f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f}) | Angle: {math.degrees(angle):.1f}°")

        output_time += output_step

    
    if vis.Run() == 0:
        break