import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000)  
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))  
ground.SetBodyFixed(True)  
system.Add(ground)


bob_radius = 0.1
bob = chrono.ChBodyEasySphere(bob_radius, 1000)  
bob.SetPos(chrono.ChVectorD(0, 2, 0))  
system.Add(bob)


pivot_point = chrono.ChVectorD(0, 3, 0)  
joint = chrono.ChLinkLockRevolute()  
joint.Initialize(ground, bob, chrono.ChCoordsysD(pivot_point))
system.Add(joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 5), chrono.ChVectorD(0, 1.5, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 1.5, 0), 3, 1, 10, 40, 512)


ground.GetVisualModel().AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))
bob.GetVisualModel().AddAsset(chrono.ChColorAsset(0.8, 0.2, 0.2))


time_step = 0.01
sim_time = 10.0
output_step = 0.1


while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.ChIrrTools.DrawAllCOGs(vis, 0.1)  
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    current_time = system.GetChTime()
    if math.fmod(current_time, output_step) < time_step:
        bob_pos = bob.GetPos()
        bob_vel = bob.GetPos_dt()
        print(f"Time: {current_time:.2f}s | Position: ({bob_pos.x:.2f}, {bob_pos.y:.2f}, {bob_pos.z:.2f}) | Velocity: ({bob_vel.x:.2f}, {bob_vel.y:.2f}, {bob_vel.z:.2f})")

    
    if current_time > sim_time:
        break