import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('../data/')


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
system.AddBody(ground)


pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))  
pendulum.SetMass(1.0)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))  
pendulum.SetCollide(True)
system.AddBody(pendulum)


revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(revolute)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))


step_size = 0.01
total_time = 10.0


simulation_time = 0.0
log_interval = 0.1
log_time = 0.0


while simulation_time < total_time:
    system.DoStepDynamics(step_size)
    simulation_time += step_size

    
    if simulation_time >= log_time:
        pendulum_pos = pendulum.GetPos()
        pendulum_vel = pendulum.GetPos_dt()
        print(f"Time: {simulation_time:.2f}, Position: {pendulum_pos}, Velocity: {pendulum_vel}")
        log_time += log_interval

    
    vis.Run()


vis.Close()