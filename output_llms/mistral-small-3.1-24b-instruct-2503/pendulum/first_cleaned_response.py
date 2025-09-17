import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('path/to/chrono/data')  


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))  
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))  
system.Add(pendulum)


revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(revolute)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddLightWithShadow(chrono.ChVectorD(1.0, -1.0, 1.0), chrono.ChVectorD(0.8, 0.8, 0.8), 1000)


camera = vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
camera.SetNearFarPlane(0.1, 100)


step_size = 0.01
end_time = 10.0
current_time = 0.0

while current_time < end_time:
    system.DoStepDynamics(step_size)
    vis.Render()
    vis.Update()

    
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetPos_dt()
    print(f"Time: {current_time:.2f}, Position: {pendulum_pos}, Velocity: {pendulum_vel}")

    current_time += step_size


vis.Close()