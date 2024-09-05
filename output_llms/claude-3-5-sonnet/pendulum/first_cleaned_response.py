import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath("C:/Program Files/Chrono/data/")
system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True)
ground.SetBodyFixed(True)
system.Add(ground)


pendulum_length = 1.0
pendulum_mass = 1.0
pendulum_radius = 0.05

pendulum = chrono.ChBodyEasyCylinder(pendulum_radius, pendulum_length, pendulum_mass)
pendulum.SetPos(chrono.ChVectorD(0, -pendulum_length/2, 0))
pendulum.SetInertiaXX(chrono.ChVectorD(1/12 * pendulum_mass * (3*pendulum_radius**2 + pendulum_length**2), 
                                       1/2 * pendulum_mass * pendulum_radius**2,
                                       1/12 * pendulum_mass * (3*pendulum_radius**2 + pendulum_length**2)))
system.Add(pendulum)


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X)))
system.AddLink(joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, 3))
vis.AddTypicalLights()


time_step = 0.01
total_time = 10.0
current_time = 0

while current_time < total_time:
    system.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if int(current_time / time_step) % 10 == 0:  
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {current_time:.2f}, Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), "
              f"Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")
    
    current_time += time_step


vis.GetDevice().closeDevice()

print("Simulation completed.")