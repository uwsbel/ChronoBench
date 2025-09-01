import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("path/to/chrono/data")  
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


pendulum_mass = 1.0
pendulum_length = 1.0
pendulum_inertia = chrono.ChVectorD(0.1, 0.1, 0.1)  
pendulum = chrono.ChBodyEasySphere(pendulum_mass, pendulum_inertia)
pendulum.SetPos(chrono.ChVectorD(0, pendulum_length, 0))
system.Add(pendulum)


joint = chrono.ChLinkLockRevolute(ground, pendulum,
                                  chrono.ChVectorD(0, 0, 0),
                                  chrono.ChVectorD(0, 0, 1))
system.Add(joint)


vis.Initialize(chrono.ChVectorD(800, 600, 0))
vis.Add(ground)
vis.Add(pendulum)
vis.SetCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))


dt = 0.01
while True:
    system.DoStepDynamics(dt)

    
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetRotSpeed()
    print(f"Position: {pendulum_pos}, Velocity: {pendulum_vel}")

    vis.Render()