import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("data/")  


world = chrono.ChSystemNSC()


ground = chrono.ChBodyEasySphere(1.0, 100.0)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
world.AddBody(ground)


pendulum_mass = 1.0
pendulum_radius = 0.1
pendulum_inertia = chrono.ChVectorD(pendulum_mass * pendulum_radius**2, 0, 0)
pendulum = chrono.ChBodyEasyCylinder(pendulum_mass, pendulum_radius, 0.5)
pendulum.SetPos(chrono.ChVectorD(0, 0.5, 0))
pendulum.SetInertiaXX(pendulum_inertia)
world.AddBody(pendulum)


joint = chrono.ChLinkRevolute(ground, pendulum,
                            chrono.ChVectorD(0, 0.5, 0),
                            chrono.ChVectorD(0, 0, 1))
world.AddLink(joint)


vis.Initialize(chrono.GetChWorld(), "Simple Pendulum")


dt = 0.01
while True:
    world.DoStepDynamics(dt)
    
    
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetRotVel()
    print(f"Position: {pendulum_pos}, Velocity: {pendulum_vel}")
    
    vis.Render()