import pychrono as chrono
import pychrono.irrlicht as chronoirr







my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -2, 0))
my_system.Add(ground)







pendulum_length = 1.0
pendulum_mass = 1.0

pendulum = chrono.ChBodyEasySphere(0.2, 1000, True, True)
pendulum.SetPos(chrono.ChVectorD(0, pendulum_length, 0))


inertia = chrono.ChVectorD(
    0.4 * pendulum_mass * 0.2**2, 0.4 * pendulum_mass * 0.2**2, 0.4 * pendulum_mass * 0.2**2
)
pendulum.SetInertia(inertia)

my_system.Add(pendulum)







joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
my_system.Add(joint)







vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.Initialize()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(5, 5, 5))







while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    my_system.DoStepDynamics(0.01)

    
    print(f"Pendulum position: {pendulum.GetPos()}")
    print(f"Pendulum velocity: {pendulum.GetLinVel()}")