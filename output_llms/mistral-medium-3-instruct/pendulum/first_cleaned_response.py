import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -2, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


pendulum_mass = 1.0
pendulum_radius = 0.1
pendulum_length = 2.0
pendulum_inertia = 0.5 * pendulum_mass * pendulum_radius * pendulum_radius  

pendulum = chrono.ChBodyEasySphere(pendulum_radius, pendulum_mass, True, True)
pendulum.SetPos(chrono.ChVectorD(pendulum_length, 0, 0))
pendulum.SetInertiaXX(chrono.ChVectorD(pendulum_inertia, pendulum_inertia, pendulum_inertia))
my_system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
my_system.Add(joint)


my_application = chronoirr.ChIrrApp(my_system, 'Pendulum Simulation', chronoirr.dimension2du(800, 600))
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalLights()
my_application.AddTypicalCamera(chrono.ChVectorD(0, 1, 5))
my_application.AssetBindAll()
my_application.AssetUpdateAll()


time_step = 0.01
simulation_time = 10.0
output_step = 0.1


current_time = 0
output_counter = 0
while current_time < simulation_time:
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStepDynamics(time_step)
    my_application.EndScene()

    
    if current_time >= output_counter * output_step:
        print(f"Time: {current_time:.2f} s")
        print(f"Position: {pendulum.GetPos()}")
        print(f"Velocity: {pendulum.GetPos_dt()}")
        print("-------------------")
        output_counter += 1

    current_time += time_step