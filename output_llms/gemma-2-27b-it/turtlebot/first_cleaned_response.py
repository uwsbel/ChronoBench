import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math







my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
my_system.Add(ground)







robot = chrono.ChTurtlebot()


robot.SetPos(chrono.ChVectorD(0, 0.5, 0))
robot.SetRot(chrono.Q_from_AngAxis(math.pi / 4, chrono.ChVectorD(0, 0, 1)))


my_system.Add(robot)







vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Turtlebot Simulation")


camera = vis.GetCamera()
camera.SetLocation(chrono.ChVectorD(2, 2, 2))
camera.SetLookAt(chrono.ChVectorD(0, 0.5, 0))
vis.AddLight(chrono.ChLight(chrono.ChVectorD(2, 5, 2)))







time_step = 0.01
simulation_time = 10


left_motor_speed = 10
right_motor_speed = -10
motor_change_time = 3


while my_system.GetChTime() < simulation_time:
    
    if my_system.GetChTime() < motor_change_time:
        robot.SetWheelSpeed(left_motor_speed, right_motor_speed)
    else:
        robot.SetWheelSpeed(-left_motor_speed, -right_motor_speed)

    
    my_system.DoStepDynamics(time_step)

    
    vis.Render()