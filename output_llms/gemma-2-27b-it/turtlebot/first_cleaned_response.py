import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math







my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
my_system.Add(ground)







robot_path = "path/to/turtlebot.obj"
robot_asset = robot.ChRobotAsset(robot_path)
robot = robot.ChRobot(robot_asset)


robot.SetPos(chrono.ChVectorD(0, 0.5, 0))
robot.SetRot(chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 0, 1)))


my_system.Add(robot)







myapplication = chronoirr.ChIrrApp(my_system, "Turtlebot Simulation", chronoirr.dimension2du(1280, 720))


myapplication.AddTypicalCamera(chronoirr.vector3df(2, 2, 2), chronoirr.vector3df(0, 0, 0))


myapplication.AddLightWithShadow(chronoirr.vector3df(1, 1, 1), chronoirr.vector3df(0, 0, 0), 1000, 50)







timestep = 0.01
simulation_time = 10

while myapplication.GetDevice().run() and my_system.GetChTime() < simulation_time:
    
    if my_system.GetChTime() < 3:
        robot.GetMotorizedLink("left_wheel").SetSpeed(10)
        robot.GetMotorizedLink("right_wheel").SetSpeed(10)
    elif my_system.GetChTime() < 6:
        robot.GetMotorizedLink("left_wheel").SetSpeed(-10)
        robot.GetMotorizedLink("right_wheel").SetSpeed(10)
    else:
        robot.GetMotorizedLink("left_wheel").SetSpeed(0)
        robot.GetMotorizedLink("right_wheel").SetSpeed(0)

    
    my_system.DoStepDynamics(timestep)

    
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()


myapplication.GetDevice().closeDevice()