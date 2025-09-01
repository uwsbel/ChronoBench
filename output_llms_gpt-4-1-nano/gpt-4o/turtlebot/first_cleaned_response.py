import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


chrono.SetChronoDataPath("path_to_chrono_data")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


turtlebot = robot.ChTurtlebot()
turtlebot.SetChassisPosition(chrono.ChVectorD(0, 0, 0))
turtlebot.SetChassisRotation(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
system.Add(turtlebot.GetChassisBody())
system.Add(turtlebot.GetLeftWheelBody())
system.Add(turtlebot.GetRightWheelBody())


application = chronoirr.ChIrrApp(system, "Turtlebot Simulation", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 2, -6))
application.AddTypicalLights()


time_step = 0.01
left_motor_speed = 1.0  
right_motor_speed = 1.0  
turn_duration = 2.0  


application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(time_step)

current_time = 0.0
turning_left = True

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    if current_time < turn_duration:
        if turning_left:
            turtlebot.SetMotorSpeeds(left_motor_speed, -right_motor_speed)
        else:
            turtlebot.SetMotorSpeeds(-left_motor_speed, right_motor_speed)
    else:
        turning_left = not turning_left
        current_time = 0.0

    
    application.DoStep()
    current_time += time_step

    application.EndScene()