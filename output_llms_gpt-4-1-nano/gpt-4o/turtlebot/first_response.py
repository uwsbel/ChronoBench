import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot

# Initialize the Chrono system
chrono.SetChronoDataPath("path_to_chrono_data")

# Create the Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Initialize the Turtlebot
turtlebot = robot.ChTurtlebot()
turtlebot.SetChassisPosition(chrono.ChVectorD(0, 0, 0))
turtlebot.SetChassisRotation(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
system.Add(turtlebot.GetChassisBody())
system.Add(turtlebot.GetLeftWheelBody())
system.Add(turtlebot.GetRightWheelBody())

# Create the Irrlicht application for visualization
application = chronoirr.ChIrrApp(system, "Turtlebot Simulation", chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 2, -6))
application.AddTypicalLights()

# Simulation parameters
time_step = 0.01
left_motor_speed = 1.0  # Speed for left motor
right_motor_speed = 1.0  # Speed for right motor
turn_duration = 2.0  # Duration for each turn

# Simulation loop
application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(time_step)

current_time = 0.0
turning_left = True

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Control the Turtlebot's motors
    if current_time < turn_duration:
        if turning_left:
            turtlebot.SetMotorSpeeds(left_motor_speed, -right_motor_speed)
        else:
            turtlebot.SetMotorSpeeds(-left_motor_speed, right_motor_speed)
    else:
        turning_left = not turning_left
        current_time = 0.0

    # Update the simulation
    application.DoStep()
    current_time += time_step

    application.EndScene()