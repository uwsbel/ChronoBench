import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# -------------------------------------------------------------------
# Create the Chrono system and set default parameters
# -------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# -------------------------------------------------------------------
# Create the ground body
# -------------------------------------------------------------------
ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)

# -------------------------------------------------------------------
# Create the Turtlebot robot
# -------------------------------------------------------------------
# Import the Chrono::Vehicle library
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)
chrono.SetChronoDataPath('../data/')

# Create the Turtlebot vehicle (replace 'path/to/turtlebot/' with the actual path)
vehicle = chrono.ChVehicleModelDataShared('path/to/turtlebot/') 
my_turtlebot = chrono.ChVehicle(vehicle, chrono.ChMaterialSurface.NSC)
my_turtlebot.SetChassisFixed(False)
my_turtlebot.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.4, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(my_turtlebot)

# -------------------------------------------------------------------
# Create the Irrlicht visualization
# -------------------------------------------------------------------
application = chronoirr.ChIrrApp(system)
application.AddTypicalSky()
application.AddTypicalLights()
application.SetShowInfos(True)
application.SetCameraPos(chrono.ChVectorD(3, 1, 3), chrono.ChVectorD(0, 0, 0))

# -------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------
time_step = 0.01
time_end = 10

# Motor control parameters
turn_left_start = 2  # Time to start turning left
turn_left_end = 4    # Time to stop turning left
turn_right_start = 6  # Time to start turning right
turn_right_end = 8    # Time to stop turning right

while application.GetDevice().run():
    time = system.GetChTime()

    # Apply motor control based on time
    if turn_left_start <= time < turn_left_end:
        my_turtlebot.GetVehicle().GetMotor(0).SetMotorSpeed(2)  # Left motor
        my_turtlebot.GetVehicle().GetMotor(1).SetMotorSpeed(-2) # Right motor
    elif turn_right_start <= time < turn_right_end:
        my_turtlebot.GetVehicle().GetMotor(0).SetMotorSpeed(-2) # Left motor
        my_turtlebot.GetVehicle().GetMotor(1).SetMotorSpeed(2)  # Right motor
    else:
        my_turtlebot.GetVehicle().GetMotor(0).SetMotorSpeed(0)  # Left motor
        my_turtlebot.GetVehicle().GetMotor(1).SetMotorSpeed(0)  # Right motor

    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)

    if time >= time_end:
        break