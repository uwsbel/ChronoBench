# Import necessary modules
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# Set the simulation parameters
gears_radius = [0.2, 0.1]  # radius of the gears
gears_thickness = 0.05  # thickness of the gears
bar_length = 0.5  # length of the rotating bar
truss_length = 1.0  # length of the fixed truss
motor_speed = 2.0  # rotation speed of the gear motor

# Initialize the Chrono environment
my_system = chrono.ChSystemNSC()

# Create the Irrlicht visualization system
application = chronoirr.ChIrrApp(my_system, 'Epicyclic Gears Simulation', chronoirr.dimension2du(800, 600))

# Create the fixed truss
truss_body = chrono.ChBodyEasyBox(my_system, truss_length, 0.1, 0.1, 1000, True)
truss_body.SetPos(chrono.ChVectorD(0, 0, 0))
truss_body.SetBodyFixed(True)
my_system.Add(truss_body)

# Create the rotating bar
bar_body = chrono.ChBodyEasyBox(my_system, bar_length, 0.05, 0.05, 100, False)
bar_body.SetPos(chrono.ChVectorD(0, 0, 0.5))
bar_body.SetMass(10)
my_system.Add(bar_body)

# Create the gears
gear1_body = chrono.ChBodyEasyCylinder(my_system, gears_radius[0], gears_thickness, 100, False)
gear1_body.SetPos(chrono.ChVectorD(-0.25, 0, 0.5))
gear1_body.SetMass(5)
my_system.Add(gear1_body)

gear2_body = chrono.ChBodyEasyCylinder(my_system, gears_radius[1], gears_thickness, 100, False)
gear2_body.SetPos(chrono.ChVectorD(0.25, 0, 0.5))
gear2_body.SetMass(5)
my_system.Add(gear2_body)

# Create the gear motor
motor = chrono.ChLinkMotorRotationSpeed(my_system)
motor.Initialize(gear1_body, gear2_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5)))
motor.SetMotorFunction(chrono.ChFunction_Ramp(0, motor_speed))
my_system.Add(motor)

# Add constraints to connect the gears to the rotating bar
bar_gear1_joint = chrono.ChLinkRevolute(my_system)
bar_gear1_joint.Initialize(bar_body, gear1_body, chrono.ChFrameD(chrono.ChVectorD(-0.25, 0, 0.5)))
my_system.Add(bar_gear1_joint)

bar_gear2_joint = chrono.ChLinkRevolute(my_system)
bar_gear2_joint.Initialize(bar_body, gear2_body, chrono.ChFrameD(chrono.ChVectorD(0.25, 0, 0.5)))
my_system.Add(bar_gear2_joint)

# Add a constraint to connect the rotating bar to the fixed truss
truss_bar_joint = chrono.ChLinkRevolute(my_system)
truss_bar_joint.Initialize(truss_body, bar_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5)))
my_system.Add(truss_bar_joint)

# Set visualization assets
application.AssetBindAll()
application.AssetUpdateAll()

# Set the camera position
application.SetCamera(chronoirr.vector3df(1, 1, 1), chronoirr.vector3df(0, 0, 0))

# Simulate the system
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()