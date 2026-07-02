import pychrono as chrono
import math

# Create the PyChrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create ground body
ground = chrono.ChBodyEasyBox(100, 100, 0.1, 1000, True, False)
ground.SetName("Rigid terrain")
ground.SetFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
system.Add(ground)

# Create Turtlebot chassis
chassis = chrono.ChBodyEasyBox(0.4, 0.2, 0.1, 1000, True, False)
chassis.SetName("Turtlebot chassis")
chassis.SetMass(10.0)
chassis.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
chassis.SetPos(chrono.ChVector3d(0, 0, 0.1))
chassis.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.4, 0.8))
system.Add(chassis)

# Create left and right wheels
wheel_radius = 0.08
wheel_mass = 1.0
wheel_inertia = 0.5 * wheel_mass * wheel_radius * wheel_radius

left_wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, wheel_radius, 0.15, 1000, True, False)
left_wheel.SetName("Left wheel")
left_wheel.SetMass(wheel_mass)
left_wheel.SetInertiaXX(chrono.ChVector3d(wheel_inertia, wheel_inertia, wheel_inertia))
left_wheel.SetPos(chrono.ChVector3d(-0.15, 0, 0.1))
left_wheel.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.2))
system.Add(left_wheel)

right_wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, wheel_radius, 0.15, 1000, True, False)
right_wheel.SetName("Right wheel")
right_wheel.SetMass(wheel_mass)
right_wheel.SetInertiaXX(chrono.ChVector3d(wheel_inertia, wheel_inertia, wheel_inertia))
right_wheel.SetPos(chrono.ChVector3d(0.15, 0, 0.1))
right_wheel.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.2))
system.Add(right_wheel)

# Create joints between chassis and wheels
left_joint = chrono.ChLinkLockRevolute()
left_joint.SetName("Left wheel joint")
left_joint.Initialize(chassis, left_wheel, chrono.ChFramed(chrono.ChVector3d(-0.15, 0, 0.1), chrono.QUNIT))
system.Add(left_joint)

right_joint = chrono.ChLinkLockRevolute()
right_joint.SetName("Right wheel joint")
right_joint.Initialize(chassis, right_wheel, chrono.ChFramed(chrono.ChVector3d(0.15, 0, 0.1), chrono.QUNIT))
system.Add(right_joint)

# Create motor systems for wheels
left_motor = chrono.ChLinkMotorRotationSpeed()
left_motor.SetName("Left wheel motor")
left_motor.Initialize(left_wheel, chassis, chrono.ChFramed(chrono.ChVector3d(-0.15, 0, 0.1), chrono.QUNIT))
system.Add(left_motor)

right_motor = chrono.ChLinkMotorRotationSpeed()
right_motor.SetName("Right wheel motor")
right_motor.Initialize(right_wheel, chassis, chrono.ChFramed(chrono.ChVector3d(0.15, 0, 0.1), chrono.QUNIT))
system.Add(right_motor)

# Create visualization
visual_system = chrono.ChVisualSystemIrrlicht()
visual_system.AttachSystem(system)
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle("PyChrono Turtlebot Simulation")
visual_system.Initialize()
visual_system.AddSkyBox()
visual_system.AddCamera(chrono.ChFramed(chrono.ChVector3d(0, 2, 3), chrono.Q_ROTATE_Z_TO_X))
visual_system.AddLightDirectional(chrono.ChVector3d(0, 0, 1), chrono.ChColor(0.8, 0.8, 0.8))

# Simulation loop
time = 0.0
step = 0.001
while visual_system.Run():
    time += step
    visual_system.BeginScene()
    visual_system.Render()
    visual_system.EndScene()
    system.DoStepDynamics(step)
    
    # Motor control logic
    if time < 2.0:
        left_motor.SetSpeedTarget(0.0)
        right_motor.SetSpeedTarget(0.0)
    elif time < 3.0:
        left_motor.SetSpeedTarget(-10.0)
        right_motor.SetSpeedTarget(10.0)
    elif time < 4.0:
        left_motor.SetSpeedTarget(10.0)
        right_motor.SetSpeedTarget(-10.0)
    else:
        left_motor.SetSpeedTarget(0.0)
        right_motor.SetSpeedTarget(0.0)