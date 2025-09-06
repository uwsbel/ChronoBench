import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
from math import pi

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation time parameters
time_step = 0.01
end_time = 30.0

# Terrain dimensions
terrain_length = 20.0
terrain_width = 20.0
terrain_height = 0.5

# Rover parameters
rover_length = 1.0
rover_width = 0.8
rover_height = 0.5
wheel_radius = 0.2
wheel_width = 0.1

# =============================================================================
# Create the physical system
# =============================================================================

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set solver settings
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetMaxItersSolverSpeed(50)
system.SetMaxItersSolverStab(50)
system.SetTolForce(1e-5)

# =============================================================================
# Create the terrain
# =============================================================================

# Create a flat rigid terrain
terrain = chrono.ChBodyEasyBox(terrain_length, terrain_height, terrain_width, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -terrain_height/2, 0))
terrain.SetBodyFixed(True)
terrain.SetCollide(True)
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurfaceNSC().SetFriction(0.8)
system.Add(terrain)

# =============================================================================
# Create the rover chassis
# =============================================================================

# Create the rover chassis
rover_chassis = chrono.ChBodyEasyBox(rover_length, rover_height, rover_width, 50, True, True)
rover_chassis.SetPos(chrono.ChVectorD(0, rover_height/2 + wheel_radius, 0))
rover_chassis.SetCollide(True)
rover_chassis.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
rover_chassis.GetMaterialSurfaceNSC().SetFriction(0.5)
system.Add(rover_chassis)

# =============================================================================
# Create the wheels
# =============================================================================

# Wheel positions (relative to chassis center)
wheel_positions = [
    chrono.ChVectorD(rover_length/2 - wheel_width/2, 0, rover_width/2 - wheel_width/2),  # Front right
    chrono.ChVectorD(rover_length/2 - wheel_width/2, 0, -rover_width/2 + wheel_width/2), # Front left
    chrono.ChVectorD(-rover_length/2 + wheel_width/2, 0, rover_width/2 - wheel_width/2), # Rear right
    chrono.ChVectorD(-rover_length/2 + wheel_width/2, 0, -rover_width/2 + wheel_width/2)  # Rear left
]

wheels = []
wheel_motors = []

for i, pos in enumerate(wheel_positions):
    # Create wheel body
    wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 1000, True, True)
    wheel.SetPos(rover_chassis.GetPos() + pos)
    wheel.SetCollide(True)
    wheel.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    wheel.GetMaterialSurfaceNSC().SetFriction(0.8)
    system.Add(wheel)
    wheels.append(wheel)

    # Create motor between chassis and wheel
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(rover_chassis, wheel, chrono.ChFrameD(pos, chrono.QUNIT))
    motor.SetSpeedFunction(chrono.ChFunction_Const(0))  # Will be controlled later
    system.Add(motor)
    wheel_motors.append(motor)

    # Create suspension (simple spring-damper)
    suspension = chrono.ChLinkLockRevolute()
    suspension.Initialize(rover_chassis, wheel, chrono.ChFrameD(pos, chrono.QUNIT))
    system.Add(suspension)

# =============================================================================
# Motor control driver
# =============================================================================

class RoverMotorDriver(chrono.ChReportContactCallback):
    def __init__(self, wheel_motors):
        super().__init__()
        self.wheel_motors = wheel_motors
        self.steering_angle = 0.0
        self.throttle = 0.0

    def SetSteering(self, angle):
        self.steering_angle = angle

    def SetThrottle(self, throttle):
        self.throttle = throttle

    def OnReportContact(self, contact_info):
        # Simple differential steering control
        left_speed = self.throttle * (1 - self.steering_angle)
        right_speed = self.throttle * (1 + self.steering_angle)

        # Apply speeds to motors
        for i in range(4):
            if i < 2:  # Front wheels
                self.wheel_motors[i].SetSpeedFunction(chrono.ChFunction_Const(right_speed))
            else:      # Rear wheels
                self.wheel_motors[i].SetSpeedFunction(chrono.ChFunction_Const(left_speed))

# Create the motor driver
motor_driver = RoverMotorDriver(wheel_motors)
system.GetContactContainer().ReportAllContacts(motor_driver)

# =============================================================================
# Visualization setup
# =============================================================================

# Create the Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 3, 0), chrono.ChVectorD(0, 0, 0))
vis.AddLightWithShadow(chrono.ChVectorD(5, 10, 5), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, 512)
vis.SetShadows(True)

# Attach visualization assets to the bodies
vis.AttachSystem(system)

# Customize terrain appearance
terrain_vis = chrono.ChObjShapeFile()
terrain_vis.SetFilename(chrono.GetChronoDataFile("textures/terrain.jpg"))
terrain.AddVisualShape(terrain_vis)

# Customize rover appearance
rover_vis = chrono.ChObjShapeFile()
rover_vis.SetFilename(chrono.GetChronoDataFile("textures/metal.jpg"))
rover_chassis.AddVisualShape(rover_vis)

for wheel in wheels:
    wheel_vis = chrono.ChObjShapeFile()
    wheel_vis.SetFilename(chrono.GetChronoDataFile("textures/rubber.jpg"))
    wheel.AddVisualShape(wheel_vis)

# =============================================================================
# Simulation loop
# =============================================================================

# Simulation loop parameters
vis.SetUserInteractionCallback(motor_driver)

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update simulation time
    system.DoStepDynamics(time_step)

    # Simple control logic (can be replaced with more sophisticated control)
    if system.GetChTime() < 5:
        motor_driver.SetThrottle(2.0)
        motor_driver.SetSteering(0.0)
    elif system.GetChTime() < 10:
        motor_driver.SetThrottle(1.5)
        motor_driver.SetSteering(0.5)
    elif system.GetChTime() < 15:
        motor_driver.SetThrottle(1.5)
        motor_driver.SetSteering(-0.5)
    else:
        motor_driver.SetThrottle(0.0)
        motor_driver.SetSteering(0.0)

    # Exit condition
    if system.GetChTime() > end_time:
        break

# =============================================================================
# Cleanup
# =============================================================================

vis.Close()