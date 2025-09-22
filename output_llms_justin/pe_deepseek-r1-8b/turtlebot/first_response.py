import os
import math
import numpy as np
from pychrono import chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *

# Initialize the PyChrono system
sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))
sys.SetGroundBody(chrono.ChBody())
sys.GetGroundBody().SetPos(chrono.ChVector3d(0, 0, 0))
sys.GetGroundBody().SetFixed(True)
sys.GetGroundBody().EnableCollision(True)

# Set up visualization
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('TurtleBot Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()

# Create TurtleBot body
robot_body = ChBody()
robot_body.SetMass(30)
robot_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))  # Inertia tensor
robot_body.SetPos(chrono.ChVector3d(0, 1, 0))  # Initial position (0.5 meters above ground)
robot_body.SetVisualShape(ChVisualShapeBox(0.5, 0.5, 0.5))
sys.Add(robot_body)

# Initialize TurtleBot with a specific position and orientation
robot = RoboSimian(sys, True, True)
robot.Initialize(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  # Initial orientation

# Setup the ground body
terrain_body = ChBody()
terrain_body.SetMass(0)
terrain_body.SetVisualShape(ChVisualShapeBox(100, 0.5, 0.5))
sys.Add(terrain_body)

# Create a link between the robot and the ground
joint = ChLinkLockLock()
joint.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(joint)

# Setup camera position
vis.AddCamera(chrono.ChVector3d(0, 5, 10))

# Create a motor joint for rotation
motor = ChLinkMotorRotationSpeed()
motor.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(motor)

# Define speed functions for left and right movement
def left_motor_speed(t):
    return 0.5 * math.sin(t * 2)

def right_motor_speed(t):
    return 0.5 * math.sin(t * 2 + 0.5)

# Register the speed functions
motor.SetSpeedFunction(left_motor_speed)
motor.SetSpeedFunction(right_motor_speed)

# Define the simulation time step
time_step = 0.01

# Define the simulation loop
while True:
    # Update the simulation
    sys.DoStepDynamics(time_step)

    # Update the motor speeds based on time
    motor.SetSpeedFunction(left_motor_speed)
    motor.SetSpeedFunction(right_motor_speed)

    # Check if the robot is in contact with the ground
    if robot_body.GetContactCount() > 0:
        # Update normal forces based on contact
        normal_force = robot_body.GetContactForce().GetNormal()
        robot_body.SetLinearDamping(normal_force * 0.1)

    # Render the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Check if the simulation should continue
    if not vis.Run():
        break

import os
import math
import numpy as np
from pychrono import chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *

# Initialize the PyChrono system
sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))
sys.SetGroundBody(chrono.ChBody())
sys.GetGroundBody().SetPos(chrono.ChVector3d(0, 0, 0))
sys.GetGroundBody().SetFixed(True)
sys.GetGroundBody().EnableCollision(True)

# Set up visualization
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('TurtleBot Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()

# Create TurtleBot body
robot_body = ChBody()
robot_body.SetMass(30)
robot_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))  # Inertia tensor
robot_body.SetPos(chrono.ChVector3d(0, 1, 0))  # Initial position (0.5 meters above ground)
robot_body.SetVisualShape(ChVisualShapeBox(0.5, 0.5, 0.5))
sys.Add(robot_body)

# Initialize TurtleBot with a specific position and orientation
robot = RoboSimian(sys, True, True)
robot.Initialize(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  # Initial orientation

# Setup the ground body
terrain_body = ChBody()
terrain_body.SetMass(0)
terrain_body.SetVisualShape(ChVisualShapeBox(100, 0.5, 0.5))
sys.Add(terrain_body)

# Create a link between the robot and the ground
joint = ChLinkLockLock()
joint.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(joint)

# Setup camera position
vis.AddCamera(chrono.ChVector3d(0, 5, 10))

# Create a motor joint for rotation
motor = ChLinkMotorRotationSpeed()
motor.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(motor)

# Define speed functions for left and right movement
def left_motor_speed(t):
    return 0.5 * math.sin(t * 2)

def right_motor_speed(t):
    return 0.5 * math.sin(t * 2 + 0.5)

# Register the speed functions
motor.SetSpeedFunction(left_motor_speed)
motor.SetSpeedFunction(right_motor_speed)

# Define the simulation time step
time_step = 0.01

# Define the simulation loop
while True:
    # Update the simulation
    sys.DoStepDynamics(time_step)

    # Update the motor speeds based on time
    motor.SetSpeedFunction(left_motor_speed)
    motor.SetSpeedFunction(right_motor_speed)

    # Check if the robot is in contact with the ground
    if robot_body.GetContactCount() > 0:
        # Update normal forces based on contact
        normal_force = robot_body.GetContactForce().GetNormal()
        robot_body.SetLinearDamping(normal_force * 0.1)

    # Render the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Check if the simulation should continue
    if not vis.Run():
        break