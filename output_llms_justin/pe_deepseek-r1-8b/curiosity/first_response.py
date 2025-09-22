import os
import math
import numpy as np
from pychrono import chrono as chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *

# Initialize the system
sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))  # Replace with your data path

# Set up the simulation time step
time_step = 0.01

# Create the terrain body
terrain_body = chrono.ChBody()
terrain_body.SetMass(0)  # No mass for the ground
terrain_body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # No inertia
terrain_body.SetFixed(True)  # Fixed in space

# Create a visual shape for the terrain
terrain_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(100, 100, 5))
terrain_body.AddVisualShape(terrain_shape)

# Add the terrain to the system
sys.Add(terrain_body)

# Create the rover body
rover_body = chrono.ChBodyEasySphere(chrono.ChVector3d(1, 1, 1), 1000, True, True, chrono.ChContactMaterialSMC())
rover_body.SetPos(chrono.ChVector3d(0, 5, 0))  # Position above the ground

# Add the rover body to the system
sys.Add(rover_body)

# Create the axle body for the wheels
axle_body = chrono.ChBody()
axle_body.SetMass(0)
axle_body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))
axle_body.SetFixed(True)
sys.Add(axle_body)

# Create the wheels
wheel_radius = 0.5
wheel_height = 0.4

# Front wheels
wheel1 = chrono.ChBodyEasySphere(wheel_radius, 1000, True, True, chrono.ChContactMaterialSMC())
wheel1.SetPos(chrono.ChVector3d(-wheel_radius, wheel_height, 0))
wheel1.SetRot(chrono.ChVector3d(0, 0, 0))  # Initial rotation
sys.Add(wheel1)

# Back wheels
wheel2 = chrono.ChBodyEasySphere(wheel_radius, 1000, True, True, chrono.ChContactMaterialSMC())
wheel2.SetPos(chrono.ChVector3d(wheel_radius, wheel_height, 0))
wheel2.SetRot(chrono.ChVector3d(0, 0, 0))  # Initial rotation
sys.Add(wheel2)

# Create the joints for the wheels
# Front left wheel
joint_fl = chrono.ChLinkLockPrismatic()
joint_fl.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # Initial position and rotation
joint_fl.SetAxis(chrono.ChVector3d(-1, 0, 0))  # X-axis for left-right movement
sys.Add(joint_fl)

# Front right wheel
joint_fr = chrono.ChLinkLockPrismatic()
joint_fr.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
joint_fr.SetAxis(chrono.ChVector3d(1, 0, 0))  # X-axis for right movement
sys.Add(joint_fr)

# Back left wheel
joint_bl = chrono.ChLinkLockPrismatic()
joint_bl.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
joint_bl.SetAxis(chrono.ChVector3d(-1, 0, 0))  # X-axis for left movement
sys.Add(joint_bl)

# Back right wheel
joint_br = chrono.ChLinkLockPrismatic()
joint_br.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
joint_br.SetAxis(chrono.ChVector3d(1, 0, 0))  # X-axis for right movement
sys.Add(joint_br)

# Create the motor for steering
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
motor.SetSpeedFunction(chrono.ChFunction_Sine(0.1, 1.0))  # Sine wave for steering
sys.Add(motor)

# Initialize the visualization system
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()

# Add camera
vis.AddCamera(chrono.ChVector3d(0, 5, 10))

# Add typical lights
vis.AddTypicalLights()

# Add skybox
vis.AddSkyBox()

# Add logo
vis.AddLogo(chrono.GetChronoDataFile("pychrono.png"))

# Set up camera movement
current_cam_pos = vis.GetCameraPosition()
current_cam_rot = vis.GetCameraRotation()

# Set up collision model
collision_model = sys.GetCollisionModel()
collision_model.SetDefaultSuggestedEnvelope(0.01)
collision_model.SetDefaultSuggestedMargin(0.005)

# Create contact material for the terrain
terrain_material = chrono.ChContactMaterialSMC()
terrain_material.SetFriction(0.5)
terrain_material.SetDampingF(0.1)
terrain_material.SetCompliance(0.01)
terrain_body.SetMaterial(terrain_material)

# Create contact material for the rover
rover_material = chrono.ChContactMaterialSMC()
rover_material.SetFriction(0.5)
rover_material.SetDampingF(0.1)
rover_material.SetCompliance(0.01)
rover_body.SetMaterial(rover_material)

# Define the callback for contact reporting
def on_contact_report(pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
    print(f"Contact between {pA} and {pB} at distance {distance} with force {cforce} and torque {ctorque}")

# Register the contact callback
sys.GetContactContainer().ReportAllContacts(on_contact_report)

# Run the simulation loop
while True:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    if vis.Run() == False:
        break

import os
import math
import numpy as np
from pychrono import chrono as chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *

# Initialize the system
sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))  # Replace with your data path

# Set up the simulation time step
time_step = 0.01

# Create the terrain
terrain_body = chrono.ChBody()
terrain_body.SetMass(0)  # No mass for the ground
terrain_body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # No inertia
terrain_body.SetFixed(True)  # Fixed in space

# Create a visual shape for the terrain
terrain_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(100, 100, 5))
terrain_body.AddVisualShape(terrain_shape)

# Add the terrain to the system
sys.Add(terrain_body)

# Create the rover body
rover_body = chrono.ChBodyEasySphere(chrono.ChVector3d(1, 1, 1), 1000, True, True, chrono.ChContactMaterialSMC())
rover_body.SetPos(chrono.ChVector3d(0, 5, 0))  # Position above the ground

# Add the rover body to the system
sys.Add(rover_body)

# Create the axle body for the wheels
axle_body = chrono.ChBody()
axle_body.SetMass(0)
axle_body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))
axle_body.SetFixed(True)
sys.Add(axle_body)

# Create the wheels
wheel_radius = 0.5
wheel_height = 0.4

# Front wheels
wheel1 = chrono.ChBodyEasySphere(wheel_radius, 1000, True, True, chrono.ChContactMaterialSMC())
wheel1.SetPos(chrono.ChVector3d(-wheel_radius, wheel_height, 0))
wheel1.SetRot(chrono.ChVector3d(0, 0, 0))  # Initial rotation
sys.Add(wheel1)

# Back wheels
wheel2 = chrono.ChBodyEasySphere(wheel_radius, 1000, True, True, chrono.ChContactMaterialSMC())
wheel2.SetPos(chrono.ChVector3d(wheel_radius, wheel_height, 0))
wheel2.SetRot(chrono.ChVector3d(0, 0, 0))  # Initial rotation
sys.Add(wheel2)

# Create the joints for the wheels
# Front left wheel
joint_fl = chrono.ChLinkLockPrismatic()
joint_fl.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # Initial position and rotation
joint_fl.SetAxis(chrono.ChVector3d(-1, 0, 0))  # X-axis for left-right movement
sys.Add(joint_fl)

# Front right wheel
joint_fr = chrono.ChLinkLockPrismatic()
joint_fr.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
joint_fr.SetAxis(chrono.ChVector3d(1, 0, 0))  # X-axis for right movement
sys.Add(joint_fr)

# Back left wheel
joint_bl = chrono.ChLinkLockPrismatic()
joint_bl.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
joint_bl.SetAxis(chrono.ChVector3d(-1, 0, 0))  # X-axis for left movement
sys.Add(joint_bl)

# Back right wheel
joint_br = chrono.ChLinkLockPrismatic()
joint_br.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
joint_br.SetAxis(chrono.ChVector3d(1, 0, 0))  # X-axis for right movement
sys.Add(joint_br)

# Create the motor for steering
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rover_body, axle_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
motor.SetSpeedFunction(chrono.ChFunction_Sine(0.1, 1.0))  # Sine wave for steering
sys.Add(motor)

# Configure the visualization
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()

# Add camera
vis.AddCamera(chrono.ChVector3d(0, 5, 10))

# Add typical lights
vis.AddTypicalLights()

# Add skybox
vis.AddSkyBox()

# Add logo
vis.AddLogo(chrono.GetChronoDataFile("pychrono.png"))

# Set up collision model
collision_model = sys.GetCollisionModel()
collision_model.SetDefaultSuggestedEnvelope(0.01)
collision_model.SetDefaultSuggestedMargin(0.005)

# Create contact materials
terrain_material = chrono.ChContactMaterialSMC()
terrain_material.SetFriction(0.5)
terrain_material.SetDampingF(0.1)
terrain_material.SetCompliance(0.01)
terrain_body.SetMaterial(terrain_material)

rover_material = chrono.ChContactMaterialSMC()
rover_material.SetFriction(0.5)
rover_material.SetDampingF(0.1)
rover_material.SetCompliance(0.01)
rover_body.SetMaterial(rover_material)

# Define the callback for contact reporting
def on_contact_report(pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
    print(f"Contact between {pA} and {pB} at distance {distance} with force {cforce} and torque {ctorque}")

# Register the contact callback
sys.GetContactContainer().ReportAllContacts(on_contact_report)

# Run the simulation loop
while True:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    if vis.Run() == False:
        break