import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, veh, irrlight, fea
from pychrono.veh import RigidTerrain, RigidTire
from pychrono.robot import InteractiveDriver
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere
from pychrono.contact import ChContactMaterialSMC
from pychrono.systems import ChSystemSMC

# Set the data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Initialize the system
sys = ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)

# Initialize the visualization
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()
vis.AddSkyBox()

# Vehicle parameters
vehicle_width = 2.5  # Width of the vehicle
vehicle_length = 4.0  # Length of the vehicle
tire_radius = 0.5  # Tire radius
tire_hardness = 0.3  # Tire hardness
soil_young_modulus = 1.0e6  # Soil young modulus
soil_shear_modulus = 0.2 * soil_young_modulus  # Soil shear modulus
soil_friction = 0.6  # Soil friction coefficient
deformable_terrain_height = 1.0  # Maximum height of the terrain patch

# Vehicle position and orientation
init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.QuatFromAngleX(chrono.CH_PI / 2)  # Initial orientation (90 degrees)

# Initialize the vehicle
car = veh.ARTcar(sys)
car.SetContactMethod(veh.ContactMethod_SMC)
car.SetChassisCollisionType(veh.ChassisCollisionType_Rigid)
car.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
car.Initialize()

# Add the vehicle to the system
sys.Add(car)

# Initialize the terrain
terrain = RigidTerrain(sys)
terrain.SetGroundType(terrain.GroundType_Deformable)
terrain.SetYoungModulus(soil_young_modulus)
terrain.SetShearModulus(soil_shear_modulus)
terrain.SetFriction(soil_friction)
terrain.Initialize()

# Create a moving patch for the vehicle
patch = terrain.AddPatch(
    patch_material=chrono.ChContactMaterialSMC(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=vehicle_length,
    width=vehicle_width,
    height=deformable_terrain_height
)
terrain.patch = patch  # Store the patch reference for dynamic updates

# Update the patch position each simulation step
def update_patch_position():
    patch_pos = car.GetBody().GetPos()
    patch.SetPos(chrono.ChCoordsysd(patch_pos, init_rot))

sys.GetContactContainer().RegisterAddContactCallback(update_patch_position)

# Initialize the driver
driver = InteractiveDriver(sys, 0, 0, 0)  # Use the first controller (gamepad)
driver.EnableThrottle(True)
driver.EnableSteering(True)
driver.EnableBrake(True)

# Define the control mappings
driver.SetControlMapping(
    throttle_axis=1,
    steer_axis=0,
    brake_axis=-1
)

# Add visual shapes for the vehicle
# Example: Add visual shapes for the chassis and wheels
chassis_shape = ChVisualShapeBox(chrono.ChVector3d(vehicle_length, vehicle_width, 0.2))
chassis_shape.SetColor(chrono.ChColor(1, 0.5, 0))
car.AddVisualShape(chassis_shape)

# Add visual shapes for the wheels
wheel_radius = tire_radius + 0.1  # Add a small radius to the wheel visualization
for i in range(4):
    wheel_pos = car.GetBody().GetPos() + chrono.ChVector3d(
        (vehicle_width / 2 - 1) * (1 if i % 2 == 0 else -1),
        0,
        0
    )
    wheel_shape = ChVisualShapeSphere(wheel_radius)
    wheel_shape.SetColor(chrono.ChColor(0, 0.5, 1))  # Blue wheels
    car.AddVisualShape(wheel_shape)

# Set the simulation time step
sys.SetTimeStep(0.01)

# Set the frame rate
vis.SetFrameLimit(50)  # 50 frames per second
vis.EnableVSync(True)

# Run the simulation
while vis.Run():
    # Update the patch position based on the vehicle's current position
    update_patch_position()
    
    # Perform a dynamics step
    sys.DoStepDynamics(sys.GetTimeStep())
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, veh, irrlight, fea
from pychrono.veh import RigidTerrain, RigidTire
from pychrono.robot import InteractiveDriver
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere
from pychrono.contact import ChContactMaterialSMC
from pychrono.systems import ChSystemSMC

# Set the data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Initialize the system
sys = ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)

# Initialize the visualization
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()
vis.AddSkyBox()

# Vehicle parameters
vehicle_width = 2.5  # Width of the vehicle
vehicle_length = 4.0  # Length of the vehicle
tire_radius = 0.5  # Tire radius
tire_hardness = 0.3  # Tire hardness
soil_young_modulus = 1.0e6  # Soil young modulus
soil_shear_modulus = 0.2 * soil_young_modulus  # Soil shear modulus
soil_friction = 0.6  # Soil friction coefficient
deformable_terrain_height = 1.0  # Maximum height of the terrain patch

# Vehicle position and orientation
init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.QuatFromAngleX(chrono.CH_PI / 2)  # Initial orientation (90 degrees)

# Initialize the vehicle
car = veh.ARTcar(sys)
car.SetContactMethod(veh.ContactMethod_SMC)
car.SetChassisCollisionType(veh.ChassisCollisionType_Rigid)
car.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
car.Initialize()

# Add the vehicle to the system
sys.Add(car)

# Initialize the terrain
terrain = RigidTerrain(sys)
terrain.SetGroundType(terrain.GroundType_Deformable)
terrain.SetYoungModulus(soil_young_modulus)
terrain.SetShearModulus(soil_shear_modulus)
terrain.SetFriction(soil_friction)
terrain.Initialize()

# Create a moving patch for the vehicle
patch = terrain.AddPatch(
    patch_material=chrono.ChContactMaterialSMC(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=vehicle_length,
    width=vehicle_width,
    height=deformable_terrain_height
)
terrain.patch = patch  # Store the patch reference for dynamic updates

# Update the patch position each simulation step
def update_patch_position():
    patch_pos = car.GetBody().GetPos()
    patch.SetPos(chrono.ChCoordsysd(patch_pos, init_rot))

sys.GetContactContainer().RegisterAddContactCallback(update_patch_position)

# Initialize the driver
driver = InteractiveDriver(sys, 0, 0, 0)  # Use the first controller (gamepad)
driver.EnableThrottle(True)
driver.EnableSteering(True)
driver.EnableBrake(True)

# Define the control mappings
driver.SetControlMapping(
    throttle_axis=1,
    steer_axis=0,
    brake_axis=-1
)

# Add visual shapes for the vehicle
# Example: Add visual shapes for the chassis and wheels
chassis_shape = ChVisualShapeBox(chrono.ChVector3d(vehicle_length, vehicle_width, 0.2))
chassis_shape.SetColor(chrono.ChColor(1, 0.5, 0))
car.AddVisualShape(chassis_shape)

# Add visual shapes for the wheels
wheel_radius = tire_radius + 0.1  # Add a small radius to the wheel visualization
for i in range(4):
    wheel_pos = car.GetBody().GetPos() + chrono.ChVector3d(
        (vehicle_width / 2 - 1) * (1 if i % 2 == 0 else -1),
        0,
        0
    )
    wheel_shape = ChVisualShapeSphere(wheel_radius)
    wheel_shape.SetColor(chrono.ChColor(0, 0.5, 1))  # Blue wheels
    car.AddVisualShape(wheel_shape)

# Set the simulation time step
sys.SetTimeStep(0.01)

# Set the frame rate
vis.SetFrameLimit(50)  # 50 frames per second
vis.EnableVSync(True)

# Run the simulation
while vis.Run():
    # Update the patch position based on the vehicle's current position
    update_patch_position()
    
    # Perform a dynamics step
    sys.DoStepDynamics(sys.GetTimeStep())
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()