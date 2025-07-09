import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, vehicle, irrlicht as chronoirr
from pychrono.robot import robosimian
from pychrono.vehicle import ARTcar, RigidTerrain, DrivetrainType, EngineType

# Initialize the PyChrono system with Non-Smooth Contact (NSC) for robust contact handling
sys = chrono.ChSystemNSC()

# Set the data path for Chrono resources
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Initialize the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()

# Create an HMMWV instance
car = ARTcar(sys, True, True)  # True for kinematics, True for visualization

# Set vehicle properties
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  # Initial position at origin
car.SetDrivetrainType(veh.DrivetrainType_Diff4)  # Differential 4-wheel drive
car.SetEngineType(veh.EngineType_Diesel)  # Diesel engine type

# Add visual elements to the vehicle
# Wheels
wheel_visual = chrono.ChVisualShapeCylinder(0.5, 0.2)
car.AddVisualShape(wheel_visual)
wheel_visual.SetColor(chrono.ChColor(1, 0, 0))  # Red for wheels

# Hood and cabin
hood_visual = chrono.ChVisualShapeBox(1.2, 1.5, 1.0)
hood_visual.SetColor(chrono.ChColor(0, 0.5, 0))  # Dark green for hood
car.AddVisualShape(hood_visual)

# Engine
engine_visual = chrono.ChVisualShapeBox(1.5, 1.0, 1.0)
engine_visual.SetColor(chrono.ChColor(0.8, 0.2, 0))  # Orange for engine
car.AddVisualShape(engine_visual)

# Create a complex terrain with multiple patches
terrain = veh.RigidTerrain(sys)

# Create different terrain patches
# Flat patch with texture
flat_patch = terrain.AddPatch(
    patch_material='asphalt',
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=100,  # 100 units in x-direction
    width=50   # 50 units in y-direction
)

# Mesh-based patch for a bump
bump_patch = terrain.AddPatch(
    patch_material='grass',
    position=chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 5), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=60,  # 60 units in x-direction
    width=40    # 40 units in y-direction
)

# Heightmap-based patch for elevation
heightmap_patch = terrain.AddPatch(
    patch_material='dirt',
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=80,  # 80 units in x-direction
    width=60   # 60 units in y-direction
)

# Visualize the terrain components
# Flat patch
flat_visual = chrono.ChVisualShapeBox(100, 5, 5)
flat_visual.SetColor(chrono.ChColor(1, 0.5, 0))  # Yellow for flat areas
terrain.AddVisualShape(flat_visual)

# Bump patch
bump_visual = chrono.ChVisualShapeMesh('bump_mesh', 1.0, 1.0)
bump_visual.SetColor(chrono.ChColor(0, 0.8, 0))  # Teal for bumps
terrain.AddVisualShape(bump_visual)

# Heightmap patch
heightmap_visual = chrono.ChVisualShapeSphere(5, 5)
heightmap_visual.SetColor(chrono.ChColor(0, 0.8, 1))  # Blue for elevation
terrain.AddVisualShape(heightmap_visual)

# Create an interactive driver system
driver = robosimian.RS_Driver(sys, 0.1)  # 0.1 seconds timestep
driver.SetSteeringDeadband(0.3)  # 30% deadband
driver.SetThrottleRange(0.5, 0.5)  # 0-50% throttle
driver.SetBrakeRange(0.5, 0.5)    # 0-50% brake

# Define driver controls
def driver_controls():
    # Get current speed
    speed = car.GetSpeedX()
    
    # Throttle control
    speed = min(speed + driver.GetThrottle() * 0.1, 12.0)  # Max speed 12 m/s
    
    # Steering control
    angle = driver.GetSteering() * 2.5  # Convert to radians
    
    # Braking control
    brake = max(0, 1 - driver.GetBrake())
    
    return speed, angle, brake

# Register the driver function
driver.RegisterDriverFunction('hmmwv_driver', driver_controls)

# Add the vehicle and terrain to the system
sys.Add(car)
sys.Add(terrain)

# Continuous simulation loop
while True:
    # Update the simulation
    sys.DoStepDynamics(0.01)  # 10ms timestep

    # Update the driver controls
    speed, angle, brake = driver.GetDriverFunction('hmmwv_driver')()

    # Apply driver controls
    car.SetSpeedX(speed)
    car.SetSteering(angle)
    car.SetBrake(break)

    # Visualize the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Check if the simulation should continue
    if not vis.Run():
        break