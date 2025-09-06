import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random  # Added for random box generation

# Initialize Chrono data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(-8, 0, 0.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Add randomly positioned boxes
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
for _ in range(10):
    while True:
        x = random.uniform(-40, 40)
        y = random.uniform(-40, 40)
        distance = math.sqrt((x + 8)**2 + y**2)
        if distance > 5:  # Ensure distance from vehicle's initial position
            break
    z = terrainHeight + 0.5  # Above terrain
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVectorD(x, y, z))
    vehicle.GetSystem().Add(box)

# Create the SCM deformable terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                          0,     # Bekker Kc
                          1.1,   # Bekker n exponent
                          0,     # Mohr cohesive limit (Pa)
                          30,    # Mohr friction limit (degrees)
                          0.01,  # Janosi shear coefficient (m)
                          2e8,   # Elastic stiffness (Pa/m)
                          3e4)   # Damping (Pa s/m)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)  # Initialize terrain mesh

# Create the vehicle Irrlicht interface
vis = irr.ChIrrApp(vehicle.GetSystem(), 'HMMWV Demo', irr.dimension2du(1280, 1024))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.SetCameraPosition(chrono.ChVectorD(0, 3, 15))  # Initial camera position
vis.SetChaseCamera(trackPoint, 6.0, 0.5)

# Add point lights to the scene
vis.AddLightPoint(chrono.ChVectorD(20, 20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(-20, -20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(0, 30, 10), chrono.ChColor(1, 1, 1), 2.0)

# Configure camera settings
camera = vis.GetCamera()
camera.SetFov(60)  # Set field of view to 60 degrees

vis.AddSkyBox()
vis.EnableShadowVolumes()
vis.SetSymbolsScale(0.01)
vis.SetCameraSensitivity(0.001, 0.001)
vis.SetTimestep(step_size)
vis.Initialize()

# Create the driver system
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# Simulation loop
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()
print("VEHICLE MASS: ", vehicle.GetMass())  # Fixed print statement

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update simulation modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    
    # Update visualization
    vis Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random

# Initialize data paths
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Vehicle setup parameters
initLoc = chrono.ChVectorD(-8, 0, 0.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID
contact_method = chrono.ChContactMethod_SMC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

# Create vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Add random boxes
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
for _ in range(10):
    while True:
        x = random.uniform(-40, 40)
        y = random.uniform(-40, 40)
        distance = math.sqrt((x + 8)**2 + y**2)
        if distance > 5:
            break
    z = 0.5
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVectorD(x, y, z))
    vehicle.GetSystem().Add(box)

# Create terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

# Visualization setup
vis = irr.ChIrrApp(vehicle.GetSystem(), 'Enhanced HMMWV Simulation', irr.dimension2du(1280, 1024))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.SetCameraPosition(chrono.ChVectorD(0, 3, 15))
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 1.71), 6.0, 0.5)

# Lighting setup
vis.AddLightPoint(chrono.ChVectorD(20, 20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(-20, -20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(0, 30, 10), chrono.ChColor(1, 1, 1), 2.0)

# Camera configuration
camera = vis.GetCamera()
camera.SetFov(60)
vis.AddSkyBox()
vis.EnableShadowVolumes()
vis.SetSymbolsScale(0.01)
vis.SetCameraSensitivity(0.001, 0.001)
vis.SetTimestep(step_size)
vis.Initialize()

# Driver setup
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# Simulation loop
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()
print("VEHICLE MASS:", vehicle.GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    driver_inputs = driver.GetInputs()
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)