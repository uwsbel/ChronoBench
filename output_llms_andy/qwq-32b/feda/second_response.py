import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (changed to (-50, 0, 0.5))
initLoc = chrono.ChVectorD(-50, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision types remain as per original
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

terrainHeight = 0
terrainLength = 200.0  # Increased to 200.0
terrainWidth = 100.0

trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

# Create and initialize the FEDA vehicle
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Visualization settings (unchanged except for camera track point)
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Removed conflicting collision system setup line
# vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with increased length
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())  # Assuming vehicle.GetVehicle() is correct

# Path follower driver setup
path = chrono.ChPath()
path.AddPoint(chrono.ChVectorD(-50, 0, 0.5))  # Starting position
path.AddPoint(chrono.ChVectorD(-30, 0, 0.5))  # Straight for 20m (2 seconds)
path.AddPoint(chrono.ChVectorD(-10, -4, 0.5)) # Left lane change after 40m (4 seconds)
path.AddPoint(chrono.ChVectorD(10, 0, 0.5))   # Back to center after 60m (6 seconds)
path.AddPoint(chrono.ChVectorD(30, 4, 0.5))   # Right lane change at 80m (8 seconds)
path.AddPoint(chrono.ChVectorD(50, 0, 0.5))   # Back to center at 100m
path.AddPoint(chrono.ChVectorD(70, 0, 0.5))   # Continue straight
path.SetSplineOrder(3)  # Use cubic spline for smooth path

driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetTargetSpeed(10.0)  # Target speed of 10 m/s
driver.SetLookAheadDistance(5.0)  # Look-ahead distance
driver.SetSteeringProportionalGain(2.0)  # Steering proportional gain
driver.SetSteeringDerivativeGain(0.1)    # Steering derivative gain
driver.SetSpeedProportionalGain(0.5)     # Speed proportional gain
driver.SetSpeedIntegralGain(0.05)        # Speed integral gain
driver.Initialize()

# Simulation setup
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)