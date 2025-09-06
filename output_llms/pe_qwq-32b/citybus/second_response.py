import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision settings
vis_type = veh.VisualizationType_PRIMITIVES
vis_type_mesh = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Tire model and terrain parameters
tire_model = veh.TireModelType_TMEASY
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0
trackPoint = chrono.ChVectorD(-15.0, 10.0, 5.8)

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # 50 FPS

# Create and initialize the vehicle
vehicle = veh.CityBus()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Visualization settings for vehicle components
vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh)

# Set collision system to Bullet for better performance
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with specified material properties
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

# Initialize visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('City Bus Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 3.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Create data-driven driver with predefined inputs
entries = veh.vector_Entry()
entries.push_back(veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0))  # Time 0: idle
entries.push_back(veh.DataDriverEntry(0.1, 1.0, 0.0, 0.0))  # Accelerate forward
entries.push_back(veh.DataDriverEntry(0.5, 1.0, 0.7, 0.0))  # Steer right
driver = veh.ChDataDriver(vehicle, entries)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetMass())

# Simulation control variables
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at specified intervals
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update driver inputs and synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    # Advance simulation for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)