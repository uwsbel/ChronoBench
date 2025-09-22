import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_PRIMITIVES  # Corrected from NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method (removed conflicting Bullet collision system)
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # 50 FPS

# Create MAN vehicle
vehicle = veh.MAN_10t()
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

# Removed conflicting collision system setup
# vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with grass texture
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Add random boxes to the simulation
for i in range(5):
    box = chrono.ChBodyEasyBox(1, 1, 1, 2000, True, True)
    box.SetPos(chrono.ChVectorD(5 + i*2, 0, terrainHeight + 0.5))
    box.SetMaterialSurface(patch_mat)
    vehicle.GetSystem().Add(box)

# Initialize visualization
vis = irr.ChIrrApp(vehicle.GetSystem(), 'MAN 10t Demo', irr.dimension2du(1280, 1024))
vis.SetCameraPosition(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.SetSymbolscale(0.002)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.SetTimestep(render_step_size)
vis.Initialize()
vis.SetVideoframeSave(1)  # Enable frame saving if needed

# Sensor setup
sensor_mgr = sens.ChSensorManager(vehicle.GetSystem())
lidar = sens.ChLidar(sensor_mgr)
lidar.SetName("lidar")
lidar.SetPos(chrono.ChVectorD(0, 0, 1.5))  # Position relative to vehicle's chassis frame
lidar.SetRot(chrono.Q_from_Euler_123(chrono.ChVectorD(0, 0, 0)))
lidar.SetWidth(640)
lidar.SetHeight(480)
lidar.SetFov(60)
lidar.SetRange(100)
lidar.SetNoise(0.01)
lidar.SetUpdateRate(1.0/30)  # 30 Hz
sensor_mgr.AddSensor(lidar)

# Driver setup
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize(vehicle.GetVehicle())

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Rendering
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    
    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Update sensor manager
    sensor_mgr.Update()
    
    step_number += 1
    realtime_timer.Spin(step_size)

# Cleanup (if needed)
sensor_mgr.Shutdown()