import math
import pychrono.core     as chrono
import pychrono.irrlicht as irr          # still imported for the GUI back-end
import pychrono.vehicle  as veh

# ---------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ---------------------------------------------------------------------
# Initial vehicle pose
# ---------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# ---------------------------------------------------------------------
# User-selectable parameters
# ---------------------------------------------------------------------
vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE       # <==  fixed
tire_model             = veh.TireModelType_TMEASY

# Terrain size
terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0

trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)     # point tracked by camera

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis    = False

# Step sizes
step_size       = 1e-3
tire_step_size  = step_size
render_step_size = 1.0 / 50.0          # 50 FPS

# ---------------------------------------------------------------------
# Create the vehicle
# ---------------------------------------------------------------------
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# ----  PERFORMANCE CHANGES  -----------------------------------------
vehicle.SetMaxMotorVoltageRatio(0.26)       # was 0.16
vehicle.SetStallTorque(0.4)                 # was 0.3
vehicle.SetTireRollingResistance(0.03)      # was 0.06
# ---------------------------------------------------------------------

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Switch to Bullet narrow-phase collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
# Create the terrain
# ---------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight),
                                            chrono.QUNIT),
                         terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------
# Visualization (Irrlicht)
# ---------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ---------------------------------------------------------------------
# Driver (interactive keyboard)
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps     = math.ceil(render_step_size / step_size)
realtime_timer   = chrono.ChRealtimeStepTimer()
step_number      = 0
render_frame     = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Collect inputs
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

    step_number += 1
    realtime_timer.Spin(step_size)