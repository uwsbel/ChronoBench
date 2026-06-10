import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -------------------------------------------------------------------------
# 1. Initial vehicle location – changed from (0,0,0.5) to (1,0,0.5)
# -------------------------------------------------------------------------
initLoc = chrono.ChVector3d(1, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# -------------------------------------------------------------------------
# 2. Visualization type – changed from MESH to PRIMITIVES
# -------------------------------------------------------------------------
vis_type = veh.VisualizationType_PRIMITIVES

# -------------------------------------------------------------------------
# 3. Chassis collision type – changed from NONE to MESH
# -------------------------------------------------------------------------
chassis_collision_type = veh.CollisionType_MESH

# -------------------------------------------------------------------------
# 4. Tire model type – changed from TMEASY to FIALA
# -------------------------------------------------------------------------
tire_model = veh.TireModelType_FIALA

# -------------------------------------------------------------------------
# Terrain parameters
# -------------------------------------------------------------------------
terrainHeight = 0        # not used directly
terrainLength = 100.0    # size in X direction
terrainWidth  = 100.0    # size in Y direction

# Point on the chassis that the camera tracks
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)

# -------------------------------------------------------------------------
# Contact and simulation settings
# -------------------------------------------------------------------------
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size

render_step_size = 1.0 / 50   # 50 FPS

# -------------------------------------------------------------------------
# Create the ARTcar vehicle
# -------------------------------------------------------------------------
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# Electric‑motor parameters (example – keep as in original)
vehicle.SetMaxMotorVoltageRatio(0.16)
vehicle.SetStallTorque(0.3)
vehicle.SetTireRollingResistance(0.06)

vehicle.Initialize()

# Visualisation of vehicle subsystems
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use the Bullet collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -------------------------------------------------------------------------
# Rigid terrain
# -------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -------------------------------------------------------------------------
# Irrlicht visualisation
# -------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('dart')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

# -----------------------------------------------------------------
# FIX: Attach the vehicle directly (no need for extra .GetVehicle())
# -----------------------------------------------------------------
vis.AttachVehicle(vehicle)

# -------------------------------------------------------------------------
# Driver (keyboard) system
# -------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

steering_time  = 1.0   # time to go from 0 to +1 (or 0 to -1)
throttle_time  = 1.0   # time to go from 0 to +1
braking_time   = 0.3   # time to go from 0 to +1

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# -------------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------------
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronise all subsystems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    # -----------------------------------------------------------------
    # FIX: vehicle.Synchronize expects only (time, driver_inputs)
    # -----------------------------------------------------------------
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance one step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)