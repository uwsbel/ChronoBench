import pychrono.core as chrono
import pychrono.vehicle as veh
import math

# Set paths (assumes CHRONO_DATA is already set in your environment)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# 1) Initial vehicle location and orientation
initLoc = chrono.ChVector3d(1, 0, 0.5)            # moved from (0,0,0.5) to (1,0,0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# 2) Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES       # changed from MESH to PRIMITIVES

# 3) Collision type for chassis
chassis_collision_type = veh.CollisionType_MESH    # changed from NONE to MESH

# 4) Tire model type
tire_model = veh.TireModelType_FIALA               # changed from TMEASY to FIALA

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0

# Point on chassis to track with the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Time stepping
step_size        = 1e-3
tire_step_size   = step_size
render_step_size = 1.0 / 50  # 50 FPS

# ------------------------------------------------
# Create the ARTcar vehicle, set parameters, init
# ------------------------------------------------
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetMaxMotorVoltageRatio(0.16)
vehicle.SetStallTorque(0.3)
vehicle.SetTireRollingResistance(0.06)

vehicle.Initialize()

# Visualization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use Bullet for contacts
vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystem.Type_BULLET
)

# ------------------
# Create the terrain
# ------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -----------------------------------------
# Create the vehicle Irrlicht visualization
# -----------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# --------------------
# Create the driver
# --------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Steering, throttle, braking response times
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ----------------
# Simulation loop
# ----------------
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps   = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get inputs and synchronize
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)