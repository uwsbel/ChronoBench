import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# -----------------------------------------------------------------------------
# 1) DATA PATH SETUP
# -----------------------------------------------------------------------------
# Make sure CHRONO data path is set properly before loading assets.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# -----------------------------------------------------------------------------
# 2) GLOBAL PARAMETERS
# -----------------------------------------------------------------------------
# Initial poses for the two vehicles
initLoc1 = chrono.ChVector3d(0,   0.0, 0.5)
initRot1 = chrono.QUNIT
# Place second car 5 m in Y direction
initLoc2 = chrono.ChVector3d(0,  -5.0, 0.5)
initRot2 = chrono.QUNIT

# Visualization and collision
vis_type             = veh.VisualizationType_MESH
chassis_collision    = veh.CollisionType_NONE
tire_model           = veh.TireModelType_TMEASY

# Terrain dimensions
terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0

# Camera track point (follows first vehicle)
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Time stepping
step_size       = 1e-3
tire_step_size  = step_size
render_fps      = 50
render_step     = 1.0 / render_fps

# -----------------------------------------------------------------------------
# 3) CREATE VEHICLE #1
# -----------------------------------------------------------------------------
vehicle1 = veh.BMW_E90()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot1))
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(tire_step_size)

vehicle1.Initialize()

# Optional: switch to Bullet in case you want high‐speed contact
vehicle1.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Visualization
vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)

# -----------------------------------------------------------------------------
# 4) CREATE VEHICLE #2 (identical to #1, shifted in Y)
# -----------------------------------------------------------------------------
vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot2))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)

vehicle2.Initialize()
vehicle2.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)

# Print out masses
print("Vehicle #1 mass:", vehicle1.GetMass())
print("Vehicle #2 mass:", vehicle2.GetMass())

# -----------------------------------------------------------------------------
# 5) CREATE RIGID TERRAIN
# -----------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle1.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0,0,terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth
)
# Changed texture to concrete.jpg
patch.SetTexture(
    chrono.GetChronoDataFile("terrain/textures/concrete.jpg"),
    200, 200
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -----------------------------------------------------------------------------
# 6) IRRLICHT VISUAL SYSTEM
# -----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Two Sedans with Sinusoidal Steering")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
# attach *both* vehicles so they are rendered
vis.AttachVehicle(vehicle1)
vis.AttachVehicle(vehicle2)

# -----------------------------------------------------------------------------
# 7) CREATE TWO INTERACTIVE DRIVERS
# -----------------------------------------------------------------------------
driver1 = veh.ChInteractiveDriverIRR(vis)
driver2 = veh.ChInteractiveDriverIRR(vis)

# responsiveness
driver1.SetSteeringDelta(render_step / 1.0)
driver1.SetThrottleDelta(render_step / 1.0)
driver1.SetBrakingDelta(render_step / 0.3)

driver2.SetSteeringDelta(render_step / 1.0)
driver2.SetThrottleDelta(render_step / 1.0)
driver2.SetBrakingDelta(render_step / 0.3)

driver1.Initialize()
driver2.Initialize()

# -----------------------------------------------------------------------------
# 8) SIMULATION LOOP
# -----------------------------------------------------------------------------
render_steps = math.ceil(render_step / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    t = vehicle1.GetSystem().GetChTime()

    # render scene at fixed intervals
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # 8.a) compute sinusoidal steering law
    #    amplitude = 0.4, frequency = 0.5 Hz
    steer = 0.4 * math.sin(2 * math.pi * 0.5 * t)

    # 8.b) get driver inputs, override steering
    in1 = driver1.GetInputs()
    in2 = driver2.GetInputs()
    in1.m_steering = steer
    in2.m_steering = steer
    # throttle/brake remain whatever keys the user pressed

    # 8.c) SYNCHRONIZE all modules
    driver1.Synchronize(t)
    driver2.Synchronize(t)
    terrain.Synchronize(t)
    vehicle1.Synchronize(t, in1, terrain)
    vehicle2.Synchronize(t, in2, terrain)
    vis.Synchronize(t, in1)   # camera follows vehicle1's input

    # 8.d) ADVANCE all modules by one timestep
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)