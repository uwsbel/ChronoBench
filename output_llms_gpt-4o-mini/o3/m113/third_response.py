import math

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
# Chrono/vehicle initialisation
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (MODIFIED)
initLoc = chrono.ChVectorD(-5, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Rigid terrain dimensions
terrainHeight = 0.0
terrainLength = 100.0
terrainWidth  = 100.0

# Point tracked by chase-camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.1)

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size       = 5e-4
render_step_size = 1.0 / 50.0                 # 50 FPS
render_steps     = math.ceil(render_step_size / step_size)

# ---------------------------------------------------------------------
# Create the M113 vehicle
# ---------------------------------------------------------------------
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetChassisCollisionType(chassis_collision_type)

vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.Initialize()

# Visualisation for all subsystems
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Use BULLET narrow-phase
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
# Build a rigid terrain
# ---------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch   = terrain.AddPatch(patch_mat,
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                                              chrono.QUNIT),
                           terrainLength, terrainWidth, terrainHeight)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------
# EXTRA OBJECT – long box used as an obstacle (MODIFIED)
# ---------------------------------------------------------------------
sys = vehicle.GetSystem()
box_len, box_wid, box_hei = 15.0, 2.0, 0.4        # size of obstacle
box_density = 600                                    # kg/m³

box_body = chrono.ChBodyEasyBox(box_len, box_wid, box_hei,
                                box_density, True, True)  # collide & visualise
box_body.SetPos(chrono.ChVectorD(10, 0, 0.2))            # in front of vehicle
box_body.SetBodyFixed(False)                              # can be pushed
box_body.GetMaterialSurface().SetFriction(0.8)
sys.Add(box_body)

# ---------------------------------------------------------------------
# Irrlicht visualisation and interactive driver
# ---------------------------------------------------------------------
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 mobility test – PyChrono')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)

# Keyboard response times
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# ---------------------------------------------------------------------
# Solver / integrator
# ---------------------------------------------------------------------
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Print vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass(), "kg")

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
step_number  = 0
render_frame = 0

while vis.Run():
    time = sys.GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs and force throttle = 0.8 (MODIFIED)
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.8    # force constant throttle
    # (steering & braking still come from the user)

    # ---- module synchronisation ---------------
    driver.Synchronize(time)                # keyboard polling
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # ---- advance simulation -------------------
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1