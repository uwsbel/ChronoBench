import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 2e-3                                                       # simulation time step (s)
tire_step_size = step_size                                             # tire step matches physics step
render_fps = 50                                                        # target render frames per second
render_every = max(1, round(1.0 / (render_fps * step_size)))          # render cadence (untagged)
sim_end = 15.0                                                         # simulation end time (s) — enough for lane-change maneuver

init_loc = chrono.ChVector3d(-50, 0, 0.5)                             # initial vehicle location (prompt)
init_rot = chrono.QuatFromAngleZ(0)                                   # facing +X direction

feda = veh.FEDA()                                                      # FED-Alpha vehicle wrapper
feda.SetContactMethod(chrono.ChContactMethod_NSC)                      # NSC for rigid terrain (truth)
feda.SetChassisCollisionType(veh.CollisionType_NONE)                   # no chassis collision box
feda.SetChassisFixed(False)                                            # chassis must be free to move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))           # set start pose
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)                     # engine model
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # transmission model
feda.SetTireType(veh.TireModelType_PAC02)                              # PAC02 tire for good grip
feda.SetTireStepSize(tire_step_size)                                   # tire integrator step
feda.Initialize()
system = feda.GetSystem()                                              # take system ownership from wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required for contact/terrain

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())                   # truth's literal mass banner

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)           # mesh chassis visual
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitives for suspension
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # primitives for steering
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)             # mesh wheels
feda.SetTireVisualizationType(veh.VisualizationType_MESH)              # mesh tires

terrainLength = 200.0                                                  # terrain length increased to 200 m (prompt)
terrainWidth  = 100.0                                                  # terrain width
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                              # NSC material for rigid terrain
patch_mat.SetFriction(0.9)                                             # road friction
patch_mat.SetRestitution(0.01)                                         # low restitution
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Path-follower driver — ISO double lane change (prompt)
path_start = chrono.ChVector3d(-50, 0, 0.5)                           # path start matches vehicle init
path = veh.DoubleLaneChangePath(
    path_start,
    13.5,    # maneuver section length (m)
    4.0,     # lane width (m)
    11.0,    # lateral offset (m)
    50.0,    # total path length (m)
    True     # to left
)
target_speed = 10.0                                                    # cruise target speed (prompt, m/s)
driver = veh.ChPathFollowerDriver(feda.GetVehicle(), path, "iso_dlc_path", target_speed)
driver.GetSteeringController().SetLookAheadDistance(5.0)               # look-ahead distance (prompt)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)                # steering PID gains (KP, KI, KD)
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)                   # speed PID gains
driver.Initialize()

# Steering target marker (renderer-agnostic, fixed body with sphere)
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
sphere = chrono.ChVisualShapeSphere(0.1)
sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
target_marker.AddVisualShape(sphere, chrono.ChFramed())
system.AddBody(target_marker)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # Irrlicht visual system
vis.SetWindowTitle("FEDA Double Lane Change")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)           # chase camera
vis.Initialize()                                                        # Initialize FIRST (Irrlicht rule)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                               # vehicle truths use directional light
vis.AttachVehicle(feda.GetVehicle())

realtime_timer = chrono.ChRealtimeStepTimer()                          # real-time pacing
step_number = 0


while vis.Run() and system.GetChTime() < sim_end:
    time = feda.GetSystem().GetChTime()

    if step_number % render_every == 0:                                # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    # Update steering target marker position each frame
    target_marker.SetPos(driver.GetSteeringController().GetTargetLocation())

    driver.Synchronize(time)
    terrain.Synchronize(time)
    feda.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    feda.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                                     # spin to maintain real-time pace
