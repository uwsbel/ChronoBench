import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(-50, 0, 0.5)                             # spawn so the DLC fits the patch
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation
step_size = 2e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire substep

feda = veh.FEDA()                                                     # FED-Alpha catalog vehicle
feda.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision mesh
feda.SetChassisFixed(False)                                          # chassis must be free to move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)                   # simple-map engine
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # automatic transmission
feda.SetTireType(veh.TireModelType_PAC02)                           # Pacejka 2002 tire
feda.SetTireStepSize(tire_step_size)                                 # tire integration step
feda.Initialize()                                                    # build the vehicle

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)       # mesh chassis
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitive suspension
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # primitive steering
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)         # mesh wheels
feda.SetTireVisualizationType(veh.VisualizationType_MESH)          # mesh tires

system = feda.GetSystem()                                            # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for terrain contact
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())                # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                          # tire grip
patch_mat.SetRestitution(0.01)                                      # nearly no bounce
terrainLength = 200.0                                                # X size — fits the DLC
terrainWidth = 100.0                                                 # Y size
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # sandy color
terrain.Initialize()                                                # build the terrain

path = veh.DoubleLaneChangePath(init_loc, 13.5, 4.0, 11.0, 50.0, True)  # ISO double lane change path
target_speed = 10.0                                                 # cruise-control target speed (m/s)
driver = veh.ChPathFollowerDriver(feda.GetVehicle(), path, "DLC_path", target_speed)  # path-follower driver
driver.GetSteeringController().SetLookAheadDistance(5.0)            # look-ahead distance
driver.GetSteeringController().SetGains(0.8, 0, 0)                  # steering KP, KI, KD
driver.GetSpeedController().SetGains(0.4, 0, 0)                     # speed KP, KI, KD
driver.Initialize()                                                 # build the driver

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht window
vis.SetWindowTitle("FEDA Double Lane Change")                       # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera
vis.Initialize()                                                    # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional light (vehicle scenes)
vis.AttachVehicle(feda.GetVehicle())                               # bind vehicle visual assets

render_step_size = 1.0 / 50.0                                       # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)             # physics steps per frame
sim_end = 16.0                                                      # simulation end time


realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                     # step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver commands

    driver.Synchronize(time)                                       # advance driver
    terrain.Synchronize(time)                                      # advance terrain
    feda.Synchronize(time, driver_inputs, terrain)               # feed inputs to the vehicle
    vis.Synchronize(time, driver_inputs)                          # update HUD

    driver.Advance(step_size)                                     # step driver
    terrain.Advance(step_size)                                    # step terrain
    feda.Advance(step_size)                                       # step the wrapper-owned system
    vis.Advance(step_size)                                        # step visualization


    step_number += 1                                              # advance step counter
    realtime_timer.Spin(step_size)                               # spin so wall-clock matches sim time
