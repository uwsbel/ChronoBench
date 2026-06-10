import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate bundled vehicle data files

init_loc = chrono.ChVector3d(-50, 0, 0.5)                            # vehicle start moved to fit the DLC maneuver
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (facing +X)
step_size = 1e-3                                                      # integration step (s)
tire_step_size = 1e-3                                                 # tire force model step (s)

terrainLength = 200.0                                                 # X extent enlarged so the DLC fits the patch
terrainWidth = 100.0                                                  # Y extent of the rigid patch

feda = veh.FEDA()                                                     # FED-Alpha catalog vehicle wrapper
feda.SetContactMethod(chrono.ChContactMethod_NSC)                    # FEDA truth uses NSC rigid-terrain contact
feda.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
feda.SetChassisFixed(False)                                          # MANDATORY — fixed chassis never moves
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose in world frame
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)                  # FEDA engine model
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # FEDA transmission model
feda.SetTireType(veh.TireModelType_PAC02)                          # FEDA Pacejka tire model
feda.SetTireStepSize(tire_step_size)                               # tire sub-step
feda.Initialize()                                                    # build the vehicle subsystems

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)    # chassis mesh
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheel mesh
feda.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire mesh

system = feda.GetSystem()                                            # take the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # flat rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material to match the vehicle
patch_mat.SetFriction(0.9)                                          # road friction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic road
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # 200 x 100 flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch base color
terrain.Initialize()                                                # finalize terrain

path = veh.DoubleLaneChangePath(init_loc, 13.5, 4.0, 11.0, 50.0, True)  # ISO double lane change path from start
target_speed = 10.0                                                  # cruise-control target speed (m/s)
driver = veh.ChPathFollowerDriver(feda.GetVehicle(), path, "DLC_path", target_speed)  # path-follower + cruise control
driver.GetSteeringController().SetLookAheadDistance(5.0)            # look-ahead distance for the steering PID
driver.GetSteeringController().SetGains(0.8, 0, 0)                 # steering controller gains (KP, KI, KD)
driver.GetSpeedController().SetGains(0.4, 0, 0)                    # speed controller gains (KP, KI, KD)
driver.Initialize()                                                  # build the driver

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht visual system
vis.SetWindowTitle("FEDA Double Lane Change")                     # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)      # chase camera tracking the chassis
vis.Initialize()                                                    # create the Irrlicht device (call first)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # vehicle scenes use a directional light
vis.AttachVehicle(feda.GetVehicle())                              # bind chassis/wheel/tire visual assets

render_step_size = 1.0 / 50.0                                       # one rendered frame every 1/50 s
render_steps = math.ceil(render_step_size / step_size)            # physics steps between rendered frames
render_every = render_steps                                        # untagged cadence constant for the record batch
sim_end = 16.0                                                      # simulation duration (s)


realtime_timer = chrono.ChRealtimeStepTimer()                     # spin loop to wall-clock real time
step_number = 0                                                    # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                      # current simulation time

    if step_number % render_steps == 0:                           # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                            # current driver command

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    feda.Synchronize(time, driver_inputs, terrain)               # update vehicle with inputs + terrain
    vis.Synchronize(time, driver_inputs)                         # update visualization

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    feda.Advance(step_size)                                        # advance vehicle (steps the owned system)
    vis.Advance(step_size)                                         # advance visualization


    step_number += 1                                              # advance step counter
    realtime_timer.Spin(step_size)                               # spin in place to match wall-clock time
