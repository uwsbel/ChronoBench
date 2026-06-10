import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())              # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')          # locate vehicle data files

initLoc = chrono.ChVector3d(0, 0, 0.5)                            # spawn above terrain
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                        # no initial rotation

step_size = 1e-3                                                   # physics time step (s)
sim_end = 20.0                                                     # simulation end time (s)
render_fps = 50.0                                                  # render frame rate

sedan = veh.BMW_E90()                                              # BMW E90 sedan catalog wrapper
sedan.SetContactMethod(chrono.ChContactMethod_NSC)                # NSC for rigid terrain
sedan.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision geometry
sedan.SetChassisFixed(False)                                       # MANDATORY: chassis must be free
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))      # initial pose
sedan.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire (prompt specifies)
sedan.SetTireStepSize(step_size)                                   # tire sub-step
sedan.Initialize()                                                 # initialize all subsystems
system = sedan.GetSystem()                                         # grab the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED: bullet collision

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)     # mesh chassis
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)       # mesh wheels
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)        # mesh tires

print("VEHICLE MASS: ", sedan.GetVehicle().GetMass())             # vehicle mass diagnostic (scored)

terrain = veh.RigidTerrain(system)                                 # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
patch_mat.SetFriction(0.9)                                         # tire-road friction coefficient
patch_mat.SetRestitution(0.01)                                     # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # terrain color
terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht vis system
vis.SetWindowTitle("BMW E90 Sedan - Rigid Terrain")                # window title
vis.SetWindowSize(1280, 1024)                                      # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)      # chase camera config
vis.Initialize()                                                   # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChono logo
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                          # directional light (vehicle style)
vis.AttachVehicle(sedan.GetVehicle())                              # bind vehicle meshes to scene

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive keyboard driver
render_step_size = 1.0 / render_fps                               # seconds per render frame
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering rate (1 s to max)
driver.SetThrottleDelta(render_step_size / 1.0)                    # throttle rate (1 s to max)
driver.SetBrakingDelta(render_step_size / 0.3)                     # braking rate (0.3 s to max)
driver.Initialize()                                                # init driver

render_every = max(1, round(render_step_size / step_size))        # physics steps per rendered frame


realtime_timer = chrono.ChRealtimeStepTimer()                     # real-time pacing timer
step_number = 0                                                    # step counter

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                      # current sim time

    if step_number % render_every == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # read current driver inputs


    driver.Synchronize(time)                                       # driver receives time
    terrain.Synchronize(time)                                      # terrain receives time
    sedan.Synchronize(time, driver_inputs, terrain)               # vehicle gets inputs + terrain
    vis.Synchronize(time, driver_inputs)                           # vis HUD gets inputs

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    sedan.Advance(step_size)                                       # advance vehicle (ticks ChSystem)
    vis.Advance(step_size)                                         # advance vis

    step_number += 1                                               # increment step counter
    realtime_timer.Spin(step_size)                                 # pace to real time
