import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 1e-3                                                       # integration step
init_loc = chrono.ChVector3d(0, 0, 0.5)                               # HMMWV chassis spawn
init_rot = chrono.QuatFromAngleZ(0)                                   # facing +X

hmmwv = veh.HMMWV_Full()                                              # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                         # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire on rigid road
hmmwv.SetTireStepSize(step_size)                                     # tire integration step
hmmwv.Initialize()                                                   # build the vehicle
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = hmmwv.GetSystem()                                           # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid terrain owner

patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                           # road friction
patch_mat.SetRestitution(0.01)                                       # low bounce

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # flat highway patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road tiling
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # light grey road

bump_pos = chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT)   # new patch location
bump = terrain.AddPatch(patch_mat, bump_pos, veh.GetDataFile("terrain/meshes/bump.obj"))  # bump.obj mesh patch
bump.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                        # requested patch color
bump.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)    # dirt texture, 6x6 scaling

terrain.Initialize()                                                # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht window
vis.SetWindowTitle("Rigid Highway")                                # window title
vis.SetWindowSize(1280, 1024)                                      # window pixels
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera on chassis
vis.Initialize()                                                   # build device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                          # vehicle-style directional light
vis.AttachVehicle(hmmwv.GetVehicle())                              # bind vehicle visuals

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive driver (truth default)
steering_time = 1.0                                                # 0 -> 1 steering time
throttle_time = 1.0                                                # 0 -> 1 throttle time
braking_time = 0.3                                                 # 0 -> 1 brake time
render_step_size = 1.0 / 50.0                                      # render cadence (s)
driver.SetSteeringDelta(render_step_size / steering_time)          # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)            # brake rate
driver.Initialize()                                                # build driver

render_every = max(1, round(render_step_size / step_size))         # untagged cadence constant
sim_end = 10.0                                                     # total sim time
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = hmmwv.GetSystem().GetChTime()                       # current sim time
        driver_inputs = driver.GetInputs()                         # driver command
        driver.Synchronize(time)                                   # update driver
        terrain.Synchronize(time)                                  # update terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)            # update vehicle
        vis.Synchronize(time, driver_inputs)                       # update vis HUD
        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        hmmwv.Advance(step_size)                                   # advance vehicle (steps system)
        vis.Advance(step_size)                                     # advance vis
        realtime_timer.Spin(step_size)                            # spin to real time
        if hmmwv.GetSystem().GetChTime() >= sim_end:
            break
