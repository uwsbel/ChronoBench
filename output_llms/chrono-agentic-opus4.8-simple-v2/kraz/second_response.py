import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

terrainLength = 100.0                                                  # terrain size in X
terrainWidth = 100.0                                                   # terrain size in Y

step_size = 1e-3                                                       # integration step
tire_step_size = 1e-3                                                  # tire force step
render_step_size = 1.0 / 50.0                                          # 50 FPS render cadence

init_loc = chrono.ChVector3d(-15, 0, 0.5)                             # initial chassis location
init_rot = chrono.QuatFromAngleZ(0)                                    # initial heading (no yaw)

truck = veh.Kraz()                                                     # semi-trailer tractor wrapper
truck.SetChassisFixed(False)                                          # MANDATORY — fixed chassis won't move
truck.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # spawn pose
truck.SetTireStepSize(tire_step_size)                                 # tire substep
truck.SetInitFwdVel(0.0)                                               # start from rest
truck.Initialize()                                                    # build the truck subsystems

truck.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
truck.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

system = truck.GetSystem()                                            # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", truck.GetTractor().GetMass())                # report tractor mass

terrain = veh.RigidTerrain(truck.GetSystem())                        # flat rigid road
patch_mat = chrono.ChContactMaterialNSC()                            # NSC material for rigid terrain
patch_mat.SetFriction(0.9)                                            # road friction
patch_mat.SetRestitution(0.01)                                        # near-zero bounciness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetColor(chrono.ChColor(0.5, 0.5, 1.0))                        # tint
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # road texture
terrain.Initialize()                                                 # build terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht view
vis.SetWindowTitle("Semi-trailer truck :: Double Lane Change")       # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(3, 0, 2.1), 25.0, 10.5)        # track point + chase distance/height
vis.Initialize()                                                     # build device first
vis.AddLightDirectional()                                           # vehicle truths use a directional light
vis.AddSkyBox()                                                      # sky
vis.AddLogo()                                                        # logo overlay
vis.AttachVehicle(truck.GetTractor())                               # bind tractor visual assets

# Scripted double-lane-change maneuver driven by simulation time (steering, throttle, braking).
driver_data = veh.vector_Entry([                                     # (time, steering, throttle, braking)
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                        # rest
    veh.DataDriverEntry(0.5, 0.0, 0.7, 0.0),                        # accelerate straight
    veh.DataDriverEntry(2.0, 0.0, 0.7, 0.0),                        # cruise straight
    veh.DataDriverEntry(2.5, 0.4, 0.7, 0.0),                        # steer left into lane 2
    veh.DataDriverEntry(3.0, 0.0, 0.7, 0.0),                        # straighten in lane 2
    veh.DataDriverEntry(3.5, -0.4, 0.7, 0.0),                       # steer right back toward lane 1
    veh.DataDriverEntry(4.0, 0.0, 0.7, 0.0),                        # straighten in lane 1
    veh.DataDriverEntry(4.5, -0.4, 0.7, 0.0),                       # steer right into lane 0
    veh.DataDriverEntry(5.0, 0.0, 0.7, 0.0),                        # straighten in lane 0
    veh.DataDriverEntry(5.5, 0.4, 0.7, 0.0),                        # steer left back toward center
    veh.DataDriverEntry(6.0, 0.0, 0.7, 0.0),                        # recover heading
    veh.DataDriverEntry(10.0, 0.0, 0.0, 0.0),                       # coast
])
driver = veh.ChDataDriver(truck.GetTractor(), driver_data)          # time-table scripted driver
driver.Initialize()                                                 # build driver

render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame
sim_end = 10.0                                                       # simulation length (s)


realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                     # physics step counter
while vis.Run() and truck.GetSystem().GetChTime() < sim_end:
    time = truck.GetSystem().GetChTime()                            # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # scripted inputs for this time

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    truck.Synchronize(time, driver_inputs, terrain)               # apply inputs to truck
    vis.Synchronize(time, driver_inputs)                          # update HUD/view

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    truck.Advance(step_size)                                       # advances the wrapper-owned system
    vis.Advance(step_size)                                         # advance view


    step_number += 1                                               # advance counter
    realtime_timer.Spin(step_size)                                 # pace to real time
