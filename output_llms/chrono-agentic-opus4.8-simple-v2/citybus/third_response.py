import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate bundled vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn over the road
init_rot = chrono.QuatFromAngleZ(0)                                  # heading along +X
step_size = 5e-4                                                     # prompt: reduced sim step for stability
tire_step_size = 5e-4                                                # prompt: reduced tire step for stability

terrainLength = 200.0                                               # X size of the rigid patch
terrainWidth = 200.0                                                # Y size of the rigid patch

bus = veh.CityBus()                                                 # city bus catalog wrapper (owns its system)
bus.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision against road
bus.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
bus.SetTireType(veh.TireModelType_PAC89)                           # prompt: Pacejka 89 tire model
bus.SetTireStepSize(tire_step_size)                                # tire integration step
bus.Initialize()                                                   # build the vehicle subsystems

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheel mesh
bus.SetTireVisualizationType(veh.VisualizationType_MESH)           # tire mesh

system = bus.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                # rigid terrain on the bus system
patch_mat = chrono.ChContactMaterialNSC()                         # NSC contact material for the road
patch_mat.SetFriction(0.9)                                        # road friction
patch_mat.SetRestitution(0.01)                                    # near-zero bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)  # prompt: dirt road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # tint
terrain.Initialize()                                             # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                 # vehicle-specific Irrlicht visual system
vis.SetWindowTitle("City Bus on Dirt Road")                     # window title
vis.SetWindowSize(1280, 1024)                                  # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)   # chase camera behind the bus
vis.Initialize()                                               # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                               # sky box
vis.AddLightDirectional()                                    # vehicle truths use a directional light
vis.AttachVehicle(bus.GetVehicle())                          # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                     # interactive driver bound to the vis
steering_time = 1.0                                          # s to go 0 -> +1 steering
throttle_time = 1.0                                          # s to go 0 -> +1 throttle
braking_time = 0.3                                           # s to go 0 -> +1 brake
render_step_size = 1.0 / 50.0                               # render cadence (50 FPS)
driver.SetSteeringDelta(render_step_size / steering_time)   # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)   # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)     # brake ramp rate
driver.Initialize()                                         # build the driver

render_steps = math.ceil(render_step_size / step_size)      # physics steps between rendered frames
sim_end = 20.0                                              # total simulated seconds
realtime_timer = chrono.ChRealtimeStepTimer()              # wall-clock pacing
step_number = 0                                            # physics step counter


while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                              # current sim time

    if step_number % render_steps == 0:                    # throttled rendering
        vis.BeginScene()                                   # begin frame
        vis.Render()                                       # draw the scene
        vis.EndScene()                                     # end frame

    driver_inputs = driver.GetInputs()                     # read driver inputs

    driver.Synchronize(time)                               # sync driver
    terrain.Synchronize(time)                              # sync terrain
    bus.Synchronize(time, driver_inputs, terrain)          # sync vehicle with inputs + terrain
    vis.Synchronize(time, driver_inputs)                   # sync visualization

    driver.Advance(step_size)                              # advance driver
    terrain.Advance(step_size)                             # advance terrain
    bus.Advance(step_size)                                 # advances the wrapper-owned system
    vis.Advance(step_size)                                 # advance visualization

    step_number += 1                                       # next physics step
    realtime_timer.Spin(step_size)                         # spin so wall-clock matches sim time
