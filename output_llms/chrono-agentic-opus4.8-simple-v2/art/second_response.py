import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(1, 0, 0.5)                                # initial chassis location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # initial chassis orientation (identity)

step_size = 1e-3                                                       # integration step (s)
tire_step_size = 1e-3                                                  # tire force-model step (s)

contact_method = chrono.ChContactMethod_NSC                           # rigid terrain -> NSC
vis_type = veh.VisualizationType_PRIMITIVES                           # prompt: PRIMITIVES vehicle parts
chassis_collision_type = veh.CollisionType_MESH                       # prompt: chassis collision MESH
tire_model = veh.TireModelType_FIALA                                  # prompt: FIALA tire model

car = veh.ARTcar()                                                     # RC-scale catalog vehicle
car.SetContactMethod(contact_method)                                  # NSC contact for rigid terrain
car.SetChassisCollisionType(chassis_collision_type)                  # chassis collision geometry
car.SetChassisFixed(False)                                            # MANDATORY — fixed chassis won't move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))          # spawn pose
car.SetTireType(tire_model)                                           # FIALA tire force model
car.SetTireStepSize(tire_step_size)                                   # tire sub-step
car.Initialize()                                                      # build the vehicle subsystems

car.SetChassisVisualizationType(vis_type)                            # chassis as PRIMITIVES
car.SetSuspensionVisualizationType(vis_type)                         # suspension as PRIMITIVES
car.SetSteeringVisualizationType(vis_type)                           # steering as PRIMITIVES
car.SetWheelVisualizationType(vis_type)                              # wheels as PRIMITIVES
car.SetTireVisualizationType(vis_type)                               # tires as PRIMITIVES

system = car.GetSystem()                                              # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", car.GetVehicle().GetMass())                  # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                            # tire-road friction
patch_mat.SetRestitution(0.01)                                        # nearly inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)   # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # ground color
terrain.Initialize()                                                 # build terrain collision/visual

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht window
vis.SetWindowTitle("ARTcar Demo")                                    # window title
vis.SetWindowSize(1280, 1024)                                        # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.2), 1.5, 0.3)          # chase-cam track point / dist / height
vis.Initialize()                                                     # create the Irrlicht device
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # vehicle demos use a directional light
vis.AttachVehicle(car.GetVehicle())                                 # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                            # real-time interactive driver
steering_time = 1.0                                                  # s to go 0 -> +1 steering
throttle_time = 1.0                                                  # s to go 0 -> +1 throttle
braking_time = 0.3                                                   # s to go 0 -> +1 brake
driver.SetSteeringDelta(0.02 / steering_time)                       # per-frame steering increment
driver.SetThrottleDelta(0.02 / throttle_time)                       # per-frame throttle increment
driver.SetBrakingDelta(0.02 / braking_time)                         # per-frame brake increment
driver.Initialize()                                                 # build the driver

render_step_size = 1.0 / 50.0                                       # render at 50 FPS
render_every = max(1, round(render_step_size / step_size))         # untagged cadence constant
sim_end = 12.0                                                      # total sim duration (s)
realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing


while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                               # start frame
    vis.Render()                                                   # draw scene
    vis.EndScene()                                                 # end frame
    for _ in range(render_every):
        time = system.GetChTime()                                  # current sim time
        driver_inputs = driver.GetInputs()                         # latest driver inputs
        driver.Synchronize(time)                                   # update driver
        terrain.Synchronize(time)                                  # update terrain
        car.Synchronize(time, driver_inputs, terrain)             # feed inputs + terrain to vehicle
        vis.Synchronize(time, driver_inputs)                      # update vis HUD/state
        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        car.Advance(step_size)                                     # advances the wrapper-owned system
        vis.Advance(step_size)                                     # advance vis
        realtime_timer.Spin(step_size)                            # spin so wall-clock matches sim time
        if system.GetChTime() >= sim_end:
            break
