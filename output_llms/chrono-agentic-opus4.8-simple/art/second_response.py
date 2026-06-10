import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(1, 0, 0.5)                               # prompt: initial location (1, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation
step_size = 1e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire model sub-step
vis_type = veh.VisualizationType_PRIMITIVES                          # prompt: vehicle parts as PRIMITIVES

car = veh.ARTcar()                                                   # ART 1/10 scale catalog car
car.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
car.SetChassisCollisionType(veh.CollisionType_MESH)                  # prompt: chassis collision MESH
car.SetChassisFixed(False)                                          # MANDATORY — fixed chassis won't move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))          # spawn pose
car.SetTireType(veh.TireModelType_FIALA)                            # prompt: FIALA tire model
car.SetTireStepSize(tire_step_size)                                 # tire sub-step
car.Initialize()                                                    # build the vehicle subsystems

car.SetChassisVisualizationType(vis_type)                          # PRIMITIVES chassis
car.SetSuspensionVisualizationType(vis_type)                       # PRIMITIVES suspension
car.SetSteeringVisualizationType(vis_type)                         # PRIMITIVES steering
car.SetWheelVisualizationType(vis_type)                            # PRIMITIVES wheels
car.SetTireVisualizationType(vis_type)                             # PRIMITIVES tires

system = car.GetSystem()                                            # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED contact scene
print("VEHICLE MASS: ", car.GetVehicle().GetMass())                # truth's literal banner

terrain = veh.RigidTerrain(system)                                  # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                          # tire grip
patch_mat.SetRestitution(0.01)                                      # low bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # sandy color
terrain.Initialize()                                               # build terrain collision

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht view
vis.SetWindowTitle("ARTcar Demo")                                  # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.2), 1.5, 0.3)         # chase the chassis
vis.Initialize()                                                  # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo overlay
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                        # vehicle demos use a directional light
vis.AttachVehicle(car.GetVehicle())                              # bind chassis/wheel/tire visuals

render_step_size = 1.0 / 50.0                                     # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)           # physics steps per frame

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive keyboard driver (truth default)
driver.SetSteeringDelta(render_step_size / 1.0)                  # 1 s to full steering
driver.SetThrottleDelta(render_step_size / 1.0)                  # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)                  # 0.3 s to full brake
driver.Initialize()                                            # bind the driver

sim_end = 10.0                                                  # simulation duration (s)
render_every = max(1, round(1.0 / (50.0 * step_size)))         # untagged render-cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                  # spin to real time
step_number = 0                                                # loop counter
while vis.Run():
    time = system.GetChTime()                                 # current sim time

    if step_number % render_steps == 0:                       # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                        # current driver command

    driver.Synchronize(time)                                  # update driver
    terrain.Synchronize(time)                                 # update terrain
    car.Synchronize(time, driver_inputs, terrain)            # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                     # update HUD/view

    driver.Advance(step_size)                                # advance driver
    terrain.Advance(step_size)                               # advance terrain
    car.Advance(step_size)                                   # advances the wrapper-owned system
    vis.Advance(step_size)                                   # advance view


    step_number += 1                                         # next step
    realtime_timer.Spin(step_size)                           # match wall-clock to sim time
    if time >= sim_end:                                      # stop at duration
        break
