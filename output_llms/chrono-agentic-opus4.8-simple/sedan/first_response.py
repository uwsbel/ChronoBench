import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate bundled vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                                # chassis spawn above the road
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # facing +X, no rotation
step_size = 2e-3                                                       # integration step
tire_step_size = 1e-3                                                  # TMEASY tire substep

vehicle = veh.BMW_E90()                                                # BMW E90 sedan catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # chassis collision off (no props)
vehicle.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # prompt: TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                              # tire substep
vehicle.Initialize()                                                  # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)      # configurable visualization
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

system = vehicle.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # rigid terrain under the vehicle
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                            # road friction
patch_mat.SetRestitution(0.01)                                        # nearly inelastic road
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # flat 200x200 m road
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # customizable road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                        # road surface color
terrain.Initialize()                                                  # build terrain bodies

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht system
vis.SetWindowTitle("BMW E90 Sedan on Rigid Terrain")                 # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)         # chase camera tracking the chassis
vis.Initialize()                                                      # create the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # customizable logo overlay
vis.AddSkyBox()                                                      # sky box backdrop
vis.AddLightDirectional()                                            # directional lighting (vehicle demos)
vis.AttachVehicle(vehicle.GetVehicle())                             # bind chassis/wheel/tire visuals

render_step_size = 1.0 / 50.0                                        # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive keyboard driver
driver.SetSteeringDelta(render_step_size / 1.0)                     # 1 s to full steering
driver.SetThrottleDelta(render_step_size / 1.0)                     # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)                     # 0.3 s to full braking
driver.Initialize()                                                 # arm the driver

realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                     # physics-step counter
sim_end = 20.0                                                      # simulation duration (s)
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                      # current simulation time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver command


    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)             # feed driver inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                          # update HUD/visuals

    driver.Advance(step_size)                                     # advance driver
    terrain.Advance(step_size)                                    # advance terrain
    vehicle.Advance(step_size)                                    # advances the wrapper-owned system
    vis.Advance(step_size)                                        # advance visuals

    step_number += 1                                              # next step
    realtime_timer.Spin(step_size)                                # pace to wall clock
