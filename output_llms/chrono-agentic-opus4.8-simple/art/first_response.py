import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())             # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')         # locate bundled vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                          # spawn the ARTcar above the road plane
init_rot = chrono.QuatFromAngleZ(0)                             # heading along +X, no yaw
step_size = 1e-3                                                 # integration time step (s)

car = veh.ARTcar()                                               # ARTcar catalog wrapper (owns its system)
car.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
car.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision against ground
car.SetChassisFixed(False)                                       # MANDATORY — a fixed chassis never moves
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # initial pose
car.SetTireType(veh.TireModelType_TMEASY)                       # TMeasy tire on rigid terrain
car.SetTireStepSize(step_size)                                  # tire force model step
car.Initialize()                                                # build the vehicle subsystems

car.SetChassisVisualizationType(veh.VisualizationType_MESH)         # chassis mesh
car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)# suspension primitives
car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)  # steering primitives
car.SetWheelVisualizationType(veh.VisualizationType_MESH)           # wheel mesh
car.SetTireVisualizationType(veh.VisualizationType_MESH)            # tire mesh

system = car.GetSystem()                                         # take ownership of the wrapper's system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", car.GetVehicle().GetMass())            # report total vehicle mass

terrain = veh.RigidTerrain(system)                              # rigid terrain attached to the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                      # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                      # road friction coefficient
patch_mat.SetRestitution(0.01)                                 # nearly inelastic contact
terrainLength = 100.0                                           # X size of the terrain (m)
terrainWidth = 100.0                                            # Y size of the terrain (m)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # custom road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                  # patch tint
terrain.Initialize()                                           # finalize the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()               # vehicle-specific Irrlicht visual system
vis.SetWindowTitle("ARTcar on Rigid Terrain")                  # window title
vis.SetWindowSize(1280, 1024)                                  # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 4.0, 0.5)     # chase camera tracking the chassis
vis.Initialize()                                               # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                # sky box
vis.AddLightDirectional()                                      # single directional light (vehicle look)
vis.AttachVehicle(car.GetVehicle())                            # bind chassis/wheel/tire visual assets

render_step_size = 1.0 / 50.0                                  # render at 50 frames per second
render_steps = math.ceil(render_step_size / step_size)        # physics steps between rendered frames

driver = veh.ChInteractiveDriverIRR(vis)                       # interactive keyboard driver
steering_time = 1.0                                            # seconds 0 -> full steering
throttle_time = 1.0                                            # seconds 0 -> full throttle
braking_time = 0.3                                             # seconds 0 -> full brake
driver.SetSteeringDelta(render_step_size / steering_time)      # steering response rate
driver.SetThrottleDelta(render_step_size / throttle_time)      # throttle response rate
driver.SetBrakingDelta(render_step_size / braking_time)        # braking response rate
driver.Initialize()                                            # finalize the driver

render_every = max(1, render_steps)                            # untagged render-cadence constant
sim_end = 12.0                                                 # simulation duration (s)

realtime_timer = chrono.ChRealtimeStepTimer()                 # keep wall-clock in step with sim time
while vis.Run() and car.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()                                          # start the frame
    vis.Render()                                              # draw the scene
    vis.EndScene()                                            # finish the frame
    for _ in range(render_every):
        time = car.GetSystem().GetChTime()                    # current sim time
        driver_inputs = driver.GetInputs()                    # current driver command

        driver.Synchronize(time)                              # update driver
        terrain.Synchronize(time)                             # update terrain
        car.Synchronize(time, driver_inputs, terrain)         # apply inputs to the vehicle
        vis.Synchronize(time, driver_inputs)                  # update the HUD/view

        driver.Advance(step_size)                             # advance driver
        terrain.Advance(step_size)                            # advance terrain
        car.Advance(step_size)                                # advances the wrapper-owned system
        vis.Advance(step_size)                                # advance the visual system

        realtime_timer.Spin(step_size)                        # spin so wall-clock matches sim time
        if car.GetSystem().GetChTime() >= sim_end:
            break
