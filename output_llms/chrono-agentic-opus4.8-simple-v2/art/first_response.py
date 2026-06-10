import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())              # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')          # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                            # initial chassis location (world)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                        # initial orientation (identity)

step_size = 1e-3                                                   # integration time step (s)
tire_step_size = step_size                                        # tire force model step (s)

contact_method = chrono.ChContactMethod_NSC                        # rigid terrain -> NSC contact
vis_type = veh.VisualizationType_MESH                              # mesh visualization for the car

car = veh.ARTcar()                                                 # ARTcar catalog wrapper
car.SetContactMethod(contact_method)                              # NSC contact for rigid terrain
car.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision geometry
car.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
car.SetTireType(veh.TireModelType_TMEASY)                         # TMeasy tire force model
car.SetTireStepSize(tire_step_size)                              # tire integration step
car.Initialize()                                                  # build the vehicle subsystems

car.SetChassisVisualizationType(vis_type)                        # chassis mesh
car.SetSuspensionVisualizationType(vis_type)                     # suspension mesh
car.SetSteeringVisualizationType(vis_type)                       # steering mesh
car.SetWheelVisualizationType(vis_type)                          # wheel mesh
car.SetTireVisualizationType(vis_type)                           # tire mesh

system = car.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", car.GetVehicle().GetMass())              # report total vehicle mass

terrainLength = 100.0                                            # terrain size in X (m)
terrainWidth = 100.0                                             # terrain size in Y (m)

terrain = veh.RigidTerrain(system)                               # rigid terrain on the car's system
patch_mat = chrono.ChContactMaterialNSC()                       # NSC patch material
patch_mat.SetFriction(0.9)                                       # tire-ground friction
patch_mat.SetRestitution(0.01)                                   # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM,            # flat patch at origin
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"),  # custom road texture
                 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                   # patch tint
terrain.Initialize()                                            # build the terrain body

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                # vehicle-aware Irrlicht system
vis.SetWindowTitle("ARTcar on Rigid Terrain")                  # window title
vis.SetWindowSize(1280, 1024)                                   # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)     # chase camera behind the car
vis.Initialize()                                               # create the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                # sky box backdrop
vis.AddLightDirectional()                                      # directional light (vehicle truths)
vis.AttachVehicle(car.GetVehicle())                            # bind chassis/wheel/tire assets

render_step_size = 1.0 / 50.0                                  # display 50 frames per second
render_steps = math.ceil(render_step_size / step_size)        # physics steps between frames

driver = veh.ChInteractiveDriverIRR(vis)                       # interactive keyboard driver
steering_time = 1.0                                            # s to ramp steering 0 -> 1
throttle_time = 1.0                                            # s to ramp throttle 0 -> 1
braking_time = 0.3                                             # s to ramp braking 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)     # steering increment per frame
driver.SetThrottleDelta(render_step_size / throttle_time)     # throttle increment per frame
driver.SetBrakingDelta(render_step_size / braking_time)       # braking increment per frame
driver.Initialize()                                           # initialize the driver

render_every = max(1, render_steps)                           # untagged render-cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                # spin to match wall-clock to sim time
sim_end = 12.0                                                # total simulated time (s)
step_number = 0                                              # physics step counter

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                               # current simulation time

    vis.BeginScene()                                        # begin the render frame
    vis.Render()                                            # draw the scene
    vis.EndScene()                                          # finish the render frame

    for _ in range(render_every):
        time = system.GetChTime()                          # advance-loop time

        driver_inputs = driver.GetInputs()                 # current steering/throttle/brake

        driver.Synchronize(time)                           # sync driver
        terrain.Synchronize(time)                          # sync terrain
        car.Synchronize(time, driver_inputs, terrain)      # sync vehicle with inputs + terrain
        vis.Synchronize(time, driver_inputs)               # sync visualization HUD

        driver.Advance(step_size)                          # advance driver
        terrain.Advance(step_size)                         # advance terrain
        car.Advance(step_size)                             # advances the wrapper-owned system
        vis.Advance(step_size)                             # advance visualization


        step_number += 1                                   # count physics steps
        realtime_timer.Spin(step_size)                     # spin in place to match real time

        if system.GetChTime() >= sim_end:
            break
