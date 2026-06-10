import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 2e-3                                                       # physics step (s)
tire_step_size = 1e-3                                                  # tire substep (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                               # chassis spawn above terrain
init_rot = chrono.QUNIT                                                # no initial yaw

vehicle = veh.Gator()                                                  # Gator catalog wrapper (owns its system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # no wrapper chassis collision
vehicle.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire on rigid terrain
vehicle.SetTireStepSize(tire_step_size)                              # tire integration substep
vehicle.Initialize()                                                  # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis as mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension as primitives
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering as primitives
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheels as mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)        # tires as mesh

system = vehicle.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED contact backend
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # rigid ground under the vehicle
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                            # high grip
patch_mat.SetRestitution(0.01)                                        # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # flat 200 x 200 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # sandy color
terrain.Initialize()                                                  # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht window
vis.SetWindowTitle("Gator Vehicle")                                  # window title
vis.SetWindowSize(1280, 720)                                          # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)         # chase cam track point / dist / height
vis.Initialize()                                                      # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                       # sky box
vis.AddLightDirectional()                                            # vehicle scenes use a directional light
vis.AttachVehicle(vehicle.GetVehicle())                              # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive (keyboard) driver
render_step_size = 1.0 / 50.0                                        # 50 fps render cadence
steering_time = 1.0                                                   # s to go 0 -> +1 steering
throttle_time = 1.0                                                   # s to go 0 -> +1 throttle
braking_time = 0.3                                                    # s to go 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)           # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)             # braking ramp rate
driver.Initialize()                                                  # arm the driver

manager = sens.ChSensorManager(system)                              # sensor manager on the vehicle system
intensity = 1.0                                                      # point-light intensity
manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100),       # key point light
                            chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100, 100, 100),      # fill point light
                            chrono.ChColor(intensity, intensity, intensity), 500.0)

offset_pose = chrono.ChFramed(                                       # camera offset on the chassis
    chrono.ChVector3d(-5.0, 0.0, 2.0),                              # behind and above the chassis
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),      # look forward (+X)
)
cam = sens.ChCameraSensor(                                          # chassis-mounted RGB camera
    vehicle.GetChassisBody(),                                       # rides on the chassis body
    30,                                                             # physical update_rate (Hz)
    offset_pose,                                                    # offset frame on the chassis
    1280, 720,                                                      # image width, height
    1.408,                                                          # horizontal FOV (rad)
)
cam.SetName("Chassis Camera")                                       # sensor name
cam.SetLag(0)                                                       # no lag
cam.SetCollectionWindow(0)                                          # instantaneous exposure
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis Camera"))  # live preview of the camera
cam.PushFilter(sens.ChFilterRGBA8Access())                         # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                      # save rendered camera images
manager.AddSensor(cam)                                              # register the camera

render_steps = math.ceil(render_step_size / step_size)             # untagged: physics steps per frame
realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacing
step_number = 0                                                    # untagged: loop step counter
while vis.Run():
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver command

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)             # update vehicle with inputs + terrain
    vis.Synchronize(time, driver_inputs)                          # update visualization

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    vehicle.Advance(step_size)                                     # advance vehicle (steps the system)
    vis.Advance(step_size)                                         # advance visualization

    manager.Update()                                               # pump sensors once per step

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # pace to real time
