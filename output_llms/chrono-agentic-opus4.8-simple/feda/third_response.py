import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate bundled vehicle data

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn (Z above flat terrain)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # no initial yaw
step_size = 1e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire substep
terrainLength = 100.0                                                # terrain X size (m)
terrainWidth = 100.0                                                 # terrain Y size (m)
sim_end = 12.0                                                       # simulation duration (s)

feda = veh.FEDA()                                                    # FED-Alpha catalog wrapper
feda.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
feda.SetChassisFixed(False)                                         # chassis must be free to move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)                  # simple engine map
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # auto transmission
feda.SetTireType(veh.TireModelType_PAC02)                          # Pacejka tire
feda.SetTireStepSize(tire_step_size)                               # tire integration step
feda.Initialize()                                                  # build the vehicle

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)    # chassis mesh visuals
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)      # wheel mesh visuals
feda.SetTireVisualizationType(veh.VisualizationType_MESH)       # tire mesh visuals

system = feda.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC terrain material
patch_mat.SetFriction(0.9)                                          # high tire grip
patch_mat.SetRestitution(0.01)                                      # nearly inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                      # neutral patch tint
terrain.Initialize()                                               # build the terrain

manager = sens.ChSensorManager(system)                             # sensor manager for the camera
intensity = 1.0                                                    # point-light intensity
manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100),     # well-lit scene
                            chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100, -100, 100),   # second fill light
                            chrono.ChColor(intensity, intensity, intensity), 500.0)

offset_pose = chrono.ChFramed(                                     # FPV pose on the chassis
    chrono.ChVector3d(0.0, 0.0, 1.5),                             # forward-looking, above the hood
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),      # local +X faces vehicle forward
)
cam = sens.ChCameraSensor(
    feda.GetChassisBody(),                                         # ride on the chassis body
    30,                                                           # update_rate (physical Hz)
    offset_pose,                                                  # offset pose on the chassis
    1280, 720,                                                    # high-resolution image
    1.408,                                                        # horizontal FOV (rad)
)
cam.SetName("FPV Camera")                                          # camera label
cam.SetLag(0)                                                     # no lag
cam.SetCollectionWindow(0)                                        # instantaneous exposure
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "FPV Camera"))  # render the image to a window
cam.PushFilter(sens.ChFilterRGBA8Access())                       # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/fpv/"))                    # save the FPV frames
manager.AddSensor(cam)                                            # include the camera in the manager

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht window
vis.SetWindowTitle("FEDA First Person View")                     # window title
vis.SetWindowSize(1280, 1024)                                     # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)      # chase camera behind chassis
vis.Initialize()                                                 # build the Irrlicht device
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                  # sky box
vis.AddLightDirectional()                                        # directional light (vehicle truth)
vis.AttachVehicle(feda.GetVehicle())                            # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive driver (truth default)
steering_time = 1.0                                              # time 0->+1 steering
throttle_time = 1.0                                             # time 0->+1 throttle
braking_time = 0.3                                              # time 0->+1 braking
driver.SetSteeringDelta(0.02 / steering_time)                  # steering rate
driver.SetThrottleDelta(0.02 / throttle_time)                 # throttle rate
driver.SetBrakingDelta(0.02 / braking_time)                  # braking rate
driver.Initialize()                                            # build the driver

render_step_size = 1.0 / 50.0                                  # 50 FPS render cadence
render_steps = math.ceil(render_step_size / step_size)        # physics steps per render
realtime_timer = chrono.ChRealtimeStepTimer()                # spin to wall-clock
step_number = 0                                              # physics step counter

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                # current sim time

    if step_number % render_steps == 0:                      # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                       # current driver inputs

    driver.Synchronize(time)                                 # update driver
    terrain.Synchronize(time)                                # update terrain
    feda.Synchronize(time, driver_inputs, terrain)           # update vehicle
    vis.Synchronize(time, driver_inputs)                     # update visualization

    driver.Advance(step_size)                                # advance driver
    terrain.Advance(step_size)                               # advance terrain
    feda.Advance(step_size)                                  # advance vehicle (steps the system)
    vis.Advance(step_size)                                   # advance visualization

    manager.Update()                                         # update camera every step


    step_number += 1                                         # advance the step count
    realtime_timer.Spin(step_size)                           # match wall-clock to sim time
