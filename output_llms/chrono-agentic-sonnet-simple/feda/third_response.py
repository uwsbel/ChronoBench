import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                    # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                # locate vehicle data files

step_size = 2e-3                                                         # physics time step (s)
sim_end = 30.0                                                           # simulation end time (s)
render_fps = 50.0                                                        # render frame rate (Hz)
render_every = max(1, round(1.0 / (render_fps * step_size)))            # render cadence (untagged)

terrainLength = 300.0                                                    # terrain length (m)
terrainWidth = 300.0                                                     # terrain width (m)

init_pos = chrono.ChVector3d(0, 0, 0.5)                                 # vehicle initial position
init_rot = chrono.QUNIT                                                  # identity rotation

feda = veh.FEDA()                                                        # FEDA vehicle wrapper
feda.SetContactMethod(chrono.ChContactMethod_NSC)                       # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                                              # MANDATORY — fixed chassis won't move
feda.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(step_size)
feda.Initialize()

system = feda.GetSystem()                                                # vehicle owns its system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # REQUIRED after Initialize

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())                    # report total vehicle mass

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)                                       # rigid flat terrain

patch_mat = chrono.ChContactMaterialNSC()                               # NSC contact material
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                                     # centered at origin
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass terrain texture
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.3))                           # green tint for grass

terrain.Initialize()

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA Vehicle — Camera Sensor Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                                # vehicle truths use directional light
vis.AttachVehicle(feda.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)                                # interactive driver (scored-core default)

steering_time = 1.0                                                      # s to reach max steering
throttle_time = 1.0                                                      # s to reach max throttle
braking_time = 0.3                                                       # s to reach max braking

driver.SetSteeringDelta(render_fps / steering_time / 50.0)
driver.SetThrottleDelta(render_fps / throttle_time / 50.0)
driver.SetBrakingDelta(render_fps / braking_time / 50.0)
driver.Initialize()

# --- sensor manager + lighting (scored core) ---
manager = sens.ChSensorManager(system)                                   # sensor manager for the vehicle system

intensity = 1.0                                                          # light intensity
manager.scene.AddPointLight(                                             # point light 1
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                             # point light 2
    chrono.ChVector3f(50, 50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                             # point light 3
    chrono.ChVector3f(-50, -50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# --- camera sensor attached to chassis for First Person View (scored core) ---
chassis_body = feda.GetChassisBody()                                     # attach camera to chassis body

# Offset pose: place the camera at the front of the chassis, at eye level, looking forward
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(2.0, 0, 1.0),                                     # front, center, eye-level offset in chassis frame
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),          # no tilt — looking straight ahead
)

cam = sens.ChCameraSensor(
    chassis_body,                                                        # attach to the chassis (FPV rides with vehicle)
    30,                                                                  # update_rate Hz — physical rate
    cam_offset,                                                          # offset pose on the chassis
    1920, 1080,                                                          # high resolution: 1920×1080
    1.408,                                                               # horizontal FOV ~80.6° (standard)
)
cam.SetName("FPV Camera Sensor")
cam.SetLag(0)                                                            # no sensor lag
cam.SetCollectionWindow(0)                                               # camera: exposure window = 0

cam.PushFilter(sens.ChFilterVisualize(1920, 1080, "FPV Camera"))        # live preview window
cam.PushFilter(sens.ChFilterRGBA8Access())                               # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/fpv/"))                           # save FPV frames
manager.AddSensor(cam)                                                   # add camera to manager after all filters

realtime_timer = chrono.ChRealtimeStepTimer()                            # real-time pacing
step_number = 0


while vis.Run():
    time = feda.GetSystem().GetChTime()

    if step_number % render_every == 0:                                  # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    feda.Synchronize(time, driver_inputs, terrain)                       # 3-arg wheeled vehicle
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    feda.Advance(step_size)                                              # advances the wrapper-owned system
    vis.Advance(step_size)

    manager.Update()                                                     # pump sensors every step (camera + filter chain)


    step_number += 1
    realtime_timer.Spin(step_size)                                       # spin to match wall-clock
