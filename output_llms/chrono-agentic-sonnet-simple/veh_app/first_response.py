import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# simulation parameters
step_size = 5e-3                   # physics time step
render_fps = 50                    # target render frame rate
render_steps = math.ceil((1.0 / render_fps) / step_size)

# Gator vehicle setup
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)           # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)                                  # MANDATORY — fixed chassis won't move
init_loc = chrono.ChVector3d(0, 0, 0.5)                      # initial spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                   # no initial rotation
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetTireType(veh.TireModelType_TMEASY)                  # TMEASY tires (good on rigid terrain)
gator.SetTireStepSize(step_size)
gator.Initialize()

# collision system — REQUIRED for terrain/contact scene
system = gator.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())         # truth's literal banner

# visualization types for vehicle parts (VisualizationType lives in veh namespace, not chrono)
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                     # NSC to match vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Irrlicht vehicle visualization system (Initialize first, then scene elements)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle with Sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)  # chase camera settings
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                      # directional light (vehicle truths use this)
vis.AttachVehicle(gator.GetVehicle())

# interactive driver — scored-core default for catalog vehicles
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_steps * step_size / 1.0)        # 1.0 s to full steering
driver.SetThrottleDelta(render_steps * step_size / 1.0)        # 1.0 s to full throttle
driver.SetBrakingDelta(render_steps * step_size / 0.3)         # 0.3 s to full braking
driver.Initialize()

# sensor manager with point lights for camera sensor
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(                                   # scene light #1
    chrono.ChVector3f(10, 10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                   # scene light #2
    chrono.ChVector3f(-10, 10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# camera sensor attached to the vehicle's chassis
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 3),                               # behind and above the chassis
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)), # slight downward tilt
)
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),                                    # attach to chassis body
    30,                                                        # update_rate Hz (physical rate)
    offset_pose,
    1280, 720,                                                 # width, height
    1.408,                                                     # horizontal FOV (rad)
)
cam.SetName("Camera Sensor")
cam.SetLag(0)                                                  # no lag
cam.SetCollectionWindow(0)                                     # instantaneous exposure
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera")) # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                     # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                  # save color PNGs (scored core)
manager.AddSensor(cam)

# real-time step timer
realtime_timer = chrono.ChRealtimeStepTimer()


step_number = 0
while vis.Run():
    time = system.GetChTime()

    if step_number % render_steps == 0:                       # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    # synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)                                  # advances wrapper-owned system
    vis.Advance(step_size)

    manager.Update()                                          # update sensor manager (scored core)


    step_number += 1
    realtime_timer.Spin(step_size)                            # real-time pacing

# check camera buffer once after final step
buf = cam.GetMostRecentRGBA8Buffer()                         # access sensor buffer
if buf.HasData():
    print('Buffer received. Resolution: {0}x{1}'.format(buf.Width, buf.Height))
