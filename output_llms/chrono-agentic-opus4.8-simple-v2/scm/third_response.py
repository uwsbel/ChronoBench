import math                                                           # render-cadence + box placement
import random                                                         # random box positions
import pychrono.core as chrono                                        # core PyChrono
import pychrono.vehicle as veh                                        # wheeled-vehicle catalog
import pychrono.sensor as sens                                        # OptiX sensor framework
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer

random.seed(1)                                                        # deterministic box layout

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                                                      # physics step (s)
tire_step_size = 1e-3                                                 # tire integration step (s)
sim_end = 20.0                                                        # total simulation time (s)

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (X, Y, Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation

hmmwv = veh.HMMWV_Full()                                              # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial chassis pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                                # tire substep
hmmwv.Initialize()                                                   # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheels mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # tires mesh

system = hmmwv.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                           # tire grip
patch_mat.SetRestitution(0.01)                                       # near-inelastic ground
terrainLength = 100.0                                                # X extent (m)
terrainWidth = 100.0                                                 # Y extent (m)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # centered flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # sandy color
terrain.Initialize()                                                # finalize terrain

box_mat = chrono.ChContactMaterialNSC()                             # NSC material for the random boxes
box_mat.SetFriction(0.9)                                            # box-ground friction
num_boxes = 30                                                      # how many random boxes
box_clear_radius = 6.0                                              # keep boxes this far from the vehicle spawn
for i in range(num_boxes):                                         # introduce randomly positioned boxes
    size = random.uniform(0.3, 0.8)                                # cube edge length (m)
    while True:                                                    # reject positions on top of the vehicle
        bx = random.uniform(-terrainLength / 2 + 2, terrainLength / 2 - 2)  # random X
        by = random.uniform(-terrainWidth / 2 + 2, terrainWidth / 2 - 2)    # random Y
        if math.hypot(bx - init_loc.x, by - init_loc.y) > box_clear_radius:  # not inside the vehicle
            break                                                  # accept this position
    box = chrono.ChBodyEasyBox(size, size, size, 1000, True, True, box_mat)  # visualized + collidable box
    box.SetPos(chrono.ChVector3d(bx, by, size / 2))               # rest the box on the ground
    box.SetFixed(True)                                            # static obstacle prop
    system.Add(box)                                              # add the box to the scene

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-aware Irrlicht window
vis.SetWindowTitle("HMMWV Camera Sensor Scene")                    # window title (before Initialize)
vis.SetWindowSize(1280, 1024)                                     # window size (before Initialize)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)       # follow camera (track point, dist, height)
vis.Initialize()                                                 # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # corner logo (after Initialize)
vis.AddSkyBox()                                                  # sky box (after Initialize)
vis.AddLightDirectional()                                        # vehicle demos use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive driver bound to the vis
render_step_size = 1.0 / 50.0                                     # driver/render cadence (s)
steering_time = 1.0                                              # s to ramp steering 0 -> 1
throttle_time = 1.0                                              # s to ramp throttle 0 -> 1
braking_time = 0.3                                               # s to ramp braking 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)        # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)          # braking rate
driver.Initialize()                                             # finalize the driver

manager = sens.ChSensorManager(system)                           # OptiX sensor manager on the shared system
intensity = 1.0                                                 # point-light intensity
manager.scene.AddPointLight(                                    # integrate point lights at various positions
    chrono.ChVector3f(10, 10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-10, 10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(10, -10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-10, -10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

offset_pose = chrono.ChFramed(                                   # camera pose on the chassis
    chrono.ChVector3d(-5.0, 0, 2.0),                            # behind and above the chassis origin
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),  # look forward (+X)
)
cam = sens.ChCameraSensor(
    hmmwv.GetChassisBody(),                                     # attach the camera to the vehicle chassis
    30,                                                        # update_rate (Hz) — physical rate
    offset_pose,                                               # offset pose on the chassis
    1280, 720,                                                 # image width, height
    1.408,                                                     # horizontal FOV (rad)
)
cam.SetName("Chassis Camera")                                  # sensor name
cam.SetLag(0)                                                  # no extra lag
cam.SetCollectionWindow(0)                                     # instantaneous exposure
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis Camera"))  # live preview window
cam.PushFilter(sens.ChFilterRGBA8Access())                     # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))           # SAVE stream: color PNGs
manager.AddSensor(cam)                                         # register the camera (filters pushed first)

render_steps = math.ceil(render_step_size / step_size)         # physics steps per rendered frame
render_every = render_steps                                    # untagged cadence constant
realtime_timer = chrono.ChRealtimeStepTimer()                  # spin to real time
step_number = 0                                               # step counter for render throttling
while vis.Run() and system.GetChTime() < sim_end:             # main real-time loop
    time = system.GetChTime()                                # current sim time

    if step_number % render_steps == 0:                      # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                       # poll driver inputs

    driver.Synchronize(time)                                 # driver <- time
    terrain.Synchronize(time)                                # terrain <- time
    hmmwv.Synchronize(time, driver_inputs, terrain)          # vehicle <- inputs + terrain
    vis.Synchronize(time, driver_inputs)                     # vis <- inputs

    driver.Advance(step_size)                                # advance driver
    terrain.Advance(step_size)                               # advance terrain
    hmmwv.Advance(step_size)                                 # advance vehicle (steps the system)
    vis.Advance(step_size)                                   # advance vis

    manager.Update()                                         # pump sensors once per step

    buffer = cam.GetMostRecentRGBA8Buffer()                  # read most recent camera buffer
    if buffer.HasData():                                     # only once the sensor has ticked
        print('Buffer received. Resolution: {0}x{1}'.format(buffer.Width, buffer.Height))


    step_number += 1                                         # advance counter
    realtime_timer.Spin(step_size)                           # spin to wall-clock
