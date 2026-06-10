import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn location
init_rot = chrono.QuatFromAngleZ(0)                                  # facing +X
step_size = 1e-3                                                      # integration step (s)
sim_end = 10.0                                                        # simulation duration (s)

feda = veh.FEDA()                                                     # FED-Alpha catalog wheeled vehicle
feda.SetContactMethod(chrono.ChContactMethod_NSC)                    # FEDA truth uses NSC rigid contact
feda.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision against terrain
feda.SetChassisFixed(False)                                          # MANDATORY — fixed chassis won't move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)                  # simple-map engine
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # automatic simple-map transmission
feda.SetTireType(veh.TireModelType_PAC02)                          # Pacejka 2002 tire
feda.SetTireStepSize(step_size)                                     # tire integration step
feda.Initialize()                                                    # build the vehicle

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)       # chassis mesh
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)         # wheel mesh
feda.SetTireVisualizationType(veh.VisualizationType_MESH)          # tire mesh

system = feda.GetSystem()                                            # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())                # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                           # tire-ground friction
patch_mat.SetRestitution(0.01)                                       # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # tint
terrain.Initialize()                                                 # build the terrain

render_step_size = 1.0 / 50.0                                        # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht window
vis.SetWindowTitle("FEDA Camera Sensor")                           # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase-cam track point/dist/height
vis.Initialize()                                                     # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # vehicle truths use a directional light
vis.AttachVehicle(feda.GetVehicle())                               # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive driver bound to the vis
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                    # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                     # braking rate
driver.Initialize()                                                 # init driver

manager = sens.ChSensorManager(system)                              # sensor manager on the vehicle system
intensity = 1.0                                                      # point-light intensity
manager.scene.AddPointLight(                                        # illuminate the scene for the camera
    chrono.ChVector3f(100, 100, 100),
    chrono.ChColor(intensity, intensity, intensity),
    5000.0,
)
manager.scene.AddPointLight(                                        # second point light for even fill
    chrono.ChVector3f(-100, 100, 100),
    chrono.ChColor(intensity, intensity, intensity),
    5000.0,
)

offset_pose = chrono.ChFramed(                                      # camera offset on the chassis (FPV, looking +X)
    chrono.ChVector3d(1.0, 0, 1.5),                                # forward of and above the chassis origin
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),      # no tilt — look straight ahead
)
cam = sens.ChCameraSensor(                                          # first-person-view camera on the chassis
    feda.GetChassisBody(),                                         # ride on the REAL chassis body
    30,                                                            # update_rate (physical Hz)
    offset_pose,                                                   # offset frame on the chassis
    1280, 720,                                                     # high resolution width x height
    1.408,                                                         # horizontal FOV (rad) ~ 80 deg
)
cam.SetName("FPV Camera")                                           # sensor name
cam.SetLag(0)                                                       # truth: lag = 0
cam.SetCollectionWindow(0)                                          # camera: exposure window = 0
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "FPV Camera"))    # live preview window (vis filter)
cam.PushFilter(sens.ChFilterRGBA8Access())                         # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/fpv/"))                      # save the RGB stream as PNGs
manager.AddSensor(cam)                                              # register camera (after all filters)

render_every = render_steps                                        # untagged render-cadence constant
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                               # begin Irrlicht frame
    vis.Render()                                                   # draw the scene
    vis.EndScene()                                                 # end Irrlicht frame
    for _ in range(render_every):
        time = system.GetChTime()                                  # current sim time
        driver_inputs = driver.GetInputs()                         # latest driver inputs

        driver.Synchronize(time)                                   # sync driver
        terrain.Synchronize(time)                                  # sync terrain
        feda.Synchronize(time, driver_inputs, terrain)            # sync vehicle with inputs + terrain
        vis.Synchronize(time, driver_inputs)                      # sync visuals

        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        feda.Advance(step_size)                                    # advance vehicle (steps the system)
        vis.Advance(step_size)                                     # advance visuals

        manager.Update()                                           # pump the camera sensor every physics step

        buffer = cam.GetMostRecentRGBA8Buffer()                   # read the camera frame buffer
        if buffer.HasData():                                       # only after the first sensor tick
            print('Buffer received. Resolution: {0}x{1}'.format(buffer.Width, buffer.Height))
        if system.GetChTime() >= sim_end:
            break
