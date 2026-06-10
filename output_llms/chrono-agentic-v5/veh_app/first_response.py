"""Gator wheeled-vehicle demo on flat rigid terrain (NSC contact).

Models a John-Deere-style Gator utility vehicle driving on a flat RigidTerrain
patch, visualized with an Irrlicht chase-camera window. The chassis, suspension,
steering, wheels, and tires are each rendered with a deliberately chosen
visualization type (mesh vs. primitives) so the part breakdown is visible. An
interactive driver (keyboard) controls steering / throttle / braking in real
time. A camera sensor riding on the chassis renders an onboard RGB image stream
through an OptiX ChSensorManager lit by point lights.

System: ChSystemNSC owned by the veh.Gator wrapper. Bodies: Gator chassis +
suspension/steering links + four spindles/wheels/tires, plus the rigid terrain
patch body. Expected behavior: the vehicle rests on the terrain at spawn and
drives under driver input; the onboard camera produces a forward-looking RGB
stream of the moving scene.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Parameters === geometry / physics constants (no bare literals downstream)
step_size = 2e-3                       # integration step (s)
sim_end = 12.0                         # bounded run length for the recording (s)
render_fps = 50.0                      # Irrlicht render cadence (frames/s)
init_loc = chrono.ChVector3d(0, 0, 0.5)        # chassis spawn (origin, on terrain)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    # spawn orientation (identity)
terrain_length = 100.0                 # rigid patch X extent (m)
terrain_width = 100.0                  # rigid patch Y extent (m)
cam_update_rate = 30.0                 # camera sensor physical rate (Hz)
cam_w, cam_h = 1280, 720               # camera image resolution (px)
cam_fov = 1.408                        # camera horizontal FOV (rad)

render_steps = math.ceil((1.0 / render_fps) / step_size)   # precomputed once

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === Gator catalog wrapper (owns its ChSystemNSC + all bodies)
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetTireType(veh.TireModelType_TMEASY)          # deformable tire model on rigid road
gator.SetTireStepSize(step_size)
gator.Initialize()

# Various vehicle parts set to different visualization types (after Initialize).
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.Gator wrapper) ===
system = gator.GetSystem()                # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED, after Initialize
chassis = gator.GetChassisBody()          # cache: main chassis rigid body, reused for camera
# spindles/wheels: gator.GetVehicle().GetAxle(i)... ; suspension+steering joints live in the wrapper
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())                # report total vehicle mass

# === Terrain === flat rigid patch under the vehicle
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window (chase camera + sky + lights)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                 # vehicle truths use a directional light
vis.AttachVehicle(gator.GetVehicle())

# === Driver === interactive (keyboard) steering / throttle / braking
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                        # s to ramp steering 0 -> 1
throttle_time = 1.0                        # s to ramp throttle 0 -> 1
braking_time = 0.3                         # s to ramp braking 0 -> 1
driver.SetSteeringDelta((1.0 / render_fps) / steering_time)
driver.SetThrottleDelta((1.0 / render_fps) / throttle_time)
driver.SetBrakingDelta((1.0 / render_fps) / braking_time)
driver.Initialize()

# === Sensor manager === OptiX scene + point lights + onboard chassis camera
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-2.0, 0, 1.2),                              # behind & above the chassis
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),    # look forward (+X)
)
cam = sens.ChCameraSensor(chassis, cam_update_rate, offset_pose, cam_w, cam_h, cam_fov)
cam.SetName("Onboard Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(cam_w, cam_h, "Onboard Camera"))   # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                               # host RGBA8 access
cam.PushFilter(sens.ChFilterSave("cam/onboard/"))                        # RGB image stream
manager.AddSensor(cam)

# === Main loop === advance driver/terrain/vehicle/vis + sensor manager each step
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)            # advances the wrapper-owned system
        vis.Advance(step_size)

        manager.Update()                    # pump the camera sensor once per step

        step_number += 1
        realtime_timer.Spin(step_size)      # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video(s) + plot, then drop PNG dirs
