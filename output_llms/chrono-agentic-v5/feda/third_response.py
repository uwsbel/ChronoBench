"""FED-Alpha (FEDA) wheeled vehicle on a grass-textured rigid terrain with an
onboard first-person camera sensor.

System type: NSC (rigid-terrain catalog vehicle, Chrono default contact method).
Main bodies: the FEDA wrapper's chassis + four suspended wheels (created by the
veh.FEDA() wrapper, which owns the ChSystemNSC), and a single flat RigidTerrain
patch textured with grass.

Sensors: a ChSensorManager drives an onboard ChCameraSensor rigidly attached to
the vehicle chassis, offset forward to give a first-person view as the vehicle
moves. Point lights illuminate the scene so the camera image is well exposed. The
camera filter chain ends in a ChFilterVisualize (live render) plus a ChFilterSave
stream, and the manager is pumped once per physics step so the camera tracks the
chassis.

Expected behavior: the vehicle rests on the grass terrain at the start, the
interactive driver controls it in real time, and the onboard camera renders a
forward-looking first-person image stream throughout the run.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants === geometry / timing / camera configuration (precomputed once)
step_size = 1e-3                       # physics integration step (s)
sim_end = 12.0                         # bounded recording horizon (s)
render_step_size = 1.0 / 50.0         # 50 FPS visualization cadence
render_steps = math.ceil(render_step_size / step_size)   # precomputed once: steps per frame

terrain_length = 200.0                 # X extent of the grass patch (m)
terrain_width = 200.0                  # Y extent of the grass patch (m)
terrain_height = 0.0                   # top surface Z of the flat terrain (m)

SUSPENSION_REF_HEIGHT = 0.5            # chassis origin above wheel-bottom at rest (m)
init_z = terrain_height + SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(0, 0, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# First-person camera placement: forward of and above the chassis origin, looking ahead.
CAM_W, CAM_H = 1920, 1080              # high-resolution FPV image
CAM_FOV = 1.396                        # ~80 deg horizontal field of view (rad)
CAM_UPDATE_RATE = 30.0                 # physical camera frame rate (Hz), not 1/dt
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(1.8, 0, 1.0),                                  # forward + up, in chassis frame
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),         # look straight ahead (local +X)
)

# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

# === Vehicle === FEDA catalog wrapper owns its ChSystemNSC; NSC for rigid terrain
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)        # NSC matches the rigid-terrain FEDA truth
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                              # MANDATORY — a fixed chassis never moves
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(step_size)
feda.Initialize()

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()                                # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for terrain contact
chassis_body = feda.GetChassisBody()                     # cache: main chassis rigid body, reused below
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())     # report total vehicle mass

# Footprint assert: wheels must rest on (not through) the grass surface after Initialize.
veh_obj = feda.GetVehicle()
TIRE_RADIUS = 0.4636                                     # FEDA Pac02 tire radius (m)
ZTOL = 0.1                                               # allowed wheel-bottom clearance/overlap (m)
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(axle, side).z
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_height - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_height:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === flat rigid patch with a grass texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass terrain texture
terrain.Initialize()

# === Visualization === full vehicle-aware Irrlicht scene: window + sky + chase camera + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA - First Person Camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                # vehicle truths use a directional light
vis.AttachVehicle(feda.GetVehicle())

# === Sensors === sensor manager + point lights + onboard first-person camera
manager = sens.ChSensorManager(system)
light_intensity = 1.0                                    # well-illuminated scene for the camera
manager.scene.AddPointLight(
    chrono.ChVector3f(20, 20, 40),
    chrono.ChColor(light_intensity, light_intensity, light_intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-20, -20, 40),
    chrono.ChColor(light_intensity, light_intensity, light_intensity),
    500.0,
)

fpv_cam = sens.ChCameraSensor(
    chassis_body,                                        # ride on the vehicle chassis (first-person)
    CAM_UPDATE_RATE,                                     # physical update rate (Hz)
    cam_offset,                                          # forward/up offset in the chassis frame
    CAM_W, CAM_H,                                        # high-resolution image
    CAM_FOV,                                             # horizontal field of view (rad)
)
fpv_cam.SetName("FPV Camera Sensor")
fpv_cam.SetLag(0)
fpv_cam.SetCollectionWindow(0)
fpv_cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H, "First Person View"))  # render the image live
fpv_cam.PushFilter(sens.ChFilterRGBA8Access())           # host access to the RGBA8 buffer
fpv_cam.PushFilter(sens.ChFilterSave("cam/fpv/"))        # save the first-person image stream
manager.AddSensor(fpv_cam)

# === Driver === interactive real-time driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === real-time render-throttled Synchronize/Advance, sensors pumped each step

try:

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:                          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        feda.Advance(step_size)
        vis.Advance(step_size)

        manager.Update()                                             # pump the camera once per step

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:    # solver divergence / invalid state during stepping
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + per-sensor video, drop frame dirs
