"""FEDA wheeled vehicle on rigid grass terrain with an onboard first-person camera.

Models a single FEDA (Ford Expedition derivative) wheeled vehicle driving on a
flat rigid terrain patch textured with grass. The vehicle uses TMEASY tires and a
simple-map engine/transmission, controlled by a scripted (open-loop) driver that
accelerates gently forward with a slight steering sweep. A chase-camera Irrlicht
window provides the review visualization, while an OptiX camera sensor is rigidly
attached to the chassis body to produce a high-resolution first-person (FPV) view
through the sensor manager's visualize + save filter chain.

System: ChSystemSMC owned by the veh.FEDA wrapper (vehicle/terrain contact).
Main bodies: FEDA chassis + 4 wheels/spindles (wrapper-created), rigid terrain patch.
Expected behavior: the vehicle rolls forward on the grass, the chase view tracks it,
and the onboard sensor renders a forward-looking FPV stream from the chassis.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr


# === Parameters === geometry / physics / control constants (no bare literals downstream)
time_step = 2.0e-3                 # integration step (s)
tire_step = 1.0e-3                 # tire force-model substep (s)
sim_end = 8.0                      # simulated duration (s)
render_fps = 50.0                  # Irrlicht review-frame cadence

terrain_length = 200.0             # rigid patch X size (m)
terrain_width = 200.0              # rigid patch Y size (m)

init_x = 0.0                       # vehicle spawn X (m)
init_y = 0.0                       # vehicle spawn Y (m)
init_z = 0.5                       # chassis-origin height above flat terrain (m)
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.QUNIT

cam_res_w = 1920                   # FPV camera horizontal resolution (high res)
cam_res_h = 1080                   # FPV camera vertical resolution (high res)
cam_fov = 1.2217                   # FPV horizontal field of view (rad, ~70 deg)
cam_update_rate = 30.0             # FPV sensor update rate (Hz) — modest so render < timeout
light_intensity = 1000.0          # point-light intensity for sensor scene illumination

render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once: physics steps per frame

# === System & vehicle (created by the veh.FEDA wrapper) ===
# The wrapper builds and owns its ChSystemSMC plus the chassis, spindles, and
# suspension/steering joints internally; we configure, initialize, then borrow handles.
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_SMC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_TMEASY)   # TMEASY: stable on rigid terrain (PAC02 can diverge here)
feda.SetTireStepSize(tire_step)
feda.Initialize()

feda.SetChassisVisualizationType(chrono.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(chrono.VisualizationType_MESH)
feda.SetTireVisualizationType(chrono.VisualizationType_MESH)

# Borrow the wrapper-created system + bodies into named locals (visible essentials).
system = feda.GetSystem()                     # ChSystemSMC owned by the FEDA wrapper
chassis_body = feda.GetChassisBody()          # cache: main chassis rigid body, reused every step
# wheels/spindles: feda.GetVehicle().GetAxles()[i] ; joints: suspension + steering inside wrapper

# === Collision system === Bullet narrow-phase for the vehicle/terrain contact pairs.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch textured with grass under the vehicle.
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass surface
terrain.Initialize()

# Assert the wheels rest on (not through) the terrain after Initialize.
veh_obj = feda.GetVehicle()
spindle_bottom_z = min(
    veh_obj.GetSpindlePos(axle, side).z
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - veh_obj.GetAxle(0).GetWheels()[0].GetTire().GetRadius()
assert spindle_bottom_z >= -0.10, (
    f"vehicle sinks into terrain: wheel bottom z={spindle_bottom_z:.3f} "
    f"vs terrain top z=0.0; raise init_z by {-spindle_bottom_z:.3f} m"
)

# === Driver === scripted open-loop control (no human-in-the-loop in batch runs).
class ScriptedDriver(veh.ChDriver):
    """Gentle forward acceleration with a slow steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 1.0:
            self.SetThrottle(0.0)
            self.SetBraking(0.4)        # settle on the terrain first
        else:
            self.SetThrottle(0.5)
            self.SetBraking(0.0)
        self.SetSteering(0.15 * math.sin(0.4 * time))

driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Sensor manager & lighting === scene illumination for the OptiX FPV camera.
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(50, 50, 100),
                            chrono.ChColor(1.0, 1.0, 1.0), light_intensity)
manager.scene.AddPointLight(chrono.ChVector3f(-50, -50, 100),
                            chrono.ChColor(1.0, 1.0, 1.0), light_intensity)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.4, 0.4, 0.4))  # fill so the scene is well lit

# === FPV camera sensor === rigidly attached to the chassis for a first-person view.
# Offset frame: just ahead of and above the chassis origin, looking forward (+X local).
fpv_offset = chrono.ChVector3d(1.6, 0.0, 1.1)
fpv_frame = chrono.ChFramed(fpv_offset, chrono.QUNIT)  # +X local = forward = viewing dir
fpv_cam = sens.ChCameraSensor(
    chassis_body,          # rides on the chassis -> first-person view that moves with the vehicle
    cam_update_rate,       # Hz
    fpv_frame,
    cam_res_w, cam_res_h,
    cam_fov,
)
fpv_cam.SetName("fpv_camera")
fpv_cam.PushFilter(sens.ChFilterVisualize(cam_res_w, cam_res_h))  # render the FPV image to a live window
fpv_cam.PushFilter(sens.ChFilterSave("cam/sensor_fpv/"))          # PNG frames -> sensor mp4
fpv_cam.PushFilter(sens.ChFilterRGBA8Access())                    # frame-buffer access
manager.AddSensor(fpv_cam)

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + lights + logo.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA First-Person View")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Output setup === ensure the sensor frame output directory exists.

# === Main loop === render once per frame; advance the full vehicle stack in the inner batch.
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            manager.Update()   # pump the FPV sensor every physics step (sees each post-step pose)


            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            feda.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(time_step)
            terrain.Advance(time_step)
            feda.Advance(time_step)          # advances the wrapper-owned system
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review + sensor videos and plot logged motion.
