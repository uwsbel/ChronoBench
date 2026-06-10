"""Gator wheeled-vehicle demo on flat rigid terrain with an onboard chassis camera.

Models a Gator utility vehicle (SMC contact, TMEASY tires) driven by a scripted
time-based driver across a flat RigidTerrain patch. The vehicle subsystems
(chassis, suspension, steering, wheels, tires) are rendered with deliberately
mixed visualization types via veh.ChWheeledVehicleVisualSystemIrrlicht with a
chase camera. A ChSensorManager carries point lights plus a ChCameraSensor rigidly
attached to the chassis body, producing an onboard point-of-view image stream.

Expected behavior: the Gator launches from rest, accelerates forward under throttle
with a gentle sinusoidal steering sweep, and stays upright on the terrain. The onboard
camera shows the forward driving view tracking with the chassis.

System type: SMC (the Gator wrapper owns its ChSystemSMC); collision uses Bullet.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Parameters === geometry / physics constants and derived scheduling values
time_step = 2e-3                       # integration step (s)
tire_step = 1e-3                       # tire force-model substep (s)
sim_end = 12.0                         # total simulated time (s)
render_fps = 50.0                      # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
cam_update_rate = 30.0                 # onboard camera Hz (modest -> render < timeout)
cam_w, cam_h = 1280, 720               # onboard camera resolution
cam_fov = 1.408                        # onboard camera horizontal FOV (rad)

terrain_length = 200.0                 # terrain patch X extent (m)
terrain_width = 100.0                  # terrain patch Y extent (m)
terrain_height = 0.0                   # flat terrain top Z (m)
suspension_ref_height = 0.5            # chassis-origin height above wheel-bottom at rest
init_x, init_y = -80.0, 0.0            # spawn XY on the long patch
init_z = terrain_height + suspension_ref_height   # derived spawn Z
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)


# === Driver === scripted time-based ChDriver subclass (no human-in-the-loop)
class GatorDriver(veh.ChDriver):
    """Open-loop driver: brief settle, then accelerate with a gentle steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 1.0:
            self.SetThrottle(0.0)      # let the vehicle settle on its suspension
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)      # steady forward drive
            self.SetBraking(0.0)
        self.SetSteering(0.25 * math.sin(0.4 * time))   # gentle sinusoidal sweep


# === Vehicle === build the Gator wrapper, configure tires/contact, then Initialize
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_SMC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so the vehicle drives
gator.SetTireStepSize(tire_step)
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.Initialize()

# Mixed visualization types across the vehicle subsystems (as requested).
gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.Gator wrapper) ===
sys = gator.GetSystem()                       # ChSystemSMC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet contacts for terrain/tires
chassis = gator.GetChassisBody()              # cache: main chassis rigid body, reused every step
veh_obj = gator.GetVehicle()                  # cache: vehicle handle for spindle queries
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links created inside the wrapper; terrain: RigidTerrain patch body added below.

# Footprint check: wheels must start on (not through) the flat terrain.
TIRE_RADIUS = 0.285                           # Gator tire radius (m), from wheel geometry
ZTOL = 0.08                                   # allowed wheel-bottom clearance/overlap (m)
spindle_world = []
for axle_i in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle_i, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_height - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_height:.3f}; raise suspension_ref_height"
)

# === Terrain === flat rigid patch under the vehicle
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver instance === scripted controller attached to the vehicle
driver = GatorDriver(veh_obj)
driver.Initialize()

# === Sensors === ChSensorManager with point lights + onboard chassis camera
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100, -100, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))   # no directional light on ChScene

# Onboard camera: rigidly mounted on the chassis, looking forward (+X) over the hood.
onboard_offset = chrono.ChFramed(
    chrono.ChVector3d(0.5, 0.0, 1.2),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 0, 1)),
)
cam = sens.ChCameraSensor(chassis, cam_update_rate, onboard_offset, cam_w, cam_h, cam_fov)
cam.SetName("onboard_cam")
cam.PushFilter(sens.ChFilterVisualize(cam_w, cam_h))   # live preview window
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))   # PNG frames -> sensor video
cam.PushFilter(sens.ChFilterRGBA8Access())             # frame-buffer access
manager.AddSensor(cam)

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + lights + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.5), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height + 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)                     # enables the steering/throttle/brake HUD

# === Main loop === throttled render + Synchronize/Advance subsystem stepping

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()
            driver.Synchronize(sim_time)        # update scripted inputs for THIS step first
            driver_inputs = driver.GetInputs()  # HUD/vehicle read current-step inputs
            terrain.Synchronize(sim_time)
            gator.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            manager.Update()                    # pump the onboard camera every physics step
            driver.Advance(time_step)
            terrain.Advance(time_step)
            gator.Advance(time_step)            # advances the wrapper-owned ChSystemSMC
            vis.Advance(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:       # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
